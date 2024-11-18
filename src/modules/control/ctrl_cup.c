#include "cup_detection.h"
#include "error_handling.h"
#include "predictive_maintenance.h"
#include "timer.h"
#include "RC.h"

// Detection state machine states
typedef enum {
    DETECT_STATE_IDLE,
    DETECT_STATE_CHECKING,
    DETECT_STATE_MEASURING,
    DETECT_STATE_ANALYZING,
    DETECT_STATE_COMPLETE,
    DETECT_STATE_ERROR
} detect_state_t;

// Internal state tracking
static struct {
    detect_state_t state;
    cup_detect_status_t status;
    cup_characteristics_t characteristics;
    uint32_t detection_start_time;
    bool override_active;
    cup_type_t override_type;
    uint8_t retry_count;
    bool calibration_needed;
} cup_state;

// Timer ID for detection sequence
static uint8_t tid_detect;

// Constants
#define MAX_DETECTION_TIME_MS 2000
#define MAX_RETRY_COUNT 3
#define MIN_VALID_WEIGHT 10    // 1.0g
#define MAX_VALID_WEIGHT 500   // 50.0g
#define MIN_VALID_HEIGHT 50    // 50mm
#define MAX_VALID_HEIGHT 200   // 200mm
#define MIN_VALID_DIAMETER 50  // 50mm
#define MAX_VALID_DIAMETER 120 // 120mm

void cup_init(void)
{
    memset(&cup_state, 0, sizeof(cup_state));
    cup_state.state = DETECT_STATE_IDLE;
    cup_state.status = CUP_DETECT_OK;
    cup_state.characteristics.material = CUP_TYPE_UNKNOWN;
    cup_state.calibration_needed = true;
    
    tid_detect = timer_alloc();
    if (tid_detect == 0xFF) {
        error_record(ERR_TIMER_ALLOC);
    }
}

void cup_process_cycle(void)
{
    if (cup_state.calibration_needed) {
        cup_calibrate_sensors();
        return;
    }

    switch (cup_state.state) {
        case DETECT_STATE_IDLE:
            // Nothing to do in idle state
            break;

        case DETECT_STATE_CHECKING:
            if (!cup_is_present()) {
                cup_state.state = DETECT_STATE_IDLE;
                break;
            }
            
            // Start measurement sequence
            cup_state.state = DETECT_STATE_MEASURING;
            cup_state.detection_start_time = timer_get_ms();
            break;

        case DETECT_STATE_MEASURING:
            if (timer_get_ms() - cup_state.detection_start_time > MAX_DETECTION_TIME_MS) {
                cup_state.status = CUP_DETECT_ERROR_SENSOR;
                cup_state.state = DETECT_STATE_ERROR;
                error_record(ERR_SENSOR_TIMEOUT);
                break;
            }

            // Read sensor values
            cup_state.characteristics.weight = rc_get_weight() * 10;
            cup_state.characteristics.height = rc_get_height();
            cup_state.characteristics.diameter = rc_get_diameter();
            cup_state.characteristics.transparency = rc_get_transparency();
            cup_state.characteristics.moisture = rc_get_moisture();

            // Move to analysis once all measurements are complete
            if (rc_measurements_complete()) {
                cup_state.state = DETECT_STATE_ANALYZING;
            }
            break;

        case DETECT_STATE_ANALYZING:
            // Validate measurements
            if (!validate_measurements()) {
                if (cup_state.retry_count < MAX_RETRY_COUNT) {
                    cup_state.retry_count++;
                    cup_state.state = DETECT_STATE_MEASURING;
                    break;
                }
                cup_state.status = CUP_DETECT_ERROR_SENSOR;
                cup_state.state = DETECT_STATE_ERROR;
                error_record(ERR_SENSOR_RANGE);
                break;
            }

            // Determine cup material
            if (cup_state.override_active) {
                cup_state.characteristics.material = cup_state.override_type;
            } else {
                cup_state.characteristics.material = analyze_material();
            }

            cup_state.state = DETECT_STATE_COMPLETE;
            break;

        case DETECT_STATE_COMPLETE:
            // Stay in complete state until next detection is started
            break;

        case DETECT_STATE_ERROR:
            // Stay in error state until next detection is started
            break;
    }
}

cup_detect_status_t cup_start_detection(void)
{
    if (cup_state.state != DETECT_STATE_IDLE && 
        cup_state.state != DETECT_STATE_COMPLETE && 
        cup_state.state != DETECT_STATE_ERROR) {
        return CUP_DETECT_ERROR_POSITION;
    }

    cup_state.state = DETECT_STATE_CHECKING;
    cup_state.status = CUP_DETECT_OK;
    cup_state.retry_count = 0;
    memset(&cup_state.characteristics, 0, sizeof(cup_characteristics_t));
    
    return CUP_DETECT_OK;
}

const cup_characteristics_t* cup_get_characteristics(void)
{
    return &cup_state.characteristics;
}

bool cup_is_present(void)
{
    return rc_cup_present();
}

cup_detect_status_t cup_get_status(void)
{
    return cup_state.status;
}

cup_detect_status_t cup_calibrate_sensors(void)
{
    if (rc_calibrate_sensors() != 0) {
        error_record(ERR_SENSOR_CAL);
        return CUP_DETECT_ERROR_SENSOR;
    }
    
    cup_state.calibration_needed = false;
    return CUP_DETECT_OK;
}

void cup_set_type_override(cup_type_t type)
{
    cup_state.override_active = true;
    cup_state.override_type = type;
}

// Internal helper functions
static bool validate_measurements(void)
{
    if (cup_state.characteristics.weight < MIN_VALID_WEIGHT ||
        cup_state.characteristics.weight > MAX_VALID_WEIGHT ||
        cup_state.characteristics.height < MIN_VALID_HEIGHT ||
        cup_state.characteristics.height > MAX_VALID_HEIGHT ||
        cup_state.characteristics.diameter < MIN_VALID_DIAMETER ||
        cup_state.characteristics.diameter > MAX_VALID_DIAMETER) {
        return false;
    }
    return true;
}

static cup_type_t analyze_material(void)
{
    // High transparency and low moisture typically indicates plastic
    if (cup_state.characteristics.transparency > 70 && 
        cup_state.characteristics.moisture < 20) {
        return CUP_TYPE_PLASTIC;
    }
    
    // Low transparency and higher moisture typically indicates paper
    if (cup_state.characteristics.transparency < 30 && 
        cup_state.characteristics.moisture > 40) {
        return CUP_TYPE_PAPER;
    }
    
    // If characteristics don't clearly indicate material type
    return CUP_TYPE_INVALID;
}
