#ifndef CUP_DETECTION_H
#define CUP_DETECTION_H

#include <stdint.h>
#include <stdbool.h>

// Cup material types
typedef enum {
    CUP_TYPE_UNKNOWN = 0,
    CUP_TYPE_PLASTIC,
    CUP_TYPE_PAPER,
    CUP_TYPE_INVALID
} cup_type_t;

// Cup detection status
typedef enum {
    CUP_DETECT_OK = 0,
    CUP_DETECT_ERROR_SENSOR,
    CUP_DETECT_ERROR_POSITION,
    CUP_DETECT_ERROR_MATERIAL
} cup_detect_status_t;

// Cup characteristics
typedef struct {
    cup_type_t material;
    uint16_t weight;        // in grams * 10
    uint16_t height;        // in mm
    uint16_t diameter;      // in mm
    uint8_t transparency;   // 0-100%
    uint8_t moisture;       // 0-100%
} cup_characteristics_t;

// Initialize cup detection module
void cup_init(void);

// Process cup detection cycle
void cup_process_cycle(void);

// Start cup detection sequence
cup_detect_status_t cup_start_detection(void);

// Get current cup characteristics
const cup_characteristics_t* cup_get_characteristics(void);

// Check if cup is present in detection zone
bool cup_is_present(void);

// Get current detection status
cup_detect_status_t cup_get_status(void);

// Calibrate sensors
cup_detect_status_t cup_calibrate_sensors(void);

// Manual override for cup type (for testing/maintenance)
void cup_set_type_override(cup_type_t type);

#endif // CUP_DETECTION_H
