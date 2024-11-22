#ifndef CFG_HARDWARE_H
#define CFG_HARDWARE_H

#include "../platform/common/plat_types.h"
#include "cfg_debug.h"

// UART Configuration
#define UART_BAUD_RATE    115200UL
#define UART_BUFFER_SIZE  256
typedef enum {
    BAUD_9600   = 9600UL,
    BAUD_19200  = 19200UL,
    BAUD_38400  = 38400UL,
    BAUD_57600  = 57600UL,
    BAUD_115200 = 115200UL
} uart_baud_t;

// Hardware Limits
#define MAX_COMPONENTS 16
#define MAX_SENSORS 8
#define MAX_MOTORS 4

// Timing Configuration
#define SYSTEM_TICK_MS 10
#define SENSOR_SAMPLE_INTERVAL_MS 100
#define MOTOR_UPDATE_INTERVAL_MS 50
#define NETWORK_TIMEOUT_MS 5000

// Safety Thresholds with Hysteresis
typedef struct {
    int16_t threshold;
    int16_t hysteresis;
    uint16_t debounce_ms;
} safety_threshold_t;

// Temperature Thresholds
#define MOTOR_TEMP_WARN_C      70
#define MOTOR_TEMP_CRITICAL_C  80
#define MOTOR_TEMP_HYSTERESIS  5
#define TEMP_DEBOUNCE_MS      1000

// Current Thresholds
#define SYSTEM_CURRENT_WARN_MA     4500
#define SYSTEM_CURRENT_CRITICAL_MA 5000
#define CURRENT_HYSTERESIS_MA      200
#define CURRENT_DEBOUNCE_MS        100

// Voltage Thresholds
#define SUPPLY_VOLTAGE_MIN_MV      4500
#define SUPPLY_VOLTAGE_MAX_MV      5500
#define SUPPLY_VOLTAGE_WARN_MV     4700
#define VOLTAGE_HYSTERESIS_MV      100
#define VOLTAGE_DEBOUNCE_MS        50
#define VOLTAGE_SAG_THRESHOLD_MV   300

#endif // CFG_HARDWARE_H
