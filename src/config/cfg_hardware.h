#ifndef CFG_HARDWARE_H
#define CFG_HARDWARE_H

#include "../platform/common/plat_types.h"

// UART Configuration
#define BAUD_9600     9600UL
#define BAUD_19200    19200UL
#define BAUD_38400    38400UL
#define BAUD_57600    57600UL
#define BAUD_115200   115200UL

// System Configuration Parameters
#define SYSTEM_VERSION "1.0.0"
#define MAX_COMPONENTS 16
#define MAX_SENSORS 8
#define MAX_MOTORS 4
#define MAX_ERROR_QUEUE 32

// Timing Configuration
#define SYSTEM_TICK_MS 10
#define WATCHDOG_TIMEOUT_MS 1000
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
#define SYSTEM_CURRENT_WARN_MA    4500
#define SYSTEM_CURRENT_CRITICAL_MA 5000
#define CURRENT_HYSTERESIS_MA     200
#define CURRENT_DEBOUNCE_MS       100

// Voltage Thresholds
#define SUPPLY_VOLTAGE_MIN_MV     4500
#define SUPPLY_VOLTAGE_MAX_MV     5500
#define SUPPLY_VOLTAGE_WARN_MV    4700
#define VOLTAGE_HYSTERESIS_MV     100
#define VOLTAGE_DEBOUNCE_MS       50
#define VOLTAGE_SAG_THRESHOLD_MV  300

// Watchdog Configuration
typedef struct {
    uint16_t timeout_ms;
    uint8_t reset_source;
    bool window_mode;
    uint16_t window_start_ms;
    uint16_t window_end_ms;
} watchdog_config_t;

#define WDT_TIMEOUT_NORMAL_MS    1000
#define WDT_TIMEOUT_CRITICAL_MS  250
#define WDT_WINDOW_START_MS      0
#define WDT_WINDOW_END_MS        900

// Communication Timeouts
#define PACKET_TIMEOUT_BASE_MS   50
#define PACKET_TIMEOUT_PER_BYTE_MS (PACKET_TIMEOUT_BASE_MS / 32)
#define PACKET_MAX_TIMEOUT_MS    500

// Calculate packet timeout based on size
#define CALC_PACKET_TIMEOUT(size) \
    ((size) * PACKET_TIMEOUT_PER_BYTE_MS + PACKET_TIMEOUT_BASE_MS)

// Validate timeout is within bounds
#define VALIDATE_PACKET_TIMEOUT(timeout) \
    ((timeout) < PACKET_MAX_TIMEOUT_MS ? (timeout) : PACKET_MAX_TIMEOUT_MS)

// Cup Detection Parameters
#define CUP_DETECT_TIMEOUT_MS 2000
#define CUP_DETECT_RETRIES 3
#define MIN_CUP_WEIGHT 10       // grams
#define MAX_CUP_WEIGHT 500      // grams

// Cleaning Parameters
#define MIN_WATER_TEMPERATURE 40  // Celsius
#define MAX_WATER_TEMPERATURE 60  // Celsius
#define MIN_WATER_PRESSURE 100    // kPa
#define MAX_WATER_PRESSURE 300    // kPa

// Communication Parameters
#define UART_BAUD_RATE 115200
#define MAX_PACKET_SIZE 256

// Debug Configuration
#ifdef DEBUG_BUILD
    #define DEBUG_LEVEL 2
    #define ENABLE_LOGGING 1
    #define ENABLE_ASSERTIONS 1
#else
    #define DEBUG_LEVEL 0
    #define ENABLE_LOGGING 0
    #define ENABLE_ASSERTIONS 0
#endif

#endif // CFG_HARDWARE_H
