#ifndef __ERROR_HANDLING_H__
#define __ERROR_HANDLING_H__

#include <stdint.h>
#include "types.h"

/**
 * @brief Error code type
 */
typedef uint32_t error_code_t;

/**
 * @brief Error categories (preserved for backward compatibility)
 */
#define ERR_CAT_NONE          0x00000000
#define ERR_CAT_SENSOR        0x10000000
#define ERR_CAT_MOTOR         0x20000000
#define ERR_CAT_DOOR          0x30000000
#define ERR_CAT_COMMUNICATION 0x40000000
#define ERR_CAT_SYSTEM        0x50000000
#define ERR_CAT_DRIVER        0x60000000
#define ERR_CAT_PERIPHERAL    0x70000000
#define ERR_CAT_APPLICATION   0x80000000

/**
 * @brief Common error codes
 */
#define ERR_SUCCESS           0x00000000
#define ERR_INVALID_PARAM     0x00000001
#define ERR_NOT_INITIALIZED   0x00000002
#define ERR_TIMEOUT          0x00000003
#define ERR_BUSY             0x00000004
#define ERR_OVERFLOW         0x00000005
#define ERR_UNDERFLOW        0x00000006
#define ERR_NOT_SUPPORTED    0x00000007
#define ERR_NOT_READY        0x00000008

/**
 * @brief Sensor error codes (preserved and extended)
 */
#define ERR_SENSOR_TIMEOUT    (ERR_CAT_SENSOR | 0x01)
#define ERR_SENSOR_RANGE      (ERR_CAT_SENSOR | 0x02)
#define ERR_SENSOR_CALIBRATION (ERR_CAT_SENSOR | 0x03)
#define ERR_SENSOR_DISCONNECT  (ERR_CAT_SENSOR | 0x04)

/**
 * @brief Motor error codes (preserved and extended)
 */
#define ERR_MOTOR_STALL       (ERR_CAT_MOTOR | 0x01)
#define ERR_MOTOR_OVERLOAD    (ERR_CAT_MOTOR | 0x02)
#define ERR_MOTOR_POSITION    (ERR_CAT_MOTOR | 0x03)
#define ERR_MOTOR_SPEED       (ERR_CAT_MOTOR | 0x04)
#define ERR_MOTOR_CURRENT     (ERR_CAT_MOTOR | 0x05)
#define ERR_MOTOR_TEMP        (ERR_CAT_MOTOR | 0x06)

/**
 * @brief Door error codes (preserved and extended)
 */
#define ERR_DOOR_JAM          (ERR_CAT_DOOR | 0x01)
#define ERR_DOOR_SENSOR       (ERR_CAT_DOOR | 0x02)
#define ERR_DOOR_LIMIT        (ERR_CAT_DOOR | 0x03)
#define ERR_DOOR_TIMEOUT      (ERR_CAT_DOOR | 0x04)

/**
 * @brief Communication error codes (preserved and extended)
 */
#define ERR_COMM_TIMEOUT      (ERR_CAT_COMMUNICATION | 0x01)
#define ERR_COMM_CHECKSUM     (ERR_CAT_COMMUNICATION | 0x02)
#define ERR_COMM_OVERFLOW     (ERR_CAT_COMMUNICATION | 0x03)
#define ERR_COMM_FRAMING      (ERR_CAT_COMMUNICATION | 0x04)
#define ERR_COMM_PARITY       (ERR_CAT_COMMUNICATION | 0x05)
#define ERR_COMM_NOISE        (ERR_CAT_COMMUNICATION | 0x06)

/**
 * @brief System error codes (preserved and extended)
 */
#define ERR_SYS_INIT          (ERR_CAT_SYSTEM | 0x01)
#define ERR_SYS_CONFIG        (ERR_CAT_SYSTEM | 0x02)
#define ERR_SYS_MEMORY        (ERR_CAT_SYSTEM | 0x03)
#define ERR_SYS_STACK         (ERR_CAT_SYSTEM | 0x04)
#define ERR_SYS_TASK          (ERR_CAT_SYSTEM | 0x05)
#define ERR_SYS_INTERRUPT     (ERR_CAT_SYSTEM | 0x06)

/**
 * @brief Driver error codes (new)
 */
#define ERR_DRIVER_INIT       (ERR_CAT_DRIVER | 0x01)
#define ERR_DRIVER_CONFIG     (ERR_CAT_DRIVER | 0x02)
#define ERR_DRIVER_STATE      (ERR_CAT_DRIVER | 0x03)
#define ERR_DRIVER_TIMEOUT    (ERR_CAT_DRIVER | 0x04)

/**
 * @brief Peripheral error codes (new)
 */
#define ERR_PERIPH_INIT       (ERR_CAT_PERIPHERAL | 0x01)
#define ERR_PERIPH_CONFIG     (ERR_CAT_PERIPHERAL | 0x02)
#define ERR_PERIPH_TIMEOUT    (ERR_CAT_PERIPHERAL | 0x03)
#define ERR_PERIPH_BUSY       (ERR_CAT_PERIPHERAL | 0x04)

// Enhanced error handling structures
typedef struct {
    uint32_t timestamp;      // Time when error occurred
    error_code_t code;       // Error code
    const char* file;        // Source file where error occurred
    uint32_t line;          // Line number where error occurred
    const char* function;    // Function where error occurred
    uint8_t severity;       // Error severity level
    void* context;          // Additional error context
} error_info_t;

// Error severity levels
typedef enum {
    ERROR_SEVERITY_INFO = 0,     // Informational, no action needed
    ERROR_SEVERITY_WARNING = 1,   // Warning, system can continue
    ERROR_SEVERITY_ERROR = 2,     // Error, needs attention
    ERROR_SEVERITY_CRITICAL = 3,  // Critical, system cannot continue
    ERROR_SEVERITY_FATAL = 4      // Fatal, immediate shutdown required
} error_severity_t;

// Error history
#define ERROR_HISTORY_SIZE 16
typedef struct {
    error_info_t errors[ERROR_HISTORY_SIZE];
    uint8_t count;
    uint8_t index;
} error_history_t;

// Error callback type
typedef void (*error_callback_t)(const error_info_t* error);

// Error history structure (preserved)
typedef struct {
    error_code_t code;       // Error code (expanded to 32-bit)
    uint32_t timestamp;      // Time when error occurred
    uint8_t retry_count;     // Number of retries attempted
    uint8_t resolved;        // Whether error was resolved
    uint16_t context;        // Additional context information
} ErrorRecord;

#define MAX_ERROR_HISTORY 10

typedef struct {
    ErrorRecord history[MAX_ERROR_HISTORY];
    uint8_t current_index;
    uint8_t total_errors;
} ErrorHistory;

// Function declarations
void error_init(void);
void error_record(error_code_t error_code);
error_code_t error_get_last(void);
void error_clear(void);
uint8_t error_attempt_recovery(error_code_t error_code);
ErrorHistory* error_get_history(void);
void error_print_status(void);

/**
 * @brief Get error category from error code
 */
static inline uint32_t error_get_category(error_code_t error_code) {
    return (error_code & 0xF0000000);
}

/**
 * @brief Get specific error from error code
 */
static inline uint32_t error_get_specific(error_code_t error_code) {
    return (error_code & 0x0000FFFF);
}

#endif // __ERROR_HANDLING_H__
