/**
 * @file error_codes.h
 * @brief Error handling system interface
 */

#ifndef ERROR_CODES_H
#define ERROR_CODES_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/**
 * @brief Error code type definition
 */
typedef uint16_t error_code_t;

/**
 * @brief Error category type definition
 */
typedef uint8_t error_category_t;

/* Configuration constants */
#define MAX_ERROR_HISTORY           5    /* Maximum number of errors to store */
#define MAX_RECOVERY_ATTEMPTS       3    /* Maximum recovery attempts per error */
#define CRITICAL_RETRY_THRESHOLD    3    /* Retries before critical failure */
#define CONSECUTIVE_ERROR_THRESHOLD 3    /* Consecutive errors before escalation */
#define MAX_RECOVERY_TIMEOUT_MS     1000 /* Maximum time to wait for recovery */
#define MIN_RECOVERY_INTERVAL_MS    100  /* Minimum time between recovery attempts */
#define RECOVERY_BACKOFF_FACTOR     2    /* Exponential backoff factor for retries */
#define MAX_CONSECUTIVE_RECOVERIES  3    /* Maximum consecutive recovery attempts */

/**
 * @brief Error category definitions (high 4 bits: 0xX0)
 * Each category must be unique and a multiple of 0x10
 */
#define ERR_CAT_SUCCESS       0x00 /* Success/no error */
#define ERR_CAT_COMMON        0x10 /* Common errors */
#define ERR_CAT_SENSOR        0x20 /* Sensor-related errors */
#define ERR_CAT_MOTOR         0x40 /* Motor-related errors */
#define ERR_CAT_DOOR          0x50 /* Door-related errors */
#define ERR_CAT_SYSTEM        0x70 /* System-level errors */
#define ERR_CAT_DRIVER        0x80 /* Driver-related errors */
#define ERR_CAT_PERIPHERAL    0x90 /* Peripheral errors */
#define ERR_CAT_APPLICATION   0xA0 /* Application errors */
#define ERR_CAT_CRITICAL      0xF0 /* Critical errors */

/* Error category bit manipulation */
#define ERR_CAT_SHIFT     4      /* Shift amount to get category from error code */
#define ERR_CAT_MASK      0xF0   /* Mask to isolate category bits */
#define ERR_SPECIFIC_MASK 0x0F   /* Mask for specific error bits */
#define ERR_BASE_MASK     0x0F00 /* Mask for base offset bits */

/* Error code helper macros */
#define MAKE_ERROR_CODE(category, specific) ((category) | (specific))
#define GET_ERROR_SEVERITY(category)                                                               \
  ((category) >= ERR_CAT_CRITICAL ? ERR_SEV_CRITICAL                                               \
   : (category) >= ERR_CAT_SYSTEM ? ERR_SEV_NORMAL                                                 \
   : (category) >= ERR_CAT_COMMON ? ERR_SEV_WARNING                                                \
                                  : ERR_SEV_INFO)

/* Error code validation */
#define IS_VALID_ERROR_CODE(code)                                                                  \
  (IS_VALID_ERROR_CATEGORY(((code) >> ERR_CAT_SHIFT) & ERR_CAT_MASK))
#define IS_VALID_ERROR_CATEGORY(cat) (((cat) <= ERR_CAT_MAX) && (((cat) & 0x0F) == 0))

/* Error severity strings for logging */
extern const char *const ERROR_SEVERITY_STRINGS[];

/* Subsystem base offsets */
#define ERR_BASE_OFFSET     0x100 /* Standard base offset */
#define ERR_EXTENDED_OFFSET 0x200 /* Extended offset for subsystems with many errors */

/**
 * @brief Common error codes (0x10-0x1F)
 */
#define ERR_SUCCESS              0x00                     /* Operation successful */
#define ERR_INVALID_PARAM        (ERR_CAT_COMMON | 0x01) /* Invalid parameter */
#define ERR_NOT_INIT            (ERR_CAT_COMMON | 0x02) /* Not initialized */
#define ERR_NO_RESOURCE         (ERR_CAT_COMMON | 0x03) /* No resource available */
#define ERR_ALREADY_EXISTS      (ERR_CAT_COMMON | 0x04) /* Already exists */
#define ERR_NOT_FOUND          (ERR_CAT_COMMON | 0x05) /* Not found */
#define ERR_OUT_OF_MEMORY      (ERR_CAT_COMMON | 0x06) /* Memory allocation failed */
#define ERR_TIMEOUT            (ERR_CAT_COMMON | 0x07) /* Operation timed out */
#define ERR_BUSY              (ERR_CAT_COMMON | 0x08) /* Resource busy */
#define ERR_NOT_SUPPORTED     (ERR_CAT_COMMON | 0x09) /* Not supported */
#define ERR_INVALID_STATE     (ERR_CAT_COMMON | 0x0A) /* Invalid state */
#define ERR_OVERFLOW          (ERR_CAT_COMMON | 0x0B) /* Overflow */
#define ERR_UNDERFLOW         (ERR_CAT_COMMON | 0x0C) /* Underflow */
#define ERR_BUFFER_FULL       (ERR_CAT_COMMON | 0x0D) /* Buffer full */
#define ERR_ALREADY_INIT      (ERR_CAT_COMMON | 0x0E) /* Already initialized */
#define ERR_INIT_FAILED       (ERR_CAT_COMMON | 0x0F) /* Initialization failed */

/**
 * @brief Motor error codes (0x40-0x4F)
 */
#define ERR_MOTOR_STALL       (ERR_CAT_MOTOR | 0x01) /* Motor stall detected */
#define ERR_MOTOR_OVERCURRENT (ERR_CAT_MOTOR | 0x02) /* Overcurrent detected */
#define ERR_MOTOR_OVERTEMP    (ERR_CAT_MOTOR | 0x03) /* Overtemperature detected */
#define ERR_MOTOR_POSITION    (ERR_CAT_MOTOR | 0x04) /* Position error */
#define ERR_MOTOR_LIMIT       (ERR_CAT_MOTOR | 0x05) /* Limit switch triggered */
#define ERR_MOTOR_COMM        (ERR_CAT_MOTOR | 0x06) /* Communication error */
#define ERR_MOTOR_CONFIG      (ERR_CAT_MOTOR | 0x07) /* Configuration error */
#define ERR_MOTOR_SYSTEM      (ERR_CAT_MOTOR | 0x08) /* System error */
#define ERR_MOTOR_NOT_INIT    (ERR_CAT_MOTOR | 0x09) /* Motor not initialized */
#define ERR_MOTOR_NOT_ENABLED (ERR_CAT_MOTOR | 0x0A) /* Motor not enabled */
#define ERR_MOTOR_BUSY        (ERR_CAT_MOTOR | 0x0B) /* Motor busy */
#define ERR_MOTOR_OBSTACLE    (ERR_CAT_MOTOR | 0x0C) /* Obstacle detected */

/**
 * @brief UART error codes (0x801-0x80F)
 */
#define ERR_UART_FRAME        ((ERR_CAT_PERIPHERAL | ERR_BASE_OFFSET) | 0x01) /* Frame error */
#define ERR_UART_PARITY       ((ERR_CAT_PERIPHERAL | ERR_BASE_OFFSET) | 0x02) /* Parity error */
#define ERR_UART_NO_DATA      ((ERR_CAT_PERIPHERAL | ERR_BASE_OFFSET) | 0x03) /* No data */
#define ERR_UART_OVERFLOW     ((ERR_CAT_PERIPHERAL | ERR_BASE_OFFSET) | 0x04) /* Buffer overflow */
#define ERR_UART_OVERRUN      ((ERR_CAT_PERIPHERAL | ERR_BASE_OFFSET) | 0x05) /* Overrun */
#define ERR_UART_NOT_ENABLED  ((ERR_CAT_PERIPHERAL | ERR_BASE_OFFSET) | 0x06) /* Not enabled */
#define ERR_UART_NOISE        ((ERR_CAT_PERIPHERAL | ERR_BASE_OFFSET) | 0x07) /* Noise detected */
#define ERR_UART_BREAK        ((ERR_CAT_PERIPHERAL | ERR_BASE_OFFSET) | 0x08) /* Break detected */
#define ERR_UART_INVALID_PORT ((ERR_CAT_PERIPHERAL | ERR_BASE_OFFSET) | 0x09) /* Invalid port */
#define ERR_UART_FLUSH_FAILED ((ERR_CAT_PERIPHERAL | ERR_BASE_OFFSET) | 0x0A) /* Flush failed */
#define ERR_UART_REGS         ((ERR_CAT_PERIPHERAL | ERR_BASE_OFFSET) | 0x0B) /* Register error */

/**
 * @brief Clock error codes (0x901-0x90F)
 */
#define ERR_CLOCK_INVALID_PRESCALER ((ERR_CAT_PERIPHERAL | ERR_EXTENDED_OFFSET) | 0x01) /* Invalid prescaler */

/**
 * @brief Timer error codes (0x701-0x740)
 */
/* Core timer errors (0x701-0x710) */
#define ERR_TIMER_INVALID_ID     ((ERR_CAT_DRIVER | ERR_BASE_OFFSET) | 0x01) /* Invalid ID */
#define ERR_TIMER_NOT_ALLOC      ((ERR_CAT_DRIVER | ERR_BASE_OFFSET) | 0x02) /* Not allocated */
#define ERR_TIMER_ALLOC_FAILED   ((ERR_CAT_DRIVER | ERR_BASE_OFFSET) | 0x03) /* Allocation failed */
#define ERR_TIMER_BUSY           ((ERR_CAT_DRIVER | ERR_BASE_OFFSET) | 0x04) /* Busy */
#define ERR_TIMER_NOT_RUNNING    ((ERR_CAT_DRIVER | ERR_BASE_OFFSET) | 0x05) /* Not running */

/* Timer configuration errors (0x711-0x720) */
#define ERR_TIMER_INVALID_MODE     ((ERR_CAT_DRIVER | ERR_BASE_OFFSET) | 0x11) /* Invalid mode */
#define ERR_TIMER_INVALID_DIV      ((ERR_CAT_DRIVER | ERR_BASE_OFFSET) | 0x12) /* Invalid prescaler */
#define ERR_TIMER_INVALID_PERIOD   ((ERR_CAT_DRIVER | ERR_BASE_OFFSET) | 0x13) /* Invalid period */
#define ERR_TIMER_INVALID_COMPARE  ((ERR_CAT_DRIVER | ERR_BASE_OFFSET) | 0x14) /* Invalid compare */
#define ERR_TIMER_INVALID_PRIORITY ((ERR_CAT_DRIVER | ERR_BASE_OFFSET) | 0x15) /* Invalid priority */

/* Timer PWM errors (0x721-0x730) */
#define ERR_TIMER_INVALID_PWM      ((ERR_CAT_DRIVER | ERR_BASE_OFFSET) | 0x21) /* Invalid PWM */
#define ERR_TIMER_INVALID_CHANNEL  ((ERR_CAT_DRIVER | ERR_BASE_OFFSET) | 0x22) /* Invalid channel */
#define ERR_TIMER_INVALID_DUTY     ((ERR_CAT_DRIVER | ERR_BASE_OFFSET) | 0x23) /* Invalid duty */
#define ERR_TIMER_INVALID_ALIGN    ((ERR_CAT_DRIVER | ERR_BASE_OFFSET) | 0x24) /* Invalid align */
#define ERR_TIMER_INVALID_DEADTIME ((ERR_CAT_DRIVER | ERR_BASE_OFFSET) | 0x25) /* Invalid deadtime */

/* Timer runtime errors (0x731-0x740) */
#define ERR_TIMER_CALLBACK_FAILED  ((ERR_CAT_DRIVER | ERR_BASE_OFFSET) | 0x31) /* Callback failed */
#define ERR_TIMER_OVERFLOW         ((ERR_CAT_DRIVER | ERR_BASE_OFFSET) | 0x32) /* Overflow */
#define ERR_TIMER_RECOVERY_FAILED  ((ERR_CAT_DRIVER | ERR_BASE_OFFSET) | 0x33) /* Recovery failed */

/**
 * @brief Peripheral error codes (0xB01-0xB0F)
 */
#define ERR_PERIPHERAL_DISABLE     ((ERR_CAT_PERIPHERAL | 0x300) | 0x01) /* Disable failed */
#define ERR_PERIPHERAL_EEPROM      ((ERR_CAT_PERIPHERAL | 0x300) | 0x02) /* EEPROM error */
#define ERR_HARDWARE              ((ERR_CAT_PERIPHERAL | 0x300) | 0x03) /* Hardware error */

/**
 * @brief Module error codes (0x901-0x90F)
 */
#define ERR_MODULE_NOT_FOUND      (ERR_CAT_APPLICATION | ERR_NOT_FOUND)  /* Not found */
#define ERR_MODULE_EXISTS         (ERR_CAT_APPLICATION | ERR_ALREADY_EXISTS) /* Already exists */
#define ERR_MODULE_STARTED        (ERR_CAT_APPLICATION | 0x03) /* Started */
#define ERR_MODULE_NOT_STARTED    (ERR_CAT_APPLICATION | 0x04) /* Not started */
#define ERR_MODULE_DEP_EXISTS     (ERR_CAT_APPLICATION | 0x05) /* Dep exists */
#define ERR_MODULE_NO_RESOURCE    (ERR_CAT_APPLICATION | ERR_NO_RESOURCE) /* No resource */
#define ERR_MODULE_UNSUPPORTED    (ERR_CAT_APPLICATION | ERR_NOT_SUPPORTED) /* Not supported */

/**
 * @brief Critical error codes (0xFX)
 */
#define ERR_CRITICAL_FAILURE (ERR_CAT_CRITICAL | 0x0F) /* System critical failure */

/* Error severity levels */
typedef enum {
  ERR_SEV_INFO     = 0x00, /* Informational only */
  ERR_SEV_WARNING  = 0x01, /* Warning, operation can continue */
  ERR_SEV_ERROR    = 0x02, /* Error, operation may be impaired */
  ERR_SEV_CRITICAL = 0x03  /* Critical, operation cannot continue */
} error_severity_t;

/* Error handling flags */
typedef enum {
  ERROR_FLAG_NONE           = 0x00,
  ERROR_FLAG_RECOVERABLE    = 0x01, /* Error can be recovered from */
  ERROR_FLAG_PERSISTENT     = 0x02, /* Error persists until explicitly cleared */
  ERROR_FLAG_REQUIRES_RESET = 0x04, /* Error requires system reset */
  ERROR_FLAG_LOG_ONLY       = 0x08  /* Error should only be logged */
} error_flags_t;

/* Error context flags */
typedef enum {
  ERR_CTX_NONE        = 0x0000,
  ERR_CTX_STARTUP     = 0x0001, /* Error occurred during startup */
  ERR_CTX_SHUTDOWN    = 0x0002, /* Error occurred during shutdown */
  ERR_CTX_INTERRUPT   = 0x0004, /* Error occurred in interrupt context */
  ERR_CTX_CRITICAL    = 0x0008, /* Error occurred in critical section */
  ERR_CTX_LOW_POWER   = 0x0010, /* Error occurred in low power mode */
  ERR_CTX_RECOVERY    = 0x0020, /* Error occurred during recovery */
  ERR_CTX_CALIBRATION = 0x0040, /* Error occurred during calibration */
  ERR_CTX_TEST        = 0x0080  /* Error occurred during testing */
} error_context_flags_t;

/* Extended error information */
typedef struct {
  uint32_t timestamp;        /* Time when error occurred */
  uint32_t uptime;           /* System uptime when error occurred */
  uint16_t context_flags;    /* Error context flags */
  uint8_t task_id;           /* ID of task where error occurred */
  uint8_t core_id;           /* ID of core where error occurred */
  void *caller_address;      /* Address of caller */
  const char *function_name; /* Name of function where error occurred */
} error_extended_info_t;

/* Error recovery status */
typedef struct {
  uint32_t last_attempt_time;   /* Timestamp of last recovery attempt */
  uint8_t consecutive_attempts; /* Count of consecutive recovery attempts */
  uint16_t backoff_interval;    /* Current backoff interval */
  bool recovery_in_progress;    /* Flag indicating recovery in progress */
} error_recovery_status_t;

/* Error callback type */
typedef void (*error_callback_t)(error_code_t code, error_severity_t severity, void *context);

/**
 * @brief Extended error record structure
 */
typedef struct {
  error_code_t code;                   /**< Error code */
  uint32_t timestamp;                  /**< Time when error occurred */
  uint8_t retry_count;                 /**< Number of retries attempted */
  uint8_t resolved;                    /**< Whether error was resolved */
  uint16_t context;                    /**< Additional context information */
  error_severity_t severity;           /**< Error severity level */
  error_flags_t flags;                 /**< Error handling flags */
  const char *description;             /**< Error description string */
  const char *location;                /**< Error location (file:line) */
  error_extended_info_t extended_info; /**< Extended error information */
} error_record_t;

/**
 * @brief Error history structure
 */
typedef struct {
  error_record_t history[MAX_ERROR_HISTORY]; /**< Array of error records */
  uint8_t current_index;                     /**< Current index in history */
  uint8_t total_errors;                      /**< Total number of errors */
} error_history_t;

/**
 * @brief Error category statistics
 */
typedef struct {
  uint32_t count;           /* Total errors in this category */
  uint32_t recovered_count; /* Successfully recovered errors */
  uint32_t last_time;       /* Timestamp of last error in this category */
  uint8_t consecutive;      /* Count of consecutive errors in this category */
} error_category_stats_t;

/**
 * @brief Error statistics structure
 */
typedef struct {
  uint32_t total_errors;                     /**< Total number of errors recorded */
  uint32_t recovered_errors;                 /**< Number of errors successfully recovered */
  uint32_t critical_errors;                  /**< Number of critical errors */
  uint32_t last_error_time;                  /**< Timestamp of last error */
  error_category_stats_t category_stats[16]; /**< Detailed stats per category */
} error_stats_t;

/* Error handling helper macros */
#define ERROR_LOCATION() __FILE__ ":" STRINGIFY(__LINE__)
#define ERROR_RECORD_EXTENDED(code, sev, flg, desc)                                                \
  error_record_extended(code, sev, flg, desc, ERROR_LOCATION())

/* Error record access helpers */
#define GET_ERROR_RECORD(history, idx)    (&(history)->history[(idx) % MAX_ERROR_HISTORY])
#define GET_CURRENT_ERROR_RECORD(history) GET_ERROR_RECORD(history, (history)->current_index)

/* Error severity helpers */
#define IS_CRITICAL_ERROR(category) ((category) >= ERR_CAT_CRITICAL)
#define GET_DEFAULT_SEVERITY(category)                                                             \
  (IS_CRITICAL_ERROR(category) ? ERR_SEV_CRITICAL : ERR_SEV_ERROR)

/* Error category helpers */
#define GET_ERROR_CATEGORY(code) ((error_category_t)(((code) >> ERR_CAT_SHIFT) & ERR_CAT_MASK))

/* Error code validation enhancements */
#define IS_ERROR_IN_CATEGORY(code, category) (((code) & ERR_CAT_MASK) == (category))
#define IS_RECOVERABLE_ERROR(code) (!IS_CRITICAL_ERROR((code) & ERR_CAT_MASK))
#define HAS_HIGHER_SEVERITY(code1, code2) \
    (GET_ERROR_SEVERITY((code1) & ERR_CAT_MASK) > GET_ERROR_SEVERITY((code2) & ERR_CAT_MASK))

/**
 * @brief Error context structure for detailed error information
 */
typedef struct {
    error_code_t code;           /* The error code */
    const char* file;           /* Source file where error occurred */
    int line;                   /* Line number where error occurred */
    const char* function;       /* Function where error occurred */
    uint32_t timestamp;         /* Time when error occurred */
    void* additional_context;   /* Optional additional context */
} error_context_t;

/**
 * @brief Create an error context
 * @param code Error code
 * @param file Source file
 * @param line Line number
 * @param function Function name
 * @return Populated error context
 */
error_context_t error_create_context(error_code_t code, const char* file, 
                                   int line, const char* function);

/**
 * @brief Get human-readable error message
 * @param error_code Error code to translate
 * @return Constant string containing error message
 */
const char* error_get_message(error_code_t error_code);

/**
 * @brief Combine two error codes, keeping the higher severity
 * @param error1 First error code
 * @param error2 Second error code
 * @return Combined error code
 */
error_code_t error_combine(error_code_t error1, error_code_t error2);

/* Convenience macro for creating error context */
#define ERROR_CONTEXT(code) error_create_context(code, __FILE__, __LINE__, __func__)

/* Static assertions for error code uniqueness */
_Static_assert((ERR_CAT_SUCCESS & ERR_CAT_COMMON) == 0, "Error categories must not overlap");
_Static_assert((ERR_CAT_SENSOR & ERR_CAT_MOTOR) == 0, "Error categories must not overlap");
_Static_assert((ERR_CAT_CRITICAL & 0x0F) == 0, "Error categories must be properly aligned");

/* Function declarations */

/**
 * @brief Initialize the error handling system
 * @return ERR_SUCCESS on success, error code otherwise
 */
error_code_t error_init(void);

/**
 * @brief Record an error in the error history
 * @param error_code Error code to record
 */
void error_record(error_code_t error_code);

/**
 * @brief Get the last recorded error code
 * @return Last recorded error code
 */
error_code_t error_get_last(void);

/**
 * @brief Clear the last recorded error
 */
void error_clear(void);

/**
 * @brief Attempt to recover from an error
 * @param error_code Error code to recover from
 * @return 1 if recovery successful, 0 otherwise
 */
uint8_t error_attempt_recovery(error_code_t error_code);

/**
 * @brief Get the error history
 * @return Pointer to error history structure
 */
volatile error_history_t *error_get_history(void);

/**
 * @brief Print the error history status
 */
void error_print_status(void);

/**
 * @brief Get error category from error code
 * @param error_code Error code to get category from
 * @return Error category
 */
static inline uint8_t error_get_category(error_code_t error_code)
{
  return (uint8_t)(error_code & 0xF0);
}

/**
 * @brief Get specific error from error code
 * @param error_code Error code to get specific error from
 * @return Specific error code
 */
static inline uint8_t error_get_specific(error_code_t error_code)
{
  return (uint8_t)(error_code & 0x0F);
}

/**
 * @brief Register an error callback
 * @param callback Error callback function
 * @param context Context for the callback
 * @return ERR_SUCCESS on success, error code otherwise
 */
error_code_t error_register_callback(error_callback_t callback, void *context);

/**
 * @brief Unregister an error callback
 * @param callback Error callback function
 * @return ERR_SUCCESS on success, error code otherwise
 */
error_code_t error_unregister_callback(error_callback_t callback);

/**
 * @brief Get the description of an error code
 * @param error_code Error code to get description for
 * @return Error description string
 */
const char *error_get_description(error_code_t error_code);

/**
 * @brief Get the severity of an error code
 * @param error_code Error code to get severity for
 * @return Error severity level
 */
error_severity_t error_get_severity(error_code_t error_code);

/**
 * @brief Get the flags of an error code
 * @param error_code Error code to get flags for
 * @return Error flags
 */
error_flags_t error_get_flags(error_code_t error_code);

/**
 * @brief Record an extended error in the error history
 * @param code Error code to record
 * @param severity Error severity level
 * @param flags Error handling flags
 * @param description Error description string
 * @param location Error location (file:line)
 */
void error_record_extended(error_code_t code, error_severity_t severity, error_flags_t flags,
                           const char *description, const char *location);

#define ERR_CAT_COMMUNICATION 0x60 /* Communication errors */
#define ERR_CAT_SYSTEM        0x70 /* System-level errors */

/* Function declarations */

#endif /* ERROR_CODES_H */
