#ifndef DRV_TIMER_H
#define DRV_TIMER_H

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include "../../../config/cfg_build.h"
#include "../../../platform/common/plat_types.h"
#include "../../../utils/common/error_codes.h"
#include "../../common/drv_interface.h"
#include "../../common/drv_types.h"
#include "../common/mcu_types.h"

/**
 * @brief Timer specific error codes
 */
#define ERR_TIMER_BASE         (ERR_CAT_PERIPHERAL | 0x0200)
#define ERR_TIMER_OVERFLOW     (ERR_TIMER_BASE | 0x01)
#define ERR_TIMER_INVALID_FREQ (ERR_TIMER_BASE | 0x02)
#define ERR_TIMER_INVALID_DIV  (ERR_TIMER_BASE | 0x03)
#define ERR_TIMER_BUSY         (ERR_TIMER_BASE | 0x04)
#define ERR_TIMER_ALLOC        (ERR_TIMER_BASE | 0x05)
#define ERR_TIMER_INVALID_ID   (ERR_TIMER_BASE | 0x06)
#define ERR_TIMER_NOT_ALLOCATED (ERR_TIMER_BASE | 0x07)
#define ERR_TIMER_INVALID_VALUE (ERR_TIMER_BASE | 0x08)

// Maximum number of timers
#define MAX_SYS_TIMER 32

/**
 * @brief Timer modes
 */
typedef enum {
    TIMER_MODE_NORMAL = 0,     // Free running counter
    TIMER_MODE_CTC,            // Clear timer on compare match
    TIMER_MODE_PWM,            // PWM mode
    TIMER_MODE_FAST_PWM,       // Fast PWM mode
    TIMER_MODE_INPUT_CAPTURE   // Input capture mode
} timer_mode_t;

/**
 * @brief Timer prescaler values
 */
typedef enum {
    TIMER_DIV_1 = 0,
    TIMER_DIV_8,
    TIMER_DIV_64,
    TIMER_DIV_256,
    TIMER_DIV_1024
} timer_prescaler_t;

/**
 * @brief Timer priority levels
 */
#define TIMER_PRIORITY_HIGH    0
#define TIMER_PRIORITY_MEDIUM  1
#define TIMER_PRIORITY_LOW     2
#define TIMER_PRIORITY_IDLE    3

/**
 * @brief Timer statistics
 */
typedef struct {
    uint32_t overflows;         // Number of timer overflows
    uint32_t compare_matches;   // Number of compare matches
    uint32_t captures;          // Number of input captures
    uint32_t errors;           // Number of errors
} timer_stats_t;

/**
 * @brief Timer status structure
 */
typedef struct {
    uint32_t value;            // Current timer value
    uint32_t target;           // Target value for comparison
    uint8_t active;            // Timer active flag
    uint32_t current_tick;     // Current timer tick value
    uint32_t elapsed_time;     // Elapsed time in microseconds
    timer_stats_t stats;       // Timer statistics
    error_code_t last_error;   // Last error code
} timer_status_t;

/**
 * @brief Timer configuration structure
 */
typedef struct {
    timer_mode_t mode;           // Timer operation mode
    timer_prescaler_t prescaler; // Clock prescaler
    uint32_t frequency;          // Timer frequency in Hz
    uint16_t period;            // Timer period value
    uint16_t compare_value;     // Compare match value
    bool interrupt_enable;      // Enable timer interrupts
    uint8_t priority;           // Timer priority level
    
    struct {
        uint8_t channel;        // PWM channel number
        uint16_t duty_cycle;    // PWM duty cycle (0-100%)
        bool inverted;          // Inverted PWM output
    } pwm_config;
    
    struct {
        uint8_t edge;           // Capture edge (rising/falling)
        uint16_t prescaler;     // Input capture prescaler
        bool noise_cancel;      // Enable noise canceller
    } capture_config;
    
    // Callbacks
    void (*overflow_callback)(void);
    void (*compare_callback)(uint16_t value);
    void (*capture_callback)(uint16_t value);
    void (*error_callback)(error_code_t error);
    void (*expiry_callback)(void*);     // Timer expiry callback
    void* expiry_callback_arg;         // Callback argument
    
    // Driver configuration
    driver_config_t driver;    // Common driver configuration
} timer_config_t;

/**
 * @brief Initialize the timer system
 */
void timer_init(void);

/**
 * @brief Allocate a new timer
 * 
 * @return Timer ID if successful, ERR_TIMER_ALLOC if no timers available
 */
int timer_alloc(void);

/**
 * @brief Free an allocated timer
 * 
 * @param timer_id Timer ID to free
 * @return 0 if successful, error code if failed
 */
int timer_free(int timer_id);

/**
 * @brief Set timer value
 * 
 * @param timer_id Timer ID to set
 * @param time_value Time value
 * @return 0 if successful, error code if failed
 */
int timer_set(int timer_id, uint32_t time_value);

/**
 * @brief Get current timer value
 * 
 * @param timer_id Timer ID to query
 * @return Current timer value, 0 if error
 */
uint32_t timer_get(int timer_id);

/**
 * @brief Check if timer has fired
 * 
 * @param timer_id Timer ID to check
 * @return true if fired, false otherwise
 */
bool timer_isfired(int timer_id);

/**
 * @brief Clear timer
 * 
 * @param timer_id Timer ID to clear
 * @return 0 if successful, error code if failed
 */
int timer_clear(int timer_id);

/**
 * @brief Get timer status
 * 
 * @param timer_id Timer ID to query
 * @return Pointer to timer status structure, NULL if error
 */
const timer_status_t* timer_get_status(int timer_id);

/**
 * @brief Reset timer statistics
 * 
 * @param timer_id Timer ID to reset
 */
void timer_reset_statistics(int timer_id);

#endif // DRV_TIMER_H
