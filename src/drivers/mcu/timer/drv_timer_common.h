#ifndef DRV_TIMER_COMMON_H
#define DRV_TIMER_COMMON_H

#include <stdint.h>
#include <stdbool.h>
#include "../../../utils/common/error_codes.h"
#include "../../../config/cfg_build.h"

// Forward declarations for hardware abstraction types
typedef struct timer_hal_t timer_hal_t;  // Forward declaration

// Timer modes
typedef enum {
    TIMER_MODE_NORMAL      = 0,
    TIMER_MODE_CTC        = 1,
    TIMER_MODE_PWM        = 2,
    TIMER_MODE_FAST_PWM   = 3,
    TIMER_MODE_INPUT_CAPTURE = 4,
    TIMER_MODE_ONESHOT    = 5   // One-shot mode for timeouts and watchdogs
} timer_mode_t;

// Timer prescaler values
typedef enum {
    TIMER_DIV_1     = 0,
    TIMER_DIV_8     = 1,
    TIMER_DIV_64    = 2,
    TIMER_DIV_256   = 3,
    TIMER_DIV_1024  = 4
} timer_prescaler_t;

// Clock source options (common for all timers)
typedef enum {
    TIMER_CLK_INTERNAL      = 0,   // Internal clock source
    TIMER_CLK_EXTERNAL_FALL = 1,   // External clock on falling edge
    TIMER_CLK_EXTERNAL_RISE = 2    // External clock on rising edge
} timer_clock_src_t;

// Timer priority levels
typedef enum {
    TIMER_PRIORITY_IDLE    = 0,
    TIMER_PRIORITY_NORMAL  = 1,
    TIMER_PRIORITY_HIGH    = 2
} timer_priority_t;

// PWM alignment modes
typedef enum {
    TIMER_PWM_INVERTED      = 0,
    TIMER_PWM_NON_INVERTED  = 1,
    TIMER_PWM_PHASE_CORRECT = 2
} timer_pwm_align_t;

// Input capture configuration
typedef struct {
    bool noise_canceler;        // Enable noise canceler
    bool sync_enable;           // Enable input synchronization
    uint8_t prescaler;         // Input capture prescaler
    bool edge_select;          // Edge selection (true = rising, false = falling)
} timer_capture_config_t;

// PWM configuration
typedef struct {
    uint8_t channel;            // PWM channel number
    uint16_t duty_cycle;        // PWM duty cycle
    timer_pwm_align_t align;    // PWM alignment mode
    uint16_t dead_time;         // Dead time for complementary PWM
} timer_pwm_config_t;

// Base timer configuration structure
typedef struct {
    timer_mode_t      mode;              // Timer operating mode
    timer_prescaler_t prescaler;         // Timer prescaler value
    timer_clock_src_t clock_source;      // Clock source selection
    timer_priority_t  priority;          // Timer priority level
    uint16_t          period;            // Timer period value
    uint16_t          compare_value;     // Compare match value
    
    // Common hardware configuration
    uint8_t compare_output_mode;    // Compare output mode (common across timers)
    bool force_output_compare;      // Force output compare option
    
    // Callback configuration
    void            (*expiry_callback)(void* arg);  // Callback function when timer expires
    void             *expiry_callback_arg;          // Argument passed to expiry callback
    
    // Optional configurations
    timer_pwm_config_t      pwm_config;      // PWM configuration if needed
    timer_capture_config_t  capture_config;   // Input capture configuration if needed
} timer_base_config_t;

// System time configuration structure
typedef struct {
    const struct timer_ops *hw_ops;         // Hardware timer operations
    const timer_hal_t      *hw_hal;         // Hardware abstraction layer
    timer_base_config_t     base;           // Base timer configuration
    void                   *specific_config; // Pointer to timer-specific config (if needed)
} timer_systime_config_t;

// Timer status structure
typedef struct {
    bool running;        // Timer is currently running
    bool overflow;       // Overflow has occurred
    bool compare_match;  // Compare match has occurred
    uint32_t counter;    // Current counter value
    uint32_t overflows;  // Number of overflows
    uint32_t matches;    // Number of compare matches
} timer_status_t;

// Timer interrupt types
typedef enum {
    TIMER_INT_OVERFLOW = 0,
    TIMER_INT_COMPARE_MATCH,
    TIMER_INT_INPUT_CAPTURE
} timer_interrupt_type_t;

// Hardware abstraction layer interface
typedef struct timer_hal_t {
    // Atomic operations
    void (*enter_atomic)(void);
    void (*exit_atomic)(void);
    
    // Timer control
    void (*set_mode)(timer_mode_t mode);
    void (*set_prescaler)(timer_prescaler_t prescaler);
    void (*set_period)(uint16_t period);
    void (*set_compare)(uint16_t compare);
    void (*enable_interrupt)(timer_interrupt_type_t int_type);
    void (*disable_interrupt)(timer_interrupt_type_t int_type);
    uint16_t (*get_counter)(void);
    void (*clear_counter)(void);
    
    // PWM specific
    void (*set_pwm_config)(const timer_pwm_config_t* config);
    
    // Input capture
    void (*set_input_capture)(const timer_capture_config_t* config);
} timer_hal_t;

// Timer operations interface
typedef struct timer_ops {
    error_code_t (*init)(void* specific_config);
    error_code_t (*deinit)(void);
    error_code_t (*start)(void);
    error_code_t (*stop)(void);
    error_code_t (*set_period)(uint16_t period);
    error_code_t (*get_counter)(uint16_t* counter);
    error_code_t (*get_status)(timer_status_t* status);
    error_code_t (*recover)(void);
    error_code_t (*register_callback)(void (*callback)(void*), void* arg);
    error_code_t (*enter_critical)(void);
    error_code_t (*exit_critical)(void);
} timer_ops_t;

// Interrupt notification function (called by hardware layer)
void timer_notify_interrupt(timer_interrupt_type_t int_type);

// Error code validation helpers
#define IS_TIMER_ERROR(code) (((code) & ERR_CAT_MASK) == ERR_CAT_DRIVER)
#define IS_TIMER_SUCCESS(code) ((code) == ERR_SUCCESS)

#endif // DRV_TIMER_COMMON_H
