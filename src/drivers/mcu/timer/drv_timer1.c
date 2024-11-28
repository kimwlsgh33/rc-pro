#include "drv_timer1.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include "../common/mcu_power.h"

// Forward declarations of static functions
static error_code_t timer1_init(void* specific_config);
static error_code_t timer1_deinit(void);
static error_code_t timer1_start(void);
static error_code_t timer1_stop(void);
static error_code_t timer1_set_period(uint16_t period);
static error_code_t timer1_get_counter(uint16_t* counter);
static error_code_t timer1_get_status(timer_status_t* status);
static error_code_t timer1_recover(void);
static error_code_t timer1_register_callback(void (*callback)(void*), void* arg);

// Timer1 state
static struct {
    volatile uint16_t counter;     // Direct hardware counter value
    volatile bool compare_match;   // Hardware compare match flag
    void (*callback)(void*);       // Callback function
    void* callback_arg;            // Callback argument
    timer_base_config_t base_config;    // Base configuration
    timer1_specific_config_t specific_config; // Timer1-specific configuration
} timer1_state;

// Timer1 HAL implementation
static void timer1_hal_enter_atomic(void)
{
    cli();  // Disable interrupts
}

static void timer1_hal_exit_atomic(void)
{
    sei();  // Enable interrupts
}

static void timer1_hal_set_mode(timer_mode_t mode)
{
    // Clear mode bits
    TCCR1A &= ~((1 << WGM11) | (1 << WGM10));
    TCCR1B &= ~((1 << WGM13) | (1 << WGM12));
    
    switch (mode) {
        case TIMER_MODE_NORMAL:
            // Already cleared
            break;
        case TIMER_MODE_CTC:
            TCCR1B |= (1 << WGM12);
            break;
        case TIMER_MODE_PWM:
            TCCR1A |= (1 << WGM11);
            break;
        case TIMER_MODE_FAST_PWM:
            TCCR1A |= (1 << WGM11) | (1 << WGM10);
            TCCR1B |= (1 << WGM12);
            break;
        default:
            break;
    }
    
    // Set compare output mode if configured
    if (timer1_state.base_config.compare_output_mode) {
        TCCR1A |= (timer1_state.base_config.compare_output_mode << COM1A0);
    }
    
    // Handle force output compare
    if (timer1_state.base_config.force_output_compare) {
        TCCR1C |= (1 << FOC1A);
    }
}

static void timer1_hal_set_prescaler(timer_prescaler_t prescaler)
{
    uint8_t cs_bits;
    
    switch (prescaler) {
        case TIMER_DIV_1:
            cs_bits = (1 << CS10);
            break;
        case TIMER_DIV_8:
            cs_bits = (1 << CS11);
            break;
        case TIMER_DIV_64:
            cs_bits = (1 << CS11) | (1 << CS10);
            break;
        case TIMER_DIV_256:
            cs_bits = (1 << CS12);
            break;
        case TIMER_DIV_1024:
            cs_bits = (1 << CS12) | (1 << CS10);
            break;
        default:
            cs_bits = 0;  // No clock source
            break;
    }
    
    TCCR1B = (TCCR1B & ~((1 << CS12) | (1 << CS11) | (1 << CS10))) | cs_bits;
}

static void timer1_hal_set_period(uint16_t period)
{
    OCR1A = period;
}

static void timer1_hal_set_compare(uint16_t compare)
{
    OCR1B = compare;
}

static void timer1_hal_enable_interrupt(timer_interrupt_type_t int_type)
{
    switch (int_type) {
        case TIMER_INT_OVERFLOW:
            TIMSK1 |= (1 << TOIE1);
            break;
        case TIMER_INT_COMPARE_MATCH:
            TIMSK1 |= (1 << OCIE1A);
            break;
        default:
            break;
    }
}

static void timer1_hal_disable_interrupt(timer_interrupt_type_t int_type)
{
    switch (int_type) {
        case TIMER_INT_OVERFLOW:
            TIMSK1 &= ~(1 << TOIE1);
            break;
        case TIMER_INT_COMPARE_MATCH:
            TIMSK1 &= ~(1 << OCIE1A);
            break;
        default:
            break;
    }
}

static uint16_t timer1_hal_get_counter(void)
{
    uint16_t count;
    timer1_hal_enter_atomic();
    count = TCNT1;
    timer1_hal_exit_atomic();
    return count;
}

static void timer1_hal_clear_counter(void)
{
    timer1_hal_enter_atomic();
    TCNT1 = 0;
    timer1_hal_exit_atomic();
}

static void timer1_hal_set_pwm_config(const timer_pwm_config_t* config)
{
    // Not implemented yet
}

static void timer1_hal_set_input_capture(const timer_capture_config_t* config)
{
    // Not implemented yet
}

// Timer1 HAL instance
static const timer_hal_t timer1_hal = {
    .enter_atomic = timer1_hal_enter_atomic,
    .exit_atomic = timer1_hal_exit_atomic,
    .set_mode = timer1_hal_set_mode,
    .set_prescaler = timer1_hal_set_prescaler,
    .set_period = timer1_hal_set_period,
    .set_compare = timer1_hal_set_compare,
    .enable_interrupt = timer1_hal_enable_interrupt,
    .disable_interrupt = timer1_hal_disable_interrupt,
    .get_counter = timer1_hal_get_counter,
    .clear_counter = timer1_hal_clear_counter,
    .set_pwm_config = timer1_hal_set_pwm_config,
    .set_input_capture = timer1_hal_set_input_capture
};

// Timer1 operations
static error_code_t timer1_init(void* specific_config)
{
    // Initialize timer state
    memset(&timer1_state, 0, sizeof(timer1_state));
    
    // Configure timer hardware
    if (specific_config != NULL) {
        // Apply specific configuration if provided
        timer1_specific_config_t* specific_config_ptr = (timer1_specific_config_t*)specific_config;
        timer1_state.base_config = specific_config_ptr->base_config;
        timer1_state.specific_config = *specific_config_ptr;
        timer1_hal_set_mode(timer1_state.base_config.mode);
        timer1_hal_set_prescaler(timer1_state.base_config.prescaler);
        timer1_hal_set_period(timer1_state.base_config.period);
    }
    
    return ERR_SUCCESS;
}

static error_code_t timer1_deinit(void)
{
    // Stop timer
    timer1_stop();
    
    // Disable interrupts
    timer1_hal_disable_interrupt(TIMER_INT_OVERFLOW);
    timer1_hal_disable_interrupt(TIMER_INT_COMPARE_MATCH);
    
    // Clear all registers
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1 = 0;
    OCR1A = 0;
    OCR1B = 0;
    
    return ERR_SUCCESS;
}

static error_code_t timer1_start(void)
{
    timer1_hal_enable_interrupt(TIMER_INT_OVERFLOW);
    timer1_hal_enable_interrupt(TIMER_INT_COMPARE_MATCH);
    timer1_hal_set_prescaler(timer1_state.base_config.prescaler);  // Start with prescaler
    return ERR_SUCCESS;
}

static error_code_t timer1_stop(void)
{
    timer1_hal_set_prescaler(TIMER_DIV_1);  // No clock source
    return ERR_SUCCESS;
}

static error_code_t timer1_set_period(uint16_t period)
{
    if (period == 0) {
        return ERR_INVALID_PARAMETER;
    }
    
    timer1_hal_set_period(period);
    return ERR_SUCCESS;
}

static error_code_t timer1_get_counter(uint16_t* counter)
{
    if (!counter) {
        return ERR_INVALID_PARAMETER;
    }
    
    *counter = timer1_hal_get_counter();
    return ERR_SUCCESS;
}

static error_code_t timer1_get_status(timer_status_t* status)
{
    if (!status) {
        return ERR_INVALID_PARAMETER;
    }
    
    timer1_hal_enter_atomic();
    status->counter = timer1_hal_get_counter();
    status->compare_match = timer1_state.compare_match;
    timer1_state.compare_match = false;  // Clear on read
    timer1_hal_exit_atomic();
    
    return ERR_SUCCESS;
}

static error_code_t timer1_recover(void)
{
    timer1_deinit();
    return timer1_init(NULL);
}

static error_code_t timer1_register_callback(void (*callback)(void*), void* arg)
{
    if (!callback) {
        return ERR_INVALID_PARAMETER;
    }
    
    timer1_hal_enter_atomic();
    timer1_state.callback = callback;
    timer1_state.callback_arg = arg;
    timer1_hal_exit_atomic();
    
    return ERR_SUCCESS;
}

// Timer1 operations instance
static const timer_ops_t timer1_ops = {
    .init = timer1_init,
    .deinit = timer1_deinit,
    .start = timer1_start,
    .stop = timer1_stop,
    .set_period = timer1_set_period,
    .get_counter = timer1_get_counter,
    .get_status = timer1_get_status,
    .recover = timer1_recover,
    .register_callback = timer1_register_callback
};

// Get Timer1 operations
const timer_ops_t* timer1_get_ops(void)
{
    return &timer1_ops;
}

// Get Timer1 HAL
const timer_hal_t* timer1_get_hal(void)
{
    return &timer1_hal;
}

// Get current system time in milliseconds
uint32_t timer_get_ms(void)
{
    // This is a basic implementation that assumes a 1ms timer tick
    // You might need to adjust this based on your specific timer configuration
    static volatile uint32_t system_ms = 0;
    
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        // Increment millisecond counter
        system_ms++;
    }
    
    return system_ms;
}

// Timer1 interrupt handlers
ISR(TIMER1_OVF_vect)
{
    timer1_state.counter += 0x10000;  // 16-bit overflow
}

ISR(TIMER1_COMPA_vect)
{
    timer1_state.compare_match = true;
    if (timer1_state.callback) {
        timer1_state.callback(timer1_state.callback_arg);
    }
}
