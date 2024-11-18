#include "drv_timer1.h"
#include "drv_timer.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>

// Static driver instance
static timer_driver_t timer1_driver;
static timer_status_t timer1_status;
static timer_config_t timer1_config;

// Prescaler lookup table
static const uint16_t prescaler_values[] = {1, 8, 64, 256, 1024};

// Forward declarations
static error_code_t timer1_start(void);
static error_code_t timer1_stop(void);
static error_code_t timer1_reset(void);
static error_code_t timer1_set_frequency(uint32_t frequency);
static error_code_t timer1_set_period(uint16_t period);
static error_code_t timer1_set_compare(uint16_t value);
static error_code_t timer1_set_pwm_duty(uint8_t channel, uint16_t duty);
static uint32_t timer1_get_tick(void);
static uint32_t timer1_get_elapsed_time(void);
static const timer_status_t* timer1_get_status(void);

// Driver interface implementation
static error_code_t timer1_driver_init(void) {
    return timer1_init(&timer1_config);
}

static error_code_t timer1_driver_deinit(void) {
    return timer1_deinit();
}

static error_code_t timer1_driver_configure(const driver_config_t* config) {
    if (!config) return ERR_INVALID_PARAM;
    memcpy(&timer1_config.driver, config, sizeof(driver_config_t));
    return ERR_SUCCESS;
}

static error_code_t timer1_driver_get_status(driver_status_t* status) {
    if (!status) return ERR_INVALID_PARAM;
    status->state = timer1_status.state;
    status->error = timer1_status.last_error;
    return ERR_SUCCESS;
}

// Initialize Timer1 driver interface
static void timer1_init_driver(void) {
    // Setup driver interface
    timer1_driver.driver.init = timer1_driver_init;
    timer1_driver.driver.deinit = timer1_driver_deinit;
    timer1_driver.driver.configure = timer1_driver_configure;
    timer1_driver.driver.get_status = timer1_driver_get_status;
    
    // Setup Timer specific functions
    timer1_driver.start = timer1_start;
    timer1_driver.stop = timer1_stop;
    timer1_driver.reset = timer1_reset;
    timer1_driver.set_frequency = timer1_set_frequency;
    timer1_driver.set_period = timer1_set_period;
    timer1_driver.set_compare = timer1_set_compare;
    timer1_driver.set_pwm_duty = timer1_set_pwm_duty;
    timer1_driver.get_tick = timer1_get_tick;
    timer1_driver.get_elapsed_time = timer1_get_elapsed_time;
    timer1_driver.get_status = timer1_get_status;
}

error_code_t timer1_init(const timer_config_t* config) {
    if (!config) return ERR_INVALID_PARAM;
    
    // Store configuration
    memcpy(&timer1_config, config, sizeof(timer_config_t));
    
    // Configure Timer1 based on mode
    switch (config->mode) {
        case TIMER_MODE_NORMAL:
            TCCR1A = 0x00;  // Normal operation
            TCCR1B = 0x00;  // No clock source yet
            break;
            
        case TIMER_MODE_CTC:
            TCCR1A = 0x00;  // CTC mode
            TCCR1B = (1 << WGM12);  // CTC mode, OCR1A as top
            break;
            
        case TIMER_MODE_PWM:
            TCCR1A = (1 << WGM11) | (1 << WGM10);  // Phase correct PWM
            TCCR1B = 0x00;
            break;
            
        case TIMER_MODE_FAST_PWM:
            TCCR1A = (1 << WGM11);
            TCCR1B = (1 << WGM13) | (1 << WGM12);  // Fast PWM, ICR1 as top
            break;
            
        case TIMER_MODE_INPUT_CAPTURE:
            TCCR1A = 0x00;
            TCCR1B = (1 << ICNC1);  // Enable noise canceller if configured
            break;
            
        default:
            return ERR_INVALID_PARAM;
    }
    
    // Set prescaler
    uint8_t prescaler_bits;
    switch (config->prescaler) {
        case TIMER_DIV_1:    prescaler_bits = 0x01; break;
        case TIMER_DIV_8:    prescaler_bits = 0x02; break;
        case TIMER_DIV_64:   prescaler_bits = 0x03; break;
        case TIMER_DIV_256:  prescaler_bits = 0x04; break;
        case TIMER_DIV_1024: prescaler_bits = 0x05; break;
        default: return ERR_TIMER_INVALID_DIV;
    }
    
    // Set compare value
    OCR1A = config->compare_value;
    
    // Configure interrupts
    if (config->interrupt_enable) {
        TIMSK1 |= (1 << TOIE1) | (1 << OCIE1A);  // Enable overflow and compare match interrupts
    }
    
    // Configure PWM if in PWM mode
    if (config->mode == TIMER_MODE_PWM || config->mode == TIMER_MODE_FAST_PWM) {
        // Set PWM period
        ICR1 = config->period;
        
        // Configure PWM channels
        if (config->pwm_config.channel & 0x01) {
            TCCR1A |= (1 << COM1A1) | (config->pwm_config.inverted ? (1 << COM1A0) : 0);
            OCR1A = (uint16_t)((uint32_t)config->pwm_config.duty_cycle * config->period / 100);
        }
        if (config->pwm_config.channel & 0x02) {
            TCCR1A |= (1 << COM1B1) | (config->pwm_config.inverted ? (1 << COM1B0) : 0);
            OCR1B = (uint16_t)((uint32_t)config->pwm_config.duty_cycle * config->period / 100);
        }
    }
    
    // Set clock source (starts timer)
    TCCR1B |= prescaler_bits;
    
    // Initialize status
    memset(&timer1_status, 0, sizeof(timer_status_t));
    timer1_status.state = DRIVER_STATE_INITIALIZED;
    
    // Initialize driver interface
    timer1_init_driver();
    
    return ERR_SUCCESS;
}

error_code_t timer1_deinit(void) {
    // Stop timer
    TCCR1B = 0x00;  // No clock source
    TIMSK1 = 0x00;  // Disable all interrupts
    
    // Clear status
    memset(&timer1_status, 0, sizeof(timer_status_t));
    timer1_status.state = DRIVER_STATE_UNINITIALIZED;
    
    return ERR_SUCCESS;
}

// Timer specific function implementations
static error_code_t timer1_start(void) {
    if (timer1_status.state != DRIVER_STATE_INITIALIZED) {
        return ERR_NOT_INITIALIZED;
    }
    
    // Restore clock source from config
    TCCR1B |= timer1_config.prescaler + 1;
    timer1_status.state = DRIVER_STATE_RUNNING;
    
    return ERR_SUCCESS;
}

static error_code_t timer1_stop(void) {
    // Remove clock source
    TCCR1B &= ~0x07;
    timer1_status.state = DRIVER_STATE_INITIALIZED;
    
    return ERR_SUCCESS;
}

static error_code_t timer1_reset(void) {
    // Clear counter
    TCNT1 = 0;
    
    // Clear status counters
    timer1_status.current_tick = 0;
    timer1_status.elapsed_time = 0;
    
    return ERR_SUCCESS;
}

static error_code_t timer1_set_frequency(uint32_t frequency) {
    if (frequency == 0) return ERR_TIMER_INVALID_FREQ;
    
    // Calculate period based on frequency
    uint32_t period = F_CPU / frequency / prescaler_values[timer1_config.prescaler];
    if (period > 65535) return ERR_TIMER_INVALID_FREQ;
    
    return timer1_set_period((uint16_t)period);
}

static error_code_t timer1_set_period(uint16_t period) {
    if (timer1_config.mode == TIMER_MODE_PWM || 
        timer1_config.mode == TIMER_MODE_FAST_PWM) {
        ICR1 = period;
    } else {
        OCR1A = period;
    }
    return ERR_SUCCESS;
}

static error_code_t timer1_set_compare(uint16_t value) {
    OCR1A = value;
    return ERR_SUCCESS;
}

static error_code_t timer1_set_pwm_duty(uint8_t channel, uint16_t duty) {
    if (duty > 100) return ERR_INVALID_PARAM;
    
    uint16_t period = (timer1_config.mode == TIMER_MODE_PWM || 
                      timer1_config.mode == TIMER_MODE_FAST_PWM) ? ICR1 : OCR1A;
    
    uint16_t compare = (uint16_t)((uint32_t)duty * period / 100);
    
    if (channel & 0x01) OCR1A = compare;
    if (channel & 0x02) OCR1B = compare;
    
    return ERR_SUCCESS;
}

static uint32_t timer1_get_tick(void) {
    return TCNT1;
}

static uint32_t timer1_get_elapsed_time(void) {
    uint32_t ticks = TCNT1;
    return (ticks * prescaler_values[timer1_config.prescaler]) / (F_CPU / 1000000UL);
}

static const timer_status_t* timer1_get_status(void) {
    timer1_status.current_tick = TCNT1;
    timer1_status.elapsed_time = timer1_get_elapsed_time();
    return &timer1_status;
}

// Interrupt handlers
ISR(TIMER1_OVF_vect) {
    timer1_status.stats.overflows++;
    if (timer1_config.overflow_callback) {
        timer1_config.overflow_callback();
    }
}

ISR(TIMER1_COMPA_vect) {
    timer1_status.stats.compare_matches++;
    if (timer1_config.compare_callback) {
        timer1_config.compare_callback(OCR1A);
    }
}

ISR(TIMER1_CAPT_vect) {
    timer1_status.stats.captures++;
    if (timer1_config.capture_callback) {
        timer1_config.capture_callback(ICR1);
    }
}

// Public function to get driver interface
const timer_driver_t* timer1_get_driver(void) {
    return &timer1_driver;
}
