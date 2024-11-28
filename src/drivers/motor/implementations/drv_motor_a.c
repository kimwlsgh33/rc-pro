#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>

#include "../../../config/cfg_hardware.h"
#include "../../../utils/common/error_codes.h"
#include "../../../drivers/mcu/timer/drv_timer.h"
#include "../common/drv_motor_common.h"
#include "../hal/drv_motor_a_pins.h"
#include "drv_motor_a.h"

// Forward declarations of interface functions
static error_code_t motor_a_init_fn(motor_driver_t* driver);
static error_code_t motor_a_deinit_fn(motor_driver_t* driver);
static error_code_t motor_a_enable_fn(motor_driver_t* driver);
static error_code_t motor_a_disable_fn(motor_driver_t* driver);
static error_code_t motor_a_move_to_fn(motor_driver_t* driver, int32_t position);
static error_code_t motor_a_set_speed_fn(motor_driver_t* driver, uint16_t speed);
static error_code_t motor_a_stop_fn(motor_driver_t* driver);
static error_code_t motor_a_emergency_stop_fn(motor_driver_t* driver);
static error_code_t motor_a_reset_error_fn(motor_driver_t* driver);
static void motor_a_process_fn(motor_driver_t* driver);

// Static interface vtable
static const motor_interface_t motor_a_interface = {
    .init = motor_a_init_fn,
    .deinit = motor_a_deinit_fn,
    .enable = motor_a_enable_fn,
    .disable = motor_a_disable_fn,
    .move_to = motor_a_move_to_fn,
    .set_speed = motor_a_set_speed_fn,
    .stop = motor_a_stop_fn,
    .emergency_stop = motor_a_emergency_stop_fn,
    .reset_error = motor_a_reset_error_fn,
    .process = motor_a_process_fn
};

/*******************************************************************************
 * Hardware Configuration
 ******************************************************************************/

// Motion Parameters
#define POS_TO_STEP       (int32_t)80  // Position to step conversion (800 ppr, 0.5cm)
#define FIND_SPEED_HZ     400          // Origin finding speed (800 ppr, 1cm/sec)

// Protection Limits
#define MOTOR_A_MAX_CURRENT 1000  // Maximum allowed current (mA)

// Helper functions
static inline motor_a_specific_t* get_specific(motor_driver_t* driver) {
    return (motor_a_specific_t*)driver->specific_data;
}

static inline const motor_a_specific_t* get_specific_const(const motor_driver_t* driver) {
    return (const motor_a_specific_t*)driver->specific_data;
}

// Driver creation/destruction
error_code_t motor_a_create(uint8_t id, const motor_a_config_t* config, motor_driver_t** driver) {
    if (!config || !driver) {
        return ERR_INVALID_PARAM;
    }

    // Allocate driver structure
    motor_driver_t* new_driver = (motor_driver_t*)malloc(sizeof(motor_driver_t));
    if (!new_driver) {
        return ERR_CAT_SYSTEM | 0x01;  // System memory error
    }

    // Allocate specific data
    motor_a_specific_t* specific = (motor_a_specific_t*)malloc(sizeof(motor_a_specific_t));
    if (!specific) {
        free(new_driver);
        return ERR_CAT_SYSTEM | 0x01;  // System memory error
    }

    // Initialize driver structure
    new_driver->id = id;
    new_driver->config = config->base;
    new_driver->status.state = MOTOR_STATE_UNKNOWN;
    new_driver->status.position = 0;
    new_driver->status.target_position = 0;
    new_driver->status.current_speed = 0;
    new_driver->status.target_speed = 0;
    new_driver->status.direction = MOTOR_DIR_CW;
    new_driver->status.error = MOTOR_ERROR_NONE;
    new_driver->status.is_initialized = false;
    new_driver->status.is_enabled = false;
    new_driver->vtable = &motor_a_interface;
    new_driver->specific_data = specific;

    // Initialize specific data
    specific->move_count = 0;
    specific->soft_limit_hit = false;
    specific->current_current = 0;

    // Initialize configuration with default position limits
    new_driver->config.min_position = INT32_MIN;
    new_driver->config.max_position = INT32_MAX;

    *driver = new_driver;
    return ERR_SUCCESS;
}

error_code_t motor_a_destroy(motor_driver_t* driver) {
    if (!driver) {
        return ERR_INVALID_PARAM;
    }

    // Free specific data
    if (driver->specific_data) {
        free(driver->specific_data);
    }

    // Free driver structure
    free(driver);
    return ERR_SUCCESS;
}

// Interface implementations
static error_code_t motor_a_init_fn(motor_driver_t* driver) {
    if (!driver) {
        return ERR_NOT_INIT;
    }

    // Initialize hardware pins
    // Set PWM and direction pins as outputs
    DDRB |= (1 << MOTOR_A_PWM_PIN) | (1 << MOTOR_A_DIR_PIN);
    // Set enable pin as output
    DDRD |= (1 << MOTOR_A_EN_PIN);
    // Set current sense pin as input
    DDRC &= ~(1 << MOTOR_A_CURRENT_PIN);

    // Initialize Timer2 for PWM generation
    TCCR2A = (1 << COM2A1) | (1 << WGM21) | (1 << WGM20); // Fast PWM mode, non-inverting
    TCCR2B = (1 << CS21);  // Prescaler 8
    OCR2A = 0;  // Start with 0% duty cycle

    // Initialize ADC for current sensing
    ADMUX = (1 << REFS0) | MOTOR_A_CURRENT_PIN;  // AVCC reference, ADC0
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);  // Enable ADC, prescaler 128

    // Default to disabled state
    PORTD |= (1 << MOTOR_A_EN_PIN); // Active low enable

    driver->status.is_initialized = true;
    driver->status.state = MOTOR_STATE_STOPPED;
    return ERR_SUCCESS;
}

static error_code_t motor_a_deinit_fn(motor_driver_t* driver) {
    if (!driver) {
        return ERR_NOT_INIT;
    }

    // TODO: Free timer and cleanup hardware
    
    driver->status.state = MOTOR_STATE_UNKNOWN;
    driver->status.is_initialized = false;
    return ERR_SUCCESS;
}

static error_code_t motor_a_enable_fn(motor_driver_t* driver) {
    if (!driver || !driver->status.is_initialized) {
        return ERR_NOT_INIT;
    }

    // Enable motor hardware
    PORTD &= ~(1 << MOTOR_A_EN_PIN); // Active low enable
    driver->status.is_enabled = true;
    return ERR_SUCCESS;
}

static error_code_t motor_a_disable_fn(motor_driver_t* driver) {
    if (!driver) {
        return ERR_NOT_INIT;
    }

    // Disable motor hardware
    PORTD |= (1 << MOTOR_A_EN_PIN); // Active low enable
    driver->status.is_enabled = false;
    return ERR_SUCCESS;
}

static error_code_t motor_a_move_to_fn(motor_driver_t* driver, int32_t position) {
    if (!driver || !driver->status.is_initialized) {
        return ERR_NOT_INIT;
    }

    motor_a_specific_t* specific = get_specific(driver);
    
    // Check soft limits
    if (position < driver->config.min_position || position > driver->config.max_position) {
        specific->soft_limit_hit = true;
        driver->status.error = MOTOR_ERROR_LIMIT;
        return ERR_MOTOR_POSITION;
    }

    // Calculate direction
    if (position > driver->status.position) {
        PORTB |= (1 << MOTOR_A_DIR_PIN);  // CW direction
        driver->status.direction = MOTOR_DIR_CW;
    } else if (position < driver->status.position) {
        PORTB &= ~(1 << MOTOR_A_DIR_PIN); // CCW direction
        driver->status.direction = MOTOR_DIR_CCW;
    } else {
        return ERR_SUCCESS;  // Already at position
    }

    // Update target position and start movement
    driver->status.target_position = position;
    driver->status.state = MOTOR_STATE_MOVING;
    specific->move_count++;

    // Start PWM
    OCR2A = driver->status.target_speed;

    return ERR_SUCCESS;
}

static error_code_t motor_a_set_speed_fn(motor_driver_t* driver, uint16_t speed) {
    if (!driver || !driver->status.is_initialized) {
        return ERR_NOT_INIT;
    }

    if (speed > driver->config.max_speed) {
        speed = driver->config.max_speed;
    }

    driver->status.target_speed = speed;
    
    // Update PWM if moving
    if (driver->status.state == MOTOR_STATE_MOVING) {
        OCR2A = speed;
    }

    return ERR_SUCCESS;
}

static error_code_t motor_a_stop_fn(motor_driver_t* driver) {
    if (!driver) {
        return ERR_NOT_INIT;
    }

    // Gradually decrease PWM
    uint8_t current_pwm = OCR2A;
    while (current_pwm > 0) {
        current_pwm--;
        OCR2A = current_pwm;
        _delay_us(100);
    }

    // Update state
    driver->status.state = MOTOR_STATE_STOPPED;
    driver->status.current_speed = 0;
    driver->status.target_speed = 0;
    driver->status.target_position = driver->status.position;

    return ERR_SUCCESS;
}

static error_code_t motor_a_emergency_stop_fn(motor_driver_t* driver) {
    if (!driver) {
        return ERR_NOT_INIT;
    }

    // Immediately stop PWM
    OCR2A = 0;
    
    // Update state
    driver->status.state = MOTOR_STATE_STOPPED;
    driver->status.current_speed = 0;
    driver->status.target_speed = 0;
    driver->status.target_position = driver->status.position;

    return ERR_SUCCESS;
}

static error_code_t motor_a_reset_error_fn(motor_driver_t* driver) {
    if (!driver) {
        return ERR_NOT_INIT;
    }

    if (driver->status.state == MOTOR_STATE_ERROR) {
        driver->status.error = MOTOR_ERROR_NONE;
        driver->status.state = MOTOR_STATE_STOPPED;
    }

    return ERR_SUCCESS;
}

static void motor_a_process_fn(motor_driver_t* driver) {
    if (!driver || !driver->status.is_initialized) {
        return;
    }

    motor_a_specific_t* specific = get_specific(driver);

    // Read current sensor
    specific->current_current = ADCH; // Assuming ADC is configured for 8-bit mode

    // Check for overcurrent
    if (specific->current_current > driver->config.current_limit) {
        motor_a_emergency_stop_fn(driver);
        driver->status.error = MOTOR_ERROR_OVERCURRENT;
        return;
    }

    // Process based on state
    switch (driver->status.state) {
        case MOTOR_STATE_MOVING:
            // Update position based on encoder feedback
            // TODO: Implement position tracking
            break;

        case MOTOR_STATE_ERROR:
            // Stop all movement
            OCR2A = 0;
            break;

        default:
            break;
    }
}

// Motor A specific operations
error_code_t motor_a_set_soft_limits(motor_driver_t* driver, int32_t min_pos, int32_t max_pos) {
    if (!driver || !driver->status.is_enabled) {
        return ERR_NOT_INIT;
    }

    // Update soft limits in common config
    driver->config.min_position = min_pos;
    driver->config.max_position = max_pos;
    
    // Clear any previous soft limit errors
    motor_a_specific_t* specific = get_specific(driver);
    specific->soft_limit_hit = false;

    return ERR_SUCCESS;
}

error_code_t motor_a_set_hold_current(motor_driver_t* driver, uint16_t current) {
    if (!driver || !driver->status.is_enabled) {
        return ERR_NOT_INIT;
    }

    motor_a_specific_t* specific = get_specific(driver);
    
    if (current > MOTOR_A_MAX_CURRENT) {
        return ERR_MOTOR_CONFIG;
    }

    // TODO: Configure hardware current limit
    return ERR_SUCCESS;
}

// Motor A specific getters
bool motor_a_soft_limit_hit(const motor_driver_t* driver) {
    const motor_a_specific_t* specific = get_specific_const(driver);
    return specific ? specific->soft_limit_hit : false;
}

uint32_t motor_a_get_move_count(const motor_driver_t* driver) {
    const motor_a_specific_t* specific = get_specific_const(driver);
    return specific ? specific->move_count : 0;
}

uint16_t motor_a_get_current(const motor_driver_t* driver) {
    const motor_a_specific_t* specific = get_specific_const(driver);
    return specific ? specific->current_current : 0;
}
