/**
 * Peristaltic Pump Controller
 * using DM1, a simple DC motor controller with init
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include <string.h>

#include "../../../config/cfg_system.h"
#include "../../../config/cfg_hardware.h"
#include "../../../utils/common/error_codes.h"
#include "../../../drivers/mcu/timer/drv_timer.h"
#include "../common/drv_motor_common.h"
#include "drv_motor_b.h"
#include "drv_stepper.h"

// Forward declarations of interface functions
static error_code_t door_init_fn(motor_driver_t* driver);
static error_code_t door_deinit_fn(motor_driver_t* driver);
static error_code_t door_enable_fn(motor_driver_t* driver);
static error_code_t door_disable_fn(motor_driver_t* driver);
static error_code_t door_move_to_fn(motor_driver_t* driver, int32_t position);
static error_code_t door_set_speed_fn(motor_driver_t* driver, uint16_t speed);
static error_code_t door_stop_fn(motor_driver_t* driver);
static error_code_t door_emergency_stop_fn(motor_driver_t* driver);
static error_code_t door_reset_error_fn(motor_driver_t* driver);
static void door_process_fn(motor_driver_t* driver);

// Static interface vtable
static const motor_interface_t door_interface = {
    .init = door_init_fn,
    .deinit = door_deinit_fn,
    .enable = door_enable_fn,
    .disable = door_disable_fn,
    .move_to = door_move_to_fn,
    .set_speed = door_set_speed_fn,
    .stop = door_stop_fn,
    .emergency_stop = door_emergency_stop_fn,
    .reset_error = door_reset_error_fn,
    .process = door_process_fn
};

// Hardware specific defines
#define ORG_SENSOR_IS_ON()   ((PINH & 0x20)!=0x20)  // PH6
#define POS_TO_STEP         (int32_t)80  // Position to step conversion (800 ppr, 0.5cm)
#define FIND_SPEED_HZ       400          // 800 ppr, 1cm /sec.
#define DEFAULT_OPEN_POS    280          // Default open position in mm
#define DEFAULT_OPEN_POS2   300          // Secondary open position in mm

// Hardware pins
#define PWM_PIN     5    // Timer3 output
#define DIR_PIN     6    // Direction control
#define EN_PIN      7    // Enable control
#define LIMIT_PIN   5    // Limit switch input (PH5)
#define OBS_PIN     4    // Obstacle detection input (PH4)
#define CURRENT_PIN 1    // Current sensing ADC input

#define DOOR_MAX_CURRENT 2000  // Maximum allowed current
#define DOOR_STALL_TIME 1000   // Time in ms to detect stall

// Helper functions
static inline door_specific_t* get_specific(motor_driver_t* driver) {
    return (door_specific_t*)driver->specific_data;
}

static inline const door_specific_t* get_specific_const(const motor_driver_t* driver) {
    return (const door_specific_t*)driver->specific_data;
}

// Driver creation/destruction
error_code_t door_create(uint8_t id, const door_config_t* config, motor_driver_t** driver) {
    if (!config || !driver) {
        return ERR_INVALID_PARAM;
    }

    // Allocate driver structure
    motor_driver_t* new_driver = (motor_driver_t*)malloc(sizeof(motor_driver_t));
    if (!new_driver) {
        return ERR_OUT_OF_MEMORY;
    }

    // Allocate specific data
    door_specific_t* specific = (door_specific_t*)malloc(sizeof(door_specific_t));
    if (!specific) {
        free(new_driver);
        return ERR_OUT_OF_MEMORY;
    }

    // Initialize driver structure
    new_driver->id = id;
    new_driver->config = config->base;  // Store base motor config
    new_driver->status.state = MOTOR_STATE_UNKNOWN;
    new_driver->status.position = 0;
    new_driver->status.target_position = 0;
    new_driver->status.current_speed = 0;
    new_driver->status.target_speed = 0;
    new_driver->status.direction = MOTOR_DIR_CW;
    new_driver->status.error = MOTOR_ERROR_NONE;
    new_driver->status.is_initialized = false;
    new_driver->status.is_enabled = false;
    new_driver->vtable = &door_interface;
    new_driver->specific_data = specific;

    // Initialize specific data
    specific->door_state = DOOR_STATE_UNKNOWN;
    specific->obstacle_detected = false;
    specific->open_count = 0;
    specific->close_count = 0;
    specific->obstacle_count = 0;
    specific->auto_close_timer = 0;
    specific->config = *config;  // Store full door config in specific data

    *driver = new_driver;
    return ERR_SUCCESS;
}

error_code_t door_destroy(motor_driver_t* driver) {
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
static error_code_t door_init_fn(motor_driver_t* driver) {
    if (!driver) {
        return ERR_INVALID_PARAM;
    }

    // Initialize hardware pins
    // Set PWM and direction pins as outputs
    DDRB |= (1 << PWM_PIN) | (1 << DIR_PIN);
    // Set enable pin as output
    DDRD |= (1 << EN_PIN);
    // Set limit switch and obstacle detection as inputs with pull-up
    DDRH &= ~((1 << LIMIT_PIN) | (1 << OBS_PIN));
    PORTH |= (1 << LIMIT_PIN) | (1 << OBS_PIN);
    // Set current sense pin as input
    DDRF &= ~(1 << CURRENT_PIN);

    // Initialize Timer3 for PWM generation
    // Fast PWM mode, non-inverting output
    TCCR3A = (1 << COM3A1) | (1 << WGM31) | (1 << WGM30);
    TCCR3B = (1 << WGM32) | (1 << CS31);  // Prescaler 8
    OCR3A = 0;  // Start with 0% duty cycle

    // Initialize ADC for current sensing
    ADMUX = (1 << REFS0) | CURRENT_PIN;  // AVCC reference, ADC1
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);  // Enable ADC, prescaler 128

    // Default to disabled state
    PORTD |= (1 << EN_PIN); // Active low enable

    driver->status.is_initialized = true;
    driver->status.state = MOTOR_STATE_STOPPED;
    return ERR_SUCCESS;
}

static error_code_t door_deinit_fn(motor_driver_t* driver) {
    if (!driver) {
        return ERR_INVALID_PARAM;
    }

    // TODO: Free timer and cleanup hardware
    
    driver->status.state = MOTOR_STATE_UNKNOWN;
    driver->status.is_initialized = false;
    return ERR_SUCCESS;
}

static error_code_t door_enable_fn(motor_driver_t* driver) {
    if (!driver) {
        return ERR_INVALID_PARAM;
    }
    if (!driver->status.is_initialized) {
        return ERR_MOTOR_NOT_INIT;
    }

    // Enable motor hardware
    PORTD &= ~(1 << EN_PIN); // Active low enable
    driver->status.is_enabled = true;
    return ERR_SUCCESS;
}

static error_code_t door_disable_fn(motor_driver_t* driver) {
    if (!driver) {
        return ERR_INVALID_PARAM;
    }

    // Disable motor hardware
    PORTD |= (1 << EN_PIN); // Active low enable
    driver->status.is_enabled = false;
    return ERR_SUCCESS;
}

static error_code_t door_move_to_fn(motor_driver_t* driver, int32_t position) {
    if (!driver) {
        return ERR_INVALID_PARAM;
    }
    if (!driver->status.is_initialized) {
        return ERR_MOTOR_NOT_INIT;
    }
    if (!driver->status.is_enabled) {
        return ERR_MOTOR_NOT_ENABLED;
    }
    if (driver->status.error != MOTOR_ERROR_NONE) {
        return ERR_MOTOR_SYSTEM;
    }

    door_specific_t* specific = get_specific(driver);
    if (specific->obstacle_detected) {
        return ERR_MOTOR_OBSTACLE;
    }

    // Check if we're already at the target position
    if (driver->status.position == position) {
        return ERR_SUCCESS;
    }

    // Update direction based on target position
    if (position > driver->status.position) {
        driver->status.direction = MOTOR_DIR_CW;
        specific->door_state = DOOR_STATE_OPENING;
    } else if (position < driver->status.position) {
        driver->status.direction = MOTOR_DIR_CCW;
        specific->door_state = DOOR_STATE_CLOSING;
    } else {
        return ERR_SUCCESS;  // Already at position
    }

    // Set PWM based on target speed
    OCR3A = driver->status.target_speed;
    
    // Update target position
    driver->status.target_position = position;
    driver->status.state = MOTOR_STATE_MOVING;

    // Update direction pin
    if (driver->status.direction == MOTOR_DIR_CW) {
        PORTB |= (1 << DIR_PIN);
        specific->open_count++;
    } else {
        PORTB &= ~(1 << DIR_PIN);
        specific->close_count++;
    }

    return ERR_SUCCESS;
}

static error_code_t door_set_speed_fn(motor_driver_t* driver, uint16_t speed) {
    if (!driver) {
        return ERR_INVALID_PARAM;
    }
    if (!driver->status.is_initialized) {
        return ERR_MOTOR_NOT_INIT;
    }
    if (!driver->status.is_enabled) {
        return ERR_MOTOR_NOT_ENABLED;
    }

    if (speed > driver->config.max_speed) {
        speed = driver->config.max_speed;
    }

    driver->status.target_speed = speed;
    
    return ERR_SUCCESS;
}

static error_code_t door_stop_fn(motor_driver_t* driver) {
    if (!driver) {
        return ERR_INVALID_PARAM;
    }
    if (!driver->status.is_initialized) {
        return ERR_MOTOR_NOT_INIT;
    }
    if (!driver->status.is_enabled) {
        return ERR_MOTOR_NOT_ENABLED;
    }

    door_specific_t* specific = get_specific(driver);
    
    // Gradually decrease PWM
    while (OCR3A > 0) {
        OCR3A--;
        _delay_us(100);
    }

    driver->status.state = MOTOR_STATE_STOPPED;
    driver->status.current_speed = 0;
    
    // Update door state based on position
    if (driver->status.position == 0) {
        specific->door_state = DOOR_STATE_CLOSED;
    } else if (driver->status.position >= DEFAULT_OPEN_POS) {
        specific->door_state = DOOR_STATE_OPEN;
    } else {
        specific->door_state = DOOR_STATE_PARTIALLY_OPEN;
    }

    return ERR_SUCCESS;
}

static error_code_t door_emergency_stop_fn(motor_driver_t* driver) {
    if (!driver) {
        return ERR_INVALID_PARAM;
    }
    if (!driver->status.is_initialized) {
        return ERR_MOTOR_NOT_INIT;
    }

    // Immediately stop PWM
    OCR3A = 0;
    
    driver->status.state = MOTOR_STATE_STOPPED;
    driver->status.current_speed = 0;
    driver->status.error |= MOTOR_ERROR_SYSTEM;

    door_specific_t* specific = get_specific(driver);
    specific->door_state = DOOR_STATE_STOPPED;

    return ERR_SUCCESS;
}

static error_code_t door_reset_error_fn(motor_driver_t* driver) {
    if (!driver) {
        return ERR_INVALID_PARAM;
    }
    if (!driver->status.is_initialized) {
        return ERR_MOTOR_NOT_INIT;
    }

    door_specific_t* specific = get_specific(driver);
    
    driver->status.error = MOTOR_ERROR_NONE;
    if (specific) {
        specific->obstacle_detected = false;
    }

    return ERR_SUCCESS;
}

static void door_process_fn(motor_driver_t* driver) {
    if (!driver) {
        return;
    }

    door_specific_t* specific = (door_specific_t*)driver->specific_data;
    const door_config_t* config = &specific->config;  // Access door config from specific data
    static uint32_t last_position_update = 0;
    static uint32_t stall_timer = 0;

    // Start ADC conversion
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC)); // Wait for conversion
    uint16_t current = ADC;

    // Check for overcurrent
    if (current > DOOR_MAX_CURRENT) {
        door_emergency_stop_fn(driver);
        driver->status.error = MOTOR_ERROR_OVERCURRENT;
        return;
    }

    // Check obstacle sensor
    if (!(PINH & (1 << OBS_PIN))) {
        specific->obstacle_detected = true;
        specific->obstacle_count++;
        door_emergency_stop_fn(driver);
        driver->status.error = MOTOR_ERROR_OBSTACLE;
        return;
    }

    // Process movement
    if (driver->status.state == MOTOR_STATE_MOVING) {
        // Update current speed based on target
        if (driver->status.current_speed < driver->status.target_speed) {
            driver->status.current_speed++;
            uint16_t pwm_value = (driver->status.current_speed * ICR3) / 100;
            OCR3A = pwm_value;
        } else if (driver->status.current_speed > driver->status.target_speed) {
            driver->status.current_speed--;
            uint16_t pwm_value = (driver->status.current_speed * ICR3) / 100;
            OCR3A = pwm_value;
        }

        // Update position based on encoder or time
        uint32_t current_time = timer_get_ms();
        if (current_time - last_position_update >= 10) {  // Update every 10ms
            if (driver->status.direction == MOTOR_DIR_CW) {
                driver->status.position++;
            } else {
                driver->status.position--;
            }
            last_position_update = current_time;
            
            // Reset stall timer if position changed
            stall_timer = current_time;
        } else if (current_time - stall_timer > DOOR_STALL_TIME) {
            // Stall detected
            door_emergency_stop_fn(driver);
            driver->status.error = MOTOR_ERROR_STALL;
            return;
        }

        // Check limit switches
        if (!(PINH & (1 << LIMIT_PIN))) {
            if (driver->status.direction == MOTOR_DIR_CCW) {
                // Hit close limit
                driver->status.position = 0;
                door_stop_fn(driver);
            }
        }

        // Check if target position reached
        if (driver->status.position == driver->status.target_position) {
            door_stop_fn(driver);
            
            // Handle auto-close if enabled
            if (config->auto_close_enable && 
                specific->door_state == DOOR_STATE_OPEN) {
                specific->auto_close_timer = timer_get_ms() + config->auto_close_delay;
            }
        }
    } else if (config->auto_close_enable && 
               specific->door_state == DOOR_STATE_OPEN &&
               timer_get_ms() >= specific->auto_close_timer) {
        // Auto-close timer expired
        door_close(driver);
    }
}

// Door-specific operations
error_code_t door_open(motor_driver_t* driver) {
    if (!driver) {
        return ERR_INVALID_PARAM;
    }
    if (!driver->status.is_enabled) {
        return ERR_MOTOR_NOT_ENABLED;
    }

    return door_move_to_fn(driver, DEFAULT_OPEN_POS);
}

error_code_t door_close(motor_driver_t* driver) {
    if (!driver) {
        return ERR_INVALID_PARAM;
    }
    if (!driver->status.is_enabled) {
        return ERR_MOTOR_NOT_ENABLED;
    }

    return door_move_to_fn(driver, 0);
}

error_code_t door_set_auto_close(motor_driver_t* driver, bool enable, uint32_t delay) {
    if (!driver) {
        return ERR_INVALID_PARAM;
    }
    if (!driver->status.is_enabled) {
        return ERR_MOTOR_NOT_ENABLED;
    }

    door_config_t* config = (door_config_t*)&driver->config;
    door_specific_t* specific = get_specific(driver);
    
    config->auto_close_enable = enable;
    config->auto_close_delay = delay;
    
    if (enable && specific->door_state == DOOR_STATE_OPEN) {
        specific->auto_close_timer = delay;
    }
    
    return ERR_SUCCESS;
}

error_code_t door_clear_obstacle(motor_driver_t* driver) {
    if (!driver) {
        return ERR_INVALID_PARAM;
    }
    if (!driver->status.is_enabled) {
        return ERR_MOTOR_NOT_ENABLED;
    }

    door_specific_t* specific = get_specific(driver);
    specific->obstacle_detected = false;
    
    return ERR_SUCCESS;
}

// Door-specific getters
door_state_t door_get_state(const motor_driver_t* driver) {
    const door_specific_t* specific = get_specific_const(driver);
    return specific ? specific->door_state : DOOR_STATE_UNKNOWN;
}

bool door_is_obstacle_detected(const motor_driver_t* driver) {
    const door_specific_t* specific = get_specific_const(driver);
    return specific ? specific->obstacle_detected : false;
}

uint32_t door_get_open_count(const motor_driver_t* driver) {
    const door_specific_t* specific = get_specific_const(driver);
    return specific ? specific->open_count : 0;
}

uint32_t door_get_close_count(const motor_driver_t* driver) {
    const door_specific_t* specific = get_specific_const(driver);
    return specific ? specific->close_count : 0;
}

uint32_t door_get_obstacle_count(const motor_driver_t* driver) {
    const door_specific_t* specific = get_specific_const(driver);
    return specific ? specific->obstacle_count : 0;
}
