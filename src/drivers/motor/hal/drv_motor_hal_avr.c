#include "drv_motor_hal.h"
#include <avr/io.h>

void drv_motor_hal_init(const drv_motor_pin_config_t* config) {
    // Configure pins as outputs
    DDRB |= (1 << config->pwm_pin) | (1 << config->direction_pin) | (1 << config->enable_pin);
}

void drv_motor_hal_set_pwm(uint8_t pwm_value) {
    OCR1A = pwm_value;  // Set PWM duty cycle
}

void drv_motor_hal_set_direction(uint8_t direction) {
    if (direction) {
        PORTB |= (1 << PB1);  // Set direction pin high
    } else {
        PORTB &= ~(1 << PB1); // Set direction pin low
    }
}

void drv_motor_hal_enable(bool enable) {
    if (enable) {
        PORTB |= (1 << PB2);  // Set enable pin high
    } else {
        PORTB &= ~(1 << PB2); // Set enable pin low
    }
}

void drv_motor_hal_configure_timer(void) {
    // Configure Timer1 for PWM operation
    TCCR1A = (1 << COM1A1) | (1 << WGM11);  // Fast PWM mode
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);  // Prescaler = 8
    ICR1 = 0xFFFF;  // Set TOP value for 16-bit timer
}
