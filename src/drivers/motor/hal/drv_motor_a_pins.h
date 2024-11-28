#ifndef DRV_MOTOR_A_PINS_H
#define DRV_MOTOR_A_PINS_H

/**
 * @file drv_motor_a_pins.h
 * @brief Pin definitions for Motor A driver
 *
 * This file contains all pin definitions and port mappings
 * for the Motor A driver hardware interface.
 */

// PWM and Direction pins on PORTB
#define MOTOR_A_PWM_PIN     PB3    // Timer2 output for PWM control
#define MOTOR_A_DIR_PIN     PB4    // Direction control
#define MOTOR_A_ORG_PIN     PB4    // Origin sensor input

// Control pins on PORTD
#define MOTOR_A_EN_PIN      PD7    // Enable pin (active low)

// ADC pins on PORTC
#define MOTOR_A_CURRENT_PIN PC0    // Current sensing ADC input

// Sensor macros
#define MOTOR_A_ORG_SENSOR_IS_ON() ((PINB & (1 << MOTOR_A_ORG_PIN)) != (1 << MOTOR_A_ORG_PIN))

#endif // DRV_MOTOR_A_PINS_H
