#ifndef DRV_STEPPER_PINS_H
#define DRV_STEPPER_PINS_H

// Pin Definitions for Stepper Motor Driver
// Step and Direction pins on PORTB
#define STEP_PIN    PB0    // Step pulse output
#define DIR_PIN     PB1    // Direction control

// Control pins on PORTD
#define EN_PIN      PD4    // Enable pin (active low)
#define MS1_PIN     PD5    // Microstep control 1
#define MS2_PIN     PD6    // Microstep control 2
#define MS3_PIN     PD7    // Microstep control 3

// Limit switch on PORTB
#define LIMIT_PIN   PB2    // Limit switch input with pull-up

#endif // DRV_STEPPER_PINS_H
