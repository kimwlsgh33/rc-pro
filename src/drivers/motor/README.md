# Motor Driver

This directory contains the motor driver implementation following the standardized driver organization pattern.

## Directory Structure

```
motor/
├── common/
│   └── drv_motor_common.h     # Common definitions and interfaces
├── hal/
│   ├── drv_motor_hal.h       # Hardware abstraction interface
│   ├── drv_motor_hal_avr.c   # AVR-specific implementation
│   └── drv_stepper_pins.h    # Hardware-specific pin definitions
├── core/
│   ├── drv_motor.h          # Unified motor interface
│   ├── drv_motor.c          # Core driver logic
│   └── drv_motor_config.h   # Configuration validation
└── impl/
    ├── drv_motor_a.c        # Motor A implementation
    ├── drv_motor_b.c        # Motor B implementation
    └── drv_stepper.c        # Stepper motor implementation
```

## Layer Responsibilities

### Common Layer
- Defines shared types, constants, and error codes
- Provides common interfaces used across all motor implementations

### HAL Layer
- Abstracts hardware-specific details
- Provides unified interface for different MCU architectures
- Handles pin configurations and timer settings

### Core Layer
- Implements hardware-independent motor control logic
- Provides configuration validation
- Manages motor state and parameters

### Implementations Layer
- Contains specific motor type implementations
- Utilizes core and HAL interfaces
- Implements motor-specific features and algorithms

## Configuration

To configure a motor, use the `drv_motor_config_t` structure:
```c
drv_motor_config_t config = {
    .pin_config = {
        .pwm_pin = PIN_B1,
        .direction_pin = PIN_B2,
        .enable_pin = PIN_B3
    },
    .max_speed = 1000,
    .motor_id = 1
};
```

## Error Handling

The driver uses the following error handling pattern:
- Returns `drv_status_t` for all operations
- Validates configurations before initialization
- Checks parameters before execution
- Reports hardware-specific errors through HAL layer

## Usage Example

```c
#include "core/drv_motor.h"

int main(void) {
    drv_motor_config_t config = {
        // ... configure motor parameters
    };
    
    if (drv_motor_init(&config) != DRV_STATUS_OK) {
        // Handle error
        return -1;
    }
    
    // Enable motor
    drv_motor_enable(true);
    
    // Set motor speed
    drv_motor_set_speed(500);
    
    return 0;
}
