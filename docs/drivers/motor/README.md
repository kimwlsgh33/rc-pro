# Motor Driver Documentation

## Overview
Documentation for the motor control subsystem, including stepper and DC motor drivers.

## Components

### Common Interface (`drv_motor_common.h`)
- Base types and interfaces for all motor drivers
- Error codes and states
- Configuration structures

### Motor A Driver (`drv_motor_a.h`, `drv_motor_a.c`)
- Implementation for Motor Type A
- Specific features and limitations
- Configuration options

### Motor B Driver (`drv_motor_b.h`, `drv_motor_b.c`)
- Implementation for Motor Type B
- Specific features and limitations
- Configuration options

### Stepper Driver (`drv_stepper.h`, `drv_stepper.c`)
- Stepper motor implementation
- Microstepping support
- Position control features

## Usage Examples

### Basic Motor Control
```c
// Example motor initialization and control code
```

### Error Handling
```c
// Example error handling code
```

## Implementation Details
See [Motor Driver Implementation](implementation.md) for detailed implementation notes.
