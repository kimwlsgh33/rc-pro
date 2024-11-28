# Hardware Abstraction Layer (HAL) Architecture

## Overview

The Hardware Abstraction Layer (HAL) in this project provides a clean separation between hardware-specific code and the higher-level application logic. This document explains the HAL architecture, its benefits, and how it compares to operating system driver models.

## Architecture Layers

### 1. High-Level API (`drv_timer.c`)
- Provides user-friendly functions (`timer_start()`, `timer_stop()`)
- Handles resource management (allocation/deallocation)
- Manages multiple timer instances
- Independent of hardware details

### 2. HAL Interface (`drv_timer_common.h`)
```c
// Operations interface (high-level functions)
typedef struct {
    error_code_t (*init)(void* config);
    error_code_t (*start)(void);
    error_code_t (*stop)(void);
    // ...
} timer_ops_t;

// Hardware interface (low-level functions)
typedef struct {
    void (*enable_clock)(void);
    void (*disable_clock)(void);
    void (*set_prescaler)(timer_prescaler_t prescaler);
    // ...
} timer_hal_t;
```

### 3. Hardware-Specific Implementation (`drv_timer1.c`)
- Implements actual hardware control
- Deals with specific registers and hardware details
- Provides implementations for both `timer_ops_t` and `timer_hal_t`

## Benefits of HAL Design

### 1. Hardware Independence
- Upper layers don't need to know about specific hardware details
- Support for different timers (Timer2, Timer3) without changing high-level code
- Same API works across different microcontrollers

### 2. Code Reusability
```c
// Works the same for any timer
error_code_t timer_start(timer_handle_t* handle) {
    return handle->ops->start();  // Calls hardware-specific implementation
}
```

### 3. Maintainability
- Hardware-specific code is isolated
- Easier to update or fix hardware-specific issues
- Clear separation of concerns

### 4. Portability
To port to a different microcontroller:
- Keep `drv_timer.c` and `drv_timer_common.h` unchanged
- Only rewrite the hardware-specific implementation

### 5. Testing and Debugging
- Can mock hardware layer for testing
- Clear interface boundaries
- Can implement debugging features at any layer

### 6. Flexibility
```c
// Supports different timer configurations
typedef struct {
    timer_mode_t mode;
    timer_prescaler_t prescaler;
    timer_clock_src_t clock_source;
    // ...
} timer_base_config_t;
```

## Comparison with Operating System HALs

### Linux Device Driver Model
```c
// Similar to our timer_hal_t
struct uart_ops {
    int  (*startup)(struct uart_port *);
    void (*shutdown)(struct uart_port *);
    void (*set_termios)(struct uart_port *, struct ktermios *new,
                        struct ktermios *old);
};
```

### Windows Driver Model (WDM)
```c
DRIVER_OBJECT DriverObject = {
    .MajorFunction[IRP_MJ_READ] = MyReadFunction,
    .MajorFunction[IRP_MJ_WRITE] = MyWriteFunction,
};
```

## Layered Architecture Comparison

Our Timer System:
```plaintext
Application Layer
      ↓
Generic Timer API (drv_timer.c)
      ↓
HAL Interface (timer_hal_t)
      ↓
Hardware-Specific Code (drv_timer1.c)
```

OS Equivalent:
```plaintext
User Applications
      ↓
System Calls
      ↓
Virtual File System (VFS)
      ↓
Device Drivers
```

## Implementation Example

```c
// User code
timer_handle_t timer;
timer_config_t config = {
    .base.mode = TIMER_MODE_NORMAL,
    .base.prescaler = TIMER_DIV_1024
};

// High-level API call
timer_start(&timer);

// Internal flow:
1. timer_start() → calls ops->start()
2. ops->start() → calls Timer1's start implementation
3. Timer1's start → uses HAL functions (enable_clock, set_prescaler)
4. HAL functions → directly manipulate hardware registers
```

## Key Design Principles

### 1. Resource Management
```c
// Managing multiple timer instances
static timer_handle_t timer_instances[MAX_TIMER_INSTANCES];
```

### 2. Error Handling
```c
error_code_t timer_init(void) {
    if (timer_initialized) {
        return ERR_ALREADY_INITIALIZED;
    }
    // ...
}
```

### 3. Configuration Management
```c
typedef struct {
    timer_mode_t mode;
    timer_prescaler_t prescaler;
    timer_clock_src_t clock_source;
} timer_base_config_t;
```

## Conclusion

The HAL architecture in this project follows industry-standard practices similar to those used in modern operating systems. It provides a clean, maintainable, and portable way to handle hardware interactions while keeping the complexity manageable at each level.
