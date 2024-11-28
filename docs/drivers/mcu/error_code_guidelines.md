# Error Code Guidelines

This document provides guidelines for adding new error codes to the error handling system. Following these guidelines ensures consistency and maintainability of the error handling across the project.

## Error Code Structure

Error codes are 16-bit values structured as follows:

```plaintext
[Category (4 bits)] [Specific Error (4 bits)] [Base Offset (8 bits)]
     0xF                 0xF                      0xFF
```

### Categories (0xX0)
- Categories are defined in multiples of 0x10
- Range: 0x00 - 0xF0
- Example: `ERR_CAT_COMMON = 0x10`

```c
Current Categories:
0x00 - ERR_CAT_SUCCESS       (Success/no error)
0x10 - ERR_CAT_COMMON        (Common errors)
0x20 - ERR_CAT_SENSOR        (Sensor-related)
0x30 - ERR_CAT_MOTOR         (Motor-related)
0x40 - ERR_CAT_DOOR          (Door-related)
0x50 - ERR_CAT_COMMUNICATION (Communication)
0x60 - ERR_CAT_SYSTEM        (System-level)
0x70 - ERR_CAT_DRIVER        (Driver-related)
0x80 - ERR_CAT_PERIPHERAL    (Peripheral)
0x90 - ERR_CAT_APPLICATION   (Application)
0xF0 - ERR_CAT_CRITICAL      (Critical errors)
```

## Adding New Error Codes

### Step 1: Choose the Appropriate Category
1. Review existing categories in `error_codes.h`
2. Select the most appropriate category for your error
3. If no suitable category exists, propose a new category (must be multiple of 0x10)

### Step 2: Define the Error Code
1. Use the following format:
```c
#define ERR_YOUR_ERROR_NAME (ERR_CAT_YOUR_CATEGORY | 0xXX)
```

2. Choose an available specific error number (0x01-0x0F) within your category
3. Add a descriptive comment

Example:
```c
/* Good */
#define ERR_TIMER_OVERFLOW (ERR_CAT_DRIVER | 0x01)    /* Timer overflow occurred */

/* Bad */
#define ERR_TMR_OVF 0x71    /* Don't use magic numbers or unclear names */
```

### Step 3: Document the Error Code
1. Add a descriptive comment explaining:
   - What the error means
   - When it occurs
   - Any relevant recovery actions

Example:
```c
/**
 * @brief Timer overflow error
 * Occurs when the timer counter reaches its maximum value
 * Recovery: Reset timer counter or adjust timer period
 */
#define ERR_TIMER_OVERFLOW (ERR_CAT_DRIVER | 0x01)
```

## Best Practices

### 1. Naming Conventions
- Use ALL_CAPS for error code names
- Start with `ERR_`
- Use descriptive names
- Use underscores to separate words
```c
/* Good */
ERR_INVALID_PARAMETER
ERR_BUFFER_OVERFLOW

/* Bad */
ERR_INVAL
errBuffOvf
```

### 2. Value Assignment
- Always use category macros
- Never use raw hexadecimal values
- Keep specific error values between 0x01 and 0x0F
```c
/* Good */
#define ERR_UART_TIMEOUT (ERR_CAT_COMMUNICATION | 0x01)

/* Bad */
#define ERR_UART_TIMEOUT 0x51  // Magic number
```

### 3. Comments
- Always add a descriptive comment
- Explain when the error occurs
- Include any relevant recovery information
```c
/* Good */
#define ERR_DMA_CONFIG (ERR_CAT_DRIVER | 0x02)    /* DMA configuration failed - Check alignment and size */

/* Bad */
#define ERR_DMA_CFG (ERR_CAT_DRIVER | 0x02)       /* DMA error */
```

### 4. Organization
- Group related errors together
- Maintain alphabetical order within groups
- Add separator comments between groups
```c
/* Timer-related errors */
#define ERR_TIMER_CONFIG    (ERR_CAT_DRIVER | 0x01)
#define ERR_TIMER_OVERFLOW  (ERR_CAT_DRIVER | 0x02)
#define ERR_TIMER_UNDERFLOW (ERR_CAT_DRIVER | 0x03)

/* UART-related errors */
#define ERR_UART_BUFFER_FULL (ERR_CAT_DRIVER | 0x11)
#define ERR_UART_FRAME       (ERR_CAT_DRIVER | 0x12)
```

## Example: Adding a New Error Code

Here's a complete example of adding a new error code:

1. Choose category:
```c
// Using existing ERR_CAT_DRIVER category for a timer error
```

2. Define the error:
```c
/**
 * @brief Timer configuration error
 * Occurs when timer initialization fails due to invalid configuration
 * Recovery: Check timer parameters (period, prescaler, mode)
 */
#define ERR_TIMER_CONFIG_INVALID (ERR_CAT_DRIVER | 0x04)
```

3. Usage in code:
```c
error_code_t timer_init(timer_config_t* config) {
    if (!validate_timer_config(config)) {
        return ERR_TIMER_CONFIG_INVALID;
    }
    // ... rest of initialization
    return ERR_SUCCESS;
}
```

## Error Code Ranges

Keep track of used error codes within each category to avoid conflicts:

```c
/* Common Errors (0x10-0x1F) */
0x11 - ERR_INVALID_PARAMETER
0x12 - ERR_NOT_INITIALIZED
// ... etc.

/* Driver Errors (0x70-0x7F) */
0x71 - ERR_TIMER_OVERFLOW
0x72 - ERR_TIMER_CONFIG
// ... etc.
```

## Conclusion

Following these guidelines ensures:
- Consistent error handling across the project
- Easy debugging and error tracking
- Maintainable and scalable error code system
- Clear documentation for future developers

When in doubt, review existing error codes in `error_codes.h` for examples and patterns to follow.
