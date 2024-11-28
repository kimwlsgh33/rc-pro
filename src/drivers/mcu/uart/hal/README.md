# UART Hardware Abstraction Layer (HAL)

This directory contains the UART Hardware Abstraction Layer implementation for AVR microcontrollers. The HAL provides a clean interface between the high-level UART driver and the hardware-specific implementation.

## Architecture Overview

The UART HAL is organized into four main components:

### 1. Hardware Interface (`drv_uart_hw.h`)
Contains all hardware-specific definitions and register access macros:
- Register bit definitions for UCSRnA, UCSRnB, and UCSRnC
- Pre-defined configuration masks for UART settings
- Hardware validation macros
- Error status definitions

Key features:
- Complete isolation of hardware-specific definitions
- F_CPU-aware baud rate validation
- Comprehensive error checking masks
- Clear documentation of all register bits

### 2. Private Implementation (`drv_uart_private.h`)
Defines internal structures and state management:
- UART instance structure for multi-port support
- Buffer management integration
- Hardware register access abstraction
- State tracking for each UART port

The instance-based design allows:
- Independent operation of multiple UART ports
- Separate configuration and state management
- Efficient buffer handling through ring buffers
- Clear separation of public and private interfaces

### 3. HAL Interface (`drv_uart_hal.h`)
Provides the public API for UART operations:
```c
// Initialization and configuration
error_code_t uart_hal_init(uart_port_t port, uint32_t baud_rate, 
                          uint8_t data_bits, uint8_t stop_bits, 
                          uint8_t parity);
error_code_t uart_hal_deinit(uart_port_t port);

// Data transfer operations
error_code_t uart_hal_write_byte(uart_port_t port, uint8_t byte);
error_code_t uart_hal_read_byte(uart_port_t port, uint8_t *byte);

// Event handling
void uart_hal_set_callbacks(uart_port_t port,
                          void (*rx_callback)(uint8_t),
                          void (*tx_callback)(void),
                          void (*error_callback)(uint16_t));
```

### 4. HAL Implementation (`drv_uart_hal.c`)
Implements the HAL interface with:
- Hardware initialization and configuration
- Interrupt handling and callback management
- Thread-safe operations using atomic blocks
- Error detection and reporting

## Key Features

1. **Hardware Independence**
   - All hardware-specific code is isolated in `drv_uart_hw.h`
   - Clean interface for higher-level drivers
   - Easy portability to different AVR variants

2. **Robust Error Handling**
   - Comprehensive parameter validation
   - Hardware error detection (frame, parity, overrun)
   - Consistent error reporting using standard error codes

3. **Interrupt Support**
   - Full interrupt-driven operation
   - Flexible callback system for events
   - Separate RX, TX, and error callbacks
   - Thread-safe interrupt handling

4. **Configuration Options**
   - Configurable baud rate with validation
   - Support for 5-9 bit data frames
   - Configurable stop bits and parity
   - Multiple operation modes (Async, Sync, SPI)

5. **Multi-Port Support**
   - Instance-based design
   - Independent operation of UART ports
   - Separate configuration per port
   - Individual buffer management

## Error Handling

The HAL uses standard error codes:
- `ERR_SUCCESS`: Operation completed successfully
- `ERR_INVALID_PARAM`: Invalid parameter provided
- `ERR_HARDWARE`: Hardware error occurred

Hardware errors are reported through the error callback with a bitmask:
- Frame Error: `UART_UCSRA_FE`
- Data Overrun: `UART_UCSRA_DOR`
- Parity Error: `UART_UCSRA_UPE`

## Usage Example

```c
// Initialize UART port 1 with common settings
error_code_t err = uart_hal_init(UART_PORT_1, 
                                115200,    // Baud rate
                                8,         // Data bits
                                1,         // Stop bits
                                0);        // No parity

if (err != ERR_SUCCESS) {
    // Handle error
    return err;
}

// Set up callbacks
uart_hal_set_callbacks(UART_PORT_1,
                      rx_handler,    // Handle received data
                      tx_handler,    // Handle transmission complete
                      error_handler);// Handle errors

// Basic read/write operations
uint8_t data;
err = uart_hal_read_byte(UART_PORT_1, &data);
if (err == ERR_SUCCESS) {
    err = uart_hal_write_byte(UART_PORT_1, data);
}
```

## Implementation Notes

1. **Baud Rate Calculation**
   - Automatically selects U2X mode when possible
   - Validates baud rate against F_CPU constraints
   - Ensures accurate timing within 2% error margin

2. **Buffer Management**
   - Uses ring buffers for efficient data handling
   - Separate RX and TX buffers per port
   - Configurable buffer sizes

3. **Critical Sections**
   - Uses atomic blocks for thread safety
   - Proper interrupt management
   - Safe register access

4. **Validation**
   - Parameter checking before hardware access
   - Configuration validation
   - Runtime error detection

## Dependencies

- AVR IO Definitions (`<avr/io.h>`)
- Interrupt Support (`<avr/interrupt.h>`)
- Atomic Operations (`<util/atomic.h>`)
- Ring Buffer Implementation (`ring_buffer.h`)
- Error Code Definitions (`error_codes.h`)
