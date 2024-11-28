# UART Driver Documentation

## Overview
The UART (Universal Asynchronous Receiver-Transmitter) driver provides a robust interface for serial communication on AVR microcontrollers. This document outlines recent improvements and the current architecture of the driver.

## Recent Changes (2024)

### Error Handling Improvements
The UART driver now features a comprehensive error handling system with specific error codes for different types of UART errors. These are defined in `error_codes.h` under the `ERR_CAT_PERIPHERAL` category.

#### New Error Codes
```c
ERR_UART_BASE            // Base error code for UART (0x70)
ERR_UART_FRAME           // Frame error detection
ERR_UART_PARITY         // Parity error detection
ERR_UART_OVERRUN        // Overrun error detection
ERR_UART_BUFFER_FULL    // Buffer full condition
ERR_UART_NOT_ENABLED    // UART not enabled
```

### Code Organization
The driver has been reorganized to improve maintainability and portability:

1. **Hardware Abstraction** (`drv_uart_hw.h`)
   - Contains all hardware-specific register definitions
   - Centralizes hardware-dependent code
   - Makes porting to different AVR variants easier

2. **Private Interface** (`drv_uart_private.h`)
   - Defines internal data structures
   - Manages driver state
   - Handles buffer management

3. **Public Interface** (`drv_uart.h`)
   - Provides user-facing API
   - Defines configuration structures
   - Declares public functions

## Architecture

### Components

#### 1. Hardware Layer
- Register definitions in `drv_uart_hw.h`
- Direct hardware access
- Interrupt handling

#### 2. Buffer Management
- Ring buffer implementation for TX/RX
- Overflow protection
- Atomic operations for thread safety

#### 3. Error Management
- Error detection in ISRs
- Error counting and tracking
- Error callback mechanism

### Data Flow
1. **Transmission**
   ```
   User Data → TX Buffer → UART Hardware → Serial Line
   ```

2. **Reception**
   ```
   Serial Line → UART Hardware → ISR → RX Buffer → User Read
   ```

## Usage Guide

### Basic Usage
```c
// Initialize UART
uart_config_t config = {
    .baud_rate = 115200,
    .data_bits = 8,
    .stop_bits = 1,
    .parity = UART_PARITY_NONE,
    .rx_buffer_size = 64,
    .tx_buffer_size = 64
};
uart_init(UART_PORT_1, &config);

// Write data
uint8_t data[] = "Hello";
uart_write(UART_PORT_1, data, sizeof(data));

// Read data
uint8_t rx_data[32];
uint16_t bytes_read;
uart_read(UART_PORT_1, rx_data, sizeof(rx_data), &bytes_read);
```

### Error Handling
```c
// Get status
const uart_status_t* status = uart_get_status(UART_PORT_1);
if (status->error_count > 0) {
    // Handle errors
    if (status->framing_errors > 0) {
        // Handle frame errors
    }
    if (status->parity_errors > 0) {
        // Handle parity errors
    }
}
```

## Implementation Details

### ISR Implementation
Each UART port has three interrupt handlers:
1. RX Complete (USART[n]_RX_vect)
2. TX Complete (USART[n]_TX_vect)
3. Data Register Empty (USART[n]_UDRE_vect)

Example RX ISR:
```c
ISR(USART1_RX_vect) {
    uart_instance_t* inst = &uart_instances[UART_PORT_1];
    const uart_registers_t* regs = uart_get_registers(UART_PORT_1);
    
    uint8_t status = *regs->UCSRnA;
    uint8_t data = *regs->UDRn;

    // Error checking
    if (status & UART_UCSRA_FE) {
        uart_update_status(UART_PORT_1, ERR_UART_FRAME);
        return;
    }
    // ... other error checks ...

    // Store data
    if (ring_buffer_push(&inst->rx_buffer, data) != ERR_SUCCESS) {
        uart_update_status(UART_PORT_1, ERR_UART_BUFFER_FULL);
    }
}
```

## Contributing
When modifying the UART driver:
1. Keep hardware-specific code in `drv_uart_hw.h`
2. Use appropriate error codes for new error conditions
3. Update status tracking for new error types
4. Maintain atomic operations for shared data
5. Document any new features or changes
