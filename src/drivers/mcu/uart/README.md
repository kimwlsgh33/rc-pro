# UART Driver

This directory contains the UART driver implementation following the standardized driver organization pattern.

## Directory Structure

```
uart/
├── common/
│   ├── drv_uart_common.h     # Common definitions and interfaces
│   ├── drv_uart_common.c     # Common utility functions
│   └── drv_uart_types.h      # UART-specific type definitions
├── hal/
│   ├── drv_uart_hw.h        # Hardware abstraction interface
│   └── drv_uart_private.h   # Internal HAL definitions
├── core/
│   ├── drv_uart.h          # Unified UART interface
│   └── drv_uart.c          # Core UART driver logic
└── implementations/
    ├── drv_uart1.h         # UART1 interface
    ├── drv_uart1.c         # UART1 implementation
    ├── drv_uart2.h         # UART2 interface
    ├── drv_uart2.c         # UART2 implementation
    ├── drv_uart3.h         # UART3 interface
    └── drv_uart3.c         # UART3 implementation
```

## Layer Responsibilities

### Common Layer
- Defines shared types, constants
- Provides common utility functions used across all UART implementations
- Manages UART-specific data types

### HAL Layer
- Abstracts hardware-specific details
- Provides unified interface for UART hardware access
- Handles register configurations and interrupt management

### Core Layer
- Implements hardware-independent UART communication logic
- Manages UART initialization and configuration
- Provides common transmit and receive functionality

### Implementations Layer
- Contains specific UART peripheral implementations (UART1, UART2, UART3)
- Utilizes core and HAL interfaces
- Implements peripheral-specific features and configurations

## Configuration

To configure a UART peripheral, use the appropriate configuration structure:
```c
drv_uart_config_t config = {
    .baudrate = 115200,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_NONE,
    .stop_bits = UART_STOP_1_BIT,
    .flow_control = UART_FLOW_CONTROL_NONE
};
```
