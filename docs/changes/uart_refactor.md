# UART Driver Code Reorganization

## Overview

This document describes recent changes to the UART driver code organization. The changes focus on reducing redundancy and improving code structure while maintaining full functionality.

## Key Changes

### 1. Type Definition Consolidation

- All UART-specific type definitions are now centralized in `src/drivers/mcu/uart/drv_uart_types.h`
- Removed redundant type definitions from `drv_uart.h`
- Configuration structures are now properly layered:
  - MCU-level configuration in `mcu_config.h`
  - Driver-level configuration in `drv_uart_types.h`

### 2. Common UART Functionality

- Added new files for shared UART functionality:
  - `drv_uart_common.h`: Common type definitions and function declarations
  - `drv_uart_common.c`: Implementation of common UART functions
- Implemented shared buffer management
- Added common utility functions like baud rate calculation

### 3. Error Handling Consolidation

- Merged error definitions from `uart_errors.h` into `drv_uart_types.h`
- Standardized error codes across all UART implementations
- Enhanced error tracking in status structure

### 4. File Organization

#### `drv_uart_types.h`
- UART events (`uart_event_t`)
- Port definitions (`uart_port_t`)
- Configuration flags
- Driver configuration structure (`uart_config_t`)
- Status structure (`uart_status_t`)
- Error codes and definitions

#### `drv_uart_common.h`
- Buffer management structure and functions
- Common utility functions
- Shared helper functions

#### `drv_uart.h`
- Driver interface (`uart_driver_t`)
- Hardware register definitions (`uart_registers_t`)
- Function declarations

#### Individual UART Drivers (uart1, uart2, uart3)
- Simplified to use common implementations
- Focused on hardware-specific initialization
- Consistent interface across all ports

## Migration Guide

### For Existing Code

1. Update UART buffer usage:
   ```c
   // Old way - direct buffer manipulation
   static uint8_t _rx_buff[UART1_BUFFER_SIZE];
   static uint8_t _rx_head = 0;
   static uint8_t _rx_tail = 0;

   // New way - using common buffer implementation
   static uint8_t _rx_data[UART_BUFFER_SIZE];
   static uart_buffer_t _rx_buffer;
   
   // Initialize in your init function
   uart_buffer_init(&_rx_buffer, _rx_data, UART_BUFFER_SIZE);
   ```

2. Update error handling:
   ```c
   // Old way
   if (status == UART_ERR_OVERFLOW) {
       // Handle overflow
   }

   // New way
   if (status == ERR_UART_OVERFLOW) {
       // Handle overflow
   }
   ```

3. Use common utility functions:
   ```c
   // Old way - manual calculation
   uint16_t brr = F_CPU / (16 * baud) - 1;

   // New way - using common function
   uint16_t brr = uart_calculate_brr(baud, F_CPU, false);
   ```

### For New Code

- Use `drv_uart_common.h` for buffer management and utilities
- Follow the new error code definitions in `drv_uart_types.h`
- Implement new UART ports using the common interface

## Benefits

1. **Reduced Redundancy**: 
   - Common buffer implementation
   - Shared utility functions
   - Consolidated error handling

2. **Better Organization**: 
   - Clear separation between common and port-specific code
   - Consistent interface across all UART ports
   - Improved type safety through proper layering

3. **Improved Maintainability**:
   - Single source for common functionality
   - Easier to add new UART ports
   - Consistent error handling

4. **Enhanced Features**:
   - More comprehensive error tracking
   - Thread-safe buffer operations
   - Better type checking

## Related Files

- `src/drivers/mcu/uart/drv_uart_types.h`
- `src/drivers/mcu/uart/drv_uart_common.h`
- `src/drivers/mcu/uart/drv_uart_common.c`
- `src/drivers/mcu/uart/drv_uart.h`
- `src/drivers/mcu/uart/drv_uart1.h`
- `src/drivers/mcu/uart/drv_uart2.h`
- `src/drivers/mcu/uart/drv_uart3.h`

## Next Steps

1. Review existing UART driver implementations for compliance
2. Update documentation in affected modules
3. Consider similar reorganization for other peripheral drivers
4. Add unit tests for common UART functionality

## Questions or Issues?

If you encounter any issues or have questions about these changes, please:

1. Check the updated header files for reference
2. Review the migration guide above
3. Open an issue in the project repository if needed
