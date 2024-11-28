#include "drv_uart_hw.h"

// Global array of UART registers
const uart_registers_t uart_registers[UART_PORT_MAX];

void uart_hw_init(void) {
    // Initialize each UART port's registers
    uart_registers_t temp;
    
    for (uart_port_t port = 0; port < UART_PORT_MAX; port++) {
        temp = get_uart_registers(port);
        // Copy the temporary structure to the global array
        // Note: We're using a const cast here because we only want to initialize once
        uart_registers_t* reg = (uart_registers_t*)&uart_registers[port];
        *reg = temp;
    }
}
