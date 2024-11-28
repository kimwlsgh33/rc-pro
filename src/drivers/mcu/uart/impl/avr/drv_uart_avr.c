/**
 * @file drv_uart_avr.c
 * @brief AVR-specific UART Implementation
 */

#include "drv_uart_avr.h"
#include "../../hal/drv_uart_hw.h"
#include <avr/interrupt.h>
#include <avr/io.h>
#include <string.h>
#include <util/atomic.h>

// Port status tracking
static struct {
    bool initialized;
    uint16_t error_count;
    uint16_t tx_count;
    uint16_t rx_count;
} port_status[UART_PORT_MAX];

error_code_t uart_avr_init_port(uart_port_t port, const uart_config_t *config) {
    if (port >= UART_PORT_MAX || !config) {
        return ERR_INVALID_PARAM;
    }

    if (!UART_IS_VALID_BAUD(config->baud_rate)) {
        return ERR_UART_INVALID_BAUD;
    }

    uart_registers_t regs = get_uart_registers(port);
    if (!regs.UCSRnA) {
        return ERR_UART_INVALID_PORT;
    }

    // Initialize port status
    memset(&port_status[port], 0, sizeof(port_status[0]));
    port_status[port].initialized = true;

    return ERR_SUCCESS;
}

error_code_t uart_avr_deinit_port(uart_port_t port) {
    if (port >= UART_PORT_MAX) {
        return ERR_INVALID_PARAM;
    }

    if (!port_status[port].initialized) {
        return ERR_SUCCESS; // Already deinitialized
    }

    uart_registers_t regs = get_uart_registers(port);
    if (!regs.UCSRnA) {
        return ERR_UART_INVALID_PORT;
    }

    // Disable all interrupts for this port
    *regs.UCSRnB = 0;

    // Clear status
    memset(&port_status[port], 0, sizeof(port_status[0]));

    return ERR_SUCCESS;
}

bool uart_avr_is_tx_ready(uart_port_t port) {
    if (port >= UART_PORT_MAX || !port_status[port].initialized) {
        return false;
    }

    uart_registers_t regs = get_uart_registers(port);
    return regs.UCSRnA && (*regs.UCSRnA & UART_UCSRA_UDRE);
}

bool uart_avr_is_rx_ready(uart_port_t port) {
    if (port >= UART_PORT_MAX || !port_status[port].initialized) {
        return false;
    }

    uart_registers_t regs = get_uart_registers(port);
    return regs.UCSRnA && (*regs.UCSRnA & UART_UCSRA_RXC);
}

uint16_t uart_avr_get_status(uart_port_t port) {
    if (port >= UART_PORT_MAX || !port_status[port].initialized) {
        return 0;
    }

    uint16_t status = 0;
    uart_registers_t regs = get_uart_registers(port);
    
    if (regs.UCSRnA) {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            status = *regs.UCSRnA;
            status |= (port_status[port].error_count << 8);
        }
    }

    return status;
}
