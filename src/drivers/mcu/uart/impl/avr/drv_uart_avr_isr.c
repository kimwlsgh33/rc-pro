/**
 * @file drv_uart_avr_isr.c
 * @brief AVR-specific UART Interrupt Service Routines Implementation
 *
 * This file implements the interrupt service routines for AVR UART ports.
 * It handles RX, TX, and UDRE (data register empty) interrupts for all
 * available UART ports.
 */

#include "drv_uart_avr_isr.h"
#include "../../hal/drv_uart_hal.h"
#include "../../hal/drv_uart_hw.h"
#include <avr/interrupt.h>
#include <avr/io.h>

/**
 * @brief Common RX ISR handler for all UART ports
 * 
 * @param port UART port number
 * @param regs Pointer to port-specific registers
 */
static inline void uart_rx_handler(uart_port_t port, uart_registers_t regs)
{
    uint8_t status = *regs.UCSRnA;
    uint8_t data = *regs.UDRn;

    // Check for errors first
    if (status & UART_ERR_MASK) {
        uart_hal_handle_error(port, status & UART_ERR_MASK);
        return;
    }

    // Process received data
    uart_hal_handle_rx(port, data);
}

/**
 * @brief Common TX complete ISR handler for all UART ports
 * 
 * @param port UART port number
 * @param regs Pointer to port-specific registers
 */
static inline void uart_tx_handler(uart_port_t port, uart_registers_t regs)
{
    // Clear TX complete flag by reading status and writing UDR
    uint8_t status = *regs.UCSRnA;
    (void)status; // Prevent unused variable warning

    // Notify HAL that transmission is complete
    uart_hal_handle_tx_complete(port);
}

/**
 * @brief Common UDRE (data register empty) ISR handler for all UART ports
 * 
 * @param port UART port number
 * @param regs Pointer to port-specific registers
 */
static inline void uart_udre_handler(uart_port_t port, uart_registers_t regs)
{
    // Notify HAL that we can transmit another byte
    uart_hal_handle_tx(port);
    
    // Disable UDRE interrupt until next transmission
    *regs.UCSRnB &= ~(1 << UDRIE0);
}

// Port 1 ISR implementations
#if defined(UART_PORT1_RX_VECT)
ISR(UART_PORT1_RX_VECT)
{
    uart_rx_handler(UART_PORT_1, get_uart_registers(UART_PORT_1));
}
#endif

#if defined(UART_PORT1_UDRE_VECT)
ISR(UART_PORT1_UDRE_VECT)
{
    uart_udre_handler(UART_PORT_1, get_uart_registers(UART_PORT_1));
}
#endif

#if defined(UART_PORT1_TX_VECT)
ISR(UART_PORT1_TX_VECT)
{
    uart_tx_handler(UART_PORT_1, get_uart_registers(UART_PORT_1));
}
#endif

// Port 2 ISR implementations
#if defined(UART_PORT2_RX_VECT)
ISR(UART_PORT2_RX_VECT)
{
    uart_rx_handler(UART_PORT_2, get_uart_registers(UART_PORT_2));
}
#endif

#if defined(UART_PORT2_UDRE_VECT)
ISR(UART_PORT2_UDRE_VECT)
{
    uart_udre_handler(UART_PORT_2, get_uart_registers(UART_PORT_2));
}
#endif

#if defined(UART_PORT2_TX_VECT)
ISR(UART_PORT2_TX_VECT)
{
    uart_tx_handler(UART_PORT_2, get_uart_registers(UART_PORT_2));
}
#endif

// Port 3 ISR implementations
#if defined(UART_PORT3_RX_VECT)
ISR(UART_PORT3_RX_VECT)
{
    uart_rx_handler(UART_PORT_3, get_uart_registers(UART_PORT_3));
}
#endif

#if defined(UART_PORT3_UDRE_VECT)
ISR(UART_PORT3_UDRE_VECT)
{
    uart_udre_handler(UART_PORT_3, get_uart_registers(UART_PORT_3));
}
#endif

#if defined(UART_PORT3_TX_VECT)
ISR(UART_PORT3_TX_VECT)
{
    uart_tx_handler(UART_PORT_3, get_uart_registers(UART_PORT_3));
}
#endif
