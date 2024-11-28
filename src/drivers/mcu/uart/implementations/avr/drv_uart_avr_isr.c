#include "drv_uart_avr_isr.h"
#include "../../hal/drv_uart_hal.h"
#include "../../hal/drv_uart_hw.h"
#include <avr/interrupt.h>
#include <avr/io.h>

// ISR implementations for AVR UART
#if defined(UART_PORT1_RX_VECT)
ISR(UART_PORT1_RX_VECT)
{
  uart_registers_t regs = get_uart_registers(UART_PORT_1);
  uint8_t status        = *regs.UCSRnA;
  uint8_t data          = *regs.UDRn;

  // Check for errors
  if (status & UART_ERR_MASK) {
    uart_hal_handle_error(UART_PORT_1, status & UART_ERR_MASK);
    return;
  }

  // Process received data
  uart_hal_handle_rx(UART_PORT_1, data);
}
#endif

#if defined(UART_PORT2_RX_VECT)
ISR(UART_PORT2_RX_VECT)
{
  uart_registers_t regs = get_uart_registers(UART_PORT_2);
  uint8_t status        = *regs.UCSRnA;
  uint8_t data          = *regs.UDRn;

  // Check for errors
  if (status & UART_ERR_MASK) {
    uart_hal_handle_error(UART_PORT_2, status & UART_ERR_MASK);
    return;
  }

  // Process received data
  uart_hal_handle_rx(UART_PORT_2, data);
}
#endif

#if defined(UART_PORT3_RX_VECT)
ISR(UART_PORT3_RX_VECT)
{
  uart_registers_t regs = get_uart_registers(UART_PORT_3);
  uint8_t status        = *regs.UCSRnA;
  uint8_t data          = *regs.UDRn;

  // Check for errors
  if (status & UART_ERR_MASK) {
    uart_hal_handle_error(UART_PORT_3, status & UART_ERR_MASK);
    return;
  }

  // Process received data
  uart_hal_handle_rx(UART_PORT_3, data);
}
#endif

#if defined(UART_PORT1_TX_VECT)
ISR(UART_PORT1_TX_VECT) { uart_hal_handle_tx(UART_PORT_1); }
#endif

#if defined(UART_PORT2_TX_VECT)
ISR(UART_PORT2_TX_VECT) { uart_hal_handle_tx(UART_PORT_2); }
#endif

#if defined(UART_PORT3_TX_VECT)
ISR(UART_PORT3_TX_VECT) { uart_hal_handle_tx(UART_PORT_3); }
#endif
