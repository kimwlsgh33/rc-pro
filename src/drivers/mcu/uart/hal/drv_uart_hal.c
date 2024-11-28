#include "drv_uart_hal.h"
#include "../../../../config/cfg_build.h"
#include "../common/drv_uart_common.h"
#include "drv_uart_hw.h"
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/atomic.h>

// Callback arrays for each UART port
static struct {
  void (*rx_callback)(uint8_t);
  void (*tx_callback)(void);
  void (*error_callback)(uint16_t);
  void (*tx_complete_callback)(void);
} uart_callbacks[UART_PORT_MAX] = {0};

/**
 * @brief Calculate UBRR value for given baud rate
 * @param baud_rate Desired baud rate
 * @param use_u2x Use double speed mode if true
 * @return UBRR value
 */
static uint16_t calculate_ubrr(uint32_t baud_rate, bool use_u2x)
{
  return (F_CPU / ((use_u2x ? 8UL : 16UL) * baud_rate)) - 1;
}

error_code_t uart_hal_init(uart_port_t port, uint32_t baud_rate, uint8_t data_bits,
                           uint8_t stop_bits, uint8_t parity)
{
  // Validate parameters
  if (port >= UART_PORT_MAX || !UART_IS_VALID_BAUD(baud_rate) || data_bits < 5 || data_bits > 9 ||
      stop_bits < 1 || stop_bits > 2 || parity > 2) {
    return ERR_INVALID_PARAM;
  }

  // Calculate baud rate with and without U2X
  uint16_t ubrr = calculate_ubrr(baud_rate, true);
  bool use_u2x  = true;

  // If U2X gives a high error rate, use normal speed
  if (ubrr > 4095) {
    ubrr    = calculate_ubrr(baud_rate, false);
    use_u2x = false;
  }

  // Configure UART in atomic block to prevent interrupts
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    // Set baud rate
    *uart_registers[port].UBRRnH = (uint8_t)(ubrr >> 8);
    *uart_registers[port].UBRRnL = (uint8_t)ubrr;

    // Configure UCSRnA - Enable/disable double speed
    *uart_registers[port].UCSRnA = use_u2x ? UART_UCSRA_U2X : 0;

    // Configure UCSRnC - Set frame format
    uint8_t ucsrc = 0;

    // Set data bits
    if (data_bits == 9) {
      ucsrc |= UART_UCSRC_UCSZ1 | UART_UCSRC_UCSZ0;
    } else if (data_bits >= 5) {
      ucsrc |= ((data_bits - 5) << UCSZ00);
    }

    // Set stop bits
    if (stop_bits == 2) {
      ucsrc |= UART_UCSRC_USBS;
    }

    // Set parity
    switch (parity) {
    case 1:
      ucsrc |= UART_UCSRC_UPM1;
      break; // Even parity
    case 2:
      ucsrc |= UART_UCSRC_UPM1 | UART_UCSRC_UPM0;
      break; // Odd parity
    }

    *uart_registers[port].UCSRnC = ucsrc;

    // Enable transmitter and receiver with interrupts
    *uart_registers[port].UCSRnB =
        UART_UCSRB_RXEN | UART_UCSRB_TXEN | UART_UCSRB_RXCIE | UART_UCSRB_TXCIE;

    // Set UCSZ2 for 9-bit character size if needed
    if (data_bits == 9) {
      *uart_registers[port].UCSRnB |= UART_UCSRB_UCSZ2;
    }
  }

  return ERR_SUCCESS;
}

error_code_t uart_hal_deinit(uart_port_t port)
{
  if (port >= UART_PORT_MAX) {
    return ERR_INVALID_PARAM;
  }

  // Disable all UART functionality and interrupts
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    *uart_registers[port].UCSRnB = 0;
    *uart_registers[port].UCSRnA = 0;
    *uart_registers[port].UCSRnC = 0;
  }

  // Clear callbacks
  uart_callbacks[port].rx_callback    = NULL;
  uart_callbacks[port].tx_callback    = NULL;
  uart_callbacks[port].error_callback = NULL;
  uart_callbacks[port].tx_complete_callback = NULL;

  return ERR_SUCCESS;
}

error_code_t uart_hal_write_byte(uart_port_t port, uint8_t byte)
{
  if (port >= UART_PORT_MAX) {
    return ERR_INVALID_PARAM;
  }

  // Wait for data register to be empty
  while (!(*uart_registers[port].UCSRnA & UART_UCSRA_UDRE))
    ;

  // Write byte to data register
  *uart_registers[port].UDRn = byte;
  return ERR_SUCCESS;
}

error_code_t uart_hal_read_byte(uart_port_t port, uint8_t *byte)
{
  if (port >= UART_PORT_MAX || !byte) {
    return ERR_INVALID_PARAM;
  }

  // Wait for data to be received
  while (!(*uart_registers[port].UCSRnA & UART_UCSRA_RXC))
    ;

  // Check for errors
  uint8_t status = *uart_registers[port].UCSRnA;
  if (status & UART_ERR_MASK) {
    uart_hal_handle_error(port, status & UART_ERR_MASK);
    return ERR_HARDWARE;
  }

  // Read byte from data register
  *byte = *uart_registers[port].UDRn;
  return ERR_SUCCESS;
}

error_code_t uart_hal_enable_udre_interrupt(uart_port_t port)
{
    if (port >= UART_PORT_MAX) {
        return ERR_INVALID_PARAM;
    }

    *uart_registers[port].UCSRnB |= UART_UCSRB_UDRIE;
    return ERR_SUCCESS;
}

void uart_hal_set_callbacks(uart_port_t port, void (*rx_callback)(uint8_t),
                          void (*tx_callback)(void), void (*error_callback)(uint16_t),
                          void (*tx_complete_callback)(void))
{
  if (port < UART_PORT_MAX) {
    uart_callbacks[port].rx_callback = rx_callback;
    uart_callbacks[port].tx_callback = tx_callback;
    uart_callbacks[port].error_callback = error_callback;
    uart_callbacks[port].tx_complete_callback = tx_complete_callback;
  }
}

void uart_hal_handle_rx(uart_port_t port, uint8_t data)
{
  if (port < UART_PORT_MAX && uart_callbacks[port].rx_callback) {
    uart_callbacks[port].rx_callback(data);
  }
}

void uart_hal_handle_tx(uart_port_t port)
{
  if (port < UART_PORT_MAX && uart_callbacks[port].tx_callback) {
    uart_callbacks[port].tx_callback();
  }
}

void uart_hal_handle_error(uart_port_t port, uint16_t error)
{
  if (port < UART_PORT_MAX && uart_callbacks[port].error_callback) {
    uart_callbacks[port].error_callback(error);
  }
}

void uart_hal_handle_tx_complete(uart_port_t port)
{
  if (port < UART_PORT_MAX && uart_callbacks[port].tx_complete_callback) {
    uart_callbacks[port].tx_complete_callback();
  }
}
