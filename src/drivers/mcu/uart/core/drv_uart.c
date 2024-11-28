/*
        ATMEGA uart common layer.
        - buffered uart io.
        - with RXC,TXC,UDRE interrupt

        @ver 1.1

        2015/12/04 interrupt e/d updated, var updated.
*/
#include "drv_uart.h"
#include "../../../../config/cfg_build.h"
#include "../../../../utils/common/error_codes.h"
#include "../hal/drv_uart_hw.h"
#include "../hal/drv_uart_private.h"
#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdlib.h>
#include <string.h>
#include <util/atomic.h>

// UART instances
uart_instance_t uart_instances[UART_PORT_MAX];

// Get hardware registers for a UART port
const uart_registers_t *uart_get_registers(uart_port_t port)
{
  if (port >= UART_PORT_MAX) {
    return NULL;
  }
  return &uart_registers[port];
}

// Default buffer sizes if not specified in config
#define DEFAULT_RX_BUFFER_SIZE 64
#define DEFAULT_TX_BUFFER_SIZE 64

// Maximum number of UART ports
#define MAX_UART_PORTS UART_PORT_MAX

// Forward declarations of static functions
static error_code_t uart_configure_port(uart_port_t port, const uart_config_t *config);
static void uart_update_status(uart_port_t port, error_code_t error);

// Initialize UART port
error_code_t uart_init(uart_port_t port, const uart_config_t *config)
{
  if (port >= UART_PORT_MAX || !config) {
    return ERR_INVALID_PARAM;
  }

  uart_instance_t *inst = &uart_instances[port];
  error_code_t err;

  // Store configuration
  inst->config = *config;

  // Allocate and initialize RX buffer
  inst->rx_buffer_data = malloc(config->rx_buffer_size);
  if (!inst->rx_buffer_data) {
    return ERR_OUT_OF_MEMORY;
  }

  err = ring_buffer_init(&inst->rx_buffer, inst->rx_buffer_data, config->rx_buffer_size);
  if (err != ERR_SUCCESS) {
    free(inst->rx_buffer_data);
    return err;
  }

  // Allocate and initialize TX buffer
  inst->tx_buffer_data = malloc(config->tx_buffer_size);
  if (!inst->tx_buffer_data) {
    free(inst->rx_buffer_data);
    return ERR_OUT_OF_MEMORY;
  }

  err = ring_buffer_init(&inst->tx_buffer, inst->tx_buffer_data, config->tx_buffer_size);
  if (err != ERR_SUCCESS) {
    free(inst->rx_buffer_data);
    free(inst->tx_buffer_data);
    return err;
  }

  // Initialize flags
  inst->tx_busy = false;
  inst->rx_busy = false;

  // Clear status
  memset(&inst->status, 0, sizeof(uart_status_t));

  // Configure UART hardware
  err = uart_configure_port(port, config);
  if (err != ERR_SUCCESS) {
    free(inst->rx_buffer_data);
    free(inst->tx_buffer_data);
    return err;
  }

  return ERR_SUCCESS;
}

// Configure UART hardware
static error_code_t uart_configure_port(uart_port_t port, const uart_config_t *config)
{
  const uart_registers_t *regs = uart_get_registers(port);
  uint8_t ucsr_a               = 0;
  uint8_t ucsr_b               = 0;
  uint8_t ucsr_c               = 0;

  // Calculate baud rate register value
  uint16_t baud_setting = (F_CPU / 8 / config->baud_rate - 1);

  // Configure UCSRA
  ucsr_a = (1 << U2X0); // Double speed mode

  // Configure UCSRB
  ucsr_b = regs->RXCIEn |               // RX Complete Interrupt Enable
           (1 << RXEN0) | (1 << TXEN0); // Enable RX and TX

  // Configure UCSRC
  // Set data bits
  switch (config->data_bits) {
  case 5:
    ucsr_c |= 0;
    break;
  case 6:
    ucsr_c |= (1 << UCSZ00);
    break;
  case 7:
    ucsr_c |= (1 << UCSZ01);
    break;
  case 8:
    ucsr_c |= (3 << UCSZ00);
    break;
  default:
    return ERR_INVALID_PARAM;
  }

  // Set stop bits
  if (config->stop_bits == 2) {
    ucsr_c |= (1 << USBS0);
  }

  // Set parity
  switch (config->parity) {
  case UART_PARITY_NONE:
    break;
  case UART_PARITY_EVEN:
    ucsr_c |= (2 << UPM00);
    break;
  case UART_PARITY_ODD:
    ucsr_c |= (3 << UPM00);
    break;
  default:
    return ERR_INVALID_PARAM;
  }

  // Disable interrupts during configuration
  uint8_t sreg = SREG;
  cli();

  // Apply configuration
  *regs->UCSRnA = ucsr_a;
  *regs->UCSRnB = ucsr_b;
  *regs->UCSRnC = ucsr_c;

  // Set baud rate
  *regs->UBRRnH = baud_setting >> 8;
  *regs->UBRRnL = baud_setting;

  // Restore interrupt state
  SREG = sreg;

  // Verify configuration was applied correctly
  if (*regs->UCSRnA != ucsr_a || *regs->UCSRnB != ucsr_b || *regs->UCSRnC != ucsr_c ||
      *regs->UBRRnH != (baud_setting >> 8) || *regs->UBRRnL != (baud_setting & 0xFF)) {
    return ERR_UART_FRAME;
  }

  return ERR_SUCCESS;
}

// Update UART status
static void uart_update_status(uart_port_t port, error_code_t error)
{
  uart_instance_t *inst = &uart_instances[port];

  switch (error) {
  case ERR_UART_FRAME:
    inst->status.framing_errors++;
    break;
  case ERR_UART_PARITY:
    inst->status.parity_errors++;
    break;
  case ERR_UART_OVERRUN:
    inst->status.overflow_count++;
    break;
  case ERR_BUFFER_FULL:
    inst->status.overflow_count++;
    break;
  default:
    break;
  }

  inst->status.error_count++;

  if (inst->config.error_callback) {
    inst->config.error_callback(error);
  }
}

// Write data to UART port
error_code_t uart_write(uart_port_t port, const uint8_t *data, uint16_t size)
{
  if (port >= UART_PORT_MAX || !data || size == 0) {
    return ERR_INVALID_PARAM;
  }

  uart_instance_t *inst = &uart_instances[port];
  error_code_t err;

  // Push data to TX buffer
  for (uint16_t i = 0; i < size; i++) {
    err = ring_buffer_push(&inst->tx_buffer, data[i]);
    if (err != ERR_SUCCESS) {
      return err;
    }
  }

  // Start transmission if not already busy
  if (!inst->tx_busy) {
    const uart_registers_t *regs = uart_get_registers(port);
    *regs->UCSRnB |= regs->UDRIEn; // Enable UDRE interrupt
    inst->tx_busy = true;
  }

  return ERR_SUCCESS;
}

// Read data from UART port
error_code_t uart_read(uart_port_t port, uint8_t *data, uint16_t size, uint16_t *bytes_read)
{
  if (port >= UART_PORT_MAX || !data || size == 0) {
    return ERR_INVALID_PARAM;
  }

  uart_instance_t *inst = &uart_instances[port];
  error_code_t err;
  uint16_t count = 0;

  // Pop data from RX buffer
  while (count < size) {
    err = ring_buffer_pop(&inst->rx_buffer, &data[count]);
    if (err != ERR_SUCCESS) {
      break;
    }
    count++;
  }

  if (bytes_read) {
    *bytes_read = count;
  }

  return count > 0 ? ERR_SUCCESS : ERR_UNDERFLOW;
}

// UART RX interrupt handlers
ISR(USART1_RX_vect)
{
  uart_instance_t *inst        = &uart_instances[UART_PORT_1];
  const uart_registers_t *regs = uart_get_registers(UART_PORT_1);

  uint8_t status = *regs->UCSRnA;
  uint8_t data   = *regs->UDRn;

  if (status & UART_UCSRA_FE) {
    uart_update_status(UART_PORT_1, ERR_UART_FRAME);
    return;
  }
  if (status & UART_UCSRA_UPE) {
    uart_update_status(UART_PORT_1, ERR_UART_PARITY);
    return;
  }
  if (status & UART_UCSRA_DOR) {
    uart_update_status(UART_PORT_1, ERR_UART_OVERRUN);
    return;
  }

  if (ring_buffer_push(&inst->rx_buffer, data) != ERR_SUCCESS) {
    uart_update_status(UART_PORT_1, ERR_BUFFER_FULL);
  } else {
    inst->status.rx_count++;
  }
}

ISR(USART2_RX_vect)
{
  uart_instance_t *inst        = &uart_instances[UART_PORT_2];
  const uart_registers_t *regs = uart_get_registers(UART_PORT_2);

  uint8_t status = *regs->UCSRnA;
  uint8_t data   = *regs->UDRn;

  if (status & UART_UCSRA_FE) {
    uart_update_status(UART_PORT_2, ERR_UART_FRAME);
    return;
  }
  if (status & UART_UCSRA_UPE) {
    uart_update_status(UART_PORT_2, ERR_UART_PARITY);
    return;
  }
  if (status & UART_UCSRA_DOR) {
    uart_update_status(UART_PORT_2, ERR_UART_OVERRUN);
    return;
  }

  if (ring_buffer_push(&inst->rx_buffer, data) != ERR_SUCCESS) {
    uart_update_status(UART_PORT_2, ERR_BUFFER_FULL);
  } else {
    inst->status.rx_count++;
  }
}

ISR(USART3_RX_vect)
{
  uart_instance_t *inst        = &uart_instances[UART_PORT_3];
  const uart_registers_t *regs = uart_get_registers(UART_PORT_3);

  uint8_t status = *regs->UCSRnA;
  uint8_t data   = *regs->UDRn;

  if (status & UART_UCSRA_FE) {
    uart_update_status(UART_PORT_3, ERR_UART_FRAME);
    return;
  }
  if (status & UART_UCSRA_UPE) {
    uart_update_status(UART_PORT_3, ERR_UART_PARITY);
    return;
  }
  if (status & UART_UCSRA_DOR) {
    uart_update_status(UART_PORT_3, ERR_UART_OVERRUN);
    return;
  }

  if (ring_buffer_push(&inst->rx_buffer, data) != ERR_SUCCESS) {
    uart_update_status(UART_PORT_3, ERR_BUFFER_FULL);
  } else {
    inst->status.rx_count++;
  }
}

// UART TX interrupt handlers
ISR(USART1_UDRE_vect)
{
  uart_instance_t *inst = &uart_instances[UART_PORT_1];
  // NOTE: You can write into uart hardware register even if it's const
  const uart_registers_t *regs = uart_get_registers(UART_PORT_1);
  uint8_t data;

  if (ring_buffer_pop(&inst->tx_buffer, &data) == ERR_SUCCESS) {
    *regs->UDRn = data;
    inst->status.tx_count++;
  } else {
    // No more data to send
    *regs->UCSRnB &= ~regs->UDRIEn; // Disable UDRE interrupt
    inst->tx_busy = false;
  }
}

ISR(USART2_UDRE_vect)
{
  uart_instance_t *inst        = &uart_instances[UART_PORT_2];
  const uart_registers_t *regs = uart_get_registers(UART_PORT_2);
  uint8_t data;

  if (ring_buffer_pop(&inst->tx_buffer, &data) == ERR_SUCCESS) {
    *regs->UDRn = data;
    inst->status.tx_count++;
  } else {
    // No more data to send
    *regs->UCSRnB &= ~regs->UDRIEn; // Disable UDRE interrupt
    inst->tx_busy = false;
  }
}

ISR(USART3_UDRE_vect)
{
  uart_instance_t *inst        = &uart_instances[UART_PORT_3];
  const uart_registers_t *regs = uart_get_registers(UART_PORT_3);
  uint8_t data;

  if (ring_buffer_pop(&inst->tx_buffer, &data) == ERR_SUCCESS) {
    *regs->UDRn = data;
    inst->status.tx_count++;
  } else {
    // No more data to send
    *regs->UCSRnB &= ~regs->UDRIEn; // Disable UDRE interrupt
    inst->tx_busy = false;
  }
}

// Deinitialize UART port
error_code_t uart_deinit(uart_port_t port)
{
  if (port >= UART_PORT_MAX) {
    return ERR_INVALID_PARAM;
  }

  uart_instance_t *inst = &uart_instances[port];

  // Free buffers
  free(inst->rx_buffer_data);
  free(inst->tx_buffer_data);

  // Reset instance
  memset(inst, 0, sizeof(uart_instance_t));

  return ERR_SUCCESS;
}

// Get UART status
const uart_status_t *uart_get_status(uart_port_t port)
{
  if (port >= UART_PORT_MAX) {
    return NULL;
  }

  uart_instance_t *inst = &uart_instances[port];

  return &inst->status;
}
