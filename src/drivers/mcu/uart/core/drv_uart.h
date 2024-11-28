#pragma once

#include "../common/drv_uart_common.h"
#include "../common/drv_uart_types.h"

// Uart parity types
typedef enum {
  UART_PARITY_NONE,
  UART_PARITY_EVEN,
  UART_PARITY_ODD,
} uart_parity_t;

error_code_t uart_write(uart_port_t port, const uint8_t *data, uint16_t size);
