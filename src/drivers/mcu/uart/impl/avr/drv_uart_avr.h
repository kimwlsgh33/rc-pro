/**
 * @file drv_uart_avr.h
 * @brief AVR-specific UART Implementation Interface
 *
 * This file provides the interface for AVR-specific UART operations,
 * consolidating the hardware-specific implementation with port-specific drivers.
 */

#ifndef DRV_UART_AVR_H
#define DRV_UART_AVR_H

#include "../../common/drv_uart_types.h"
#include "../../hal/drv_uart_hal.h"
#include <stdbool.h>

/**
 * @brief Initialize AVR UART hardware for a specific port
 *
 * @param port UART port to initialize
 * @param config Port configuration
 * @return error_code_t Error code
 */
error_code_t uart_avr_init_port(uart_port_t port, const uart_config_t *config);

/**
 * @brief Deinitialize AVR UART hardware for a specific port
 *
 * @param port UART port to deinitialize
 * @return error_code_t Error code
 */
error_code_t uart_avr_deinit_port(uart_port_t port);

/**
 * @brief Check if AVR UART port is ready to transmit
 *
 * @param port UART port to check
 * @return bool True if ready to transmit
 */
bool uart_avr_is_tx_ready(uart_port_t port);

/**
 * @brief Check if AVR UART port has data to receive
 *
 * @param port UART port to check
 * @return bool True if data is available
 */
bool uart_avr_is_rx_ready(uart_port_t port);

/**
 * @brief Get current status of AVR UART port
 *
 * @param port UART port to check
 * @return uint16_t Status flags
 */
uint16_t uart_avr_get_status(uart_port_t port);

#endif /* DRV_UART_AVR_H */
