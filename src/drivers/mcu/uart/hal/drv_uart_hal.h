#ifndef DRV_UART_HAL_H
#define DRV_UART_HAL_H

#include "../../../../utils/common/error_codes.h"
#include "../common/drv_uart_types.h"
#include <stdint.h>

/**
 * @file drv_uart_hal.h
 * @brief UART Hardware Abstraction Layer Interface
 *
 * This file defines the hardware abstraction layer (HAL) interface for the UART driver.
 * The HAL provides a hardware-independent interface for initializing and operating
 * the UART peripheral. It handles all direct hardware access and interrupt management.
 *
 * Features:
 * - Hardware initialization and configuration
 * - Basic read/write operations
 * - Interrupt-driven operation with callbacks
 * - Error detection and reporting
 */

/**
 * @brief Initialize UART hardware
 *
 * This function initializes the UART hardware with the specified configuration.
 * It configures the baud rate, data bits, stop bits, and parity settings.
 * The function also enables the transmitter and receiver with their respective
 * interrupts.
 *
 * @param port UART port to initialize
 * @param baud_rate Desired baud rate (e.g., 9600, 115200)
 * @param data_bits Number of data bits (5-9)
 * @param stop_bits Number of stop bits (1-2)
 * @param parity Parity setting (0=None, 1=Even, 2=Odd)
 *
 * @return ERR_SUCCESS if successful
 *         ERR_INVALID_PARAM if any parameter is invalid
 *         ERR_HARDWARE if hardware initialization fails
 */
error_code_t uart_hal_init(uart_port_t port, uint32_t baud_rate, uint8_t data_bits, uint8_t stop_bits, uint8_t parity);

/**
 * @brief Deinitialize UART hardware
 *
 * This function disables all UART functionality and interrupts for the specified port.
 * It also clears any registered callbacks.
 *
 * @param port UART port to deinitialize
 *
 * @return ERR_SUCCESS if successful
 *         ERR_INVALID_PARAM if port is invalid
 */
error_code_t uart_hal_deinit(uart_port_t port);

/**
 * @brief Write a byte to UART
 *
 * This function writes a single byte to the UART transmit buffer.
 * It blocks until the transmit buffer is ready to accept new data.
 *
 * @param port UART port to write to
 * @param byte Data byte to transmit
 *
 * @return ERR_SUCCESS if successful
 *         ERR_INVALID_PARAM if port is invalid
 */
error_code_t uart_hal_write_byte(uart_port_t port, uint8_t byte);

/**
 * @brief Read a byte from UART
 *
 * This function reads a single byte from the UART receive buffer.
 * It blocks until data is available to read.
 * If any error occurs during reception (frame, parity, overrun),
 * the error callback will be invoked if registered.
 *
 * @param port UART port to read from
 * @param byte Pointer to store received byte
 *
 * @return ERR_SUCCESS if successful
 *         ERR_INVALID_PARAM if port is invalid or byte is NULL
 *         ERR_HARDWARE if a reception error occurred
 */
error_code_t uart_hal_read_byte(uart_port_t port, uint8_t *byte);

/**
 * @brief Set UART callback functions
 *
 * This function registers callback functions for UART events:
 * - RX callback: Called when a byte is received
 * - TX callback: Called when a byte transmission is complete
 * - Error callback: Called when an error occurs (frame, parity, overrun)
 *
 * The error callback receives a bitmask of error flags:
 * - UART_UCSRA_FE: Frame Error
 * - UART_UCSRA_DOR: Data Overrun
 * - UART_UCSRA_UPE: Parity Error
 *
 * @param port UART port to set callbacks for
 * @param rx_callback Function called when byte received (can be NULL)
 * @param tx_callback Function called when byte transmitted (can be NULL)
 * @param error_callback Function called when error occurs (can be NULL)
 * @param tx_complete_callback Function called when transmission complete (can be NULL)
 */
void uart_hal_set_callbacks(uart_port_t port, void (*rx_callback)(uint8_t),
                          void (*tx_callback)(void), void (*error_callback)(uint16_t),
                          void (*tx_complete_callback)(void));

/**
 * @brief Enable UART Data Register Empty (UDRE) interrupt
 * 
 * @param port UART port to enable UDRE interrupt for
 * @return error_code_t Error code indicating success or failure
 */
error_code_t uart_hal_enable_udre_interrupt(uart_port_t port);

/**
 * @brief Internal callback handlers - Used by hardware-specific implementations
 * These functions should only be called by the hardware implementation layer
 */
void uart_hal_handle_rx(uart_port_t port, uint8_t data);
void uart_hal_handle_tx(uart_port_t port);
void uart_hal_handle_error(uart_port_t port, uint16_t error);
void uart_hal_handle_tx_complete(uart_port_t port);

#endif // DRV_UART_HAL_H
