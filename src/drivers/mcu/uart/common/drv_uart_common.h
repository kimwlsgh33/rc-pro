/**
 * @file drv_uart_common.h
 * @brief Common UART driver functionality
 */

#ifndef DRV_UART_COMMON_H
#define DRV_UART_COMMON_H

#include "drv_uart_types.h"
#include <stdbool.h>

/**
 * @brief Initialize a UART buffer
 * 
 * @param buffer Buffer structure to initialize
 * @param data Buffer data array
 * @param size Size of the buffer
 */
void uart_buffer_init(uart_buffer_t *buffer, uint8_t *data, uint16_t size);

/**
 * @brief Write a byte to the buffer
 * 
 * @param buffer Buffer to write to
 * @param byte Byte to write
 * @return true if write was successful
 * @return false if buffer is full
 */
bool uart_buffer_write(uart_buffer_t *buffer, uint8_t byte);

/**
 * @brief Read a byte from the buffer
 * 
 * @param buffer Buffer to read from
 * @param byte Pointer to store read byte
 * @return true if read was successful
 * @return false if buffer is empty
 */
bool uart_buffer_read(uart_buffer_t *buffer, uint8_t *byte);

/**
 * @brief Get number of bytes available in buffer
 * 
 * @param buffer Buffer to check
 * @return uint16_t Number of bytes available
 */
uint16_t uart_buffer_available(const uart_buffer_t *buffer);

/**
 * @brief Get free space in buffer
 * 
 * @param buffer Buffer to check
 * @return uint16_t Number of bytes free
 */
uint16_t uart_buffer_space(const uart_buffer_t *buffer);

/**
 * @brief Flush buffer
 * 
 * @param buffer Buffer to flush
 */
void uart_buffer_flush(uart_buffer_t *buffer);

/**
 * @brief Calculate UART baud rate register value
 * 
 * @param baud_rate Desired baud rate
 * @param cpu_freq CPU frequency in Hz
 * @param u2x Use double speed mode
 * @return uint16_t Baud rate register value
 */
uint16_t uart_calculate_brr(uint32_t baud_rate, uint32_t cpu_freq, bool u2x);

#endif /* DRV_UART_COMMON_H */
