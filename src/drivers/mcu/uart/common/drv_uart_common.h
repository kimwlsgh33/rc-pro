/**
 * @file drv_uart_common.h
 * @brief Common UART driver functionality and types
 */

#ifndef DRV_UART_COMMON_H
#define DRV_UART_COMMON_H

#include "drv_uart_types.h"
#include "../../../../utils/common/ring_buffer.h"

/**
 * @brief UART buffer management structure
 * 
 * Contains separate ring buffers for transmit and receive operations,
 * along with their associated data storage and status flags.
 */
typedef struct {
    ring_buffer_t tx_buffer;       // Transmit ring buffer
    ring_buffer_t rx_buffer;       // Receive ring buffer
    uint8_t *tx_buffer_data;       // Transmit buffer memory
    uint8_t *rx_buffer_data;       // Receive buffer memory
    volatile bool tx_busy;         // Transmit busy flag
    volatile bool rx_busy;         // Receive busy flag
} uart_buffer_t;

/**
 * @brief Initialize UART buffer
 * 
 * @param buffer Buffer structure to initialize
 * @param tx_data Transmit buffer memory
 * @param rx_data Receive buffer memory
 * @param buffer_size Size of each buffer
 * @return error_code_t ERR_SUCCESS if successful
 */
error_code_t uart_buffer_init(uart_buffer_t *buffer,
                            uint8_t *tx_data,
                            uint8_t *rx_data,
                            uint16_t buffer_size);

/**
 * @brief Get next byte from transmit buffer
 * 
 * @param buffer Buffer to get byte from
 * @param byte Pointer to store byte
 * @return true if byte retrieved
 */
bool uart_buffer_get_tx_byte(uart_buffer_t *buffer, uint8_t *byte);

/**
 * @brief Add byte to transmit buffer
 * 
 * @param buffer Buffer to add byte to
 * @param byte Byte to add
 * @return error_code_t ERR_SUCCESS if successful
 */
error_code_t uart_buffer_add_tx_byte(uart_buffer_t *buffer, uint8_t byte);

/**
 * @brief Store received byte in buffer
 * 
 * @param buffer Buffer to store byte in
 * @param byte Byte to store
 * @return error_code_t ERR_SUCCESS if successful
 */
error_code_t uart_buffer_store_rx_byte(uart_buffer_t *buffer, uint8_t byte);

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
