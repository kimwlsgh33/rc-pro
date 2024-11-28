/**
 * @file drv_uart_common.c
 * @brief Common UART driver functionality implementation
 */

#include "drv_uart_common.h"
#include "../../../../utils/common/error_codes.h"
#include "../../../../utils/common/ring_buffer.h"
#include <stdbool.h>

error_code_t uart_buffer_init(uart_buffer_t *buffer,
                            uint8_t *tx_data,
                            uint8_t *rx_data,
                            uint16_t buffer_size)
{
    if (!buffer || !tx_data || !rx_data || buffer_size == 0) {
        return ERR_INVALID_PARAM;
    }

    // Initialize both ring buffers
    error_code_t err = ring_buffer_init(&buffer->tx_buffer, tx_data, buffer_size);
    if (err != ERR_SUCCESS) {
        return err;
    }
    
    err = ring_buffer_init(&buffer->rx_buffer, rx_data, buffer_size);
    if (err != ERR_SUCCESS) {
        return err;
    }
    
    // Store buffer data pointers
    buffer->tx_buffer_data = tx_data;
    buffer->rx_buffer_data = rx_data;
    
    // Clear busy flags
    buffer->tx_busy = false;
    buffer->rx_busy = false;

    return ERR_SUCCESS;
}

bool uart_buffer_get_tx_byte(uart_buffer_t *buffer, uint8_t *byte)
{
    if (!buffer || !byte) {
        return false;
    }

    // Check if there's data in TX buffer
    if (ring_buffer_is_empty(&buffer->tx_buffer)) {
        buffer->tx_busy = false;
        return false;
    }

    // Get next byte from buffer
    error_code_t err = ring_buffer_pop(&buffer->tx_buffer, byte);
    if (err != ERR_SUCCESS) {
        buffer->tx_busy = false;
        return false;
    }

    buffer->tx_busy = true;
    return true;
}

error_code_t uart_buffer_add_tx_byte(uart_buffer_t *buffer, uint8_t byte)
{
    if (!buffer) {
        return ERR_INVALID_PARAM;
    }

    return ring_buffer_push(&buffer->tx_buffer, byte);
}

error_code_t uart_buffer_store_rx_byte(uart_buffer_t *buffer, uint8_t byte)
{
    if (!buffer) {
        return ERR_INVALID_PARAM;
    }

    buffer->rx_busy = true;
    error_code_t err = ring_buffer_push(&buffer->rx_buffer, byte);
    buffer->rx_busy = false;

    return err;
}

uint16_t uart_calculate_brr(uint32_t baud_rate, uint32_t cpu_freq, bool u2x)
{
    uint16_t brr;
    
    if (u2x) {
        brr = ((cpu_freq / 8) / baud_rate) - 1;
    } else {
        brr = ((cpu_freq / 16) / baud_rate) - 1;
    }
    
    return brr;
}
