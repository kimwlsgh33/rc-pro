/**
 * @file drv_uart2.c
 * @brief UART2 Driver Implementation using HAL Interface
 *
 * This file implements the UART2 driver using the UART HAL interface
 * and AVR-specific implementation.
 */
#include "drv_uart2.h"
#include "../common/drv_uart_types.h"
#include "../common/drv_uart_common.h"
#include "../hal/drv_uart_hal.h"
#include "../hal/drv_uart_hw.h"
#include "avr/drv_uart_avr.h"
#include <string.h>

// Buffer management
static uart_buffer_t uart2_buffer;
static uint8_t uart2_tx_data[UART2_BUFFER_SIZE];
static uint8_t uart2_rx_data[UART2_BUFFER_SIZE];

// UART2 status
static uart_status_t uart2_status;

// Local callback handlers
static void uart2_error_handler(uint16_t error) {
    uart2_status.error_count++;
    uart2_status.last_error = error;

    switch (error) {
        case ERR_UART_FRAME:
            uart2_status.framing_errors++;
            break;
        case ERR_UART_OVERFLOW:
            uart2_status.overflow_count++;
            break;
        case ERR_UART_PARITY:
            uart2_status.parity_errors++;
            break;
    }
}

static void uart2_rx_handler(uint8_t data) {
    uart2_status.rx_count++;
    uart_buffer_store_rx_byte(&uart2_buffer, data);
}

static void uart2_tx_handler(void) {
    uint8_t byte;
    if (uart_buffer_get_tx_byte(&uart2_buffer, &byte)) {
        uart_hal_write_byte(UART_PORT_2, byte);
        uart2_status.tx_count++;
        
        // Enable UDRE interrupt using HAL interface
        uart_hal_enable_udre_interrupt(UART_PORT_2);
    }
}

error_code_t uart2_init(uint32_t baud)
{
    // Initialize buffer management
    error_code_t err = uart_buffer_init(&uart2_buffer, uart2_tx_data, uart2_rx_data, UART2_BUFFER_SIZE);
    if (err != ERR_SUCCESS) {
        return err;
    }

    // Reset status
    memset(&uart2_status, 0, sizeof(uart2_status));

    // Initialize UART hardware
    err = uart_hal_init(UART_PORT_2, baud, 8, 1, UART_PARITY_NONE);
    if (err != ERR_SUCCESS) {
        return err;
    }

    // Set up callbacks
    uart_hal_set_callbacks(UART_PORT_2, 
                          uart2_rx_handler,
                          uart2_tx_handler,
                          uart2_error_handler,
                          NULL);  // No TX complete callback needed

    return ERR_SUCCESS;
}

error_code_t uart2_deinit(void) {
    return uart_hal_deinit(UART_PORT_2);
}

int uart2_getchar(uint8_t *c)
{
    if (!c) {
        return -1;
    }

    uint8_t byte;
    error_code_t err = ring_buffer_pop(&uart2_buffer.rx_buffer, &byte);
    if (err != ERR_SUCCESS) {
        return -1;
    }

    *c = byte;
    return 0;
}

int uart2_putchar(uint8_t c) {
    error_code_t err = uart_buffer_add_tx_byte(&uart2_buffer, c);
    if (err != ERR_SUCCESS) {
        return -1;  // Buffer full
    }

    // Start transmission if not already busy
    if (!uart2_buffer.tx_busy) {
        uart2_tx_handler();
    }

    return 0;
}

void uart2_set_callbacks(void (*error_cb)(uint16_t), void (*rx_cb)(uint8_t), void (*tx_cb)(void)) {
    uart_hal_set_callbacks(UART_PORT_2, rx_cb, tx_cb, error_cb, NULL);
}

bool uart2_is_rx_ready(void) {
    return uart_avr_is_rx_ready(UART_PORT_2);
}

bool uart2_is_tx_ready(void) {
    return uart_avr_is_tx_ready(UART_PORT_2);
}

uart_status_t uart2_get_status(void) {
    return uart2_status;
}
