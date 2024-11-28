/**
 * @file drv_uart1.c
 * @brief UART1 Driver Implementation using HAL Interface
 *
 * This file implements the UART1 driver using the UART HAL interface
 * and AVR-specific implementation.
 */
#include "drv_uart1.h"
#include "../common/drv_uart_types.h"
#include "../hal/drv_uart_hal.h"
#include "../hal/drv_uart_hw.h"
#include "avr/drv_uart_avr.h"
#include <string.h>

// Buffer management
static uart_buffer_t uart1_buffer;
static uint8_t uart1_tx_data[UART1_BUFFER_SIZE];
static uint8_t uart1_rx_data[UART1_BUFFER_SIZE];

// UART1 status
static uart_status_t uart1_status;

// Local callback handlers
static void uart1_error_handler(uint16_t error) {
    uart1_status.error_count++;
    uart1_status.last_error = error;

    switch (error) {
        case ERR_UART_FRAME:
            uart1_status.framing_errors++;
            break;
        case ERR_UART_OVERFLOW:
            uart1_status.overflow_count++;
            break;
        case ERR_UART_PARITY:
            uart1_status.parity_errors++;
            break;
    }
}

static void uart1_rx_handler(uint8_t data) {
    uart1_status.rx_count++;
    uart_buffer_store_rx_byte(&uart1_buffer, data);
}

static void uart1_tx_handler(void)
{
    uint8_t byte;
    if (uart_buffer_get_tx_byte(&uart1_buffer, &byte)) {
        uart_hal_write_byte(UART_PORT_1, byte);
        uart1_status.tx_count++;
        
        // Enable UDRE interrupt using HAL interface
        uart_hal_enable_udre_interrupt(UART_PORT_1);
    }
}

error_code_t uart1_init(uint32_t baud)
{
    // Initialize buffer management
    error_code_t err = uart_buffer_init(&uart1_buffer, uart1_tx_data, uart1_rx_data, UART1_BUFFER_SIZE);
    if (err != ERR_SUCCESS) {
        return err;
    }

    // Reset status
    memset(&uart1_status, 0, sizeof(uart1_status));

    // Initialize UART hardware
    err = uart_hal_init(UART_PORT_1, baud, 8, 1, UART_PARITY_NONE);
    if (err != ERR_SUCCESS) {
        return err;
    }

    // Set up callbacks
    uart_hal_set_callbacks(UART_PORT_1, 
                          uart1_rx_handler,
                          uart1_tx_handler,
                          uart1_error_handler,
                          NULL);  // No TX complete callback needed

    return ERR_SUCCESS;
}

error_code_t uart1_deinit(void) {
    return uart_avr_deinit_port(UART_PORT_1);
}

int uart1_getchar(uint8_t *c)
{
    if (!c) {
        return -1;
    }

    uint8_t byte;
    error_code_t err = ring_buffer_pop(&uart1_buffer.rx_buffer, &byte);
    if (err != ERR_SUCCESS) {
        return -1;
    }

    *c = byte;
    return 0;
}

int uart1_putchar(uint8_t c)
{
    error_code_t err = uart_buffer_add_tx_byte(&uart1_buffer, c);
    if (err != ERR_SUCCESS) {
        return -1;
    }

    // Start transmission if not already busy
    if (!uart1_buffer.tx_busy) {
        uart1_tx_handler();
    }

    return 0;
}

void uart1_set_callbacks(void (*error_cb)(uint16_t), void (*rx_cb)(uint8_t), void (*tx_cb)(void)) {
    uart_hal_set_callbacks(UART_PORT_1, rx_cb, tx_cb, error_cb, NULL);
}

bool uart1_is_rx_ready(void) {
    return uart_avr_is_rx_ready(UART_PORT_1);
}

bool uart1_is_tx_ready(void) {
    return uart_avr_is_tx_ready(UART_PORT_1);
}

uart_status_t uart1_get_status(void) {
    return uart1_status;
}
