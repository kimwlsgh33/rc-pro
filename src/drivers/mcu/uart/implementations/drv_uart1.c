/**
 * @file drv_uart1.c
 * @brief UART1 Driver Implementation using HAL Interface
 *
 * This file implements the UART1 driver using the UART HAL interface.
 * It provides buffered, interrupt-driven UART communication with
 * error handling and status tracking.
 */
#include "drv_uart1.h"
#include "../common/drv_uart_types.h"
#include "../hal/drv_uart_hal.h"
#include "../../../../config/cfg_build.h"
#include <string.h>

// UART1 status structure
typedef struct {
    uint16_t error_count;
    uint16_t framing_errors;
    uint16_t overflow_count;
    uint16_t parity_errors;
    uint16_t rx_count;
    uint16_t tx_count;
    uint16_t last_error;
} uart1_status_t;

// UART1 status
static uart1_status_t uart1_status;

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
}

static void uart1_tx_handler(void) {
    uart1_status.tx_count++;
}

error_code_t uart1_init(uint32_t baud) {
    // Initialize UART1 with 8N1 configuration
    error_code_t result = uart_hal_init(UART_PORT_1, baud, 8, 1, 0);
    if (result != ERR_SUCCESS) {
        return result;
    }

    // Clear status
    memset(&uart1_status, 0, sizeof(uart1_status_t));

    // Set up HAL callbacks
    uart_hal_set_callbacks(UART_PORT_1, uart1_rx_handler, uart1_tx_handler, uart1_error_handler);

    return ERR_SUCCESS;
}

error_code_t uart1_deinit(void) {
    // Deinitialize UART1 through HAL
    error_code_t result = uart_hal_deinit(UART_PORT_1);
    if (result != ERR_SUCCESS) {
        return result;
    }

    // Clear status
    memset(&uart1_status, 0, sizeof(uart1_status_t));

    return ERR_SUCCESS;
}

int uart1_getchar(uint8_t *c) {
    if (!c) {
        return 0;
    }

    error_code_t result = uart_hal_read_byte(UART_PORT_1, c);
    return (result == ERR_SUCCESS) ? 1 : 0;
}

int uart1_putchar(uint8_t c) {
    error_code_t result = uart_hal_write_byte(UART_PORT_1, c);
    return (result == ERR_SUCCESS) ? 1 : 0;
}

void uart1_set_callbacks(void (*error_cb)(uint16_t), void (*rx_cb)(uint8_t), void (*tx_cb)(void)) {
    uart_hal_set_callbacks(UART_PORT_1, rx_cb, tx_cb, error_cb);
}

bool uart1_is_rx_ready(void) {
    uint8_t dummy;
    return uart_hal_read_byte(UART_PORT_1, &dummy) == ERR_SUCCESS;
}

bool uart1_is_tx_ready(void) {
    return uart_hal_write_byte(UART_PORT_1, 0) == ERR_SUCCESS;
}

uart1_status_t uart1_get_status(void) {
    return uart1_status;
}
