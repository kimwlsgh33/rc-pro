#ifndef UART_ERRORS_H
#define UART_ERRORS_H

#include <stdint.h>

// UART Error Codes
typedef enum {
    UART_ERR_NONE = 0,
    UART_ERR_OVERFLOW = -1,
    UART_ERR_FRAMING = -2,
    UART_ERR_PARITY = -3,
    UART_ERR_NOISE = -4,
    UART_ERR_BREAK = -5,
    UART_ERR_DMA = -6,
    UART_ERR_TIMEOUT = -7
} uart_error_t;

// UART Status Structure
typedef struct {
    uint32_t rx_count;         // Total bytes received
    uint32_t tx_count;         // Total bytes transmitted
    uint32_t error_count;      // Total error count
    uint32_t overflow_count;   // Buffer overflow count
    uint32_t framing_errors;   // Framing error count
    uint32_t parity_errors;    // Parity error count
    uint32_t noise_errors;     // Noise error count
    uint32_t break_count;      // Break condition count
    uart_error_t last_error;   // Last error code
    uint8_t state;            // Current driver state
} uart_status_t;

#endif // UART_ERRORS_H
