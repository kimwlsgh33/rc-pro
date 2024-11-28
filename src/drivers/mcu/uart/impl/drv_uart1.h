#ifndef DRV_UART1_H
#define DRV_UART1_H

#include "../core/drv_uart.h"
#include "../common/drv_uart_types.h"

// UART1 buffer size
#define UART1_BUFFER_SIZE 128

/**
 * @brief Initialize UART1 with configuration
 *
 * @param config UART configuration structure
 * @return error_code_t Error code
 */
error_code_t uart1_init(uint32_t baud);

/**
 * @brief Deinitialize UART1
 *
 * @return error_code_t Error code
 */
error_code_t uart1_deinit(void);

/**
 * @brief Get UART1 driver interface
 *
 * @return const uart_driver_t* Pointer to UART1 driver interface
 */
const uart_driver_t *uart1_get_driver(void);

#endif // DRV_UART1_H
