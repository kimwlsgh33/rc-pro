#ifndef DRV_UART2_H
#define DRV_UART2_H

#include "../core/drv_uart.h"
#include "../common/drv_uart_types.h"

// UART2 buffer size
#define UART2_BUFFER_SIZE 128

/**
 * @brief Initialize UART2 with configuration
 *
 * @param baud UART baud rate
 * @return error_code_t Error code
 */
error_code_t uart2_init(uint32_t baud);

/**
 * @brief Deinitialize UART2
 *
 * @return error_code_t Error code
 */
error_code_t uart2_deinit(void);

/**
 * @brief Get UART2 driver interface
 *
 * @return const uart_driver_t* Pointer to UART2 driver interface
 */
const uart_driver_t *uart2_get_driver(void);

#endif // DRV_UART2_H
