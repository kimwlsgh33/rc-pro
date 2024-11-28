#ifndef DRV_UART3_H
#define DRV_UART3_H

#include "drv_uart_types.h"
#include "drv_uart.h"

/**
 * @brief Initialize UART3 with configuration
 * 
 * @param config UART configuration structure
 * @return error_code_t Error code
 */
error_code_t uart3_init(const uart_config_t* config);

/**
 * @brief Deinitialize UART3
 * 
 * @return error_code_t Error code
 */
error_code_t uart3_deinit(void);

/**
 * @brief Get UART3 driver interface
 * 
 * @return const uart_driver_t* Pointer to UART3 driver interface
 */
const uart_driver_t* uart3_get_driver(void);

#endif // DRV_UART3_H
