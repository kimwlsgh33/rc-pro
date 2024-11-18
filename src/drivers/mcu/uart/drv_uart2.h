#ifndef DRV_UART2_H
#define DRV_UART2_H

#include "../../../config/cfg_system.h"
#include "../../../config/cfg_hardware.h"
#include "../../../platform/common/plat_types.h"
#include "../../../utils/common/error_codes.h"
#include "drv_uart.h"

// Buffer size configuration - Aligned with other UARTs
#define UART2_BUFFER_SIZE 128

/**
 * @brief Initialize UART2 with configuration
 * 
 * @param config UART configuration structure
 * @return error_code_t Error code
 */
error_code_t uart2_init(const uart_config_t* config);

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
const uart_driver_t* uart2_get_driver(void);

// Callback setters
void uart2_set_callbacks(void (*error_cb)(uint8_t),
                        void (*rx_cb)(uint8_t),
                        void (*tx_cb)(void));

#endif // DRV_UART2_H
