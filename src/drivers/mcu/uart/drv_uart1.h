#ifndef DRV_UART1_H
#define DRV_UART1_H

#include "../../../config/cfg_system.h"
#include "../../../platform/common/plat_types.h"
#include "../../../utils/common/error_codes.h"
#include "../../driver_interface.h"

// UART1 Buffer Size
#define UART1_BUFFER_SIZE 128

// Function Declarations
error_code_t uart1_init(uint32_t baud);
error_code_t uart1_deinit(void);
int uart1_getchar(uint8_t *c);
int uart1_putchar(uint8_t c);
bool uart1_is_rx_ready(void);
bool uart1_is_tx_ready(void);

#endif // DRV_UART1_H
