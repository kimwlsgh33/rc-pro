#ifndef UI_CONSOLE_H
#define UI_CONSOLE_H

#include <stdint.h>
#include "../../drivers/mcu/uart/drv_uart.h"

// Console configuration
#define CONSOLE_UART_PORT UART_PORT_1
#define CONSOLE_RX_BUFFER_SIZE 128
#define CONSOLE_TX_BUFFER_SIZE 128
#define MAX_CMD_LEN 128

// Function declarations
void cons_init(uint32_t baud);
void cons_enable(void);
void cons_disable(void);
int cons_get_command_line(char *poutBuff);

#endif // UI_CONSOLE_H
