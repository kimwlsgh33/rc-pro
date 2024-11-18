#include <string.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>

#include "ui_console.h"
#include "../../utils/common/error_codes.h"

// Buffer for command processing
static char rxBuff[MAX_CMD_LEN];
static int rxLen = 0;

// UART configuration
static uart_config_t uart_config = {
    .baud_rate = 0,  // Set in cons_init
    .data_bits = 8,
    .stop_bits = 1,
    .parity = 0,
    .rx_buffer_size = CONSOLE_RX_BUFFER_SIZE,
    .tx_buffer_size = CONSOLE_TX_BUFFER_SIZE,
    .flow_control = false,
    .rx_callback = NULL,
    .tx_callback = NULL,
    .error_callback = NULL
};

// Initialize console
void cons_init(uint32_t baud) {
    // Configure UART
    uart_config.baud_rate = baud;
    error_code_t err = uart_init(CONSOLE_UART_PORT, &uart_config);
    if (err != ERROR_CODE_SUCCESS) {
        // Handle error - maybe set an error LED?
        return;
    }
    
    // Clear receive buffer
    rxLen = 0;
}

// Enable console
void cons_enable(void) {
    // Nothing to do - UART is already enabled
}

// Disable console
void cons_disable(void) {
    uart_deinit(CONSOLE_UART_PORT);
}

// Get command line from console
int cons_get_command_line(char *poutBuff) {
    if (!poutBuff) {
        return -ERR_INVALID_PARAM;
    }

    static char rxBuff[MAX_CMD_LEN];
    static uint8_t rxLen = 0;
    uint8_t c;
    
    // Check for new character
    if (uart_rx_available(CONSOLE_UART_PORT) > 0) {
        if (uart_read_byte(CONSOLE_UART_PORT, &c) == ERROR_CODE_SUCCESS) {
            // Echo character
            uart_write_byte(CONSOLE_UART_PORT, c);
            
            // Handle backspace
            if (c == '\b' || c == 0x7f) {
                if (rxLen > 0) {
                    rxLen--;
                    // Echo backspace sequence
                    uart_write_byte(CONSOLE_UART_PORT, '\b');
                    uart_write_byte(CONSOLE_UART_PORT, ' ');
                    uart_write_byte(CONSOLE_UART_PORT, '\b');
                }
                return -1;
            }
            
            // Handle enter
            if (c == '\r' || c == '\n') {
                uart_write_byte(CONSOLE_UART_PORT, '\r');
                uart_write_byte(CONSOLE_UART_PORT, '\n');
                
                // Copy command and reset buffer with bounds checking
                if (rxLen > 0) {
                    size_t copy_len = rxLen;
                    if (copy_len >= MAX_CMD_LEN) {
                        copy_len = MAX_CMD_LEN - 1;  // Ensure space for null terminator
                        error_record(ERR_BUFFER_OVERFLOW);
                    }
                    memcpy(poutBuff, rxBuff, copy_len);
                    poutBuff[copy_len] = '\0';  // Ensure null termination
                    int len = copy_len;
                    rxLen = 0;  // Reset buffer
                    memset(rxBuff, 0, MAX_CMD_LEN);  // Clear buffer for security
                    return len;
                }
                return 0;
            }
            
            // Add character to buffer if space available
            if (rxLen < MAX_CMD_LEN - 1) {  // Leave space for null terminator
                rxBuff[rxLen++] = c;
            } else {
                // Buffer full - emit bell character and record error
                uart_write_byte(CONSOLE_UART_PORT, '\a');
                error_record(ERR_BUFFER_OVERFLOW);
            }
        }
    }
    
    return -1;
}
