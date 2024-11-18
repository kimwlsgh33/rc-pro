#ifndef DRV_UART3_H
#define DRV_UART3_H

#include "../../../config/cfg_system.h"
#include "../../../platform/common/plat_types.h"
#include "../../../utils/common/error_codes.h"
#include "../../driver_interface.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <string.h>

// UART3 Buffer Size (for ultrasonic sensor)
#define UART3_BUFFER_SIZE 128

// UART3 Error Codes
#define UART_ERR_FRAMING   0x01
#define UART_ERR_OVERFLOW  0x02
#define UART_ERR_PARITY    0x03
#define UART_ERR_CHECKSUM  0x04

// UART3 Status Structure
typedef struct {
    uint16_t error_count;      // Total error count
    uint16_t framing_errors;   // Framing error count
    uint16_t overflow_count;   // Buffer overflow count
    uint16_t parity_errors;    // Parity error count
    uint16_t rx_count;         // Received bytes count
    uint16_t tx_count;         // Transmitted bytes count
    uint8_t last_error;        // Last error code
} uart3_status_t;

// Function Declarations
error_code_t uart3_init(uint32_t baud);
error_code_t uart3_deinit(void);
int uart3_getchar(uint8_t *c);
int uart3_putchar(uint8_t c);
bool uart3_is_rx_ready(void);
bool uart3_is_tx_ready(void);

// Ultrasonic sensor specific functions
void uart3_set_ultrasonic_mode(bool enable);
bool uart3_is_ultrasonic_data_ready(void);
uint16_t uart3_get_ultrasonic_distance(void);

// Status and callback functions
const uart3_status_t* uart3_get_status(void);
void uart3_set_callbacks(void (*error_cb)(uint8_t), 
                        void (*rx_cb)(uint8_t),
                        void (*tx_cb)(void));

#endif // DRV_UART3_H
