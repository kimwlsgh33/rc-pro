#ifndef DRV_UART_H
#define DRV_UART_H

#include "../../../config/cfg_system.h"
#include "../../../config/cfg_hardware.h"
#include "../../../platform/common/plat_types.h"
#include "../../../platform/avr/plat_avr.h"
#include "../../../utils/common/error_codes.h"
#include "../../../utils/common/ring_buffer.h"
#include "../../common/drv_interface.h"
#include "../../common/drv_types.h"
#include "../common/mcu_types.h"

/**
 * @brief UART specific error codes
 */
#define ERR_UART_BASE         (ERR_CAT_PERIPHERAL | 0x0100)
#define ERR_UART_OVERFLOW     (ERR_UART_BASE | 0x01)
#define ERR_UART_FRAMING      (ERR_UART_BASE | 0x02)
#define ERR_UART_PARITY       (ERR_UART_BASE | 0x03)
#define ERR_UART_NOISE        (ERR_UART_BASE | 0x04)
#define ERR_UART_BREAK        (ERR_UART_BASE | 0x05)
#define ERR_UART_INVALID_PORT (ERR_UART_BASE | 0x06)

/**
 * @brief UART events
 */
typedef enum {
    UART_EVENT_RX_READY = 0x01,
    UART_EVENT_TX_DONE = 0x02,
    UART_EVENT_ERROR = 0x04,
    UART_EVENT_BREAK = 0x08
} uart_event_t;

/**
 * @brief UART port numbers
 */
typedef enum {
    UART_PORT_1 = 0,
    UART_PORT_2,
    UART_PORT_3,
    UART_PORT_MAX
} uart_port_t;

/**
 * @brief UART configuration flags
 */
#define UART_FLAG_DMA_TX      0x01    // Enable DMA for TX
#define UART_FLAG_DMA_RX      0x02    // Enable DMA for RX
#define UART_FLAG_DMA_CIRCULAR 0x04   // Enable circular DMA mode

/**
 * @brief UART configuration structure
 */
typedef struct {
    uint32_t baud_rate;          // Baud rate in bps
    uint8_t data_bits;           // Data bits (5-9)
    uint8_t stop_bits;           // Stop bits (1-2)
    uint8_t parity;              // Parity (none, even, odd)
    uint16_t rx_buffer_size;     // RX buffer size
    uint16_t tx_buffer_size;     // TX buffer size
    bool flow_control;           // Hardware flow control
    uint8_t dma_flags;           // DMA configuration flags
    uint8_t dma_priority;        // DMA priority level
    
    // Callbacks
    void (*rx_callback)(void* data, uint32_t size);
    void (*tx_callback)(void);
    void (*error_callback)(error_code_t error);
    
    // Driver configuration
    driver_config_t driver;      // Common driver configuration
} uart_config_t;

/**
 * @brief UART status structure
 */
typedef struct {
    uint8_t state;              // Current state
    error_code_t last_error;    // Last error code
    uint32_t rx_count;          // Total bytes received
    uint32_t tx_count;          // Total bytes transmitted
    uint32_t error_count;       // Total errors
    uint32_t overflow_count;    // Buffer overflow count
} uart_status_t;

/**
 * @brief UART driver interface
 */
typedef struct {
    // Standard driver interface
    driver_interface_t driver;
    
    // UART specific functions
    error_code_t (*write)(const uint8_t* data, uint16_t size);
    error_code_t (*read)(uint8_t* data, uint16_t size, uint16_t* bytes_read);
    error_code_t (*write_byte)(uint8_t byte);
    error_code_t (*read_byte)(uint8_t* byte);
    error_code_t (*flush_rx)(void);
    error_code_t (*flush_tx)(void);
    uint16_t (*rx_available)(void);
    uint16_t (*tx_space)(void);
    const uart_status_t* (*get_status)(void);
} uart_driver_t;

/**
 * @brief UART hardware registers
 */
typedef struct {
    volatile uint8_t *UDRn;    // Data register
    volatile uint8_t *UCSRnA;  // Control and Status Register A
    volatile uint8_t *UCSRnB;  // Control and Status Register B
    volatile uint8_t *UCSRnC;  // Control and Status Register C
    volatile uint8_t *UBRRnH;  // Baud Rate Register High
    volatile uint8_t *UBRRnL;  // Baud Rate Register Low
    uint8_t RXCIEn;           // RX Complete Interrupt Enable bit
    uint8_t TXCIEn;           // TX Complete Interrupt Enable bit
    uint8_t UDRIEn;           // USART Data Register Empty Interrupt Enable bit
} uart_registers_t;

// Function declarations
error_code_t uart_init(uart_port_t port, const uart_config_t* config);
error_code_t uart_deinit(uart_port_t port);
error_code_t uart_write(uart_port_t port, const uint8_t* data, uint16_t size);
error_code_t uart_read(uart_port_t port, uint8_t* data, uint16_t size, uint16_t* bytes_read);
error_code_t uart_write_byte(uart_port_t port, uint8_t byte);
error_code_t uart_read_byte(uart_port_t port, uint8_t* byte);
error_code_t uart_flush_rx(uart_port_t port);
error_code_t uart_flush_tx(uart_port_t port);
uint16_t uart_rx_available(uart_port_t port);
uint16_t uart_tx_space(uart_port_t port);
const uart_status_t* uart_get_status(uart_port_t port);
const uart_driver_t* uart_get_driver(uart_port_t port);

#endif // DRV_UART_H
