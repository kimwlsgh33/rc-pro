#ifndef DRV_UART_PRIVATE_H
#define DRV_UART_PRIVATE_H

#include "../core/drv_uart.h"
#include "../common/drv_uart_types.h"
#include "../../../../utils/common/ring_buffer.h"
#include <stdbool.h>

/**
 * @file drv_uart_private.h
 * @brief UART Driver Private Definitions
 *
 * This file contains private definitions and structures used by the UART driver.
 * It defines the internal state management and hardware access structures that
 * are not exposed to the public interface.
 *
 * The UART driver uses an instance-based approach where each UART port has its
 * own configuration, status, and buffer management. This allows for independent
 * operation of multiple UART ports.
 */

/**
 * @brief UART instance data structure
 *
 * Contains all state information for a single UART port:
 * - Configuration: Baud rate, frame format, etc.
 * - Status: Current operational status
 * - Driver Interface: Function pointers for operations
 * - Buffers: Separate RX and TX ring buffers
 * - Buffer Data: Actual memory for buffers
 * - State Flags: Track busy and initialization state
 *
 * This structure is instantiated for each UART port in use.
 */
typedef struct {
    uart_config_t config;           // Configuration parameters
    uart_status_t status;          // Current status information
    uart_driver_t driver;          // Driver interface functions
    ring_buffer_t rx_buffer;       // Receive ring buffer
    ring_buffer_t tx_buffer;       // Transmit ring buffer
    uint8_t *rx_buffer_data;       // Receive buffer memory
    uint8_t *tx_buffer_data;       // Transmit buffer memory
    volatile bool tx_busy;         // Transmit busy flag
    volatile bool rx_busy;         // Receive busy flag
    bool initialized;              // Initialization flag
} uart_instance_t;

/**
 * @brief Array of UART instances
 *
 * Global array containing instance data for all UART ports.
 * Index corresponds to uart_port_t enumeration values.
 */
extern uart_instance_t uart_instances[UART_PORT_MAX];

/**
 * @brief Get hardware registers for a UART port
 *
 * Retrieves the hardware register structure for the specified UART port.
 * This function is used by the HAL layer to access the correct hardware
 * registers for each port.
 *
 * @param port UART port to get registers for
 * @return Pointer to register structure, or NULL if port is invalid
 */
const uart_registers_t* uart_get_registers(uart_port_t port);

#endif // DRV_UART_PRIVATE_H
