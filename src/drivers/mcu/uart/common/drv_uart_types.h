#pragma once

#include "../../../../utils/common/error_codes.h"
#include "../../../common/drv_interface.h"
#include "../../../common/drv_types.h"
#include <stddef.h>
#include <stdint.h>

/**
 * @brief UART port enumeration
 */
typedef enum { UART_PORT_1 = 0, UART_PORT_2, UART_PORT_3, UART_PORT_MAX } uart_port_t;

/**
 * @brief UART hardware registers structure
 */
typedef struct {
  volatile uint8_t *UDRn;   // Data register
  volatile uint8_t *UCSRnA; // Control and Status Register A
  volatile uint8_t *UCSRnB; // Control and Status Register B
  volatile uint8_t *UCSRnC; // Control and Status Register C
  volatile uint8_t *UBRRnH; // Baud Rate Register High
  volatile uint8_t *UBRRnL; // Baud Rate Register Low
  uint8_t RXCIEn;           // RX Complete Interrupt Enable bit
  uint8_t TXCIEn;           // TX Complete Interrupt Enable bit
  uint8_t UDRIEn;           // USART Data Register Empty Interrupt Enable bit
} uart_registers_t;

/**
 * @brief UART configuration structure
 */
typedef struct {
  driver_config_t driver; // Generic driver configuration
  uint32_t baud_rate;     // UART specific configs below
  uint8_t data_bits;
  uint8_t stop_bits;
  uint8_t parity;
  uint8_t flow_control;
  uint16_t rx_buffer_size;
  uint16_t tx_buffer_size;
  bool (*error_callback)(error_code_t error);
} uart_config_t;

/**
 * @brief UART buffer management structure
 */
typedef struct {
  uint8_t *data;
  uint16_t size;
  uint16_t head;
  uint16_t tail;
  uint16_t count;
} uart_buffer_t;

/**
 * @brief UART status structure
 */
typedef struct {
  driver_state_t state;    // Added state member
  uint16_t error_count;    // Total error count
  uint16_t framing_errors; // Framing error count
  uint16_t overflow_count; // Buffer overflow count
  uint16_t parity_errors;  // Parity error count
  error_code_t last_error; // Last encountered error
  uint16_t rx_count;       // Received bytes count
  uint16_t tx_count;       // Transmitted bytes count
} uart_status_t;

/**
 * @brief UART driver structure
 */
typedef struct {
  driver_interface_t driver; // Common driver interface

  // UART-specific function pointers
  error_code_t (*write)(const uint8_t *data, uint16_t size);
  error_code_t (*read)(uint8_t *data, uint16_t size, uint16_t *bytes_read);
  error_code_t (*write_byte)(uint8_t byte);
  error_code_t (*read_byte)(uint8_t *byte);
  error_code_t (*flush_rx)(void);
  error_code_t (*flush_tx)(void);
  uint16_t (*rx_available)(void);
  uint16_t (*tx_space)(void);
  uart_status_t (*get_status)(void);
} uart_driver_t;
