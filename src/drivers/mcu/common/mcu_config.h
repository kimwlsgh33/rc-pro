/**
 * @file mcu_config.h
 * @brief MCU configuration definitions
 */

#ifndef MCU_CONFIG_H
#define MCU_CONFIG_H

#include "mcu_types.h"
#include "../../../utils/common/error_codes.h"
#include "../../../config/cfg_hardware.h"
#include "../timer/drv_timer.h"

/**
 * @brief MCU UART basic configuration structure
 */
typedef struct {
    uint32_t baudrate;         // Basic baud rate setting
    uint8_t data_bits;         // Basic data bits setting
    uint8_t stop_bits;         // Basic stop bits setting
    uint8_t parity;           // Basic parity setting
    mcu_pin_config_t tx_pin;  // Hardware pin configuration
    mcu_pin_config_t rx_pin;  // Hardware pin configuration
    mcu_irq_priority_t irq_priority;  // Hardware interrupt priority
} mcu_uart_config_t;

/**
 * @brief Timer configuration structure
 */
typedef timer_config_t mcu_timer_config_t;

/**
 * @brief MCU peripheral configuration structure
 */
typedef struct {
    union {
        mcu_uart_config_t uart;
        mcu_timer_config_t timer;
    };
} mcu_peripheral_config_t;

/**
 * @brief Initialize MCU with system configuration
 * 
 * @param config System configuration
 * @return error_code_t Error code
 */
error_code_t mcu_init(const mcu_system_config_t* config);

/**
 * @brief Configure MCU peripheral
 * 
 * @param peripheral Peripheral ID
 * @param config Peripheral configuration
 * @return error_code_t Error code
 */
error_code_t mcu_configure_peripheral(mcu_peripheral_id_t peripheral, const mcu_peripheral_config_t* config);

/**
 * @brief Enable MCU peripheral
 * 
 * @param peripheral Peripheral ID
 * @return error_code_t Error code
 */
error_code_t mcu_enable_peripheral(mcu_peripheral_id_t peripheral);

/**
 * @brief Disable MCU peripheral
 * 
 * @param peripheral Peripheral ID
 * @return error_code_t Error code
 */
error_code_t mcu_disable_peripheral(mcu_peripheral_id_t peripheral);

/**
 * @brief Set MCU power mode
 * 
 * @param mode Power mode
 * @return error_code_t Error code
 */
error_code_t mcu_set_power_mode(mcu_power_mode_t mode);

#endif // MCU_CONFIG_H
