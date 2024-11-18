/**
 * @file mcu_config.h
 * @brief MCU configuration definitions
 */

#ifndef MCU_CONFIG_H
#define MCU_CONFIG_H

#include "mcu_types.h"
#include "../../../config/cfg_hardware.h"

/**
 * @brief UART configuration structure
 */
typedef struct {
    uint32_t baudrate;
    uint8_t data_bits;
    uint8_t stop_bits;
    uint8_t parity;
    uint8_t flow_control;
    mcu_pin_config_t tx_pin;
    mcu_pin_config_t rx_pin;
    mcu_irq_priority_t irq_priority;
} uart_config_t;

/**
 * @brief Timer configuration structure
 */
typedef struct {
    uint32_t frequency;
    uint16_t period;
    uint8_t mode;
    uint8_t prescaler;
    mcu_irq_priority_t irq_priority;
} timer_config_t;

/**
 * @brief MCU peripheral configuration structure
 */
typedef struct {
    union {
        uart_config_t uart;
        timer_config_t timer;
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
