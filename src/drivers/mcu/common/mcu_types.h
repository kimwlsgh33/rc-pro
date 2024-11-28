/**
 * @file mcu_types.h
 * @brief Common MCU type definitions
 */

#ifndef MCU_TYPES_H
#define MCU_TYPES_H

#include <stdint.h>

/**
 * @brief MCU clock sources
 */
typedef enum {
    MCU_CLOCK_INTERNAL = 0,   // Internal RC oscillator
    MCU_CLOCK_EXTERNAL,       // External crystal/clock
    MCU_CLOCK_PLL            // PLL derived clock
} mcu_clock_source_t;

/**
 * @brief MCU power modes
 */
typedef enum {
    MCU_POWER_RUN = 0,       // Normal run mode
    MCU_POWER_SLEEP,         // Sleep mode
    MCU_POWER_DEEP_SLEEP,    // Deep sleep mode
    MCU_POWER_STANDBY        // Standby mode
} mcu_power_mode_t;

/**
 * @brief MCU peripheral IDs
 */
typedef enum {
    MCU_PERIPHERAL_UART0 = 0,
    MCU_PERIPHERAL_UART1,
    MCU_PERIPHERAL_UART2,
    MCU_PERIPHERAL_UART3,
    MCU_PERIPHERAL_TIMER0,
    MCU_PERIPHERAL_TIMER1,
    MCU_PERIPHERAL_TIMER2,
    MCU_PERIPHERAL_TIMER3,
    MCU_PERIPHERAL_ADC,
    MCU_PERIPHERAL_SPI,
    MCU_PERIPHERAL_I2C,
    MCU_PERIPHERAL_COUNT
} mcu_peripheral_id_t;

/**
 * @brief MCU peripheral states
 */
typedef enum {
    MCU_PERIPHERAL_DISABLED = 0,
    MCU_PERIPHERAL_ENABLED
} mcu_peripheral_state_t;

/**
 * @brief MCU interrupt priority levels
 */
typedef enum {
    MCU_IRQ_PRIORITY_0 = 0,  // Highest priority
    MCU_IRQ_PRIORITY_1,
    MCU_IRQ_PRIORITY_2,
    MCU_IRQ_PRIORITY_3,      // Lowest priority
} mcu_irq_priority_t;

/**
 * @brief MCU pin configuration
 */
typedef struct {
    uint8_t port;            // GPIO port
    uint8_t pin;             // GPIO pin number
    uint8_t mode;            // Pin mode (input/output/alternate)
    uint8_t pull;            // Pull-up/down configuration
    uint8_t alternate;       // Alternate function
} mcu_pin_config_t;

/**
 * @brief MCU system configuration
 */
typedef struct {
    mcu_clock_source_t clock_source;
    uint32_t system_clock_hz;
    uint32_t peripheral_clock_hz;
    mcu_power_mode_t power_mode;
} mcu_system_config_t;

#endif // MCU_TYPES_H
