#ifndef DRV_MOTOR_HAL_H
#define DRV_MOTOR_HAL_H

#include <stdint.h>
#include "../common/drv_motor_common.h"

/**
 * @brief Hardware abstraction layer for motor drivers
 */

/* Pin configuration structure */
typedef struct {
    uint8_t pwm_pin;
    uint8_t direction_pin;
    uint8_t enable_pin;
} drv_motor_pin_config_t;

/* HAL interface functions */
void drv_motor_hal_init(const drv_motor_pin_config_t* config);
void drv_motor_hal_set_pwm(uint8_t pwm_value);
void drv_motor_hal_set_direction(uint8_t direction);
void drv_motor_hal_enable(bool enable);
void drv_motor_hal_configure_timer(void);

#endif /* DRV_MOTOR_HAL_H */
