#ifndef DRV_MOTOR_H
#define DRV_MOTOR_H

#include <stdint.h>
#include "../common/drv_motor_common.h"
#include "../hal/drv_motor_hal.h"

/* Motor configuration */
typedef struct {
    drv_motor_pin_config_t pin_config;
    uint16_t max_speed;
    uint8_t motor_id;
} drv_motor_config_t;

/* Motor control functions */
drv_status_t drv_motor_init(const drv_motor_config_t* config);
drv_status_t drv_motor_set_speed(int16_t speed);
drv_status_t drv_motor_enable(bool enable);
drv_status_t drv_motor_brake(void);

#endif /* DRV_MOTOR_H */
