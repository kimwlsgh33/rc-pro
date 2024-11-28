#include "drv_motor.h"
#include "drv_motor_config.h"

static drv_motor_config_t current_config;

drv_status_t drv_motor_init(const drv_motor_config_t* config) {
    if (!config) {
        return DRV_STATUS_INVALID_PARAM;
    }

    // Validate configuration
    if (drv_motor_validate_config(config) != DRV_STATUS_OK) {
        return DRV_STATUS_INVALID_CONFIG;
    }

    // Store configuration
    current_config = *config;

    // Initialize hardware
    drv_motor_hal_init(&config->pin_config);
    drv_motor_hal_configure_timer();
    
    return DRV_STATUS_OK;
}

drv_status_t drv_motor_set_speed(int16_t speed) {
    if (speed > current_config.max_speed || speed < -current_config.max_speed) {
        return DRV_STATUS_INVALID_PARAM;
    }

    // Set direction based on speed sign
    drv_motor_hal_set_direction(speed >= 0);
    
    // Set PWM value (absolute speed)
    drv_motor_hal_set_pwm(speed >= 0 ? speed : -speed);

    return DRV_STATUS_OK;
}

drv_status_t drv_motor_enable(bool enable) {
    drv_motor_hal_enable(enable);
    return DRV_STATUS_OK;
}

drv_status_t drv_motor_brake(void) {
    drv_motor_hal_set_pwm(0);
    return DRV_STATUS_OK;
}
