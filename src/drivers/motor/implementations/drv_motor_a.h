#ifndef DRV_MOTOR_A_H
#define DRV_MOTOR_A_H

#include "../../../config/cfg_system.h"
#include "../../../utils/common/error_codes.h"
#include "../common/drv_motor_common.h"

// Motor A specific configuration (extends base config)
typedef struct {
    motor_config_t base;           // Base motor configuration
    uint16_t hold_current;        // Current when motor is stationary
    bool enable_soft_limits;      // Enable software position limits
    int32_t soft_limit_min;       // Minimum position limit
    int32_t soft_limit_max;       // Maximum position limit
} motor_a_config_t;

// Motor A specific data
typedef struct {
    uint32_t move_count;          // Number of moves performed
    bool soft_limit_hit;         // Software limit status
    uint16_t current_current;    // Current drive current
} motor_a_specific_t;

// Motor A driver creation/destruction
error_code_t motor_a_create(uint8_t id, const motor_a_config_t* config, motor_driver_t** driver);
error_code_t motor_a_destroy(motor_driver_t* driver);

// Motor A specific operations (beyond base interface)
error_code_t motor_a_set_soft_limits(motor_driver_t* driver, int32_t min_pos, int32_t max_pos);
error_code_t motor_a_set_hold_current(motor_driver_t* driver, uint16_t current);

// Motor A specific getters
bool motor_a_soft_limit_hit(const motor_driver_t* driver);
uint32_t motor_a_get_move_count(const motor_driver_t* driver);
uint16_t motor_a_get_current(const motor_driver_t* driver);

#endif // DRV_MOTOR_A_H
