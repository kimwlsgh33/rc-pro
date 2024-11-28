#ifndef DRV_MOTOR_B_H
#define DRV_MOTOR_B_H

#include "../../../config/cfg_system.h"
#include "../../../utils/common/error_codes.h"
#include "../common/drv_motor_common.h"

// Door-specific configuration (extends base config)
typedef struct {
    motor_config_t base;           // Base motor configuration
    uint16_t close_speed;         // Speed when closing
    uint16_t open_speed;          // Speed when opening
    uint16_t slow_zone;           // Distance from limits to slow down
    uint16_t obstacle_threshold;  // Obstacle detection threshold
    bool auto_close_enable;      // Enable auto-close feature
    uint32_t auto_close_delay;   // Auto-close delay in ms
} door_config_t;

// Door-specific data
typedef struct {
    door_state_t door_state;     // Current door state
    bool obstacle_detected;      // Obstacle detection status
    uint32_t open_count;        // Number of open operations
    uint32_t close_count;       // Number of close operations
    uint32_t obstacle_count;    // Number of obstacle detections
    uint32_t auto_close_timer;  // Auto-close countdown
    door_config_t config;       // Door configuration
} door_specific_t;

// Door driver creation/destruction
error_code_t door_create(uint8_t id, const door_config_t* config, motor_driver_t** driver);
error_code_t door_destroy(motor_driver_t* driver);

// Door-specific operations (beyond base interface)
error_code_t door_open(motor_driver_t* driver);
error_code_t door_close(motor_driver_t* driver);
error_code_t door_set_auto_close(motor_driver_t* driver, bool enable, uint32_t delay);
error_code_t door_clear_obstacle(motor_driver_t* driver);

// Door-specific getters
door_state_t door_get_state(const motor_driver_t* driver);
bool door_is_obstacle_detected(const motor_driver_t* driver);
uint32_t door_get_open_count(const motor_driver_t* driver);
uint32_t door_get_close_count(const motor_driver_t* driver);
uint32_t door_get_obstacle_count(const motor_driver_t* driver);

#endif // DRV_MOTOR_B_H
