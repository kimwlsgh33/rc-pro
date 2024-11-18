#ifndef DRV_MOTOR_COMMON_H
#define DRV_MOTOR_COMMON_H

#include "../../../config/cfg_system.h"
#include "../../../platform/common/plat_types.h"
#include "../../../utils/common/error_codes.h"

// Common motor configuration structure
typedef struct {
    uint16_t steps_per_rev;     // Steps per revolution
    uint16_t max_speed;         // Maximum speed in steps/sec
    uint16_t min_speed;         // Minimum speed in steps/sec
    uint16_t acceleration;      // Acceleration in steps/sec^2
    uint16_t deceleration;      // Deceleration in steps/sec^2
    uint8_t micro_stepping;     // Microstepping mode (1,2,4,8,16,32)
    uint8_t invert_direction;   // Invert motor direction
    uint16_t current_limit;     // Current limit in mA
    uint16_t stall_threshold;   // Stall detection threshold
} motor_config_t;

// Common motor error flags
typedef enum {
    MOTOR_ERROR_NONE = 0x00,
    MOTOR_ERROR_STALL = 0x01,
    MOTOR_ERROR_OVERCURRENT = 0x02,
    MOTOR_ERROR_OVERTEMP = 0x04,
    MOTOR_ERROR_POSITION = 0x08,
    MOTOR_ERROR_LIMIT = 0x10,
    MOTOR_ERROR_COMM = 0x20,
    MOTOR_ERROR_CONFIG = 0x40,
    MOTOR_ERROR_SYSTEM = 0x80
} motor_error_t;

// Common motor direction
typedef enum {
    MOTOR_DIR_CW = 0,
    MOTOR_DIR_CCW = 1
} motor_direction_t;

// Common motor constants
#define MOTOR_MAX_SPEED 10000       // Maximum speed in steps/sec
#define MOTOR_MIN_SPEED 100         // Minimum speed in steps/sec
#define MOTOR_MAX_ACCEL 1000        // Maximum acceleration in steps/sec^2
#define MOTOR_MAX_CURRENT 2000      // Maximum current in mA
#define MOTOR_STALL_THRESHOLD 100   // Default stall threshold

// Common motor utility functions
static inline uint32_t motor_steps_to_ticks(uint32_t steps, uint32_t freq) {
    return (freq * steps) / MOTOR_MAX_SPEED;
}

static inline uint32_t motor_ticks_to_steps(uint32_t ticks, uint32_t freq) {
    return (ticks * MOTOR_MAX_SPEED) / freq;
}

#endif // DRV_MOTOR_COMMON_H
