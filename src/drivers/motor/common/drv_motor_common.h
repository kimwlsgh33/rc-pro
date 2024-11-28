#ifndef DRV_MOTOR_COMMON_H
#define DRV_MOTOR_COMMON_H

/**
 * @file drv_motor_common.h
 * @brief Common definitions and types for the motor driver
 *
 * This file contains common types, enums, and definitions that are shared
 * across all layers of the motor driver (HAL, Core, and Implementations).
 */

#include <stdint.h>
#include <stdbool.h>
#include "../../../utils/common/error_codes.h"

/*******************************************************************************
 * Type Definitions
 ******************************************************************************/

// Motor state enumeration
typedef enum {
  MOTOR_STATE_UNKNOWN = 0,
  MOTOR_STATE_READY,
  MOTOR_STATE_ERROR,
  MOTOR_STATE_INITIALIZING,
  MOTOR_STATE_MOVING,
  MOTOR_STATE_STOPPED,
  MOTOR_STATE_STALLED
} motor_state_t;

// Door state enumeration
typedef enum {
    DOOR_STATE_UNKNOWN = 0,
    DOOR_STATE_CLOSED,
    DOOR_STATE_OPEN,
    DOOR_STATE_PARTIALLY_OPEN,
    DOOR_STATE_OPENING,
    DOOR_STATE_CLOSING,
    DOOR_STATE_STOPPED
} door_state_t;

// Motor direction
typedef enum { MOTOR_DIR_CW = 0, MOTOR_DIR_CCW = 1 } motor_direction_t;

// Motor error flags
typedef enum {
  MOTOR_ERROR_NONE        = 0x00,
  MOTOR_ERROR_STALL       = 0x01,
  MOTOR_ERROR_OVERCURRENT = 0x02,
  MOTOR_ERROR_OVERTEMP    = 0x04,
  MOTOR_ERROR_POSITION    = 0x08,
  MOTOR_ERROR_LIMIT       = 0x10,
  MOTOR_ERROR_COMM        = 0x20,
  MOTOR_ERROR_CONFIG      = 0x40,
  MOTOR_ERROR_SYSTEM      = 0x80,
  MOTOR_ERROR_OBSTACLE    = 0x100  /* Obstacle detected */
} motor_error_t;

// Base motor configuration
typedef struct {
  uint16_t steps_per_rev;   // Steps per revolution
  uint16_t max_speed;       // Maximum speed in steps/sec
  uint16_t min_speed;       // Minimum speed in steps/sec
  uint16_t acceleration;    // Acceleration in steps/sec^2
  uint16_t deceleration;    // Deceleration in steps/sec^2
  uint8_t micro_stepping;   // Microstepping mode (1,2,4,8,16,32)
  uint8_t invert_direction; // Invert motor direction
  uint16_t current_limit;   // Current limit in mA
  uint16_t stall_threshold; // Stall detection threshold
  int32_t min_position;    // Minimum allowed position
  int32_t max_position;    // Maximum allowed position
} motor_config_t;

// Base motor status
typedef struct {
  motor_state_t state;         // Current motor state
  door_state_t door_state;     // Current door state
  int32_t position;            // Current position
  int32_t target_position;     // Target position
  uint16_t current_speed;      // Current speed
  uint16_t target_speed;       // Target speed
  motor_direction_t direction; // Current direction
  motor_error_t error;         // Current error state
  bool is_initialized;         // Initialization status
  bool is_enabled;             // Enable status
} motor_status_t;

// Motor driver instance
typedef struct motor_driver motor_driver_t;

// Motor interface function types
typedef error_code_t (*motor_init_fn)(motor_driver_t *driver);
typedef error_code_t (*motor_deinit_fn)(motor_driver_t *driver);
typedef error_code_t (*motor_enable_fn)(motor_driver_t *driver);
typedef error_code_t (*motor_disable_fn)(motor_driver_t *driver);
typedef error_code_t (*motor_move_to_fn)(motor_driver_t *driver, int32_t position);
typedef error_code_t (*motor_set_speed_fn)(motor_driver_t *driver, uint16_t speed);
typedef error_code_t (*motor_stop_fn)(motor_driver_t *driver);
typedef error_code_t (*motor_emergency_stop_fn)(motor_driver_t *driver);
typedef error_code_t (*motor_reset_error_fn)(motor_driver_t *driver);
typedef void (*motor_process_fn)(motor_driver_t *driver);

// Motor interface
typedef struct {
  motor_init_fn init;
  motor_deinit_fn deinit;
  motor_enable_fn enable;
  motor_disable_fn disable;
  motor_move_to_fn move_to;
  motor_set_speed_fn set_speed;
  motor_stop_fn stop;
  motor_emergency_stop_fn emergency_stop;
  motor_reset_error_fn reset_error;
  motor_process_fn process;
} motor_interface_t;

// Motor driver structure
struct motor_driver {
  uint8_t id;                      // Unique motor ID
  motor_config_t config;           // Motor configuration
  motor_status_t status;           // Motor status
  const motor_interface_t *vtable; // Virtual function table
  void *specific_data;             // Driver-specific data
};

// Common motor constants
#define MOTOR_MAX_SPEED       10000 // Maximum speed in steps/sec
#define MOTOR_MIN_SPEED       100   // Minimum speed in steps/sec
#define MOTOR_MAX_ACCEL       1000  // Maximum acceleration in steps/sec^2
#define MOTOR_MAX_CURRENT     2000  // Maximum current in mA
#define MOTOR_STALL_THRESHOLD 100   // Default stall threshold

// Common motor utility functions
static inline uint32_t motor_steps_to_ticks(uint32_t steps, uint32_t freq)
{
  return (freq * steps) / MOTOR_MAX_SPEED;
}

// Common motor helper functions
static inline bool motor_is_busy(const motor_driver_t *driver)
{
  return driver->status.state == MOTOR_STATE_MOVING ||
         driver->status.state == MOTOR_STATE_INITIALIZING;
}

static inline bool motor_has_error(const motor_driver_t *driver)
{
  return driver->status.error != MOTOR_ERROR_NONE;
}

/**
 * @brief Status codes for motor driver operations
 */
typedef enum {
    DRV_STATUS_OK = 0,            /**< Operation completed successfully */
    DRV_STATUS_ERROR,             /**< Generic error occurred */
    DRV_STATUS_INVALID_PARAM,     /**< Invalid parameter provided */
    DRV_STATUS_INVALID_CONFIG,    /**< Invalid configuration */
    DRV_STATUS_HAL_ERROR,         /**< Hardware abstraction layer error */
    DRV_STATUS_NOT_INITIALIZED,   /**< Driver not initialized */
    DRV_STATUS_BUSY              /**< Driver is busy */
} drv_status_t;

/* Common motor direction definitions */
typedef enum {
    DRV_MOTOR_DIR_FORWARD = 0,
    DRV_MOTOR_DIR_REVERSE = 1
} drv_motor_direction_t;

/* Common motor state definitions */
typedef enum {
    DRV_MOTOR_STATE_STOPPED = 0,
    DRV_MOTOR_STATE_RUNNING,
    DRV_MOTOR_STATE_ERROR
} drv_motor_state_t;

/**
 * @brief Maximum allowed motor speed
 * 
 * This value represents the maximum allowable speed setting for any motor.
 * The actual RPM depends on the specific motor implementation.
 */
#define DRV_MOTOR_MAX_SPEED   UINT16_MAX

/**
 * @brief Minimum allowed motor speed
 * 
 * This value represents the minimum speed setting for any motor.
 * Usually zero, but may be higher for some motor types.
 */
#define DRV_MOTOR_MIN_SPEED   0

#endif /* DRV_MOTOR_COMMON_H */
