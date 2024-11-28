#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "../../utils/common/error_codes.h"
#include <stdint.h>

// Motor control states
typedef enum {
  MOTOR_STATE_IDLE,
  MOTOR_STATE_ACCELERATING,
  MOTOR_STATE_RUNNING,
  MOTOR_STATE_DECELERATING,
  MOTOR_STATE_ERROR
} motor_state_t;

// Motor parameters
typedef struct {
  uint16_t current_speed;
  uint16_t target_speed;
  uint16_t acceleration;
  uint16_t load_factor;
  uint8_t temperature;
  motor_state_t state;
} motor_params_t;

// Motor optimization settings
typedef struct {
  uint16_t min_speed;
  uint16_t max_speed;
  uint16_t optimal_acceleration;
  uint16_t temperature_threshold;
  uint16_t load_threshold;
} motor_config_t;

// Motor timeout definitions
#define MOTOR_TIMEOUT_MS       5000  // 5 seconds for general operation timeout
#define MOTOR_STOP_TIMEOUT_MS  2000  // 2 seconds for stop operation
#define MOTOR_MOVE_TIMEOUT_MS  10000 // 10 seconds for movement operation
#define MOTOR_STALL_TIMEOUT_MS 1000  // 1 second for stall detection

// Motor error codes
#define ERR_MOTOR_TIMEOUT      0x01
#define ERR_MOTOR_STOP_TIMEOUT 0x02
#define ERR_MOTOR_MOVE_TIMEOUT 0x04
#define ERR_MOTOR_STALL        0x08

// Motor control structure extension
typedef struct {
  uint32_t start_time;       // Start time of current operation
  uint32_t stop_start_time;  // Start time of stop operation
  uint32_t move_start_time;  // Start time of move operation
  uint32_t last_position;    // Last recorded position
  uint32_t stall_start_time; // Time when potential stall was first detected
} motor_timing_t;

// Function declarations
void mc_init(void);
error_code_t mc_configure_motor(uint8_t motor_id, const motor_config_t *config);
error_code_t mc_adjust_speed(uint8_t motor_id, uint16_t target_speed);
error_code_t mc_optimize_performance(uint8_t motor_id);
motor_params_t mc_get_motor_status(uint8_t motor_id);
void mc_process(void);

#endif // MOTOR_CONTROL_H
