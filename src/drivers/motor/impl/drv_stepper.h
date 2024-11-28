#ifndef DRV_STEPPER_H
#define DRV_STEPPER_H

#include "../../../config/cfg_hardware.h"
#include "../../../config/cfg_system.h"
#include "../../../platform/avr/plat_avr.h"
#include "../../../utils/common/error_codes.h"
#include "../common/drv_motor_common.h"

// Stepper timing constants
#define STEP_TIMING_WINDOW 1000 // Maximum time window (in timer ticks) between steps before considering stall
#define STALL_DETECTION_THRESHOLD 3 // Number of consecutive missed steps before declaring a stall

// Stepper-specific configuration (extends base config)
typedef struct {
  motor_config_t base;           // Base motor configuration
  bool enable_position_tracking; // Enable absolute position tracking
  bool enable_stall_detection;   // Enable stall detection
  uint16_t home_speed;           // Speed for homing operation
  uint8_t home_current;          // Current during homing
} stepper_config_t;

// Stepper-specific data
typedef struct {
  uint32_t step_count;       // Total steps since last reset
  bool is_homed;            // Homing status
  bool limit_switch_hit;     // Limit switch status
  uint8_t current_mode;      // Current microstepping mode
  uint8_t missed_step_count; // Counter for potential missed steps
  uint16_t last_step_time;   // Time of last successful step
  bool stall_detected;       // Stall detection flag
} stepper_specific_t;

// Stepper driver creation/destruction
error_code_t stepper_create(uint8_t id, const stepper_config_t *config, motor_driver_t **driver);
error_code_t stepper_destroy(motor_driver_t *driver);

// Stepper-specific operations (beyond base interface)
error_code_t stepper_home(motor_driver_t *driver);
error_code_t stepper_set_micro_stepping(motor_driver_t *driver, uint8_t micro_stepping);
error_code_t stepper_set_current(motor_driver_t *driver, uint8_t current);

// Stepper-specific getters
bool stepper_is_homed(const motor_driver_t *driver);
bool stepper_limit_switch_hit(const motor_driver_t *driver);
uint32_t stepper_get_step_count(const motor_driver_t *driver);

#endif // DRV_STEPPER_H
