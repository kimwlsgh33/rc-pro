#ifndef DRV_STEPPER_H
#define DRV_STEPPER_H

#include "../../config/cfg_system.h"
#include "../../config/cfg_hardware.h"
#include "../../platform/common/plat_types.h"
#include "../../platform/avr/plat_avr.h"
#include "../../utils/common/error_codes.h"
#include "../driver_interface.h"
#include "common/drv_motor_common.h"

// Stepper Motor Configuration
typedef struct {
    motor_base_config_t base;
    uint16_t steps_per_rev;
    uint8_t micro_stepping;
    uint8_t hold_current;
    bool enable_position_tracking;
} stepper_config_t;

// Stepper Motor Status
typedef struct {
    motor_base_status_t base;
    uint32_t current_position;
    uint32_t target_position;
    uint16_t steps_remaining;
    bool is_homed;
    bool limit_switch_hit;
} stepper_status_t;

// Function Declarations
error_code_t stepper_init(uint8_t id, const stepper_config_t* config);
error_code_t stepper_configure(uint8_t id, const stepper_config_t* config);

// Motion Control
error_code_t stepper_move_steps(uint8_t id, int32_t steps, uint16_t speed);
error_code_t stepper_move_to_position(uint8_t id, uint32_t position, uint16_t speed);
error_code_t stepper_set_speed(uint8_t id, uint16_t speed);
error_code_t stepper_stop(uint8_t id);
error_code_t stepper_emergency_stop(uint8_t id);

// Position Control
error_code_t stepper_set_position(uint8_t id, uint32_t position);
error_code_t stepper_home(uint8_t id);
uint32_t stepper_get_position(uint8_t id);

// Status and Control
const stepper_status_t* stepper_get_status(uint8_t id);
error_code_t stepper_reset_error(uint8_t id);

// Advanced Control
error_code_t stepper_set_micro_stepping(uint8_t id, uint8_t micro_stepping);
error_code_t stepper_set_hold_current(uint8_t id, uint8_t current);
error_code_t stepper_enable_position_tracking(uint8_t id, bool enable);

#endif // DRV_STEPPER_H
