#ifndef DRV_MOTOR_A_H
#define DRV_MOTOR_A_H

#include "../../config/cfg_system.h"
#include "../../platform/common/plat_types.h"
#include "../../utils/common/error_codes.h"
#include "common/drv_motor_common.h"

// Motor states (using chars for backward compatibility)
#define MA_ST_UNKNOWN     'U'
#define MA_ST_READY       'R'
#define MA_ST_ERROR       'E'
#define MA_ST_INITIALIZING 'I'
#define MA_ST_MOVING      'M'

// Motor control structure
typedef struct {
    char state;              // Current state (using char for backward compatibility)
    int32_t position;        // Current position
    int32_t target_position; // Target position
    uint16_t current_speed;  // Current speed
    uint16_t target_speed;   // Target speed
    uint8_t direction;       // Current direction
    uint8_t error_flags;     // Error flags
    int init_step;          // Initialization step counter
} motor_control_t;

// Function Declarations
error_code_t ma_init(void);
error_code_t ma_init_position(void);
error_code_t ma_move_to(int32_t position);
error_code_t ma_set_speed(uint16_t speed);
error_code_t ma_stop(void);
error_code_t ma_emergency_stop(void);

// Status functions
char ma_get_state(void);
int32_t ma_get_position(void);
uint16_t ma_get_speed(void);
bool ma_is_busy(void);
uint8_t ma_get_error_flags(void);

// Configuration
error_code_t ma_set_config(const motor_config_t* config);
const motor_config_t* ma_get_config(void);

// Process function (called periodically)
void ma_process(void);

// Debug
void ma_print_state(void);

#endif // DRV_MOTOR_A_H
