#ifndef DRV_MOTOR_B_H
#define DRV_MOTOR_B_H

#include "../../config/cfg_system.h"
#include "../../platform/common/plat_types.h"
#include "../../utils/common/error_codes.h"
#include "common/drv_motor_common.h"

// Motor states (using chars for backward compatibility)
#define GD_ST_UNKNOWN     'U'
#define GD_ST_READY       'R'
#define GD_ST_ERROR       'E'
#define GD_ST_INITIALIZING 'I'
#define GD_ST_MOVING      'M'

// Door states
#define DOOR_STATE_CLOSED     'C'
#define DOOR_STATE_CLOSING    'c'
#define DOOR_STATE_OPEN       'O'
#define DOOR_STATE_OPENING    'o'

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
error_code_t gd_init(void);
error_code_t gd_init_position(void);
error_code_t gd_move_to(int32_t position);
error_code_t gd_open(void);
error_code_t gd_open2(void);
error_code_t gd_close(void);
error_code_t gd_stop(void);

// Status functions
char gd_get_state(void);
char gd_get_door_state(void);
int32_t gd_get_position(void);
bool gd_is_busy(void);
bool gd_is_open(void);
bool gd_is_closed(void);

// Process function (called periodically)
void gd_process(void);

// Debug
void gd_print_state(void);

#endif // DRV_MOTOR_B_H
