#ifndef CTRL_STATE_B_H
#define CTRL_STATE_B_H

#include "../../config/cfg_build.h"
#include "../../platform/common/plat_types.h"
#include "../../utils/common/error_codes.h"
#include "../../core/module_interface.h"

// State Machine B States
typedef enum {
    STATE_B_IDLE,
    STATE_B_INIT,
    STATE_B_RUNNING,
    STATE_B_PAUSED,
    STATE_B_ERROR
} state_b_state_t;

// State Machine B Configuration
typedef struct {
    uint16_t timeout_ms;
    uint8_t retry_count;
    uint8_t flags;
} state_b_config_t;

// State Machine B Status
typedef struct {
    state_b_state_t state;
    uint32_t state_time_ms;
    uint32_t total_transitions;
    uint32_t error_flags;
} state_b_status_t;

// Function Declarations
error_code_t state_b_init(const state_b_config_t* config);
error_code_t state_b_start(void);
error_code_t state_b_stop(void);
error_code_t state_b_pause(void);
error_code_t state_b_resume(void);
const state_b_status_t* state_b_get_status(void);
void state_b_process(void);

#endif // CTRL_STATE_B_H
