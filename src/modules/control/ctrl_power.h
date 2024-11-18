#ifndef CTRL_POWER_H
#define CTRL_POWER_H

#include "../../config/cfg_build.h"
#include "../../platform/common/plat_types.h"
#include "../../utils/common/error_codes.h"
#include "../../core/module_interface.h"

// Power Management States
typedef enum {
    POWER_STATE_NORMAL,
    POWER_STATE_LOW_POWER,
    POWER_STATE_SLEEP,
    POWER_STATE_SHUTDOWN,
    POWER_STATE_ERROR
} power_state_t;

// Power Configuration
typedef struct {
    uint16_t voltage_threshold_mv;
    uint16_t current_threshold_ma;
    uint16_t temp_threshold_c;
    uint8_t low_power_mode;
} power_config_t;

// Power Status
typedef struct {
    power_state_t state;
    uint16_t voltage_mv;
    uint16_t current_ma;
    uint16_t temperature_c;
    uint8_t battery_level;
    uint32_t uptime_ms;
} power_status_t;

// Function Declarations
error_code_t power_init(const power_config_t* config);
error_code_t power_set_state(power_state_t state);
error_code_t power_enter_low_power(void);
error_code_t power_exit_low_power(void);
const power_status_t* power_get_status(void);
void power_process(void);

#endif // CTRL_POWER_H