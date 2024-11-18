#ifndef CTRL_CLEAN_H
#define CTRL_CLEAN_H

#include "../../config/cfg_system.h"
#include "../../config/cfg_hardware.h"
#include "../../platform/common/plat_types.h"
#include "../../utils/common/error_codes.h"
#include "../../core/module/mod_manager.h"

// Cleaning cycle types
typedef enum {
    CLEAN_TYPE_QUICK,
    CLEAN_TYPE_NORMAL,
    CLEAN_TYPE_DEEP,
    CLEAN_TYPE_MAINTENANCE
} cleaning_type_t;

// Cleaning cycle phases
typedef enum {
    CLEAN_PHASE_IDLE,
    CLEAN_PHASE_PREWASH,
    CLEAN_PHASE_MAIN_WASH,
    CLEAN_PHASE_RINSE,
    CLEAN_PHASE_DRY,
    CLEAN_PHASE_COMPLETE,
    CLEAN_PHASE_ERROR
} cleaning_phase_t;

// Cleaning parameters
typedef struct {
    uint16_t water_temperature;
    uint16_t water_pressure;
    uint16_t cycle_duration;
    uint8_t detergent_level;
    uint8_t rinse_cycles;
} cleaning_params_t;

// Cleaning cycle statistics
typedef struct {
    uint32_t total_cycles;
    uint32_t water_usage_ml;
    uint32_t energy_usage_wh;
    uint16_t average_duration_s;
    uint8_t efficiency_score;
} cleaning_stats_t;

// Function declarations
void cc_init(void);
error_code_t cc_configure_cycle(cleaning_type_t type, const cleaning_params_t* params);
error_code_t cc_start_cycle(cleaning_type_t type);
error_code_t cc_stop_cycle(void);
cleaning_phase_t cc_get_current_phase(void);
cleaning_stats_t cc_get_statistics(void);
void cc_optimize_parameters(void);
void cc_process(void);

#endif // CTRL_CLEAN_H
