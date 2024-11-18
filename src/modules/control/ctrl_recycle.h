#ifndef CTRL_RECYCLE_H
#define CTRL_RECYCLE_H

#include "../../config/cfg_build.h"
#include "../../platform/common/plat_types.h"
#include "../../utils/common/error_codes.h"
#include "../../core/module_interface.h"

// Recycling States
typedef enum {
    RECYCLE_STATE_IDLE,
    RECYCLE_STATE_INIT,
    RECYCLE_STATE_DETECTING,
    RECYCLE_STATE_SORTING,
    RECYCLE_STATE_PROCESSING,
    RECYCLE_STATE_COMPLETE,
    RECYCLE_STATE_ERROR
} recycle_state_t;

// Recycling Configuration
typedef struct {
    uint16_t detection_timeout_ms;
    uint16_t sorting_timeout_ms;
    uint16_t process_timeout_ms;
    uint8_t retry_count;
    bool auto_retry;
} recycle_config_t;

// Recycling Status
typedef struct {
    recycle_state_t state;
    uint32_t cycle_time_ms;
    uint32_t items_processed;
    uint32_t items_rejected;
    uint32_t error_flags;
} recycle_status_t;

// Recycling Statistics
typedef struct {
    uint32_t total_cycles;
    uint32_t successful_cycles;
    uint32_t failed_cycles;
    uint32_t total_items;
    uint32_t sorted_items;
    uint32_t rejected_items;
    uint32_t error_count;
} recycle_stats_t;

// Function Declarations
error_code_t recycle_init(const recycle_config_t* config);
error_code_t recycle_configure(const recycle_config_t* config);
error_code_t recycle_start_cycle(void);
error_code_t recycle_stop_cycle(void);
error_code_t recycle_pause_cycle(void);
error_code_t recycle_resume_cycle(void);
error_code_t recycle_emergency_stop(void);

// Status and Control
const recycle_status_t* recycle_get_status(void);
const recycle_stats_t* recycle_get_statistics(void);
error_code_t recycle_reset_error(void);

// System Processing
void recycle_process(void);

#endif // CTRL_RECYCLE_H
