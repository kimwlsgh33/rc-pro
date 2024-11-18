#ifndef SAFE_MAINTENANCE_H
#define SAFE_MAINTENANCE_H

#include "../../config/cfg_build.h"
#include "../../platform/common/plat_types.h"
#include "../../utils/common/error_codes.h"
#include "../../core/module_interface.h"

// Maintenance States
typedef enum {
    MAINT_STATE_NORMAL,
    MAINT_STATE_DUE_SOON,
    MAINT_STATE_OVERDUE,
    MAINT_STATE_IN_PROGRESS,
    MAINT_STATE_ERROR
} maintenance_state_t;

// Component Health Status
typedef enum {
    HEALTH_STATUS_GOOD,
    HEALTH_STATUS_FAIR,
    HEALTH_STATUS_POOR,
    HEALTH_STATUS_CRITICAL,
    HEALTH_STATUS_UNKNOWN
} health_status_t;

// Maintenance Parameters
typedef struct {
    uint32_t operation_hours_limit;
    uint32_t cycle_count_limit;
    uint16_t wear_threshold;
    uint16_t performance_threshold;
    uint8_t prediction_window_days;
} maintenance_params_t;

// Component Health Data
typedef struct {
    health_status_t status;
    uint32_t operation_hours;
    uint32_t cycle_count;
    uint16_t wear_level;
    uint16_t performance_level;
    uint32_t last_maintenance;
    uint32_t next_maintenance;
} component_health_t;

// Maintenance Statistics
typedef struct {
    uint32_t total_maintenance_ops;
    uint32_t preventive_actions;
    uint32_t corrective_actions;
    uint32_t total_downtime_ms;
    float availability_percent;
} maintenance_stats_t;

// Function Declarations
error_code_t maintenance_init(const maintenance_params_t* params);
error_code_t maintenance_set_params(const maintenance_params_t* params);
error_code_t maintenance_start_service(void);
error_code_t maintenance_complete_service(void);
error_code_t maintenance_record_event(uint8_t component_id, const char* event);

// Health Monitoring
health_status_t maintenance_get_health(uint8_t component_id);
const component_health_t* maintenance_get_health_data(uint8_t component_id);
uint32_t maintenance_predict_next_service(uint8_t component_id);

// Statistics and Reporting
const maintenance_stats_t* maintenance_get_statistics(void);
error_code_t maintenance_generate_report(char* buffer, size_t size);

// System Processing
void maintenance_process(void);

#endif // SAFE_MAINTENANCE_H
