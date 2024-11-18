#ifndef SAFE_MONITOR_H
#define SAFE_MONITOR_H

#include "../../config/cfg_system.h"
#include "../../config/cfg_hardware.h"
#include "../../platform/common/plat_types.h"
#include "../../utils/common/error_codes.h"
#include "../../utils/debug/dbg_diagnostics.h"
#include "../../core/module/mod_manager.h"
#include "../../core/event/evt_manager.h"

// Safety status flags
typedef enum {
    SAFETY_OK = 0,
    SAFETY_WARNING = 1,
    SAFETY_CRITICAL = 2
} safety_status_t;

// Safety subsystem identifiers
typedef enum {
    SAFETY_SYS_MECHANICAL = 0,
    SAFETY_SYS_ELECTRICAL,
    SAFETY_SYS_THERMAL,
    SAFETY_SYS_FLUID,
    SAFETY_SYS_MOTION,
    SAFETY_SYS_COUNT
} safety_system_t;

// Safety threshold configuration
typedef struct {
    uint16_t warning_threshold;
    uint16_t critical_threshold;
    uint16_t recovery_threshold;
    uint16_t max_warnings;
    uint32_t warning_timeout_ms;
} safety_threshold_t;

// Safety statistics
typedef struct {
    uint32_t total_warnings;
    uint32_t total_criticals;
    uint32_t last_warning_time;
    uint32_t last_critical_time;
    uint16_t consecutive_warnings;
} safety_stats_t;

// Initialize safety monitoring system
void safety_init(void);

// Process safety monitoring cycle
void safety_process_cycle(void);

// Get current safety status for a subsystem
safety_status_t safety_get_status(safety_system_t system);

// Get overall system safety status
safety_status_t safety_get_system_status(void);

// Configure safety thresholds for a subsystem
void safety_configure_thresholds(safety_system_t system, const safety_threshold_t* thresholds);

// Get safety statistics for a subsystem
const safety_stats_t* safety_get_stats(safety_system_t system);

// Reset safety statistics for a subsystem
void safety_reset_stats(safety_system_t system);

// Check if emergency stop is required
bool safety_emergency_stop_required(void);

// Acknowledge and clear safety warnings
void safety_acknowledge_warnings(safety_system_t system);

// Register callback for safety status changes
typedef void (*safety_callback_t)(safety_system_t system, safety_status_t status);
void safety_register_callback(safety_callback_t callback);

// Emergency stop control
void safety_trigger_emergency_stop(void);
void safety_clear_emergency_stop(void);
bool safety_is_emergency_stopped(void);

#endif // SAFE_MONITOR_H
