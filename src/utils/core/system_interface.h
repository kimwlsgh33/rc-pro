#ifndef SYSTEM_INTERFACE_H
#define SYSTEM_INTERFACE_H

#include "../config/system_config.h"
#include "types.h"
#include "../utils/error_handling.h"

// System States
typedef enum {
    SYSTEM_STATE_INIT,
    SYSTEM_STATE_READY,
    SYSTEM_STATE_RUNNING,
    SYSTEM_STATE_ERROR,
    SYSTEM_STATE_MAINTENANCE,
    SYSTEM_STATE_SHUTDOWN
} system_state_t;

// System Events
typedef enum {
    EVENT_NONE,
    EVENT_CUP_DETECTED,
    EVENT_CLEANING_COMPLETE,
    EVENT_ERROR_DETECTED,
    EVENT_MAINTENANCE_REQUIRED,
    EVENT_SAFETY_TRIGGERED,
    EVENT_USER_INPUT
} system_event_t;

// System Configuration
typedef struct {
    uint8_t operation_mode;
    uint8_t safety_level;
    uint16_t maintenance_interval;
    uint8_t debug_level;
} system_config_t;

// System Status
typedef struct {
    system_state_t state;
    uint32_t uptime;
    uint16_t error_count;
    uint8_t components_active;
    uint8_t maintenance_needed;
} system_status_t;

// System Interface Functions
error_code_t system_init(const system_config_t* config);
error_code_t system_start(void);
error_code_t system_stop(void);
error_code_t system_reset(void);
system_status_t system_get_status(void);
error_code_t system_handle_event(system_event_t event);
void system_process(void);

// Component Management
error_code_t system_enable_component(uint8_t component_id);
error_code_t system_disable_component(uint8_t component_id);
uint8_t system_get_component_status(uint8_t component_id);

// Diagnostic Interface
error_code_t system_run_diagnostics(uint8_t test_mask);
error_code_t system_get_diagnostic_results(uint8_t* results, uint8_t* count);

// Configuration Interface
error_code_t system_update_config(const system_config_t* config);
system_config_t system_get_config(void);

#endif // SYSTEM_INTERFACE_H
