#ifndef MOD_MANAGER_H
#define MOD_MANAGER_H

#include "../../config/cfg_system.h"
#include "../../platform/common/plat_types.h"
#include "../../utils/common/error_codes.h"
#include "mod_interface.h"

// Module Manager Configuration
#define MAX_MODULES 32
#define MODULE_NAME_MAX_LENGTH 32

// Module Priority Levels
typedef enum {
    MODULE_PRIORITY_CRITICAL = 0,
    MODULE_PRIORITY_HIGH = 1,
    MODULE_PRIORITY_NORMAL = 2,
    MODULE_PRIORITY_LOW = 3,
    MODULE_PRIORITY_IDLE = 4
} module_priority_t;

// Module Categories
typedef enum {
    MODULE_CAT_SYSTEM,
    MODULE_CAT_CONTROL,
    MODULE_CAT_SAFETY,
    MODULE_CAT_UI,
    MODULE_CAT_NETWORK,
    MODULE_CAT_DIAGNOSTIC,
    MODULE_CAT_CUSTOM
} module_category_t;

// Module Manager Status
typedef struct {
    uint8_t total_modules;
    uint8_t active_modules;
    uint8_t error_modules;
    uint32_t last_process_time;
    uint32_t total_process_count;
} module_manager_status_t;

// Module Registration Info
typedef struct {
    const char* name;
    const char* version;
    module_category_t category;
    module_priority_t priority;
    const module_interface_t* interface;
} module_info_t;

// Module Manager Interface
error_code_t module_manager_init(void);
error_code_t module_manager_deinit(void);

// Module Registration
error_code_t module_register(const module_info_t* module_info);
error_code_t module_unregister(const char* module_name);

// Module Management
error_code_t module_start(const char* module_name);
error_code_t module_stop(const char* module_name);
error_code_t module_reset(const char* module_name);

// Module Status
const module_status_t* module_get_status(const char* module_name);
const module_interface_t* module_get_interface(const char* module_name);
module_state_t module_get_state(const char* module_name);

// Module Processing
void module_process_all(void);
error_code_t module_process_category(module_category_t category);
error_code_t module_process_priority(module_priority_t priority);

// Module Configuration
error_code_t module_configure(const char* module_name, const module_config_t* config);
const module_config_t* module_get_config(const char* module_name);

// Module Events
error_code_t module_send_event(const char* module_name, system_event_t event);
error_code_t module_broadcast_event(system_event_t event);

// Module Dependencies
error_code_t module_add_dependency(const char* module_name, const char* dependency_name);
error_code_t module_check_dependencies(const char* module_name);

// Module Manager Status
const module_manager_status_t* module_manager_get_status(void);
uint8_t module_manager_get_count(void);
const char** module_manager_get_module_list(uint8_t* count);

#endif // MOD_MANAGER_H
