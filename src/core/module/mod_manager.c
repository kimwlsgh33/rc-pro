#include "mod_manager.h"
#include "mod_interface.h"
#include "../../utils/common/error_codes.h"
#include <string.h>

// Internal module registry structure
typedef struct {
    char name[MODULE_NAME_MAX_LENGTH];
    const module_interface_t* interface;
    module_status_t status;
    module_config_t config;
    bool is_active;
    
    // Dependency tracking
    module_dependency_t dependencies[MAX_MODULE_DEPENDENCIES];
    uint8_t dependency_count;
} module_registry_entry_t;

// Module manager state
static struct {
    module_registry_entry_t modules[MAX_MODULES];
    uint8_t module_count;
    bool initialized;
    module_manager_status_t status;
} module_manager = {0};

// Helper function to find module by name
static module_registry_entry_t* find_module(const char* module_name) {
    if (!module_name) return NULL;
    
    for (uint8_t i = 0; i < module_manager.module_count; i++) {
        if (strcmp(module_manager.modules[i].name, module_name) == 0) {
            return &module_manager.modules[i];
        }
    }
    return NULL;
}

// Module Status Functions
const module_status_t* module_get_status(const char* module_name) {
    module_registry_entry_t* module = find_module(module_name);
    if (!module) return NULL;
    
    // Update status through interface if available
    if (module->interface && module->interface->get_status) {
        module->interface->get_status(&module->status);
    }
    
    return &module->status;
}

const module_interface_t* module_get_interface(const char* module_name) {
    module_registry_entry_t* module = find_module(module_name);
    return module ? module->interface : NULL;
}

module_state_t module_get_state(const char* module_name) {
    module_registry_entry_t* module = find_module(module_name);
    if (!module) return MODULE_STATE_UNINITIALIZED;
    
    if (module->interface && module->interface->get_state) {
        return module->interface->get_state();
    }
    return module->status.state;
}

// Module Processing Functions
void module_process_all(void) {
    for (uint8_t i = 0; i < module_manager.module_count; i++) {
        if (module_manager.modules[i].is_active && 
            module_manager.modules[i].interface && 
            module_manager.modules[i].interface->process) {
            module_manager.modules[i].interface->process();
        }
    }
    module_manager.status.last_process_time = /* Get current time */0; // TODO: Add time function
    module_manager.status.total_process_count++;
}

error_code_t module_process_category(module_category_t category) {
    bool processed = false;
    
    for (uint8_t i = 0; i < module_manager.module_count; i++) {
        module_registry_entry_t* module = &module_manager.modules[i];
        module_info_t* info = (module_info_t*)module->config.data;
        
        if (module->is_active && info && info->category == category) {
            if (module->interface && module->interface->process) {
                error_code_t err = module->interface->process();
                if (err != ERR_SUCCESS) return err;
                processed = true;
            }
        }
    }
    
    return processed ? ERR_SUCCESS : ERR_MODULE_NOT_FOUND;
}

error_code_t module_process_priority(module_priority_t priority) {
    bool processed = false;
    
    for (uint8_t i = 0; i < module_manager.module_count; i++) {
        module_registry_entry_t* module = &module_manager.modules[i];
        module_info_t* info = (module_info_t*)module->config.data;
        
        if (module->is_active && info && info->priority == priority) {
            if (module->interface && module->interface->process) {
                error_code_t err = module->interface->process();
                if (err != ERR_SUCCESS) return err;
                processed = true;
            }
        }
    }
    
    return processed ? ERR_SUCCESS : ERR_MODULE_NOT_FOUND;
}

// Module Configuration Functions
error_code_t module_configure(const char* module_name, const module_config_t* config) {
    if (!config) return ERR_INVALID_PARAM;
    
    module_registry_entry_t* module = find_module(module_name);
    if (!module) return ERR_MODULE_NOT_FOUND;
    
    if (module->interface && module->interface->configure) {
        error_code_t err = module->interface->configure(config);
        if (err == ERR_SUCCESS) {
            memcpy(&module->config, config, sizeof(module_config_t));
        }
        return err;
    }
    
    return ERR_NOT_SUPPORTED;
}

const module_config_t* module_get_config(const char* module_name) {
    module_registry_entry_t* module = find_module(module_name);
    return module ? &module->config : NULL;
}

// Module Event Functions
error_code_t module_send_event(const char* module_name, system_event_t event) {
    module_registry_entry_t* module = find_module(module_name);
    if (!module) return ERR_MODULE_NOT_FOUND;
    
    if (module->interface && module->interface->handle_event) {
        return module->interface->handle_event(&event);
    }
    
    return ERR_NOT_SUPPORTED;
}

error_code_t module_broadcast_event(system_event_t event) {
    error_code_t final_err = ERR_SUCCESS;
    
    for (uint8_t i = 0; i < module_manager.module_count; i++) {
        if (module_manager.modules[i].is_active && 
            module_manager.modules[i].interface && 
            module_manager.modules[i].interface->handle_event) {
            error_code_t err = module_manager.modules[i].interface->handle_event(&event);
            if (err != ERR_SUCCESS) {
                final_err = err; // Continue broadcasting but remember error
            }
        }
    }
    
    return final_err;
}

// Module Manager Interface
error_code_t module_manager_init(void) {
    if (module_manager.initialized) {
        return ERR_ALREADY_INIT;
    }
    
    memset(&module_manager, 0, sizeof(module_manager));
    module_manager.initialized = true;
    return ERR_SUCCESS;
}

error_code_t module_manager_deinit(void) {
    if (!module_manager.initialized) {
        return ERR_NOT_INIT;
    }
    
    // Stop and deinitialize all modules
    for (uint8_t i = 0; i < module_manager.module_count; i++) {
        module_registry_entry_t* module = &module_manager.modules[i];
        if (module->interface) {
            if (module->interface->stop) module->interface->stop();
            if (module->interface->deinit) module->interface->deinit();
        }
    }
    
    memset(&module_manager, 0, sizeof(module_manager));
    return ERR_SUCCESS;
}

// Module Registration
error_code_t module_register(const module_info_t* module_info) {
    if (!module_manager.initialized) return ERR_NOT_INIT;
    if (!module_info || !module_info->name || !module_info->interface) {
        return ERR_INVALID_PARAM;
    }
    if (module_manager.module_count >= MAX_MODULES) {
        return ERR_NO_RESOURCE;
    }
    
    // Check for duplicate module
    if (find_module(module_info->name)) {
        return ERR_ALREADY_EXISTS;
    }
    
    // Register new module
    module_registry_entry_t* new_module = &module_manager.modules[module_manager.module_count];
    strncpy(new_module->name, module_info->name, MODULE_NAME_MAX_LENGTH - 1);
    new_module->name[MODULE_NAME_MAX_LENGTH - 1] = '\0';
    new_module->interface = module_info->interface;
    new_module->is_active = false;
    
    // Initialize module config with info
    new_module->config.data = (void*)module_info;
    new_module->config.data_size = sizeof(module_info_t);
    new_module->config.version = MODULE_INTERFACE_VERSION;
    
    // Initialize dependency tracking
    new_module->dependency_count = 0;
    memset(new_module->dependencies, 0, sizeof(new_module->dependencies));
    
    module_manager.module_count++;
    module_manager.status.total_modules++;
    
    return ERR_SUCCESS;
}

error_code_t module_unregister(const char* module_name) {
    if (!module_manager.initialized) return ERR_NOT_INIT;
    if (!module_name) return ERR_INVALID_PARAM;
    
    // Find module index
    int module_index = -1;
    for (uint8_t i = 0; i < module_manager.module_count; i++) {
        if (strcmp(module_manager.modules[i].name, module_name) == 0) {
            module_index = i;
            break;
        }
    }
    
    if (module_index < 0) return ERR_NOT_FOUND;
    
    // Check if any other modules depend on this one
    for (uint8_t i = 0; i < module_manager.module_count; i++) {
        if (i == module_index) continue;
        
        module_registry_entry_t* other = &module_manager.modules[i];
        for (uint8_t j = 0; j < other->dependency_count; j++) {
            if (strcmp(other->dependencies[j].name, module_name) == 0) {
                return ERR_ALREADY_EXISTS;
            }
        }
    }
    
    // Stop and deinitialize module if active
    module_registry_entry_t* module = &module_manager.modules[module_index];
    if (module->is_active) {
        if (module->interface) {
            if (module->interface->stop) module->interface->stop();
            if (module->interface->deinit) module->interface->deinit();
        }
        module_manager.status.active_modules--;
    }
    
    // Clear module's dependencies
    memset(module->dependencies, 0, sizeof(module->dependencies));
    module->dependency_count = 0;
    
    // Shift remaining modules
    if (module_index < module_manager.module_count - 1) {
        memmove(&module_manager.modules[module_index], 
                &module_manager.modules[module_index + 1],
                (module_manager.module_count - module_index - 1) * sizeof(module_registry_entry_t));
    }
    
    module_manager.module_count--;
    module_manager.status.total_modules--;
    
    return ERR_SUCCESS;
}

// Module Management
error_code_t module_start(const char* module_name) {
    module_registry_entry_t* module = find_module(module_name);
    if (!module) return ERR_NOT_FOUND;
    
    if (module->is_active) return ERR_ALREADY_INIT;
    
    // Check dependencies before starting
    error_code_t dep_status = check_and_start_dependencies(module_name);
    if (dep_status != ERR_SUCCESS) {
        return dep_status;
    }
    
    if (module->interface) {
        // Initialize if needed
        if (module->interface->init) {
            error_code_t err = module->interface->init(&module->config);
            if (err != ERR_SUCCESS) return err;
        }
        
        // Start module
        if (module->interface->start) {
            error_code_t err = module->interface->start();
            if (err != ERR_SUCCESS) {
                if (module->interface->deinit) module->interface->deinit();
                return err;
            }
        }
    }
    
    module->is_active = true;
    module_manager.status.active_modules++;
    return ERR_SUCCESS;
}

error_code_t module_stop(const char* module_name) {
    module_registry_entry_t* module = find_module(module_name);
    if (!module) return ERR_NOT_FOUND;
    
    if (!module->is_active) return ERR_NOT_INIT;
    
    if (module->interface) {
        if (module->interface->stop) {
            error_code_t err = module->interface->stop();
            if (err != ERR_SUCCESS) return err;
        }
        if (module->interface->deinit) {
            module->interface->deinit();
        }
    }
    
    module->is_active = false;
    module_manager.status.active_modules--;
    return ERR_SUCCESS;
}

error_code_t module_reset(const char* module_name) {
    module_registry_entry_t* module = find_module(module_name);
    if (!module) return ERR_NOT_FOUND;
    
    if (module->interface && module->interface->reset) {
        return module->interface->reset();
    }
    
    // If no reset function, try stop and start
    error_code_t err = module_stop(module_name);
    if (err != ERR_SUCCESS && err != ERR_NOT_INIT) return err;
    
    return module_start(module_name);
}

// Module Dependencies
error_code_t module_add_dependency(const char* module_name, const char* dependency_name) {
    if (!module_manager.initialized) return ERR_NOT_INIT;
    if (!module_name || !dependency_name) return ERR_INVALID_PARAM;
    
    // Find the module
    module_registry_entry_t* module = find_module(module_name);
    if (!module) return ERR_NOT_FOUND;
    
    // Check if dependency exists
    if (!find_module(dependency_name)) return ERR_INVALID_PARAM;
    
    // Check for maximum dependencies
    if (module->dependency_count >= MAX_MODULE_DEPENDENCIES) {
        return ERR_NO_RESOURCE;
    }
    
    // Check for duplicate dependency
    for (uint8_t i = 0; i < module->dependency_count; i++) {
        if (strcmp(module->dependencies[i].name, dependency_name) == 0) {
            return ERR_ALREADY_EXISTS;
        }
    }
    
    // Add new dependency
    module_dependency_t* dep = &module->dependencies[module->dependency_count];
    strncpy(dep->name, dependency_name, MODULE_NAME_MAX_LENGTH - 1);
    dep->name[MODULE_NAME_MAX_LENGTH - 1] = '\0';
    dep->status = DEP_STATUS_PENDING;
    
    module->dependency_count++;
    
    return ERR_SUCCESS;
}

error_code_t module_check_dependencies(const char* module_name) {
    if (!module_manager.initialized) return ERR_NOT_INIT;
    if (!module_name) return ERR_INVALID_PARAM;
    
    // Find the module
    module_registry_entry_t* module = find_module(module_name);
    if (!module) return ERR_NOT_FOUND;
    
    // No dependencies is considered satisfied
    if (module->dependency_count == 0) {
        return ERR_SUCCESS;
    }
    
    // Check each dependency
    bool all_satisfied = true;
    for (uint8_t i = 0; i < module->dependency_count; i++) {
        module_registry_entry_t* dep_module = find_module(module->dependencies[i].name);
        
        if (!dep_module) {
            module->dependencies[i].status = DEP_STATUS_ERROR;
            all_satisfied = false;
            continue;
        }
        
        // Check if dependency is active and initialized
        if (!dep_module->is_active || 
            dep_module->status.state != MODULE_STATE_RUNNING) {
            module->dependencies[i].status = DEP_STATUS_PENDING;
            all_satisfied = false;
        } else {
            module->dependencies[i].status = DEP_STATUS_SATISFIED;
        }
    }
    
    return all_satisfied ? ERR_SUCCESS : ERR_NOT_INIT;
}

// Helper function to check dependencies before starting a module
error_code_t check_and_start_dependencies(const char* module_name) {
    module_registry_entry_t* module = find_module(module_name);
    if (!module) return ERR_NOT_FOUND;
    
    // First check if dependencies are satisfied
    error_code_t dep_status = module_check_dependencies(module_name);
    if (dep_status == ERR_SUCCESS) {
        return ERR_SUCCESS;  // Dependencies already satisfied
    }
    
    // Try to start each pending dependency
    for (uint8_t i = 0; i < module->dependency_count; i++) {
        if (module->dependencies[i].status == DEP_STATUS_PENDING) {
            error_code_t err = module_start(module->dependencies[i].name);
            if (err != ERR_SUCCESS && err != ERR_ALREADY_INIT) {
                return err;
            }
        }
    }
    
    // Check dependencies again
    return module_check_dependencies(module_name);
}

// Dependency Information Functions
uint8_t module_get_dependency_count(const char* module_name) {
    module_registry_entry_t* module = find_module(module_name);
    return module ? module->dependency_count : 0;
}

const module_dependency_t* module_get_dependencies(const char* module_name) {
    module_registry_entry_t* module = find_module(module_name);
    return module ? module->dependencies : NULL;
}

dependency_status_t module_get_dependency_status(const char* module_name, const char* dependency_name) {
    if (!module_name || !dependency_name) return DEP_STATUS_ERROR;
    
    module_registry_entry_t* module = find_module(module_name);
    if (!module) return DEP_STATUS_ERROR;
    
    for (uint8_t i = 0; i < module->dependency_count; i++) {
        if (strcmp(module->dependencies[i].name, dependency_name) == 0) {
            return module->dependencies[i].status;
        }
    }
    
    return DEP_STATUS_NONE;
}

// Module Manager Status
const module_manager_status_t* module_manager_get_status(void) {
    if (!module_manager.initialized) return NULL;
    return &module_manager.status;
}

uint8_t module_manager_get_count(void) {
    return module_manager.module_count;
}

const char** module_manager_get_module_list(uint8_t* count) {
    if (!count) return NULL;
    
    static const char* module_list[MAX_MODULES];
    *count = module_manager.module_count;
    
    for (uint8_t i = 0; i < module_manager.module_count; i++) {
        module_list[i] = module_manager.modules[i].name;
    }
    
    return module_list;
}
