#ifndef MOD_INTERFACE_H
#define MOD_INTERFACE_H

#include "../../config/cfg_system.h"
#include "../../utils/common/error_codes.h"

// Module States
typedef enum {
    MODULE_STATE_UNINITIALIZED = 0,
    MODULE_STATE_INITIALIZED = 1,
    MODULE_STATE_RUNNING = 2,
    MODULE_STATE_SUSPENDED = 3,
    MODULE_STATE_ERROR = 4
} module_state_t;

// Module Status Information
typedef struct {
    module_state_t state;
    uint32_t uptime;
    uint32_t last_process_time;
    uint32_t error_count;
    error_code_t last_error;
} module_status_t;

// Module Configuration Structure
typedef struct {
    void* data;              // Pointer to module-specific configuration data
    uint16_t data_size;      // Size of configuration data
    uint32_t version;        // Configuration version
} module_config_t;

// Module Interface Functions
typedef struct {
    // Lifecycle Management
    error_code_t (*init)(const module_config_t* config);
    error_code_t (*deinit)(void);
    error_code_t (*start)(void);
    error_code_t (*stop)(void);
    error_code_t (*reset)(void);
    
    // Runtime Operations
    error_code_t (*process)(void);
    error_code_t (*suspend)(void);
    error_code_t (*resume)(void);
    
    // Configuration Management
    error_code_t (*configure)(const module_config_t* config);
    error_code_t (*get_config)(module_config_t* config);
    
    // Event Handling
    error_code_t (*handle_event)(const void* event);
    
    // Status Management
    error_code_t (*get_status)(module_status_t* status);
    module_state_t (*get_state)(void);
    
    // Resource Management
    error_code_t (*save_state)(void);
    error_code_t (*load_state)(void);
} module_interface_t;

// Helper Macros for Interface Implementation
#define MODULE_INTERFACE_VERSION 1

// Default interface implementation macro
#define MODULE_INTERFACE_DEFAULT(name) \
    static const module_interface_t name##_interface = { \
        .init = name##_init, \
        .deinit = name##_deinit, \
        .start = name##_start, \
        .stop = name##_stop, \
        .reset = name##_reset, \
        .process = name##_process, \
        .suspend = name##_suspend, \
        .resume = name##_resume, \
        .configure = name##_configure, \
        .get_config = name##_get_config, \
        .handle_event = name##_handle_event, \
        .get_status = name##_get_status, \
        .get_state = name##_get_state, \
        .save_state = name##_save_state, \
        .load_state = name##_load_state \
    }

#endif // MOD_INTERFACE_H
