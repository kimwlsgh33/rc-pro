#ifndef EVT_MANAGER_H
#define EVT_MANAGER_H

#include "../../config/cfg_system.h"
#include "../../platform/common/plat_types.h"
#include "../../utils/common/error_codes.h"

// Event Configuration
#define MAX_EVENT_QUEUE_SIZE 64
#define MAX_EVENT_HANDLERS 32
#define EVENT_DATA_SIZE 16

// Event Types
typedef enum {
    EVENT_TYPE_SYSTEM,
    EVENT_TYPE_MODULE,
    EVENT_TYPE_DRIVER,
    EVENT_TYPE_USER,
    EVENT_TYPE_NETWORK,
    EVENT_TYPE_CUSTOM
} event_type_t;

// Event Priority
typedef enum {
    EVENT_PRIORITY_CRITICAL,
    EVENT_PRIORITY_HIGH,
    EVENT_PRIORITY_NORMAL,
    EVENT_PRIORITY_LOW
} event_priority_t;

// Event Data
typedef struct {
    uint8_t data[EVENT_DATA_SIZE];
    uint8_t size;
} event_data_t;

// Event Structure
typedef struct {
    event_type_t type;
    event_priority_t priority;
    uint32_t timestamp;
    uint16_t source_id;
    uint16_t target_id;
    event_data_t data;
} system_event_t;

// Event Handler Function Type
typedef error_code_t (*event_handler_t)(const system_event_t* event);

// Event Handler Registration
typedef struct {
    event_type_t type;
    uint16_t id;
    event_handler_t handler;
} event_handler_info_t;

// Event Manager Status
typedef struct {
    uint8_t queue_count;
    uint8_t handler_count;
    uint32_t events_processed;
    uint32_t events_dropped;
    uint32_t last_event_time;
} event_manager_status_t;

// Event Manager Interface
error_code_t event_manager_init(void);
error_code_t event_manager_deinit(void);

// Event Processing
error_code_t event_post(const system_event_t* event);
error_code_t event_process_all(void);
error_code_t event_process_type(event_type_t type);
error_code_t event_process_priority(event_priority_t priority);

// Event Handler Management
error_code_t event_register_handler(const event_handler_info_t* handler_info);
error_code_t event_unregister_handler(uint16_t handler_id);
error_code_t event_enable_handler(uint16_t handler_id);
error_code_t event_disable_handler(uint16_t handler_id);

// Event Queue Management
error_code_t event_clear_queue(void);
uint8_t event_get_queue_count(void);
bool event_is_queue_full(void);
bool event_is_queue_empty(void);

// Event Filtering
error_code_t event_set_filter(event_type_t type, bool enabled);
error_code_t event_set_priority_filter(event_priority_t min_priority);

// Event Manager Status
const event_manager_status_t* event_manager_get_status(void);
uint32_t event_manager_get_processed_count(void);
uint32_t event_manager_get_dropped_count(void);

// Utility Functions
const char* event_type_to_string(event_type_t type);
const char* event_priority_to_string(event_priority_t priority);

#endif // EVT_MANAGER_H
