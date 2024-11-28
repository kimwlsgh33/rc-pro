/**
 * Event Manager Implementation
 * - Priority-based event queue management
 * - Event handler registration and dispatch
 * - Event filtering and monitoring
 * 
 * @author tchan@TSoft
 * @date 2024/01/01
 */

#include "evt_manager.h"
#include <string.h>
#include "../../utils/common/error_codes.h"

// Static event queue
static system_event_t event_queue[MAX_EVENT_QUEUE_SIZE];
static uint8_t queue_head = 0;
static uint8_t queue_tail = 0;
static uint8_t queue_count = 0;

// Event handlers
static event_handler_info_t event_handlers[MAX_EVENT_HANDLERS];
static uint8_t handler_count = 0;
static bool handler_enabled[MAX_EVENT_HANDLERS];

// Event filtering
static bool type_filters[EVENT_TYPE_CUSTOM + 1] = {true};  // All enabled by default
static event_priority_t priority_filter = EVENT_PRIORITY_LOW;  // Accept all priorities by default

// Status tracking
static event_manager_status_t manager_status = {0};
static bool initialized = false;

// Private function declarations
static bool is_priority_acceptable(event_priority_t priority);
static bool is_type_acceptable(event_type_t type);
static error_code_t enqueue_event(const system_event_t* event);
static error_code_t dequeue_event(system_event_t* event);
static void update_status(bool event_processed);

// Initialize event manager
error_code_t event_manager_init(void) {
    if (initialized) {
        return ERR_ALREADY_INITIALIZED;
    }

    // Clear all data structures
    memset(event_queue, 0, sizeof(event_queue));
    memset(event_handlers, 0, sizeof(event_handlers));
    memset(handler_enabled, true, sizeof(handler_enabled));
    memset(&manager_status, 0, sizeof(manager_status));

    // Reset queue pointers
    queue_head = 0;
    queue_tail = 0;
    queue_count = 0;
    handler_count = 0;

    // Enable all type filters by default
    for (int i = 0; i <= EVENT_TYPE_CUSTOM; i++) {
        type_filters[i] = true;
    }

    // Set default priority filter
    priority_filter = EVENT_PRIORITY_LOW;

    initialized = true;
    return ERR_SUCCESS;
}

// Deinitialize event manager
error_code_t event_manager_deinit(void) {
    if (!initialized) {
        return ERR_NOT_INITIALIZED;
    }

    // Clear all data structures
    memset(event_queue, 0, sizeof(event_queue));
    memset(event_handlers, 0, sizeof(event_handlers));
    memset(&manager_status, 0, sizeof(manager_status));

    initialized = false;
    return ERR_SUCCESS;
}

// Post an event to the queue
error_code_t event_post(const system_event_t* event) {
    if (!initialized) {
        return ERR_NOT_INITIALIZED;
    }

    if (!event) {
        return ERR_INVALID_PARAMETER;
    }

    // Check if event should be filtered
    if (!is_type_acceptable(event->type) || !is_priority_acceptable(event->priority)) {
        manager_status.events_dropped++;
        return ERR_SUCCESS;  // Silent drop for filtered events
    }

    return enqueue_event(event);
}

// Process all events in queue
error_code_t event_process_all(void) {
    if (!initialized) {
        return ERR_NOT_INITIALIZED;
    }

    system_event_t event;
    error_code_t err;
    bool processed = false;

    while (queue_count > 0) {
        err = dequeue_event(&event);
        if (err != ERR_SUCCESS) {
            return err;
        }

        // Process event through all registered handlers
        for (uint8_t i = 0; i < handler_count; i++) {
            if (handler_enabled[i] && event_handlers[i].type == event.type) {
                err = event_handlers[i].handler(&event);
                if (err != ERR_SUCCESS) {
                    // Log error but continue processing
                    manager_status.events_dropped++;
                }
                processed = true;
            }
        }

        update_status(processed);
    }

    return ERR_SUCCESS;
}

// Process events of specific type
error_code_t event_process_type(event_type_t type) {
    if (!initialized) {
        return ERR_NOT_INITIALIZED;
    }

    system_event_t event;
    error_code_t err;
    bool processed = false;

    // Process all events of specified type
    for (uint8_t i = 0; i < queue_count; i++) {
        if (event_queue[(queue_head + i) % MAX_EVENT_QUEUE_SIZE].type == type) {
            err = dequeue_event(&event);
            if (err != ERR_SUCCESS) {
                return err;
            }

            // Process through handlers
            for (uint8_t j = 0; j < handler_count; j++) {
                if (handler_enabled[j] && event_handlers[j].type == type) {
                    err = event_handlers[j].handler(&event);
                    if (err != ERR_SUCCESS) {
                        manager_status.events_dropped++;
                    }
                    processed = true;
                }
            }

            update_status(processed);
        }
    }

    return ERR_SUCCESS;
}

// Process events of specific priority
error_code_t event_process_priority(event_priority_t priority) {
    if (!initialized) {
        return ERR_NOT_INITIALIZED;
    }

    system_event_t event;
    error_code_t err;
    bool processed = false;

    // Process all events of specified priority or higher
    for (uint8_t i = 0; i < queue_count; i++) {
        if (event_queue[(queue_head + i) % MAX_EVENT_QUEUE_SIZE].priority <= priority) {
            err = dequeue_event(&event);
            if (err != ERR_SUCCESS) {
                return err;
            }

            // Process through handlers
            for (uint8_t j = 0; j < handler_count; j++) {
                if (handler_enabled[j]) {
                    err = event_handlers[j].handler(&event);
                    if (err != ERR_SUCCESS) {
                        manager_status.events_dropped++;
                    }
                    processed = true;
                }
            }

            update_status(processed);
        }
    }

    return ERR_SUCCESS;
}

// Register event handler
error_code_t event_register_handler(const event_handler_info_t* handler_info) {
    if (!initialized) {
        return ERR_NOT_INITIALIZED;
    }

    if (!handler_info || !handler_info->handler) {
        return ERR_INVALID_PARAMETER;
    }

    if (handler_count >= MAX_EVENT_HANDLERS) {
        return ERR_NO_RESOURCE;
    }

    // Check for duplicate handler ID
    for (uint8_t i = 0; i < handler_count; i++) {
        if (event_handlers[i].id == handler_info->id) {
            return ERR_ALREADY_EXISTS;
        }
    }

    // Add new handler
    event_handlers[handler_count] = *handler_info;
    handler_enabled[handler_count] = true;
    handler_count++;

    return ERR_SUCCESS;
}

// Unregister event handler
error_code_t event_unregister_handler(uint16_t handler_id) {
    if (!initialized) {
        return ERR_NOT_INITIALIZED;
    }

    // Find and remove handler
    for (uint8_t i = 0; i < handler_count; i++) {
        if (event_handlers[i].id == handler_id) {
            // Shift remaining handlers
            for (uint8_t j = i; j < handler_count - 1; j++) {
                event_handlers[j] = event_handlers[j + 1];
                handler_enabled[j] = handler_enabled[j + 1];
            }
            handler_count--;
            return ERR_SUCCESS;
        }
    }

    return ERR_NOT_FOUND;
}

// Enable event handler
error_code_t event_enable_handler(uint16_t handler_id) {
    if (!initialized) {
        return ERR_NOT_INITIALIZED;
    }

    for (uint8_t i = 0; i < handler_count; i++) {
        if (event_handlers[i].id == handler_id) {
            handler_enabled[i] = true;
            return ERR_SUCCESS;
        }
    }

    return ERR_NOT_FOUND;
}

// Disable event handler
error_code_t event_disable_handler(uint16_t handler_id) {
    if (!initialized) {
        return ERR_NOT_INITIALIZED;
    }

    for (uint8_t i = 0; i < handler_count; i++) {
        if (event_handlers[i].id == handler_id) {
            handler_enabled[i] = false;
            return ERR_SUCCESS;
        }
    }

    return ERR_NOT_FOUND;
}

// Clear event queue
error_code_t event_clear_queue(void) {
    if (!initialized) {
        return ERR_NOT_INITIALIZED;
    }

    queue_head = 0;
    queue_tail = 0;
    queue_count = 0;
    manager_status.events_dropped += queue_count;  // Count cleared events as dropped

    return ERR_SUCCESS;
}

// Get current queue count
uint8_t event_get_queue_count(void) {
    return queue_count;
}

// Check if queue is full
bool event_is_queue_full(void) {
    return queue_count >= MAX_EVENT_QUEUE_SIZE;
}

// Check if queue is empty
bool event_is_queue_empty(void) {
    return queue_count == 0;
}

// Set event type filter
error_code_t event_set_filter(event_type_t type, bool enabled) {
    if (!initialized) {
        return ERR_NOT_INITIALIZED;
    }

    if (type > EVENT_TYPE_CUSTOM) {
        return ERR_INVALID_PARAMETER;
    }

    type_filters[type] = enabled;
    return ERR_SUCCESS;
}

// Set minimum priority filter
error_code_t event_set_priority_filter(event_priority_t min_priority) {
    if (!initialized) {
        return ERR_NOT_INITIALIZED;
    }

    priority_filter = min_priority;
    return ERR_SUCCESS;
}

// Get event manager status
const event_manager_status_t* event_manager_get_status(void) {
    return &manager_status;
}

// Get processed event count
uint32_t event_manager_get_processed_count(void) {
    return manager_status.events_processed;
}

// Get dropped event count
uint32_t event_manager_get_dropped_count(void) {
    return manager_status.events_dropped;
}

// Convert event type to string
const char* event_type_to_string(event_type_t type) {
    static const char* type_strings[] = {
        "SYSTEM",
        "MODULE",
        "DRIVER",
        "USER",
        "NETWORK",
        "CUSTOM"
    };

    if (type <= EVENT_TYPE_CUSTOM) {
        return type_strings[type];
    }
    return "UNKNOWN";
}

// Convert priority to string
const char* event_priority_to_string(event_priority_t priority) {
    static const char* priority_strings[] = {
        "CRITICAL",
        "HIGH",
        "NORMAL",
        "LOW"
    };

    if (priority <= EVENT_PRIORITY_LOW) {
        return priority_strings[priority];
    }
    return "UNKNOWN";
}

// Private function implementations
static bool is_priority_acceptable(event_priority_t priority) {
    return priority <= priority_filter;
}

static bool is_type_acceptable(event_type_t type) {
    return type <= EVENT_TYPE_CUSTOM && type_filters[type];
}

static error_code_t enqueue_event(const system_event_t* event) {
    if (queue_count >= MAX_EVENT_QUEUE_SIZE) {
        manager_status.events_dropped++;
        return ERR_NO_RESOURCE;
    }

    // Copy event to queue
    memcpy(&event_queue[queue_tail], event, sizeof(system_event_t));
    
    // Update queue pointers
    queue_tail = (queue_tail + 1) % MAX_EVENT_QUEUE_SIZE;
    queue_count++;
    
    // Update status
    manager_status.queue_count = queue_count;
    manager_status.last_event_time = event->timestamp;

    return ERR_SUCCESS;
}

static error_code_t dequeue_event(system_event_t* event) {
    if (queue_count == 0) {
        return ERR_NOT_FOUND;
    }

    // Copy event from queue
    memcpy(event, &event_queue[queue_head], sizeof(system_event_t));
    
    // Update queue pointers
    queue_head = (queue_head + 1) % MAX_EVENT_QUEUE_SIZE;
    queue_count--;
    
    // Update status
    manager_status.queue_count = queue_count;

    return ERR_SUCCESS;
}

static void update_status(bool event_processed) {
    if (event_processed) {
        manager_status.events_processed++;
    } else {
        manager_status.events_dropped++;
    }
}
