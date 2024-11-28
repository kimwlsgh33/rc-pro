# Event Manager

A lightweight, efficient event management system designed for embedded systems. The event manager provides priority-based event handling, flexible filtering, and comprehensive status monitoring.

## Features

- **Priority-based Event Processing**: Support for CRITICAL, HIGH, NORMAL, and LOW priority events
- **Event Type Management**: System, Module, Driver, User, Network, and Custom event types
- **Efficient Queue Implementation**: Circular buffer with O(1) enqueue/dequeue operations
- **Flexible Event Filtering**: Filter events by type and priority
- **Robust Error Handling**: Comprehensive error checking and status reporting
- **Resource-Efficient**: Fixed memory footprint suitable for embedded systems

## Configuration

Key configuration parameters in `evt_manager.h`:

```c
#define MAX_EVENT_QUEUE_SIZE 64    // Maximum events in queue
#define MAX_EVENT_HANDLERS 32      // Maximum registered handlers
#define EVENT_DATA_SIZE 16         // Size of event data payload
```

## Usage Examples

### 1. Initialize Event Manager

```c
#include "evt_manager.h"

void init_system(void) {
    error_code_t err = event_manager_init();
    if (err != ERR_SUCCESS) {
        // Handle initialization error
    }
}
```

### 2. Register Event Handler

```c
error_code_t my_event_handler(const system_event_t* event) {
    // Handle event
    return ERR_SUCCESS;
}

void setup_handlers(void) {
    event_handler_info_t handler = {
        .type = EVENT_TYPE_SYSTEM,
        .id = 1,
        .handler = my_event_handler
    };
    
    error_code_t err = event_register_handler(&handler);
    if (err != ERR_SUCCESS) {
        // Handle registration error
    }
}
```

### 3. Post Events

```c
void send_system_event(void) {
    system_event_t event = {
        .type = EVENT_TYPE_SYSTEM,
        .priority = EVENT_PRIORITY_NORMAL,
        .timestamp = get_system_time(),
        .source_id = MY_MODULE_ID,
        .target_id = TARGET_MODULE_ID,
        .data = { /* event data */ }
    };
    
    error_code_t err = event_post(&event);
    if (err != ERR_SUCCESS) {
        // Handle posting error
    }
}
```

### 4. Process Events

```c
void main_loop(void) {
    while (1) {
        // Process all events
        error_code_t err = event_process_all();
        if (err != ERR_SUCCESS) {
            // Handle processing error
        }
        
        // Or process specific types
        err = event_process_type(EVENT_TYPE_SYSTEM);
        
        // Or process by priority
        err = event_process_priority(EVENT_PRIORITY_HIGH);
    }
}
```

### 5. Event Filtering

```c
void configure_filters(void) {
    // Filter out network events
    event_set_filter(EVENT_TYPE_NETWORK, false);
    
    // Only process HIGH priority and above
    event_set_priority_filter(EVENT_PRIORITY_HIGH);
}
```

### 6. Status Monitoring

```c
void check_status(void) {
    const event_manager_status_t* status = event_manager_get_status();
    
    printf("Events in queue: %d\n", status->queue_count);
    printf("Events processed: %lu\n", status->events_processed);
    printf("Events dropped: %lu\n", status->events_dropped);
    printf("Last event time: %lu\n", status->last_event_time);
}
```

## Best Practices

1. **Priority Usage**:
   - CRITICAL: System-critical events requiring immediate attention
   - HIGH: Time-sensitive operations
   - NORMAL: Regular operations
   - LOW: Background tasks

2. **Queue Management**:
   - Monitor queue usage with `event_get_queue_count()`
   - Handle queue full conditions appropriately
   - Consider increasing `MAX_EVENT_QUEUE_SIZE` if drops are frequent

3. **Handler Implementation**:
   - Keep handlers lightweight
   - Avoid blocking operations in handlers
   - Return appropriate error codes

4. **Error Handling**:
   - Always check return codes
   - Implement appropriate error recovery
   - Monitor dropped event count

## Error Codes

The event manager uses the following error codes:

- `ERR_SUCCESS`: Operation successful
- `ERR_NOT_INITIALIZED`: Event manager not initialized
- `ERR_ALREADY_INITIALIZED`: Already initialized
- `ERR_INVALID_PARAMETER`: Invalid parameter provided
- `ERR_NO_RESOURCE`: Queue or handler array full
- `ERR_NOT_FOUND`: Requested item not found
- `ERR_ALREADY_EXISTS`: Handler ID already registered
