# Event Manager API Reference

## Data Structures

### system_event_t
Main event structure containing event information.
```c
typedef struct {
    event_type_t type;        // Event type
    event_priority_t priority; // Event priority
    uint32_t timestamp;       // Event timestamp
    uint16_t source_id;       // Source module ID
    uint16_t target_id;       // Target module ID
    event_data_t data;        // Event data payload
} system_event_t;
```

### event_data_t
Event data payload structure.
```c
typedef struct {
    uint8_t data[EVENT_DATA_SIZE]; // Data buffer
    uint8_t size;                  // Actual data size
} event_data_t;
```

### event_handler_info_t
Event handler registration structure.
```c
typedef struct {
    event_type_t type;           // Event type to handle
    uint16_t id;                 // Unique handler ID
    event_handler_t handler;     // Handler function pointer
} event_handler_info_t;
```

### event_manager_status_t
Event manager status structure.
```c
typedef struct {
    uint8_t queue_count;         // Current events in queue
    uint8_t handler_count;       // Registered handlers
    uint32_t events_processed;   // Total processed events
    uint32_t events_dropped;     // Total dropped events
    uint32_t last_event_time;    // Last event timestamp
} event_manager_status_t;
```

## Initialization Functions

### event_manager_init
Initialize the event manager.
```c
error_code_t event_manager_init(void);
```
**Returns:**
- `ERR_SUCCESS`: Initialization successful
- `ERR_ALREADY_INITIALIZED`: Already initialized

### event_manager_deinit
Deinitialize the event manager.
```c
error_code_t event_manager_deinit(void);
```
**Returns:**
- `ERR_SUCCESS`: Deinitialization successful
- `ERR_NOT_INITIALIZED`: Not initialized

## Event Processing Functions

### event_post
Post an event to the queue.
```c
error_code_t event_post(const system_event_t* event);
```
**Parameters:**
- `event`: Pointer to event structure

**Returns:**
- `ERR_SUCCESS`: Event posted successfully
- `ERR_NOT_INITIALIZED`: Manager not initialized
- `ERR_INVALID_PARAMETER`: Invalid event pointer
- `ERR_NO_RESOURCE`: Queue full

### event_process_all
Process all events in queue.
```c
error_code_t event_process_all(void);
```
**Returns:**
- `ERR_SUCCESS`: All events processed
- `ERR_NOT_INITIALIZED`: Manager not initialized

### event_process_type
Process events of specific type.
```c
error_code_t event_process_type(event_type_t type);
```
**Parameters:**
- `type`: Event type to process

**Returns:**
- `ERR_SUCCESS`: Events processed
- `ERR_NOT_INITIALIZED`: Manager not initialized

### event_process_priority
Process events of specific priority or higher.
```c
error_code_t event_process_priority(event_priority_t priority);
```
**Parameters:**
- `priority`: Minimum priority to process

**Returns:**
- `ERR_SUCCESS`: Events processed
- `ERR_NOT_INITIALIZED`: Manager not initialized

## Handler Management Functions

### event_register_handler
Register an event handler.
```c
error_code_t event_register_handler(const event_handler_info_t* handler_info);
```
**Parameters:**
- `handler_info`: Handler registration information

**Returns:**
- `ERR_SUCCESS`: Handler registered
- `ERR_NOT_INITIALIZED`: Manager not initialized
- `ERR_INVALID_PARAMETER`: Invalid handler info
- `ERR_NO_RESOURCE`: Handler array full
- `ERR_ALREADY_EXISTS`: Handler ID exists

### event_unregister_handler
Unregister an event handler.
```c
error_code_t event_unregister_handler(uint16_t handler_id);
```
**Parameters:**
- `handler_id`: Handler ID to unregister

**Returns:**
- `ERR_SUCCESS`: Handler unregistered
- `ERR_NOT_INITIALIZED`: Manager not initialized
- `ERR_NOT_FOUND`: Handler not found

### event_enable_handler
Enable an event handler.
```c
error_code_t event_enable_handler(uint16_t handler_id);
```
**Parameters:**
- `handler_id`: Handler ID to enable

**Returns:**
- `ERR_SUCCESS`: Handler enabled
- `ERR_NOT_INITIALIZED`: Manager not initialized
- `ERR_NOT_FOUND`: Handler not found

### event_disable_handler
Disable an event handler.
```c
error_code_t event_disable_handler(uint16_t handler_id);
```
**Parameters:**
- `handler_id`: Handler ID to disable

**Returns:**
- `ERR_SUCCESS`: Handler disabled
- `ERR_NOT_INITIALIZED`: Manager not initialized
- `ERR_NOT_FOUND`: Handler not found

## Queue Management Functions

### event_clear_queue
Clear all events from queue.
```c
error_code_t event_clear_queue(void);
```
**Returns:**
- `ERR_SUCCESS`: Queue cleared
- `ERR_NOT_INITIALIZED`: Manager not initialized

### event_get_queue_count
Get current number of events in queue.
```c
uint8_t event_get_queue_count(void);
```
**Returns:**
- Current number of events in queue

### event_is_queue_full
Check if queue is full.
```c
bool event_is_queue_full(void);
```
**Returns:**
- `true`: Queue is full
- `false`: Queue has space

### event_is_queue_empty
Check if queue is empty.
```c
bool event_is_queue_empty(void);
```
**Returns:**
- `true`: Queue is empty
- `false`: Queue has events

## Filter Management Functions

### event_set_filter
Set event type filter.
```c
error_code_t event_set_filter(event_type_t type, bool enabled);
```
**Parameters:**
- `type`: Event type to filter
- `enabled`: Enable/disable filter

**Returns:**
- `ERR_SUCCESS`: Filter set
- `ERR_NOT_INITIALIZED`: Manager not initialized
- `ERR_INVALID_PARAMETER`: Invalid type

### event_set_priority_filter
Set minimum priority filter.
```c
error_code_t event_set_priority_filter(event_priority_t min_priority);
```
**Parameters:**
- `min_priority`: Minimum priority to accept

**Returns:**
- `ERR_SUCCESS`: Filter set
- `ERR_NOT_INITIALIZED`: Manager not initialized

## Status Functions

### event_manager_get_status
Get event manager status.
```c
const event_manager_status_t* event_manager_get_status(void);
```
**Returns:**
- Pointer to status structure

### event_manager_get_processed_count
Get total processed event count.
```c
uint32_t event_manager_get_processed_count(void);
```
**Returns:**
- Number of processed events

### event_manager_get_dropped_count
Get total dropped event count.
```c
uint32_t event_manager_get_dropped_count(void);
```
**Returns:**
- Number of dropped events

## Utility Functions

### event_type_to_string
Convert event type to string.
```c
const char* event_type_to_string(event_type_t type);
```
**Parameters:**
- `type`: Event type

**Returns:**
- String representation of type

### event_priority_to_string
Convert priority to string.
```c
const char* event_priority_to_string(event_priority_t priority);
```
**Parameters:**
- `priority`: Event priority

**Returns:**
- String representation of priority
