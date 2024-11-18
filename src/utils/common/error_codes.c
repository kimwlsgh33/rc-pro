#include "error_handling.h"
#include "putil.h"
#include "timer.h"
#include <string.h>

// Define error categories and their corresponding statistics
#define ERR_CAT_MAX 5
typedef struct {
    uint16_t total_errors;
    uint16_t resolved_errors;
    uint32_t last_occurrence;
} ErrorCategoryStats;

static ErrorHistory error_history;
static uint8_t last_error = ERR_CAT_NONE;
static int error_timer_id = -1;
static bool is_initialized = false;
static ErrorCategoryStats error_category_stats[ERR_CAT_MAX];
static ErrorRecord* error_handlers[MAX_ERROR_HANDLERS];

error_code_t error_init(void) {
    if (is_initialized) {
        return ERR_ALREADY_INITIALIZED;
    }

    // Initialize error history with atomic protection
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        memset(&error_history, 0, sizeof(ErrorHistory));
        error_timer_id = timer_alloc();
        
        if (error_timer_id == -1) {
            return ERR_RESOURCE_INIT_FAILED;
        }
        
        // Initialize error categories
        for (int i = 0; i < ERR_CAT_MAX; i++) {
            error_category_stats[i].total_errors = 0;
            error_category_stats[i].resolved_errors = 0;
            error_category_stats[i].last_occurrence = 0;
        }
        
        is_initialized = true;
    }
    return ERROR_CODE_SUCCESS;
}

void error_record(error_code_t error_code) {
    if (!is_initialized) {
        // Try to initialize if not already done
        if (error_init() != ERROR_CODE_SUCCESS) {
            // If initialization fails, we can't do much
            return;
        }
    }

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        last_error = error_code;
        
        // Record in history
        ErrorRecord* record = &error_history.history[error_history.current_index];
        record->code = error_code;
        record->timestamp = timer_get(error_timer_id);
        record->retry_count = 0;
        record->resolved = 0;
        record->context = get_error_context();  // Get additional context
        
        // Update category statistics
        uint8_t category = error_code & 0xF0;
        if (category < ERR_CAT_MAX) {
            error_category_stats[category].total_errors++;
            error_category_stats[category].last_occurrence = record->timestamp;
        }
        
        error_history.current_index = (error_history.current_index + 1) % MAX_ERROR_HISTORY;
        if (error_history.total_errors < MAX_ERROR_HISTORY) {
            error_history.total_errors++;
        }
        
        // Notify error handlers if registered
        for (int i = 0; i < MAX_ERROR_HANDLERS; i++) {
            if (error_handlers[i] != NULL) {
                error_handlers[i](error_code, record);
            }
        }
        
        // Attempt recovery if appropriate
        if (should_attempt_recovery(error_code)) {
            error_attempt_recovery(error_code);
        }
    }
}

uint8_t error_attempt_recovery(error_code_t error_code) {
    uint8_t recovery_success = 0;
    
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        // Find the most recent record of this error
        for (int i = 0; i < error_history.total_errors; i++) {
            int idx = (error_history.current_index - 1 - i + MAX_ERROR_HISTORY) % MAX_ERROR_HISTORY;
            ErrorRecord* record = &error_history.history[idx];
            
            if (record->code == error_code && !record->resolved) {
                record->retry_count++;
                
                // Check if we've exceeded max retries for this category
                if (record->retry_count > get_max_retries(error_code)) {
                    error_escalate(record);
                    return 0;
                }
                
                switch (error_code & 0xF0) {
                    case ERR_CAT_SENSOR:
                        recovery_success = attempt_sensor_recovery(record);
                        break;
                        
                    case ERR_CAT_MOTOR:
                        recovery_success = attempt_motor_recovery(record);
                        break;
                        
                    case ERR_CAT_DOOR:
                        recovery_success = attempt_door_recovery(record);
                        break;
                        
                    case ERR_CAT_COMMUNICATION:
                        recovery_success = attempt_comm_recovery(record);
                        break;
                        
                    case ERR_CAT_SYSTEM:
                        // System errors require manual intervention
                        recovery_success = 0;
                        break;
                }
                
                record->resolved = recovery_success;
                if (recovery_success) {
                    uint8_t category = error_code & 0xF0;
                    if (category < ERR_CAT_MAX) {
                        error_category_stats[category].resolved_errors++;
                    }
                }
                break;
            }
        }
    }
    
    return recovery_success;
}

static void error_escalate(ErrorRecord* record) {
    // Escalate error handling based on severity and retry count
    if (record->retry_count >= CRITICAL_RETRY_THRESHOLD) {
        // Transition to critical error state
        error_record(ERR_CRITICAL_FAILURE);
        
        // Notify system of critical failure
        system_critical_failure_handler();
        
        // Attempt safe shutdown
        safe_shutdown();
    }
}

error_code_t error_get_last(void) {
    if (!is_initialized) {
        return ERR_NOT_INITIALIZED;
    }
    return last_error;
}

void error_clear(void) {
    if (!is_initialized) {
        return;
    }
    last_error = ERR_CAT_NONE;
}

ErrorHistory* error_get_history(void) {
    if (!is_initialized) {
        return NULL;
    }
    return &error_history;
}

void error_print_status(void) {
    if (!is_initialized) {
        DPRINTF("Error system not initialized\n");
        return;
    }

    DPRINTF("Error History (Total: %d):\n", error_history.total_errors);
    
    for (int i = 0; i < error_history.total_errors; i++) {
        int idx = (error_history.current_index - 1 - i + MAX_ERROR_HISTORY) % MAX_ERROR_HISTORY;
        ErrorRecord* record = &error_history.history[idx];
        
        DPRINTF("Error[%d]: Code=0x%02X, Time=%lu, Retries=%d, %s\n",
                i,
                record->code,
                record->timestamp,
                record->retry_count,
                record->resolved ? "Resolved" : "Unresolved");
    }
}
