#include "drv_dma.h"
#include "../../../utils/common/error_codes.h"
#include "../../../utils/common/logging.h"
#include <string.h>

// DMA Safety Configuration
#define DMA_SAFETY_TIMEOUT_MS 1000
#define DMA_MAX_RETRIES 3
#define DMA_ERROR_THRESHOLD 5

// DMA Channel Safety Status
typedef struct {
    uint32_t last_activity;
    uint32_t error_count;
    uint32_t timeout_count;
    uint32_t retry_count;
    bool channel_locked;
} dma_safety_status_t;

static struct {
    dma_safety_status_t channels[DMA_MAX_CHANNELS];
    bool safety_enabled;
    uint32_t total_errors;
    uint32_t total_timeouts;
} dma_safety;

// Initialize DMA safety monitoring
void dma_safety_init(void) {
    memset(&dma_safety, 0, sizeof(dma_safety));
    dma_safety.safety_enabled = true;
}

// Monitor DMA transfer start
error_code_t dma_safety_monitor_start(uint8_t channel) {
    if (channel >= DMA_MAX_CHANNELS) {
        return ERR_INVALID_PARAM;
    }
    
    dma_safety_status_t* status = &dma_safety.channels[channel];
    
    // Check if channel is locked due to errors
    if (status->channel_locked) {
        return ERR_RESOURCE_LOCKED;
    }
    
    // Reset timeout counter
    status->last_activity = timer_get_ms();
    
    return ERROR_CODE_SUCCESS;
}

// Monitor DMA transfer completion
error_code_t dma_safety_monitor_complete(uint8_t channel, bool success) {
    if (channel >= DMA_MAX_CHANNELS) {
        return ERR_INVALID_PARAM;
    }
    
    dma_safety_status_t* status = &dma_safety.channels[channel];
    
    if (!success) {
        status->error_count++;
        dma_safety.total_errors++;
        
        // Check error threshold
        if (status->error_count >= DMA_ERROR_THRESHOLD) {
            status->channel_locked = true;
            error_record(ERR_DMA_THRESHOLD);
            return ERR_DMA_THRESHOLD;
        }
    }
    
    return ERROR_CODE_SUCCESS;
}

// Monitor DMA timeouts
void dma_safety_monitor_timeouts(void) {
    if (!dma_safety.safety_enabled) {
        return;
    }
    
    uint32_t current_time = timer_get_ms();
    
    for (uint8_t channel = 0; channel < DMA_MAX_CHANNELS; channel++) {
        dma_safety_status_t* status = &dma_safety.channels[channel];
        
        // Skip inactive channels
        if (!dma_is_channel_active(channel)) {
            continue;
        }
        
        // Check for timeout
        if (current_time - status->last_activity >= DMA_SAFETY_TIMEOUT_MS) {
            status->timeout_count++;
            dma_safety.total_timeouts++;
            
            // Stop transfer and record error
            dma_stop_transfer(channel);
            error_record(ERR_DMA_TIMEOUT);
            
            // Update status
            status->last_activity = current_time;
        }
    }
}

// Reset DMA channel safety status
error_code_t dma_safety_reset_channel(uint8_t channel) {
    if (channel >= DMA_MAX_CHANNELS) {
        return ERR_INVALID_PARAM;
    }
    
    memset(&dma_safety.channels[channel], 0, sizeof(dma_safety_status_t));
    return ERROR_CODE_SUCCESS;
}

// Get DMA safety statistics
void dma_safety_get_stats(uint32_t* total_errors, uint32_t* total_timeouts) {
    if (total_errors) {
        *total_errors = dma_safety.total_errors;
    }
    if (total_timeouts) {
        *total_timeouts = dma_safety.total_timeouts;
    }
}

// Enhanced DMA interrupt handler with safety checks
void dma_safety_irq_handler(uint8_t channel) {
    if (channel >= DMA_MAX_CHANNELS) {
        return;
    }
    
    dma_safety_status_t* status = &dma_safety.channels[channel];
    bool has_error = false;
    
    // Critical section for checking DMA status
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        // Check for DMA errors
        if (DMA_CH_STATUS(channel) & DMA_ERROR_FLAG) {
            has_error = true;
            status->error_count++;
            dma_safety.total_errors++;
            
            // Clear error flags
            DMA_CH_STATUS(channel) |= DMA_ERROR_FLAG;
        }
        
        // Update activity timestamp
        status->last_activity = timer_get_ms();
    }
    
    // Handle errors
    if (has_error) {
        if (status->error_count >= DMA_ERROR_THRESHOLD) {
            status->channel_locked = true;
            error_record(ERR_DMA_THRESHOLD);
        } else if (status->retry_count < DMA_MAX_RETRIES) {
            // Attempt retry
            status->retry_count++;
            dma_restart_transfer(channel);
        } else {
            error_record(ERR_DMA_MAX_RETRIES);
        }
    } else {
        // Successful transfer - reset retry count
        status->retry_count = 0;
    }
}

// Check if DMA channel is safe to use
bool dma_safety_is_channel_safe(uint8_t channel) {
    if (channel >= DMA_MAX_CHANNELS) {
        return false;
    }
    
    return !dma_safety.channels[channel].channel_locked;
}

// Enable/disable safety monitoring
void dma_safety_set_enabled(bool enabled) {
    dma_safety.safety_enabled = enabled;
}

// Get safety monitoring state
bool dma_safety_is_enabled(void) {
    return dma_safety.safety_enabled;
}
