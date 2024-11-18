#include "drv_dma.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include "../../../utils/common/error_codes.h"

// DMA channel control blocks
static struct {
    dma_config_t config;
    dma_status_t status;
    bool initialized;
} dma_channels[DMA_MAX_CHANNELS];

// DMA registers for ATmega2560
#define DMA_BASE_ADDR     0x100
#define DMA_CH_CTRL(ch)   (*(volatile uint8_t*)(DMA_BASE_ADDR + ch * 0x10))
#define DMA_CH_STATUS(ch) (*(volatile uint8_t*)(DMA_BASE_ADDR + ch * 0x10 + 1))
#define DMA_CH_SRC_L(ch)  (*(volatile uint8_t*)(DMA_BASE_ADDR + ch * 0x10 + 2))
#define DMA_CH_SRC_H(ch)  (*(volatile uint8_t*)(DMA_BASE_ADDR + ch * 0x10 + 3))
#define DMA_CH_DST_L(ch)  (*(volatile uint8_t*)(DMA_BASE_ADDR + ch * 0x10 + 4))
#define DMA_CH_DST_H(ch)  (*(volatile uint8_t*)(DMA_BASE_ADDR + ch * 0x10 + 5))
#define DMA_CH_CNT_L(ch)  (*(volatile uint8_t*)(DMA_BASE_ADDR + ch * 0x10 + 6))
#define DMA_CH_CNT_H(ch)  (*(volatile uint8_t*)(DMA_BASE_ADDR + ch * 0x10 + 7))

// DMA control register bits
#define DMA_ENABLE        0x80
#define DMA_START         0x40
#define DMA_MODE_MASK     0x30
#define DMA_PRIO_MASK     0x0C
#define DMA_IRQ_ENABLE    0x02
#define DMA_ERROR_FLAG    0x01

error_code_t dma_init(void) {
    // Initialize DMA controller
    for (uint8_t i = 0; i < DMA_MAX_CHANNELS; i++) {
        dma_channels[i].initialized = false;
        dma_channels[i].status.busy = false;
        dma_channels[i].status.remaining = 0;
        dma_channels[i].status.transfers_complete = 0;
        dma_channels[i].status.errors = 0;
        
        // Clear DMA registers
        DMA_CH_CTRL(i) = 0;
        DMA_CH_STATUS(i) = 0;
    }
    
    // Enable DMA controller
    DMACR = DMA_ENABLE;
    
    return ERROR_CODE_SUCCESS;
}

error_code_t dma_deinit(void) {
    // Disable DMA controller
    DMACR = 0;
    
    // Stop all active transfers and verify
    error_code_t status = ERROR_CODE_SUCCESS;
    for (uint8_t i = 0; i < DMA_MAX_CHANNELS; i++) {
        if (dma_channels[i].initialized) {
            error_code_t channel_status = dma_stop_transfer(i);
            if (channel_status != ERROR_CODE_SUCCESS) {
                status = channel_status;
                error_record(ERR_DMA_STOP_FAILED);
                continue;
            }
            
            // Wait for channel to actually stop
            uint16_t timeout = 1000;  // 1 second timeout
            while (dma_is_transfer_active(i) && timeout > 0) {
                _delay_ms(1);
                timeout--;
            }
            
            if (timeout == 0) {
                status = ERR_DMA_TIMEOUT;
                error_record(ERR_DMA_STOP_TIMEOUT);
                continue;
            }
            
            // Clear channel configuration
            dma_channels[i].initialized = false;
            dma_channels[i].status.busy = false;
            memset(&dma_channels[i].config, 0, sizeof(dma_config_t));
        }
    }
    
    return status;
}

error_code_t dma_configure_channel(uint8_t channel, const dma_config_t* config) {
    if (channel >= DMA_MAX_CHANNELS || !config) {
        return ERR_INVALID_PARAM;
    }
    
    // Stop any ongoing transfer
    dma_stop_transfer(channel);
    
    // Store configuration
    dma_channels[channel].config = *config;
    dma_channels[channel].initialized = true;
    
    // Configure DMA channel
    uint8_t ctrl = DMA_ENABLE;
    ctrl |= (config->mode << 4) & DMA_MODE_MASK;
    ctrl |= (config->priority << 2) & DMA_PRIO_MASK;
    if (config->callback) {
        ctrl |= DMA_IRQ_ENABLE;
    }
    
    DMA_CH_CTRL(channel) = ctrl;
    
    // Set addresses
    uint16_t src_addr = (uint16_t)config->source;
    uint16_t dst_addr = (uint16_t)config->destination;
    
    DMA_CH_SRC_L(channel) = src_addr & 0xFF;
    DMA_CH_SRC_H(channel) = (src_addr >> 8) & 0xFF;
    DMA_CH_DST_L(channel) = dst_addr & 0xFF;
    DMA_CH_DST_H(channel) = (dst_addr >> 8) & 0xFF;
    
    // Set transfer size
    DMA_CH_CNT_L(channel) = config->buffer_size & 0xFF;
    DMA_CH_CNT_H(channel) = (config->buffer_size >> 8) & 0xFF;
    
    return ERROR_CODE_SUCCESS;
}

error_code_t dma_start_transfer(uint8_t channel) {
    if (channel >= DMA_MAX_CHANNELS || !dma_channels[channel].initialized) {
        return ERR_INVALID_PARAM;
    }
    
    if (dma_channels[channel].status.busy) {
        return ERR_BUSY;
    }
    
    // Start transfer
    DMA_CH_CTRL(channel) |= DMA_START;
    dma_channels[channel].status.busy = true;
    dma_channels[channel].status.remaining = dma_channels[channel].config.buffer_size;
    
    return ERROR_CODE_SUCCESS;
}

error_code_t dma_stop_transfer(uint8_t channel) {
    if (channel >= DMA_MAX_CHANNELS) {
        return ERR_INVALID_PARAM;
    }
    
    // Stop transfer
    DMA_CH_CTRL(channel) &= ~DMA_START;
    dma_channels[channel].status.busy = false;
    
    return ERROR_CODE_SUCCESS;
}

error_code_t dma_get_status(uint8_t channel, dma_status_t* status) {
    if (channel >= DMA_MAX_CHANNELS || !status) {
        return ERR_INVALID_PARAM;
    }
    
    *status = dma_channels[channel].status;
    return ERROR_CODE_SUCCESS;
}

bool dma_is_transfer_active(uint8_t channel) {
    return dma_channels[channel].status.busy;
}

void dma_irq_handler(uint8_t channel) {
    if (channel >= DMA_MAX_CHANNELS) {
        return;
    }
    
    // Check for errors
    if (DMA_CH_STATUS(channel) & DMA_ERROR_FLAG) {
        dma_channels[channel].status.errors++;
        DMA_CH_STATUS(channel) |= DMA_ERROR_FLAG; // Clear error flag
    }
    
    // Update status
    dma_channels[channel].status.transfers_complete++;
    dma_channels[channel].status.busy = false;
    
    // Call completion callback if configured
    if (dma_channels[channel].config.callback) {
        dma_channels[channel].config.callback();
    }
    
    // Restart transfer if in circular mode
    if (dma_channels[channel].config.circular_mode) {
        dma_start_transfer(channel);
    }
}

error_code_t dma_error_recovery(dma_channel_t channel) {
    if (channel >= DMA_MAX_CHANNELS) {
        return ERR_DMA_INVALID_CHANNEL;
    }

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        // Stop current transfer
        REG_DMA_CTRL &= ~(1 << channel);
        
        // Clear error flags
        REG_DMA_STATUS |= (DMA_ERR_MASK << (channel * DMA_STATUS_BITS));
        
        // Reset channel configuration
        dma_channels[channel].status = DMA_STATUS_IDLE;
        dma_channels[channel].transfer_count = 0;
        
        // Reset DMA registers for this channel
        REG_DMA_SRC(channel) = 0;
        REG_DMA_DEST(channel) = 0;
        REG_DMA_COUNT(channel) = 0;
        
        // Clear pending interrupts
        REG_DMA_INT_CLR |= (1 << channel);
    }
    
    return ERR_OK;
}

error_code_t dma_init_channel(dma_channel_t channel, const dma_config_t* config) {
    if (channel >= DMA_MAX_CHANNELS || !config) {
        return ERR_DMA_INVALID_PARAM;
    }
    
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        // Stop any ongoing transfer
        REG_DMA_CTRL &= ~(1 << channel);
        
        // Configure channel
        dma_channels[channel].config = *config;
        dma_channels[channel].status = DMA_STATUS_IDLE;
        dma_channels[channel].transfer_count = 0;
        dma_channels[channel].error_count = 0;
        
        // Set up DMA registers
        REG_DMA_SRC(channel) = config->src_addr;
        REG_DMA_DEST(channel) = config->dest_addr;
        REG_DMA_COUNT(channel) = config->count;
        
        // Configure channel control register
        uint32_t ctrl = 0;
        ctrl |= (config->src_increment ? DMA_CTRL_SRC_INC : 0);
        ctrl |= (config->dest_increment ? DMA_CTRL_DEST_INC : 0);
        ctrl |= (config->size << DMA_CTRL_SIZE_SHIFT);
        ctrl |= (config->priority << DMA_CTRL_PRIO_SHIFT);
        REG_DMA_CH_CTRL(channel) = ctrl;
        
        // Enable error interrupts
        REG_DMA_INT_EN |= (DMA_INT_ERR_MASK << (channel * DMA_INT_BITS));
    }
    
    return ERR_OK;
}

ISR(DMA_ERROR_VECT) {
    uint32_t status = REG_DMA_STATUS;
    uint32_t active_channels = REG_DMA_CTRL;
    
    for (uint8_t channel = 0; channel < DMA_MAX_CHANNELS; channel++) {
        if (status & (DMA_ERR_MASK << (channel * DMA_STATUS_BITS))) {
            // Record error
            dma_channels[channel].error_count++;
            error_record(ERR_DMA_TRANSFER_ERROR);
            
            // Stop transfer
            REG_DMA_CTRL &= ~(1 << channel);
            
            // Attempt recovery
            if (dma_channels[channel].error_count < DMA_MAX_RETRIES) {
                // Reset and restart transfer
                dma_error_recovery(channel);
                dma_start_transfer(channel);
            } else {
                // Too many errors, notify system
                error_record(ERR_DMA_FATAL_ERROR);
                dma_channels[channel].status = DMA_STATUS_ERROR;
            }
        }
    }
}
