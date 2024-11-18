#ifndef DRV_DMA_H
#define DRV_DMA_H

#include "../../common/drv_types.h"
#include "../../../utils/common/error_codes.h"

// DMA channel definitions for ATmega2560
#define DMA_MAX_CHANNELS     4
#define DMA_UART_TX_CHANNEL  0
#define DMA_UART_RX_CHANNEL  1

// DMA transfer modes
typedef enum {
    DMA_MODE_SINGLE = 0,    // Single block transfer
    DMA_MODE_CIRCULAR = 1,  // Circular buffer mode
    DMA_MODE_DOUBLE = 2     // Double buffer mode
} dma_mode_t;

// DMA priority levels
typedef enum {
    DMA_PRIORITY_LOW = 0,
    DMA_PRIORITY_MEDIUM = 1,
    DMA_PRIORITY_HIGH = 2,
    DMA_PRIORITY_VERY_HIGH = 3
} dma_priority_t;

// DMA configuration structure
typedef struct {
    uint8_t channel;            // DMA channel number
    dma_mode_t mode;           // Transfer mode
    dma_priority_t priority;   // Channel priority
    uint16_t buffer_size;      // Transfer buffer size
    void* source;              // Source address
    void* destination;         // Destination address
    bool circular_mode;        // Enable circular mode
    void (*callback)(void);    // Transfer complete callback
} dma_config_t;

// DMA status structure
typedef struct {
    bool busy;                 // Channel is busy
    uint16_t remaining;        // Remaining transfers
    uint32_t transfers_complete; // Total completed transfers
    uint32_t errors;           // Error count
} dma_status_t;

// Function declarations
error_code_t dma_init(void);
error_code_t dma_deinit(void);
error_code_t dma_configure_channel(uint8_t channel, const dma_config_t* config);
error_code_t dma_start_transfer(uint8_t channel);
error_code_t dma_stop_transfer(uint8_t channel);
error_code_t dma_get_status(uint8_t channel, dma_status_t* status);
void dma_irq_handler(uint8_t channel);

#endif // DRV_DMA_H
