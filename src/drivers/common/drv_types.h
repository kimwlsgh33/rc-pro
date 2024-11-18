/**
 * @file drv_types.h
 * @brief Common driver type definitions
 */

#ifndef DRV_TYPES_H
#define DRV_TYPES_H

#include <stdint.h>
#include "../../platform/common/plat_types.h"
#include "../../utils/common/error_codes.h"

/**
 * @brief Driver initialization flags
 */
typedef enum {
    DRIVER_INIT_NORMAL = 0x00,    // Normal initialization
    DRIVER_INIT_RESET = 0x01,     // Reset device during init
    DRIVER_INIT_NO_IRQ = 0x02,    // Initialize without interrupts
    DRIVER_INIT_DEBUG = 0x04      // Enable debug features
} driver_init_flags_t;

/**
 * @brief Driver operation modes
 */
typedef enum {
    DRIVER_MODE_NORMAL = 0,       // Normal operation mode
    DRIVER_MODE_LOW_POWER,        // Low power mode
    DRIVER_MODE_HIGH_PERFORMANCE, // High performance mode
    DRIVER_MODE_DEBUG            // Debug mode
} driver_mode_t;

/**
 * @brief Driver statistics
 */
typedef struct {
    uint32_t interrupts;          // Number of interrupts handled
    uint32_t errors;              // Number of errors encountered
    uint32_t overruns;           // Number of buffer overruns
    uint32_t underruns;          // Number of buffer underruns
    uint32_t timeouts;           // Number of timeouts
    uint32_t bytes_processed;    // Number of bytes processed
} driver_stats_t;

/**
 * @brief Driver buffer configuration
 */
typedef struct {
    void* buffer;                // Buffer pointer
    uint32_t size;              // Buffer size
    uint32_t threshold;         // Buffer threshold for notifications
    uint8_t flags;              // Buffer flags
} driver_buffer_t;

/**
 * @brief Driver timing configuration
 */
typedef struct {
    uint32_t timeout_ms;        // Operation timeout in milliseconds
    uint32_t retry_count;       // Number of retries
    uint32_t retry_delay_ms;    // Delay between retries
} driver_timing_t;

/**
 * @brief Driver callback configuration
 */
typedef struct {
    void (*data_callback)(void* data, uint32_t size);    // Data ready callback
    void (*error_callback)(error_code_t error);          // Error callback
    void (*event_callback)(uint32_t event);             // Event callback
    void* context;                                      // Callback context
} driver_callbacks_t;

/**
 * @brief Common driver configuration
 */
typedef struct {
    driver_init_flags_t init_flags;    // Initialization flags
    driver_mode_t mode;                // Operation mode
    driver_buffer_t rx_buffer;         // Receive buffer configuration
    driver_buffer_t tx_buffer;         // Transmit buffer configuration
    driver_timing_t timing;            // Timing configuration
    driver_callbacks_t callbacks;       // Callback configuration
} driver_config_t;

#endif // DRV_TYPES_H
