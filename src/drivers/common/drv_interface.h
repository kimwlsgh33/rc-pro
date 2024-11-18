/**
 * @file drv_interface.h
 * @brief Common driver interface definitions
 */

#ifndef DRV_INTERFACE_H
#define DRV_INTERFACE_H

#include "../../utils/common/error_codes.h"

/**
 * @brief Generic driver configuration structure
 * This should be cast to the appropriate driver-specific config
 */
typedef struct {
    uint32_t version;    // Driver version
    void* config;        // Driver specific configuration
} driver_config_t;

/**
 * @brief Generic driver status structure
 * This should be cast to the appropriate driver-specific status
 */
typedef struct {
    uint32_t state;      // Current driver state
    uint32_t error;      // Last error code
    void* status;        // Driver specific status
} driver_status_t;

/**
 * @brief Common driver interface
 * All drivers should implement these functions
 */
typedef struct {
    // Core functions
    error_code_t (*init)(void);
    error_code_t (*deinit)(void);
    error_code_t (*configure)(const driver_config_t* config);
    error_code_t (*get_status)(driver_status_t* status);
    
    // Optional functions
    error_code_t (*start)(void);
    error_code_t (*stop)(void);
    error_code_t (*reset)(void);
    error_code_t (*suspend)(void);
    error_code_t (*resume)(void);
    
    // Interrupt handling
    void (*irq_handler)(void* context);
    void* irq_context;
} driver_interface_t;

/**
 * @brief Driver states
 */
typedef enum {
    DRIVER_STATE_UNINITIALIZED = 0,
    DRIVER_STATE_INITIALIZED,
    DRIVER_STATE_CONFIGURED,
    DRIVER_STATE_RUNNING,
    DRIVER_STATE_SUSPENDED,
    DRIVER_STATE_ERROR
} driver_state_t;

/**
 * @brief Driver capabilities
 */
typedef enum {
    DRIVER_CAP_NONE = 0x00,
    DRIVER_CAP_IRQ = 0x01,        // Supports interrupts
    DRIVER_CAP_DMA = 0x02,        // Supports DMA
    DRIVER_CAP_SUSPEND = 0x04,    // Supports suspend/resume
    DRIVER_CAP_CONFIG = 0x08      // Supports runtime configuration
} driver_capabilities_t;

/**
 * @brief Driver registration structure
 */
typedef struct {
    const char* name;                     // Driver name
    uint32_t version;                     // Driver version
    driver_capabilities_t capabilities;    // Driver capabilities
    const driver_interface_t* interface;   // Driver interface
} driver_registration_t;

#endif // DRV_INTERFACE_H
