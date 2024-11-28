#ifndef DRV_TIMER_H
#define DRV_TIMER_H

#include "drv_timer_common.h"
#include <stdint.h>
#include <stdbool.h>

// Timer configuration structure
typedef struct {
    timer_base_config_t     base;            // Base timer configuration
    const timer_ops_t      *hw_ops;          // Hardware timer operations
    const timer_hal_t      *hw_hal;          // Hardware abstraction layer
    void                   *specific_config;  // Pointer to timer-specific config (if needed)
} timer_config_t;

// Timer instance handle
typedef struct {
    int                     id;              // Timer ID
    timer_config_t          config;          // Timer configuration
    timer_status_t          status;          // Current timer status
    bool                    initialized;      // Initialization flag
} timer_handle_t;

// Timer API functions
error_code_t timer_init(void);
error_code_t timer_alloc(timer_handle_t* handle, const timer_config_t* config);
error_code_t timer_free(timer_handle_t* handle);
error_code_t timer_start(timer_handle_t* handle);
error_code_t timer_stop(timer_handle_t* handle);
error_code_t timer_set_period(timer_handle_t* handle, uint16_t period);
error_code_t timer_get_counter(timer_handle_t* handle, uint16_t* counter);
error_code_t timer_get_status(timer_handle_t* handle, timer_status_t* status);
error_code_t timer_set_callback(timer_handle_t* handle, void (*callback)(void*), void* arg);

// System time initialization function
error_code_t timer_init_system_time(const timer_systime_config_t* config);

// Timer utility functions
bool timer_is_initialized(const timer_handle_t* handle);
bool timer_is_running(const timer_handle_t* handle);

// Get current system time in milliseconds
uint32_t timer_get_ms(void);

#endif // DRV_TIMER_H
