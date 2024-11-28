/**
 * Timer Driver Implementation
 * - Generic timer implementation with priority-based management
 * 
 * @author tchan@TSoft
 * @date 2024/01/01
 */
#include "drv_timer.h"
#include "drv_timer_common.h"
#include <util/atomic.h>
#include <string.h>
#include "../../../utils/common/error_codes.h"

// Maximum number of timer instances
#define MAX_TIMER_INSTANCES 4

// Timer instance state
typedef struct {
    timer_handle_t handle;
    struct {
        uint32_t counter;      // Current counter value
        uint32_t overflows;    // Number of overflows
        uint32_t matches;      // Number of compare matches
        bool running;          // Timer is currently running
        bool overflow;         // Overflow has occurred
        bool compare_match;    // Compare match has occurred
        void (*callback)(void*);
        void* callback_arg;
    } state;
} timer_instance_t;

// Static timer instances
static timer_instance_t timer_instances[MAX_TIMER_INSTANCES];
static bool timer_initialized = false;

// System time variables
static timer_handle_t system_timer;
static volatile uint32_t system_ms = 0;
static bool system_time_initialized = false;

// System timer callback
static void system_timer_callback(void* arg) {
    (void)arg;  // Unused parameter
    system_ms++;
}

// Initialize system timer for millisecond counting
error_code_t timer_init_system_time(const timer_systime_config_t* config) {
    if (!config || !config->hw_ops || !config->hw_hal) {
        return ERR_INVALID_PARAMETER;
    }

    if (system_time_initialized) {
        return ERR_ALREADY_EXISTS;
    }

    // Initialize timer subsystem if not already initialized
    error_code_t err = timer_init();
    if (err != ERR_SUCCESS && err != ERR_ALREADY_INITIALIZED) {
        return err;
    }

    // Configure timer for 1ms period
    timer_config_t timer_config = {
        .base = {
            .mode              = TIMER_MODE_NORMAL,
            .prescaler        = config->base.prescaler,
            .period           = 1,
            .expiry_callback  = NULL,  // Will be set through timer_set_callback
            .expiry_callback_arg = NULL
        },
        .hw_ops          = config->hw_ops,
        .hw_hal          = config->hw_hal,
        .specific_config = config->specific_config
    };

    // Allocate timer
    err = timer_alloc(&system_timer, &timer_config);
    if (err != ERR_SUCCESS) {
        return err;
    }

    // Set callback
    err = timer_set_callback(&system_timer, system_timer_callback, NULL);
    if (err != ERR_SUCCESS) {
        timer_free(&system_timer);
        return err;
    }

    // Start timer
    err = timer_start(&system_timer);
    if (err != ERR_SUCCESS) {
        timer_free(&system_timer);
        return err;
    }

    system_time_initialized = true;
    return ERR_SUCCESS;
}

// Get system time in milliseconds
uint32_t timer_get_ms(void) {
    uint32_t ms;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        ms = system_ms;
    }
    return ms;
}

// Interrupt notification from hardware layer
void timer_notify_interrupt(timer_interrupt_type_t int_type)
{
    // Find the timer instance that generated the interrupt
    for (int i = 0; i < MAX_TIMER_INSTANCES; i++) {
        timer_instance_t* instance = &timer_instances[i];
        if (instance->handle.initialized) {
            switch (int_type) {
                case TIMER_INT_COMPARE_MATCH:
                    instance->state.compare_match = true;
                    if (instance->state.callback) {
                        instance->state.callback(instance->state.callback_arg);
                    }
                    break;
                default:
                    break;
            }
        }
    }
}

// Initialize timer subsystem
error_code_t timer_init(void)
{
    if (timer_initialized) {
        return ERR_ALREADY_INITIALIZED;
    }

    // Initialize all timer instances
    memset(timer_instances, 0, sizeof(timer_instances));
    timer_initialized = true;

    return ERR_SUCCESS;
}

// Validate timer configuration
static error_code_t validate_timer_config(const timer_config_t* config)
{
    if (!config) {
        return ERR_INVALID_PARAMETER;
    }

    // Validate base configuration
    if (config->base.mode >= TIMER_MODE_INPUT_CAPTURE) {
        return ERR_TIMER_INVALID_MODE;
    }

    // Validate PWM configuration if in PWM mode
    if ((config->base.mode == TIMER_MODE_PWM || config->base.mode == TIMER_MODE_FAST_PWM) &&
        config->base.pwm_config.duty_cycle > config->base.period) {
        return ERR_TIMER_INVALID_PWM;
    }

    // Validate hardware interface
    if (!config->hw_ops || !config->hw_hal ||
        !config->hw_ops->init || !config->hw_ops->deinit ||
        !config->hw_ops->start || !config->hw_ops->stop ||
        !config->hw_ops->set_period || !config->hw_ops->get_counter ||
        !config->hw_ops->get_status) {
        return ERR_INVALID_PARAMETER;
    }

    return ERR_SUCCESS;
}

// Allocate a timer instance
error_code_t timer_alloc(timer_handle_t* handle, const timer_config_t* config)
{
    if (!timer_initialized) {
        return ERR_NOT_INITIALIZED;
    }

    if (!handle) {
        return ERR_INVALID_PARAMETER;
    }

    // Validate configuration
    error_code_t err = validate_timer_config(config);
    if (err != ERR_SUCCESS) {
        return err;
    }

    // Find free timer instance
    int free_id = -1;
    for (int i = 0; i < MAX_TIMER_INSTANCES; i++) {
        if (!timer_instances[i].handle.initialized) {
            free_id = i;
            break;
        }
    }

    if (free_id == -1) {
        return ERR_NO_RESOURCE;
    }

    // Initialize timer instance
    timer_instance_t* instance = &timer_instances[free_id];
    memcpy(&instance->handle.config, config, sizeof(timer_config_t));
    instance->handle.id = free_id;
    memset(&instance->state, 0, sizeof(instance->state));

    // Initialize hardware
    error_code_t init_err = config->hw_ops->init(config->specific_config);
    if (init_err != ERR_SUCCESS) {
        return init_err;
    }

    instance->handle.initialized = true;
    *handle = instance->handle;

    return ERR_SUCCESS;
}

// Validate timer handle
static error_code_t validate_handle(const timer_handle_t* handle)
{
    if (!handle || !handle->initialized) {
        return ERR_INVALID_PARAMETER;
    }
    if (handle->id >= MAX_TIMER_INSTANCES) {
        return ERR_INVALID_PARAMETER;
    }
    return ERR_SUCCESS;
}

// Free a timer instance
error_code_t timer_free(timer_handle_t* handle)
{
    error_code_t err = validate_handle(handle);
    if (err != ERR_SUCCESS) {
        return err;
    }

    // Stop timer if running
    if (timer_is_running(handle)) {
        err = timer_stop(handle);
        if (err != ERR_SUCCESS) {
            return err;
        }
    }

    // Deinitialize hardware
    if (handle->config.hw_ops && handle->config.hw_ops->deinit) {
        err = handle->config.hw_ops->deinit();
        if (err != ERR_SUCCESS) {
            return err;
        }
    }

    // Clear timer instance (this also clears the handle since it's part of the instance)
    memset(&timer_instances[handle->id], 0, sizeof(timer_instance_t));

    return ERR_SUCCESS;
}

// Start timer
error_code_t timer_start(timer_handle_t* handle)
{
    error_code_t err = validate_handle(handle);
    if (err != ERR_SUCCESS) {
        return err;
    }

    err = handle->config.hw_ops->start();
    if (err != ERR_SUCCESS) {
        return err;
    }

    // Update software state
    timer_instance_t* instance = &timer_instances[handle->id];
    handle->config.hw_hal->enter_atomic();
    instance->state.running = true;
    handle->config.hw_hal->exit_atomic();

    return ERR_SUCCESS;
}

// Stop timer
error_code_t timer_stop(timer_handle_t* handle)
{
    error_code_t err = validate_handle(handle);
    if (err != ERR_SUCCESS) {
        return err;
    }

    err = handle->config.hw_ops->stop();
    if (err != ERR_SUCCESS) {
        return err;
    }

    // Update software state
    timer_instance_t* instance = &timer_instances[handle->id];
    handle->config.hw_hal->enter_atomic();
    instance->state.running = false;
    handle->config.hw_hal->exit_atomic();

    return ERR_SUCCESS;
}

// Set timer period
error_code_t timer_set_period(timer_handle_t* handle, uint16_t period)
{
    error_code_t err = validate_handle(handle);
    if (err != ERR_SUCCESS) {
        return err;
    }

    return handle->config.hw_ops->set_period(period);
}

// Get timer counter value
error_code_t timer_get_counter(timer_handle_t* handle, uint16_t* counter)
{
    error_code_t err = validate_handle(handle);
    if (err != ERR_SUCCESS) {
        return err;
    }

    return handle->config.hw_ops->get_counter(counter);
}

// Set timer callback
error_code_t timer_set_callback(timer_handle_t* handle, void (*callback)(void*), void* arg)
{
    error_code_t err = validate_handle(handle);
    if (err != ERR_SUCCESS) {
        return err;
    }

    if (!callback) {
        return ERR_INVALID_PARAMETER;
    }

    err = handle->config.hw_ops->register_callback(callback, arg);
    if (err != ERR_SUCCESS) {
        return err;
    }

    // Update software state
    timer_instance_t* instance = &timer_instances[handle->id];
    handle->config.hw_hal->enter_atomic();
    instance->state.callback = callback;
    instance->state.callback_arg = arg;
    handle->config.hw_hal->exit_atomic();

    return ERR_SUCCESS;
}

// Get timer status
error_code_t timer_get_status(timer_handle_t* handle, timer_status_t* status)
{
    error_code_t err = validate_handle(handle);
    if (err != ERR_SUCCESS) {
        return err;
    }

    if (!status) {
        return ERR_INVALID_PARAMETER;
    }

    // Get hardware status
    err = handle->config.hw_ops->get_status(status);
    if (err != ERR_SUCCESS) {
        return err;
    }

    // Update with software state
    timer_instance_t* instance = &timer_instances[handle->id];
    handle->config.hw_hal->enter_atomic();
    status->running = instance->state.running;
    status->overflow = instance->state.overflow;
    status->compare_match = instance->state.compare_match;
    status->counter = instance->state.counter;
    status->overflows = instance->state.overflows;
    status->matches = instance->state.matches;
    handle->config.hw_hal->exit_atomic();

    return ERR_SUCCESS;
}

// Enter critical section
error_code_t timer_enter_critical(timer_handle_t* handle)
{
    if (!handle || !handle->initialized) {
        return ERR_INVALID_PARAMETER;
    }

    if (!handle->config.hw_hal || !handle->config.hw_hal->enter_atomic) {
        return ERR_INVALID_PARAMETER;
    }

    handle->config.hw_hal->enter_atomic();
    return ERR_SUCCESS;
}

// Exit critical section
error_code_t timer_exit_critical(timer_handle_t* handle)
{
    if (!handle || !handle->initialized) {
        return ERR_INVALID_PARAMETER;
    }

    if (!handle->config.hw_hal || !handle->config.hw_hal->exit_atomic) {
        return ERR_INVALID_PARAMETER;
    }

    handle->config.hw_hal->exit_atomic();
    return ERR_SUCCESS;
}

// Recover from error
error_code_t timer_recover(timer_handle_t* handle)
{
    if (!handle || !handle->initialized) {
        return ERR_INVALID_PARAMETER;
    }

    if (!handle->config.hw_ops || !handle->config.hw_ops->recover) {
        return ERR_INVALID_PARAMETER;
    }

    return handle->config.hw_ops->recover();
}

// Check if timer is initialized
bool timer_is_initialized(const timer_handle_t* handle)
{
    return handle && handle->initialized;
}

// Check if timer is running
bool timer_is_running(const timer_handle_t* handle)
{
    if (!timer_is_initialized(handle)) {
        return false;
    }

    timer_status_t status;
    // Create a non-const copy to pass to timer_get_status
    timer_handle_t mutable_handle = *handle;
    if (timer_get_status(&mutable_handle, &status) != ERR_SUCCESS) {
        return false;
    }

    return status.running;
}

// Process timer callbacks
void timer_process_callbacks(void)
{
    for (int i = 0; i < MAX_TIMER_INSTANCES; i++) {
        timer_instance_t* instance = &timer_instances[i];
        if (instance->handle.initialized && instance->state.callback) {
            timer_status_t status;
            if (timer_get_status(&instance->handle, &status) == ERR_SUCCESS) {
                if (status.compare_match) {
                    instance->state.callback(instance->state.callback_arg);
                }
            }
        }
    }
}