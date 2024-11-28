#include "safe_monitor.h"
#include <string.h>
#include "../../drivers/mcu/timer/drv_timer1.h"
#include "../../platform/avr/plat_avr.h"

// Internal state tracking
static struct {
    safety_status_t system_status[SAFETY_SYS_COUNT];
    safety_threshold_t thresholds[SAFETY_SYS_COUNT];
    safety_stats_t stats[SAFETY_SYS_COUNT];
    safety_callback_t status_callback;
    bool emergency_stop;
    uint8_t active_warnings;
    uint8_t active_criticals;
    bool initialized;
    safety_status_t last_status;
} safety_state;

// Timer ID for safety monitoring
static uint8_t tid_safety;

// Default thresholds
static const safety_threshold_t default_thresholds = {
    .warning_threshold = 75,     // 75% of max
    .critical_threshold = 90,    // 90% of max
    .recovery_threshold = 60,    // 60% of max
    .max_warnings = 3,
    .warning_timeout_ms = 60000  // 1 minute
};

void safety_init(void)
{
    memset(&safety_state, 0, sizeof(safety_state));
    
    // Initialize all subsystems with default thresholds
    for (int i = 0; i < SAFETY_SYS_COUNT; i++) {
        safety_state.system_status[i] = SAFETY_OK;
        safety_state.thresholds[i] = default_thresholds;
        safety_state.stats[i].consecutive_warnings = 0;
        safety_state.stats[i].total_warnings = 0;
        safety_state.stats[i].total_criticals = 0;
    }
    
    // Allocate timer with error handling
    tid_safety = timer_alloc();
    if (tid_safety == 0xFF) {
        error_record(ERR_TIMER_ALLOC);
        return;
    }
    
    // Set up watchdog monitoring
    plat_avr_watchdog_enable(WDTO_2S);
    
    // Initialize safety thresholds
    safety_state.thresholds[SAFETY_SYS_MOTOR].temp_max = MOTOR_TEMP_MAX;
    safety_state.thresholds[SAFETY_SYS_MOTOR].current_max = MOTOR_CURRENT_MAX;
    safety_state.thresholds[SAFETY_SYS_POWER].voltage_min = POWER_VOLTAGE_MIN;
    safety_state.thresholds[SAFETY_SYS_POWER].voltage_max = POWER_VOLTAGE_MAX;
    
    safety_state.initialized = true;
    safety_state.emergency_stop = false;
    safety_state.active_warnings = 0;
    safety_state.active_criticals = 0;
    safety_state.last_status = SAFETY_OK;
}

void safety_deinit(void)
{
    if (tid_safety != 0xFF) {
        timer_free(tid_safety);
        tid_safety = 0xFF;
    }
    safety_state.initialized = false;
}

void safety_process_cycle(void)
{
    if (!safety_state.initialized) {
        return;
    }
    
    uint32_t current_time = timer_get_ms();
    
    // Process each subsystem
    for (safety_system_t sys = 0; sys < SAFETY_SYS_COUNT; sys++) {
        // Get current measurements
        uint16_t current_value = get_system_measurement(sys);
        safety_threshold_t* threshold = &safety_state.thresholds[sys];
        safety_stats_t* stats = &safety_state.stats[sys];
        safety_status_t old_status = safety_state.system_status[sys];
        safety_status_t new_status = old_status;
        
        // Check thresholds
        if (current_value >= threshold->critical_threshold) {
            new_status = SAFETY_CRITICAL;
            stats->total_criticals++;
            stats->last_critical_time = current_time;
            safety_state.active_criticals++;
        }
        else if (current_value >= threshold->warning_threshold) {
            new_status = SAFETY_WARNING;
            stats->total_warnings++;
            stats->last_warning_time = current_time;
            stats->consecutive_warnings++;
            safety_state.active_warnings++;
            
            // Check for excessive consecutive warnings
            if (stats->consecutive_warnings >= threshold->max_warnings) {
                new_status = SAFETY_CRITICAL;
                stats->total_criticals++;
                stats->last_critical_time = current_time;
                safety_state.active_criticals++;
            }
        }
        else if (current_value <= threshold->recovery_threshold) {
            // Only clear warning/critical if value drops below recovery threshold
            if (old_status != SAFETY_OK) {
                new_status = SAFETY_OK;
                stats->consecutive_warnings = 0;
                if (old_status == SAFETY_WARNING) {
                    safety_state.active_warnings--;
                } else {
                    safety_state.active_criticals--;
                }
            }
        }
        
        // Update status and notify if changed
        if (new_status != old_status) {
            safety_state.system_status[sys] = new_status;
            if (safety_state.status_callback) {
                safety_state.status_callback(sys, new_status);
            }
            
            // Handle critical conditions
            if (new_status == SAFETY_CRITICAL) {
                error_record(ERR_SAFETY_CRITICAL);
                if (safety_emergency_stop_required()) {
                    safety_trigger_emergency_stop();
                }
            }
        }
    }
}

void safety_process(void)
{
    if (!safety_state.initialized) {
        return;
    }

    uint32_t current_time = timer_get_ms();
    
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        // Enhanced motor monitoring with trend analysis
        static int16_t prev_motor_temp = 0;
        static uint32_t temp_rise_start = 0;
        int16_t motor_temp = motor_get_temperature();
        
        // Check for rapid temperature rise
        if (motor_temp > prev_motor_temp + TEMP_RISE_THRESHOLD) {
            if (temp_rise_start == 0) {
                temp_rise_start = current_time;
            } else if (current_time - temp_rise_start > TEMP_RISE_DURATION) {
                safety_trigger_warning(SAFETY_SYS_MOTOR);
                log_safety_event(SAFETY_EVENT_TEMP_RISE);
            }
        } else {
            temp_rise_start = 0;
        }
        
        // Standard temperature checks with hysteresis
        if (motor_temp > safety_state.thresholds[SAFETY_SYS_MOTOR].temp_max) {
            if (!safety_state.warnings[SAFETY_SYS_MOTOR]) {
                safety_trigger_warning(SAFETY_SYS_MOTOR);
                log_safety_event(SAFETY_EVENT_TEMP_WARNING);
            }
            if (motor_temp > safety_state.thresholds[SAFETY_SYS_MOTOR].temp_critical) {
                safety_trigger_critical(SAFETY_SYS_MOTOR);
                log_safety_event(SAFETY_EVENT_TEMP_CRITICAL);
            }
        } else if (motor_temp < safety_state.thresholds[SAFETY_SYS_MOTOR].temp_max - TEMP_HYSTERESIS) {
            safety_clear_warning(SAFETY_SYS_MOTOR);
        }
        
        prev_motor_temp = motor_temp;
        
        // Enhanced current monitoring with spike detection
        uint16_t motor_current = motor_get_current();
        static uint16_t current_samples[CURRENT_SAMPLE_COUNT] = {0};
        static uint8_t sample_index = 0;
        
        current_samples[sample_index] = motor_current;
        sample_index = (sample_index + 1) % CURRENT_SAMPLE_COUNT;
        
        // Calculate moving average
        uint32_t avg_current = 0;
        for (uint8_t i = 0; i < CURRENT_SAMPLE_COUNT; i++) {
            avg_current += current_samples[i];
        }
        avg_current /= CURRENT_SAMPLE_COUNT;
        
        // Check for current spikes
        if (motor_current > avg_current * CURRENT_SPIKE_THRESHOLD) {
            safety_trigger_warning(SAFETY_SYS_MOTOR);
            log_safety_event(SAFETY_EVENT_CURRENT_SPIKE);
            
            if (motor_current > safety_state.thresholds[SAFETY_SYS_MOTOR].current_max) {
                safety_trigger_critical(SAFETY_SYS_MOTOR);
                log_safety_event(SAFETY_EVENT_CURRENT_CRITICAL);
            }
        }
        
        // Enhanced power supply monitoring
        uint16_t voltage = power_get_voltage();
        static uint16_t voltage_samples[VOLTAGE_SAMPLE_COUNT] = {0};
        static uint8_t voltage_index = 0;
        
        voltage_samples[voltage_index] = voltage;
        voltage_index = (voltage_index + 1) % VOLTAGE_SAMPLE_COUNT;
        
        // Calculate voltage stability
        uint16_t min_voltage = voltage_samples[0];
        uint16_t max_voltage = voltage_samples[0];
        for (uint8_t i = 1; i < VOLTAGE_SAMPLE_COUNT; i++) {
            if (voltage_samples[i] < min_voltage) min_voltage = voltage_samples[i];
            if (voltage_samples[i] > max_voltage) max_voltage = voltage_samples[i];
        }
        
        // Check for voltage instability
        if (max_voltage - min_voltage > VOLTAGE_STABILITY_THRESHOLD) {
            safety_trigger_warning(SAFETY_SYS_POWER);
            log_safety_event(SAFETY_EVENT_VOLTAGE_UNSTABLE);
        }
        
        // Absolute voltage checks
        if (voltage < SUPPLY_VOLTAGE_MIN_MV || voltage > SUPPLY_VOLTAGE_MAX_MV) {
            safety_trigger_critical(SAFETY_SYS_POWER);
            log_safety_event(SAFETY_EVENT_VOLTAGE_CRITICAL);
        }
        
        // Enhanced communication monitoring with error pattern detection
        static uint32_t comm_errors[NUM_COMM_CHANNELS] = {0};
        for (int i = 0; i < NUM_COMM_CHANNELS; i++) {
            if (current_time - comm_last_rx_time[i] > COMM_TIMEOUT_MS) {
                comm_errors[i]++;
                
                if (comm_errors[i] > COMM_ERROR_THRESHOLD) {
                    safety_trigger_warning(SAFETY_SYS_COMM);
                    log_safety_event(SAFETY_EVENT_COMM_ERROR);
                }
                
                if (current_time - comm_last_rx_time[i] > COMM_CRITICAL_TIMEOUT_MS) {
                    safety_trigger_critical(SAFETY_SYS_COMM);
                    log_safety_event(SAFETY_EVENT_COMM_CRITICAL);
                }
            } else {
                comm_errors[i] = 0;  // Reset error counter on successful communication
            }
        }
        
        // Update system status with enhanced error aggregation
        safety_status_t new_status = safety_get_system_status();
        if (new_status != safety_state.last_status) {
            if (safety_state.status_callback) {
                safety_state.status_callback(new_status);
            }
            safety_state.last_status = new_status;
            
            // Log status change with detailed diagnostics
            log_safety_event_extended(new_status, voltage, motor_temp, motor_current);
        }
        
        // Enhanced emergency stop logic with multiple trigger conditions
        if (safety_emergency_stop_required() || 
            motor_temp > EMERGENCY_TEMP_THRESHOLD ||
            motor_current > EMERGENCY_CURRENT_THRESHOLD ||
            voltage < EMERGENCY_VOLTAGE_MIN ||
            voltage > EMERGENCY_VOLTAGE_MAX) {
            
            safety_trigger_emergency_stop();
            error_record(ERR_SAFETY_EMERGENCY_STOP);
            log_safety_event(SAFETY_EVENT_EMERGENCY_STOP);
            
            // Notify all subsystems of emergency stop
            notify_emergency_stop();
        }
    }
}

// Enhanced safety event logging with extended diagnostics
static void log_safety_event_extended(safety_status_t status, uint16_t voltage, 
                                    int16_t temp, uint16_t current) {
    safety_event_t event = {
        .timestamp = timer_get_ms(),
        .status = status,
        .voltage = voltage,
        .temperature = temp,
        .current = current,
        .error_flags = error_get_flags(),
        .warning_flags = safety_state.warning_flags,
        .critical_flags = safety_state.critical_flags
    };
    
    // Store event in circular buffer with overflow protection
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        safety_events[safety_event_index] = event;
        safety_event_index = (safety_event_index + 1) % MAX_SAFETY_EVENTS;
        
        if (safety_event_count < MAX_SAFETY_EVENTS) {
            safety_event_count++;
        }
    }
}

// Notify all subsystems of emergency stop
static void notify_emergency_stop(void) {
    // Disable all motor outputs
    motor_emergency_stop();
    
    // Disable all power outputs
    power_emergency_shutdown();
    
    // Put all subsystems in safe state
    system_safe_state();
    
    // Record diagnostic information
    diagnostic_record_emergency();
}

safety_status_t safety_get_status(safety_system_t system)
{
    if (system >= SAFETY_SYS_COUNT) {
        return SAFETY_CRITICAL;
    }
    return safety_state.system_status[system];
}

safety_status_t safety_get_system_status(void)
{
    if (safety_state.active_criticals > 0) {
        return SAFETY_CRITICAL;
    }
    if (safety_state.active_warnings > 0) {
        return SAFETY_WARNING;
    }
    return SAFETY_OK;
}

void safety_configure_thresholds(safety_system_t system, const safety_threshold_t* thresholds)
{
    if (system < SAFETY_SYS_COUNT && thresholds != NULL) {
        safety_state.thresholds[system] = *thresholds;
    }
}

const safety_stats_t* safety_get_stats(safety_system_t system)
{
    if (system >= SAFETY_SYS_COUNT) {
        return NULL;
    }
    return &safety_state.stats[system];
}

void safety_reset_stats(safety_system_t system)
{
    if (system < SAFETY_SYS_COUNT) {
        memset(&safety_state.stats[system], 0, sizeof(safety_stats_t));
    }
}

bool safety_emergency_stop_required(void)
{
    return (safety_state.active_criticals > 0);
}

void safety_acknowledge_warnings(safety_system_t system)
{
    if (system < SAFETY_SYS_COUNT && 
        safety_state.system_status[system] == SAFETY_WARNING) {
        safety_state.stats[system].consecutive_warnings = 0;
        safety_state.active_warnings--;
        safety_state.system_status[system] = SAFETY_OK;
    }
}

void safety_register_callback(safety_callback_t callback)
{
    safety_state.status_callback = callback;
}

void safety_trigger_emergency_stop(void)
{
    safety_state.emergency_stop = true;
    error_record(ERR_EMERGENCY_STOP);
    rc_emergency_stop();
}

void safety_clear_emergency_stop(void)
{
    if (safety_get_system_status() != SAFETY_CRITICAL) {
        safety_state.emergency_stop = false;
        rc_clear_emergency_stop();
    }
}

bool safety_is_emergency_stopped(void)
{
    return safety_state.emergency_stop;
}

void safety_trigger_warning(safety_system_t system)
{
    safety_state.system_status[system] = SAFETY_WARNING;
    safety_state.stats[system].total_warnings++;
    safety_state.stats[system].last_warning_time = timer_get_ms();
    safety_state.stats[system].consecutive_warnings++;
    safety_state.active_warnings++;
}

void safety_trigger_critical(safety_system_t system)
{
    if (system >= SAFETY_SYS_COUNT) {
        error_record(ERR_INVALID_PARAM);
        return;
    }

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        // Update system status
        safety_state.system_status[system] = SAFETY_CRITICAL;
        safety_state.stats[system].total_criticals++;
        safety_state.stats[system].last_critical_time = timer_get_ms();
        safety_state.active_criticals++;

        // Log the critical event with extended diagnostics
        uint16_t voltage = get_system_measurement(system);
        int16_t temp = get_temperature();  // Assuming this function exists
        uint16_t current = get_current();  // Assuming this function exists
        log_safety_event_extended(SAFETY_CRITICAL, voltage, temp, current);

        // Notify registered callback if any
        if (safety_state.status_callback) {
            safety_state.status_callback(system, SAFETY_CRITICAL);
        }

        // If this is the first critical error, trigger emergency procedures
        if (safety_state.active_criticals == 1) {
            safety_trigger_emergency_stop();
            notify_emergency_stop();
        }
    }
}

// Internal helper function to get measurements for each subsystem
static uint16_t get_system_measurement(safety_system_t system)
{
    switch (system) {
        case SAFETY_SYS_MECHANICAL:
            return rc_get_motor_load();
            
        case SAFETY_SYS_ELECTRICAL:
            return rc_get_current_draw();
            
        case SAFETY_SYS_THERMAL:
            return rc_get_temperature();
            
        case SAFETY_SYS_FLUID:
            return rc_get_fluid_pressure();
            
        case SAFETY_SYS_MOTION:
            return rc_get_motion_speed();
            
        default:
            return 0;
    }
}

// Helper function to log safety events
static void log_safety_event(safety_status_t status) {
    safety_event_t event = {
        .timestamp = timer_get_ms(),
        .status = status,
        .voltage = power_get_voltage(),
        .temperature = motor_get_temperature(),
        .current = motor_get_current()
    };
    
    // Store event in circular buffer
    safety_events[safety_event_index] = event;
    safety_event_index = (safety_event_index + 1) % MAX_SAFETY_EVENTS;
}
