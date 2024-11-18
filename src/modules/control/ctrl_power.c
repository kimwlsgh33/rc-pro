#include "power_management.h"
#include "putil.h"
#include "timer.h"
#include <string.h>

// Default configuration values
#define DEFAULT_IDLE_TIMEOUT     30000   // 30 seconds
#define DEFAULT_STANDBY_TIMEOUT  300000  // 5 minutes
#define DEFAULT_SLEEP_TIMEOUT    900000  // 15 minutes
#define DEFAULT_ECO_THRESHOLD    50      // 50% activity
#define DEFAULT_LOW_POWER        20      // 20% power remaining

static PowerConfig power_config;
static PowerStats power_stats;
static uint8_t power_state = PWR_STATE_ACTIVE;
static int power_timer_id = -1;
static uint32_t last_activity_time = 0;
static uint32_t state_entry_time = 0;
static uint32_t last_error_time = 0;
static uint8_t power_error_count = 0;
static uint8_t power_recovery_attempts = 0;
static uint8_t recovery_state = RECOVERY_NONE;
static uint8_t component_power_state[NUM_POWER_COMPONENTS];
static bool power_initialized = false;

// Initialize power management system
void pwr_power_init(void) {
    // Clear power statistics
    memset(&power_stats, 0, sizeof(PowerStats));
    
    // Initialize power timer with error handling
    power_timer_id = timer_alloc();
    if (power_timer_id == 0xFF) {
        error_record(ERR_TIMER_ALLOC);
        return;
    }
    
    // Set default configuration with bounds checking
    power_config.idle_timeout_ms = DEFAULT_IDLE_TIMEOUT;
    power_config.standby_timeout_ms = DEFAULT_STANDBY_TIMEOUT;
    power_config.sleep_timeout_ms = DEFAULT_SLEEP_TIMEOUT;
    power_config.eco_threshold = DEFAULT_ECO_THRESHOLD;
    power_config.low_power_threshold = DEFAULT_LOW_POWER;
    
    // Initialize power state tracking
    power_state = PWR_STATE_ACTIVE;
    power_error_count = 0;
    power_recovery_attempts = 0;
    
    // Record initial timestamps
    state_entry_time = timer_get(power_timer_id);
    last_activity_time = state_entry_time;
    last_error_time = 0;
    
    // Initialize power monitoring
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        // Configure ADC for power monitoring
        ADMUX = (1 << REFS0);  // AVCC reference
        ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);  // Enable ADC, prescaler 128
        
        // Configure power control pins
        POWER_CTRL_DDR |= (1 << POWER_CTRL_PIN);  // Set as output
        POWER_CTRL_PORT &= ~(1 << POWER_CTRL_PIN);  // Initially off
    }
    
    // Initialize power recovery system
    recovery_state = RECOVERY_NONE;
    memset(component_power_state, 0, sizeof(component_power_state));
    
    power_initialized = true;
}

void pwr_set_config(PowerConfig* config) {
    memcpy(&power_config, config, sizeof(PowerConfig));
}

void pwr_get_config(PowerConfig* config) {
    memcpy(config, &power_config, sizeof(PowerConfig));
}

static void update_power_stats(void) {
    uint32_t current_time = timer_get(power_timer_id);
    uint32_t state_duration = current_time - state_entry_time;
    
    switch (power_state) {
        case PWR_STATE_ACTIVE:
            power_stats.active_time += state_duration;
            break;
        case PWR_STATE_IDLE:
            power_stats.idle_time += state_duration;
            break;
        case PWR_STATE_STANDBY:
            power_stats.standby_time += state_duration;
            break;
        case PWR_STATE_SLEEP:
            power_stats.sleep_time += state_duration;
            break;
    }
    
    state_entry_time = current_time;
}

void pwr_transition_state(uint8_t state) {
    if (state == power_state) return;
    
    update_power_stats();
    
    // Perform state transition actions
    switch (state) {
        case PWR_STATE_ACTIVE:
            pwr_motor_power_control(PWR_MODE_FULL);
            pwr_sensor_power_control(PWR_MODE_FULL);
            pwr_display_power_control(PWR_MODE_FULL);
            pwr_communication_power_control(PWR_MODE_FULL);
            break;
            
        case PWR_STATE_IDLE:
            pwr_motor_power_control(PWR_MODE_ECO);
            pwr_sensor_power_control(PWR_MODE_ECO);
            pwr_display_power_control(PWR_MODE_ECO);
            pwr_communication_power_control(PWR_MODE_FULL);
            break;
            
        case PWR_STATE_STANDBY:
            pwr_motor_power_control(PWR_MODE_LOW);
            pwr_sensor_power_control(PWR_MODE_LOW);
            pwr_display_power_control(PWR_MODE_LOW);
            pwr_communication_power_control(PWR_MODE_ECO);
            break;
            
        case PWR_STATE_SLEEP:
            pwr_motor_power_control(PWR_MODE_OFF);
            pwr_sensor_power_control(PWR_MODE_OFF);
            pwr_display_power_control(PWR_MODE_OFF);
            pwr_communication_power_control(PWR_MODE_LOW);
            break;
            
        case PWR_STATE_LOW_POWER:
            pwr_motor_power_control(PWR_MODE_LOW);
            pwr_sensor_power_control(PWR_MODE_LOW);
            pwr_display_power_control(PWR_MODE_LOW);
            pwr_communication_power_control(PWR_MODE_LOW);
            break;
            
        case PWR_STATE_CRITICAL:
            pwr_motor_power_control(PWR_MODE_OFF);
            pwr_sensor_power_control(PWR_MODE_OFF);
            pwr_display_power_control(PWR_MODE_OFF);
            pwr_communication_power_control(PWR_MODE_OFF);
            break;
            
        case PWR_STATE_ERROR:
            pwr_motor_power_control(PWR_MODE_OFF);
            pwr_sensor_power_control(PWR_MODE_OFF);
            pwr_display_power_control(PWR_MODE_OFF);
            pwr_communication_power_control(PWR_MODE_OFF);
            break;
    }
    
    power_state = state;
    power_stats.total_transitions++;
}

uint8_t pwr_get_state(void) {
    return power_state;
}

void pwr_process_event(power_event_t event) {
    if (!power_initialized) return;
    
    uint32_t current_time = timer_get(power_timer_id);
    
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        switch (event) {
            case PWR_EVENT_ACTIVITY:
                last_activity_time = current_time;
                if (power_state != PWR_STATE_ACTIVE) {
                    pwr_transition_state(PWR_STATE_ACTIVE);
                }
                break;
                
            case PWR_EVENT_INACTIVITY:
                if (current_time - last_activity_time >= power_config.idle_timeout_ms) {
                    if (power_state == PWR_STATE_ACTIVE) {
                        pwr_transition_state(PWR_STATE_IDLE);
                    }
                }
                break;
                
            case PWR_EVENT_LOW_POWER:
                power_stats.low_power_events++;
                if (power_state != PWR_STATE_LOW_POWER) {
                    pwr_transition_state(PWR_STATE_LOW_POWER);
                }
                break;
                
            case PWR_EVENT_CRITICAL:
                power_stats.critical_events++;
                if (power_state != PWR_STATE_CRITICAL) {
                    pwr_transition_state(PWR_STATE_CRITICAL);
                }
                break;
                
            case PWR_EVENT_ERROR:
                power_error_count++;
                last_error_time = current_time;
                if (power_error_count >= MAX_POWER_ERRORS) {
                    pwr_attempt_recovery();
                }
                break;
        }
    }
}

static void pwr_attempt_recovery(void) {
    if (power_recovery_attempts >= MAX_RECOVERY_ATTEMPTS) {
        error_record(ERR_POWER_UNRECOVERABLE);
        pwr_transition_state(PWR_STATE_ERROR);
        return;
    }
    
    power_recovery_attempts++;
    recovery_state = RECOVERY_IN_PROGRESS;
    
    // Systematic power recovery sequence
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        // 1. Save component states
        for (int i = 0; i < NUM_POWER_COMPONENTS; i++) {
            component_power_state[i] = pwr_get_component_state(i);
        }
        
        // 2. Power down all components
        pwr_power_down_all();
        
        // 3. Reset power control hardware
        POWER_CTRL_PORT &= ~(1 << POWER_CTRL_PIN);
        _delay_ms(100);
        POWER_CTRL_PORT |= (1 << POWER_CTRL_PIN);
        
        // 4. Restore components sequentially
        for (int i = 0; i < NUM_POWER_COMPONENTS; i++) {
            if (component_power_state[i]) {
                pwr_power_up_component(i);
                _delay_ms(50);  // Delay between components
            }
        }
    }
    
    // Check if recovery was successful
    if (pwr_verify_power_state()) {
        recovery_state = RECOVERY_SUCCESS;
        power_error_count = 0;
        pwr_transition_state(PWR_STATE_ACTIVE);
    } else {
        recovery_state = RECOVERY_FAILED;
        error_record(ERR_POWER_RECOVERY_FAILED);
        if (power_recovery_attempts >= MAX_RECOVERY_ATTEMPTS) {
            pwr_transition_state(PWR_STATE_ERROR);
        }
    }
}

void pwr_update_component(uint8_t component_id, uint8_t power_mode) {
    switch (component_id) {
        case 1: // Motor
            pwr_motor_power_control(power_mode);
            break;
        case 2: // Sensor
            pwr_sensor_power_control(power_mode);
            break;
        case 3: // Display
            pwr_display_power_control(power_mode);
            break;
        case 4: // Communication
            pwr_communication_power_control(power_mode);
            break;
    }
}

void pwr_get_stats(PowerStats* stats) {
    update_power_stats();  // Update stats before returning
    memcpy(stats, &power_stats, sizeof(PowerStats));
    
    // Calculate average power consumption
    uint32_t total_time = power_stats.active_time + power_stats.idle_time +
                         power_stats.standby_time + power_stats.sleep_time;
    if (total_time > 0) {
        stats->avg_power_consumption = 
            (power_stats.active_time * 1.0f +
             power_stats.idle_time * 0.7f +
             power_stats.standby_time * 0.3f +
             power_stats.sleep_time * 0.1f) / total_time;
    }
}

// Component-specific power control implementations
void pwr_motor_power_control(uint8_t power_mode) {
    switch (power_mode) {
        case PWR_MODE_FULL:
            // Set motor to full power mode
            _SET(PORTB, 0);  // Enable motor power
            _SET(PORTB, 1);  // Enable motor driver
            break;
        case PWR_MODE_ECO:
            // Set motor to eco mode (reduced current)
            _SET(PORTB, 0);  // Enable motor power
            _CLR(PORTB, 1);  // Disable motor driver
            break;
        case PWR_MODE_LOW:
            // Set motor to minimum power
            _CLR(PORTB, 0);  // Disable motor power
            _CLR(PORTB, 1);  // Disable motor driver
            break;
        case PWR_MODE_OFF:
            // Power down motor completely
            _CLR(PORTB, 0);  // Disable motor power
            _CLR(PORTB, 1);  // Disable motor driver
            break;
    }
}

void pwr_sensor_power_control(uint8_t power_mode) {
    switch (power_mode) {
        case PWR_MODE_FULL:
            _SET(PORTC, 0);  // Enable sensor power
            _SET(PORTC, 1);  // Enable sensor bias
            break;
        case PWR_MODE_ECO:
            _SET(PORTC, 0);  // Enable sensor power
            _CLR(PORTC, 1);  // Disable sensor bias
            break;
        case PWR_MODE_LOW:
            _CLR(PORTC, 0);  // Disable sensor power
            _CLR(PORTC, 1);  // Disable sensor bias
            break;
        case PWR_MODE_OFF:
            _CLR(PORTC, 0);  // Disable sensor power
            _CLR(PORTC, 1);  // Disable sensor bias
            break;
    }
}

void pwr_display_power_control(uint8_t power_mode) {
    switch (power_mode) {
        case PWR_MODE_FULL:
            _SET(PORTD, 0);  // Enable display power
            _SET(PORTD, 1);  // Full brightness
            break;
        case PWR_MODE_ECO:
            _SET(PORTD, 0);  // Enable display power
            _CLR(PORTD, 1);  // Reduced brightness
            break;
        case PWR_MODE_LOW:
            _SET(PORTD, 0);  // Enable display power
            _CLR(PORTD, 1);  // Minimum brightness
            break;
        case PWR_MODE_OFF:
            _CLR(PORTD, 0);  // Disable display power
            _CLR(PORTD, 1);  // Off
            break;
    }
}

void pwr_communication_power_control(uint8_t power_mode) {
    switch (power_mode) {
        case PWR_MODE_FULL:
            _SET(PORTE, 0);  // Enable communication power
            _SET(PORTE, 1);  // Full power mode
            break;
        case PWR_MODE_ECO:
            _SET(PORTE, 0);  // Enable communication power
            _CLR(PORTE, 1);  // Reduced power mode
            break;
        case PWR_MODE_LOW:
            _SET(PORTE, 0);  // Enable communication power
            _CLR(PORTE, 1);  // Minimum power mode
            break;
        case PWR_MODE_OFF:
            _CLR(PORTE, 0);  // Disable communication power
            _CLR(PORTE, 1);  // Off
            break;
    }
}

void pwr_print_status(void) {
    PowerStats stats;
    pwr_get_stats(&stats);
    
    const char* state_names[] = {"Active", "Idle", "Standby", "Sleep", "Low Power", "Critical", "Error"};
    
    DPRINTF("Power Management Status:\n");
    DPRINTF("Current State: %s\n", state_names[power_state]);
    DPRINTF("Time in states (ms):\n");
    DPRINTF("  Active: %lu\n", stats.active_time);
    DPRINTF("  Idle: %lu\n", stats.idle_time);
    DPRINTF("  Standby: %lu\n", stats.standby_time);
    DPRINTF("  Sleep: %lu\n", stats.sleep_time);
    DPRINTF("  Low Power: %lu\n", stats.low_power_time);
    DPRINTF("  Critical: %lu\n", stats.critical_time);
    DPRINTF("Total transitions: %lu\n", stats.total_transitions);
    DPRINTF("Average power consumption: %.2f%%\n", stats.avg_power_consumption * 100);
    
    DPRINTF("\nConfiguration:\n");
    DPRINTF("Idle timeout: %u ms\n", power_config.idle_timeout_ms);
    DPRINTF("Standby timeout: %u ms\n", power_config.standby_timeout_ms);
    DPRINTF("Sleep timeout: %u ms\n", power_config.sleep_timeout_ms);
    DPRINTF("Eco threshold: %u%%\n", power_config.eco_threshold);
    DPRINTF("Low power threshold: %u%%\n", power_config.low_power_threshold);
}