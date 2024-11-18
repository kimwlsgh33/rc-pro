#include "predictive_maintenance.h"
#include "putil.h"
#include "timer.h"
#include <string.h>

static ComponentHealth component_health[5];  // One extra for indexing simplicity
static MaintenanceEvent maintenance_history[MAX_MAINTENANCE_HISTORY];
static uint8_t history_index = 0;
static int maintenance_timer_id = -1;

static uint32_t get_threshold(uint8_t component_id) {
    switch (component_id) {
        case COMP_MOTOR:
            return MOTOR_CYCLE_THRESHOLD;
        case COMP_DOOR:
            return DOOR_CYCLE_THRESHOLD;
        case COMP_SENSOR:
            return SENSOR_READ_THRESHOLD;
        case COMP_TANK:
            return TANK_USAGE_THRESHOLD;
        default:
            return 0;
    }
}

void pm_init(void) {
    memset(component_health, 0, sizeof(component_health));
    memset(maintenance_history, 0, sizeof(maintenance_history));
    maintenance_timer_id = timer_alloc();
    
    // Initialize wear rates based on expected component lifetime
    for (int i = 1; i <= 4; i++) {
        component_health[i].wear_rate = 1.0f / get_threshold(i);
        component_health[i].health_status = HEALTH_GOOD;
        component_health[i].last_maintenance = timer_get(maintenance_timer_id);
    }
}

void pm_record_cycle(uint8_t component_id) {
    if (component_id < 1 || component_id > 4) return;
    
    ComponentHealth* health = &component_health[component_id];
    health->cycles++;
    
    // Update health status based on cycle count
    uint32_t threshold = get_threshold(component_id);
    uint32_t cycles_since_maintenance = health->cycles - health->last_maintenance;
    
    if (cycles_since_maintenance >= threshold * 0.9) {
        health->health_status = HEALTH_CRITICAL;
    } else if (cycles_since_maintenance >= threshold * 0.7) {
        health->health_status = HEALTH_WARNING;
    }
    
    // Record event if status changed
    if (health->health_status != HEALTH_GOOD) {
        MaintenanceEvent* event = &maintenance_history[history_index];
        event->timestamp = timer_get(maintenance_timer_id);
        event->component_id = component_id;
        event->event_type = health->health_status;
        event->cycle_count = health->cycles;
        
        history_index = (history_index + 1) % MAX_MAINTENANCE_HISTORY;
    }
    
    pm_update_wear_rate(component_id);
}

void pm_record_error(uint8_t component_id) {
    if (component_id < 1 || component_id > 4) return;
    
    ComponentHealth* health = &component_health[component_id];
    health->errors++;
    
    // Update wear rate based on errors per cycle
    if (health->cycles > 0) {
        health->wear_rate = (float)health->errors / health->cycles;
    }
    
    // Record error event
    MaintenanceEvent* event = &maintenance_history[history_index];
    event->timestamp = timer_get(maintenance_timer_id);
    event->component_id = component_id;
    event->event_type = HEALTH_CRITICAL;  // Errors are always critical
    event->cycle_count = health->cycles;
    
    history_index = (history_index + 1) % MAX_MAINTENANCE_HISTORY;
    
    pm_update_wear_rate(component_id);
}

void pm_record_maintenance(uint8_t component_id) {
    if (component_id < 1 || component_id > 4) return;
    
    ComponentHealth* health = &component_health[component_id];
    health->last_maintenance = timer_get(maintenance_timer_id);
    health->health_status = HEALTH_GOOD;
    health->errors = 0;  // Reset error count after maintenance
    
    // Record maintenance event
    MaintenanceEvent* event = &maintenance_history[history_index];
    event->timestamp = health->last_maintenance;
    event->component_id = component_id;
    event->event_type = HEALTH_GOOD;
    event->cycle_count = health->cycles;
    
    history_index = (history_index + 1) % MAX_MAINTENANCE_HISTORY;
    
    pm_update_wear_rate(component_id);
}

uint8_t pm_get_health_status(uint8_t component_id) {
    if (component_id < 1 || component_id > 4) return HEALTH_CRITICAL;
    return component_health[component_id].health_status;
}

uint32_t pm_get_cycles(uint8_t component_id) {
    if (component_id < 1 || component_id > 4) return 0;
    return component_health[component_id].cycles;
}

float pm_get_wear_rate(uint8_t component_id) {
    if (component_id < 1 || component_id > 4) return 0.0f;
    return component_health[component_id].wear_rate;
}

void pm_predict_maintenance(uint8_t component_id, uint32_t* cycles_remaining) {
    if (component_id < 1 || component_id > 4) {
        *cycles_remaining = 0;
        return;
    }
    
    ComponentHealth* health = &component_health[component_id];
    uint32_t base_threshold = get_threshold(component_id);
    float adjusted_threshold = base_threshold / (1.0f + health->wear_rate);
    
    *cycles_remaining = health->next_maintenance - health->cycles;
}

void pm_print_status(void) {
    DPRINTF("Predictive Maintenance Status:\n");
    
    const char* component_names[] = {"", "Motor", "Door", "Sensor", "Tank"};
    const char* health_status[] = {"Good", "Warning", "Critical"};
    
    for (int i = 1; i <= 4; i++) {
        ComponentHealth* health = &component_health[i];
        uint32_t cycles_remaining;
        pm_predict_maintenance(i, &cycles_remaining);
        
        DPRINTF("%s: Status=%s, Cycles=%lu, Errors=%lu, Wear=%.4f, Remaining=%lu\n",
                component_names[i],
                health_status[health->health_status],
                health->cycles,
                health->errors,
                health->wear_rate,
                cycles_remaining);
    }
    
    DPRINTF("\nMaintenance History:\n");
    for (int i = 0; i < MAX_MAINTENANCE_HISTORY; i++) {
        MaintenanceEvent* event = &maintenance_history[i];
        if (event->timestamp == 0) continue;
        
        DPRINTF("Time=%lu, Component=%s, Type=%s, Cycles=%lu\n",
                event->timestamp,
                component_names[event->component_id],
                health_status[event->event_type],
                event->cycle_count);
    }
}

void pm_update_wear_rate(uint8_t component_id) {
    if (component_id < 1 || component_id > 4) return;
    
    ComponentHealth* health = &component_health[component_id];
    
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        // Calculate base error rate
        float error_rate = (float)health->errors / (health->cycles > 0 ? health->cycles : 1);
        
        // Get environmental factors
        float temp_factor = get_temperature_factor();
        float load_factor = get_load_factor();
        float usage_intensity = get_usage_intensity();
        
        // Calculate weighted wear rate
        float wear_rate = (error_rate * 0.4f +      // Error history weight
                          temp_factor * 0.2f +       // Temperature impact
                          load_factor * 0.2f +       // Load impact
                          usage_intensity * 0.2f);   // Usage pattern impact
        
        // Apply wear rate smoothing
        health->wear_rate = (health->wear_rate * 0.7f) + (wear_rate * 0.3f);
        
        // Update maintenance prediction
        uint32_t base_threshold = get_threshold(component_id);
        float adjusted_threshold = base_threshold / (1.0f + health->wear_rate);
        
        // Update maintenance schedule
        health->next_maintenance = health->cycles + (uint32_t)adjusted_threshold;
        
        // Check if immediate maintenance is needed
        if (health->wear_rate > WEAR_RATE_CRITICAL) {
            health->health_status = HEALTH_CRITICAL;
            error_record(ERR_MAINTENANCE_REQUIRED);
        } else if (health->wear_rate > WEAR_RATE_WARNING) {
            health->health_status = HEALTH_WARNING;
        }
    }
}

static float get_temperature_factor(void) {
    int16_t temp = motor_get_temperature();
    float factor = 1.0f;
    
    if (temp > TEMP_CRITICAL) {
        factor = 2.0f;
    } else if (temp > TEMP_WARNING) {
        factor = 1.5f;
    } else if (temp > TEMP_NOMINAL) {
        factor = 1.2f;
    }
    
    return factor;
}

static float get_load_factor(void) {
    uint16_t current = motor_get_current();
    float factor = 1.0f;
    
    if (current > CURRENT_CRITICAL) {
        factor = 2.0f;
    } else if (current > CURRENT_WARNING) {
        factor = 1.5f;
    } else if (current > CURRENT_NOMINAL) {
        factor = 1.2f;
    }
    
    return factor;
}

static float get_usage_intensity(void) {
    uint32_t current_time = timer_get_ms();
    static uint32_t last_check_time = 0;
    static uint32_t operation_count = 0;
    float intensity = 1.0f;
    
    if (last_check_time == 0) {
        last_check_time = current_time;
        return intensity;
    }
    
    uint32_t time_diff = current_time - last_check_time;
    if (time_diff > USAGE_CHECK_INTERVAL) {
        float ops_per_hour = (float)operation_count * (3600000.0f / time_diff);
        
        if (ops_per_hour > OPS_PER_HOUR_CRITICAL) {
            intensity = 2.0f;
        } else if (ops_per_hour > OPS_PER_HOUR_WARNING) {
            intensity = 1.5f;
        } else if (ops_per_hour > OPS_PER_HOUR_NOMINAL) {
            intensity = 1.2f;
        }
        
        operation_count = 0;
        last_check_time = current_time;
    }
    
    operation_count++;
    return intensity;
}
