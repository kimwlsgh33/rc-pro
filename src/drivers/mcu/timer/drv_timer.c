/**
 * Timer Driver Implementation
 * - Generic timer implementation with priority-based management
 * 
 * @author tchan@TSoft
 * @date 2024/01/01
 */
#include "drv_timer.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>

// Timer control block structure
typedef struct {
    timer_config_t config;
    struct {
        uint32_t value;
        uint32_t target;
        uint8_t active;
        uint32_t current_tick;
        uint32_t elapsed_time;
        timer_stats_t stats;
        error_code_t last_error;
    } status;
    bool allocated;
} timer_control_block_t;

// Global variables
static timer_control_block_t timer_blocks[MAX_SYS_TIMER];
static const uint16_t prescaler_values[] = {1, 8, 64, 256, 1024};

// Initialize timer system
void timer_init(void) {
    // Clear all timer blocks
    memset(timer_blocks, 0, sizeof(timer_blocks));
    
    // Configure Timer1 as system timebase
    TCCR1A = 0;                // Normal operation
    TCCR1B = (1 << CS11);      // Prescaler = 8
    TIMSK1 = (1 << TOIE1);     // Enable overflow interrupt
    
    // Note: Interrupts are managed by the application layer
}

// Allocate a timer
int timer_alloc(void) {
    for (int i = 0; i < MAX_SYS_TIMER; i++) {
        if (!timer_blocks[i].allocated) {
            timer_blocks[i].allocated = true;
            memset(&timer_blocks[i].status, 0, sizeof(timer_blocks[i].status));
            timer_blocks[i].config.priority = TIMER_PRIORITY_IDLE;
            return i;
        }
    }
    return ERR_TIMER_ALLOC;
}

// Free a timer
int timer_free(int timer_id) {
    if (timer_id < 0 || timer_id >= MAX_SYS_TIMER) {
        return ERR_TIMER_INVALID_ID;
    }
    
    if (!timer_blocks[timer_id].allocated) {
        return ERR_TIMER_NOT_ALLOCATED;
    }
    
    timer_blocks[timer_id].allocated = false;
    return 0;
}

// Set timer value
int timer_set(int timer_id, uint32_t time_value) {
    if (timer_id < 0 || timer_id >= MAX_SYS_TIMER) {
        return ERR_TIMER_INVALID_ID;
    }
    
    if (!timer_blocks[timer_id].allocated) {
        return ERR_TIMER_NOT_ALLOCATED;
    }
    
    timer_control_block_t* tcb = &timer_blocks[timer_id];
    tcb->status.value = 0;
    tcb->status.target = time_value;
    tcb->status.active = true;
    tcb->status.current_tick = TCNT1;
    
    return 0;
}

// Get current timer value
uint32_t timer_get(int timer_id) {
    if (timer_id < 0 || timer_id >= MAX_SYS_TIMER || !timer_blocks[timer_id].allocated) {
        return 0;
    }
    
    timer_control_block_t* tcb = &timer_blocks[timer_id];
    return tcb->status.value;
}

// Check if timer has fired
bool timer_isfired(int timer_id) {
    if (timer_id < 0 || timer_id >= MAX_SYS_TIMER || !timer_blocks[timer_id].allocated) {
        return false;
    }
    
    timer_control_block_t* tcb = &timer_blocks[timer_id];
    if (!tcb->status.active) {
        return false;
    }
    
    if (tcb->status.value >= tcb->status.target) {
        tcb->status.active = false;
        if (tcb->config.expiry_callback) {
            tcb->config.expiry_callback(tcb->config.expiry_callback_arg);
        }
        return true;
    }
    
    return false;
}

// Clear timer
int timer_clear(int timer_id) {
    if (timer_id < 0 || timer_id >= MAX_SYS_TIMER) {
        return ERR_TIMER_INVALID_ID;
    }
    
    if (!timer_blocks[timer_id].allocated) {
        return ERR_TIMER_NOT_ALLOCATED;
    }
    
    timer_blocks[timer_id].status.value = 0;
    timer_blocks[timer_id].status.active = false;
    return 0;
}

// Timer overflow interrupt handler
ISR(TIMER1_OVF_vect) {
    static uint32_t overflow_count = 0;
    overflow_count++;
    
    // Process active timers in priority order
    for (uint8_t priority = TIMER_PRIORITY_HIGH; priority <= TIMER_PRIORITY_IDLE; priority++) {
        for (int i = 0; i < MAX_SYS_TIMER; i++) {
            timer_control_block_t* tcb = &timer_blocks[i];
            
            if (tcb->allocated && tcb->status.active && tcb->config.priority == priority) {
                // Update timer value and statistics
                tcb->status.value++;
                tcb->status.elapsed_time = (overflow_count * 65536UL + TCNT1 - tcb->status.current_tick) * 
                                         prescaler_values[TIMER_DIV_8] / (F_CPU / 1000000UL);
                
                // Check for expiry
                if (tcb->status.value >= tcb->status.target) {
                    tcb->status.active = false;
                    tcb->status.stats.compare_matches++;
                    
                    if (tcb->config.expiry_callback) {
                        tcb->config.expiry_callback(tcb->config.expiry_callback_arg);
                    }
                }
            }
        }
    }
    
    // Update statistics for all timers
    for (int i = 0; i < MAX_SYS_TIMER; i++) {
        if (timer_blocks[i].allocated) {
            timer_blocks[i].status.stats.overflows++;
        }
    }
}

// Get timer status
const timer_status_t* timer_get_status(int timer_id) {
    if (timer_id < 0 || timer_id >= MAX_SYS_TIMER || !timer_blocks[timer_id].allocated) {
        return NULL;
    }
    
    return (const timer_status_t*)&timer_blocks[timer_id].status;
}

// Reset timer statistics
void timer_reset_statistics(int timer_id) {
    if (timer_id >= 0 && timer_id < MAX_SYS_TIMER && timer_blocks[timer_id].allocated) {
        memset(&timer_blocks[timer_id].status.stats, 0, sizeof(timer_stats_t));
    }
}
