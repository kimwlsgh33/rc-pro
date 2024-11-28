#include <stdbool.h>
#include <string.h>
#include "error_codes.h"
#include "../../config/cfg_system.h"

// Error Recovery Configuration
#define MAX_RECOVERY_ATTEMPTS 3
#define RECOVERY_TIMEOUT_MS 5000
#define MAX_ERROR_HANDLERS 16

// Recovery Handler Type
typedef error_code_t (*error_recovery_handler_t)(error_code_t error);

// Recovery Handler Registration
typedef struct {
    error_code_t error_code;
    error_recovery_handler_t handler;
    uint8_t priority;
} recovery_handler_t;

// Recovery Statistics
typedef struct {
    uint32_t attempts;
    uint32_t successes;
    uint32_t failures;
    uint32_t last_error;
    uint32_t last_attempt_time;
} recovery_stats_t;

// Recovery System State
static struct {
    recovery_handler_t handlers[MAX_ERROR_HANDLERS];
    uint8_t handler_count;
    recovery_stats_t stats;
    bool recovery_in_progress;
    uint32_t recovery_start_time;
} recovery_system;

// Initialize error recovery system
void error_recovery_init(void) {
    memset(&recovery_system, 0, sizeof(recovery_system));
}

// Register error recovery handler
error_code_t error_recovery_register_handler(error_code_t error_code, 
                                           error_recovery_handler_t handler,
                                           uint8_t priority) {
    if (!handler || recovery_system.handler_count >= MAX_ERROR_HANDLERS) {
        return ERR_INVALID_PARAM;
    }
    
    // Check if handler already exists
    for (uint8_t i = 0; i < recovery_system.handler_count; i++) {
        if (recovery_system.handlers[i].error_code == error_code) {
            return ERR_ALREADY_EXISTS;
        }
    }
    
    // Find insertion point based on priority
    uint8_t insert_idx = recovery_system.handler_count;
    for (uint8_t i = 0; i < recovery_system.handler_count; i++) {
        if (priority < recovery_system.handlers[i].priority) {
            insert_idx = i;
            break;
        }
    }
    
    // Shift existing handlers
    if (insert_idx < recovery_system.handler_count) {
        memmove(&recovery_system.handlers[insert_idx + 1],
                &recovery_system.handlers[insert_idx],
                (recovery_system.handler_count - insert_idx) * sizeof(recovery_handler_t));
    }
    
    // Insert new handler
    recovery_system.handlers[insert_idx].error_code = error_code;
    recovery_system.handlers[insert_idx].handler = handler;
    recovery_system.handlers[insert_idx].priority = priority;
    recovery_system.handler_count++;
    
    return ERROR_CODE_SUCCESS;
}

// Attempt error recovery
error_code_t error_recovery_attempt(error_code_t error) {
    if (recovery_system.recovery_in_progress) {
        return ERR_BUSY;
    }
    
    // Update statistics
    recovery_system.stats.attempts++;
    recovery_system.stats.last_error = error;
    recovery_system.stats.last_attempt_time = timer_get_ms();
    
    // Set recovery state
    recovery_system.recovery_in_progress = true;
    recovery_system.recovery_start_time = timer_get_ms();
    
    error_code_t recovery_result = ERR_NOT_FOUND;
    
    // Try each registered handler in priority order
    for (uint8_t i = 0; i < recovery_system.handler_count; i++) {
        if (recovery_system.handlers[i].error_code == error ||
            recovery_system.handlers[i].error_code == ERR_ANY) {
            
            recovery_result = recovery_system.handlers[i].handler(error);
            
            if (recovery_result == ERROR_CODE_SUCCESS) {
                recovery_system.stats.successes++;
                break;
            }
        }
    }
    
    // Update statistics and state
    if (recovery_result != ERROR_CODE_SUCCESS) {
        recovery_system.stats.failures++;
    }
    
    recovery_system.recovery_in_progress = false;
    
    return recovery_result;
}

// Get recovery statistics
const recovery_stats_t* error_recovery_get_stats(void) {
    return &recovery_system.stats;
}

// Check if recovery is in progress
bool error_recovery_is_active(void) {
    if (!recovery_system.recovery_in_progress) {
        return false;
    }
    
    // Check for timeout
    if (timer_get_ms() - recovery_system.recovery_start_time >= RECOVERY_TIMEOUT_MS) {
        recovery_system.recovery_in_progress = false;
        recovery_system.stats.failures++;
        return false;
    }
    
    return true;
}

// Reset recovery statistics
void error_recovery_reset_stats(void) {
    memset(&recovery_system.stats, 0, sizeof(recovery_stats_t));
}

// Example recovery handlers for common errors
static error_code_t recover_from_motor_error(error_code_t error) {
    // Emergency stop all motors
    ma_emergency_stop();
    
    // Reset motor control blocks
    motor_ctrl.state = MA_ST_UNKNOWN;
    motor_ctrl.error_flags = 0;
    
    // Reinitialize motor subsystem
    return ma_init();
}

static error_code_t recover_from_event_error(error_code_t error) {
    // Clear event queue
    event_clear_queue();
    
    // Reset event manager statistics
    event_manager_status_t* status = (event_manager_status_t*)event_manager_get_status();
    if (status) {
        status->events_dropped = 0;
        status->events_processed = 0;
        status->last_event_time = 0;
    }
    
    // Reinitialize event manager
    return event_manager_init();
}

static error_code_t recover_from_buffer_error(error_code_t error) {
    switch (error) {
        case ERR_UNDERFLOW:
            // No recovery needed, just a status
            return ERROR_CODE_SUCCESS;
            
        case ERR_BUFFER_FULL:
        case ERR_BUFFER_OVERFLOW:
            // Flush affected buffers
            for (uart_port_t port = 0; port < UART_PORT_MAX; port++) {
                uart_flush_rx(port);
                uart_flush_tx(port);
            }
            return ERROR_CODE_SUCCESS;
            
        default:
            return ERR_NOT_SUPPORTED;
    }
}

static error_code_t recover_from_hardware_error(error_code_t error) {
    // Disable interrupts during recovery
    cli();
    
    // Reset all peripheral registers to default values
    for (uart_port_t port = 0; port < UART_PORT_MAX; port++) {
        uart_deinit(port);
    }
    
    // Reset DMA controller
    dma_deinit();
    
    // Reset timer system
    timer_reset_all();
    
    // Reinitialize system with default configuration
    error_code_t result = ERROR_CODE_SUCCESS;
    
    // Restore interrupts
    sei();
    
    return result;
}

// ADC Error Recovery Implementation
static error_code_t recover_from_adc_error(error_code_t error) {
    uint8_t error_flags = (uint8_t)error;
    if (error_flags & ADC_ERROR_TIMEOUT) {
        // Reset ADC and restart conversion
        ADCSRA &= ~(1 << ADEN);  // Disable ADC
        _delay_us(10);           // Short delay
        ADCSRA |= (1 << ADEN);   // Re-enable ADC
        ADCSRA |= (1 << ADSC);   // Start new conversion
    }
    if (error_flags & ADC_ERROR_REFERENCE) {
        // Switch to different reference voltage
        ADMUX &= ~(0xC0);        // Clear reference bits
        ADMUX |= (0x40);         // Set AVCC as reference
    }
    return ERROR_CODE_SUCCESS;
}

// Enhanced UART Error Recovery Implementation
static error_code_t recover_from_uart_error(error_code_t error) {
    uint8_t error_flags = (uint8_t)error;
    error_code_t err = ERROR_CODE_SUCCESS;
    
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        if (error_flags & (UART_ERROR_OVERFLOW | UART_ERROR_BUFFER)) {
            // Flush buffers and reset UART with enhanced error handling
            uint8_t dummy;
            while (UCSR0A & (1 << RXC0)) {
                dummy = UDR0;
                __asm__ __volatile__ ("nop");  // Memory barrier
            }
            
            // Disable interrupts during reset
            UCSR0B &= ~((1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0) | (1 << TXCIE0));
            
            // Clear error flags
            UCSR0A |= (1 << FE0) | (1 << DOR0) | (1 << UPE0);
            
            // Reset baud rate
            UBRR0H = (UART_BAUD_SETTING >> 8) & 0xFF;
            UBRR0L = UART_BAUD_SETTING & 0xFF;
            
            // Wait for stable clock
            _delay_ms(10);
            
            // Re-enable with all features
            UCSR0B |= (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0) | (1 << TXCIE0);
        }
        
        if (error_flags & UART_ERROR_DMA) {
            // Enhanced DMA recovery
            err = recover_from_dma_error(error);
            if (err != ERROR_CODE_SUCCESS) {
                error_record(ERR_DMA_RECOVERY);
            }
        }
        
        if (error_flags & UART_ERROR_PARITY) {
            // Handle parity errors by resetting UART configuration
            UCSR0C = (1 << UCSZ01) | (1 << UCSZ00) | (1 << UPM01) | (1 << UPM00);
            _delay_ms(1);
        }
        
        if (error_flags & UART_ERROR_FRAMING) {
            // Reset frame format and clear framing error
            UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
            UCSR0A |= (1 << FE0);
            _delay_ms(1);
        }
    }
    
    // Log recovery attempt
    log_safety_event(SAFETY_EVENT_UART_RECOVERY);
    
    return err;
}

static error_code_t recover_from_dma_error(error_code_t error) {
    uint8_t error_flags = (uint8_t)error;
    
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        if (error_flags & DMA_ERROR_TRANSFER) {
            // Reset DMA controller
            DMAMUX0 = 0;  // Reset DMA multiplexer
            DMA0CTL = 0;  // Reset DMA control register
            
            // Clear all pending transfers
            DMA0CNT = 0;  // Clear transfer count
            DMA0SA = 0;   // Clear source address
            DMA0DA = 0;   // Clear destination address
            
            // Reinitialize channels with safe defaults
            DMA0CTL = DMA_SIZE_8 | DMA_SINGLE;  // 8-bit transfers, single mode
            DMAEN = 1;    // Re-enable DMA
        }
        
        if (error_flags & DMA_ERROR_BUFFER) {
            // Reset buffer pointers and state
            dma_buffer_head = 0;
            dma_buffer_tail = 0;
            dma_buffer_count = 0;
            
            // Clear buffer contents
            memset(dma_buffer, 0, DMA_BUFFER_SIZE);
            
            // Reset DMA buffer configuration
            DMA0SA = (uint16_t)&dma_buffer[0];
            DMA0DA = (uint16_t)&UDR0;
            DMA0CNT = DMA_BUFFER_SIZE - 1;
        }
        
        if (error_flags & DMA_ERROR_ALIGNMENT) {
            // Fix alignment issues
            uint16_t addr = (uint16_t)dma_buffer;
            if (addr & 0x01) {  // If address is not 2-byte aligned
                addr = (addr + 1) & ~0x01;  // Align to 2-byte boundary
                dma_buffer = (uint8_t*)addr;
            }
        }
    }
    
    return ERROR_CODE_SUCCESS;
}

static error_code_t recover_from_timer_error(error_code_t error) {
    // Stop all active timers
    for (int i = 0; i < MAX_SYS_TIMER; i++) {
        timer_clear(i);
    }
    
    // Reset timer system
    timer_reset_all();
    
    // Reinitialize timer system
    return timer_init();
}

static error_code_t recover_from_pwm_error(error_code_t error) {
    uint8_t error_flags = (uint8_t)error;
    if (error_flags & PWM_ERROR_CONFIG) {
        // Reset PWM configuration
        TCCR1A = 0;  // Clear all PWM settings
        TCCR1B = 0;
        // Reconfigure with safe values
        TCCR1A = (1 << WGM11);
        TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);
    }
    if (error_flags & PWM_ERROR_DUTY_CYCLE) {
        // Set safe duty cycle
        OCR1A = ICR1 / 2;  // 50% duty cycle
    }
    return ERROR_CODE_SUCCESS;
}

static error_code_t recover_from_sensor_error(error_code_t error) {
    uint8_t error_flags = (uint8_t)error;
    if (error_flags & SENSOR_ERROR_TIMEOUT) {
        // Reset sensor communication
        // Implementation depends on sensor type
    }
    if (error_flags & SENSOR_ERROR_CALIBRATION) {
        // Attempt sensor recalibration
        // Implementation depends on sensor type
    }
    return ERROR_CODE_SUCCESS;
}

static error_code_t recover_from_stepper_error(error_code_t error) {
    uint8_t error_flags = (uint8_t)error;
    
    if (error_flags & STEPPER_ERROR_STALL) {
        // Handle motor stall
        sm1_stop();
        sm2_stop();
        _delay_ms(100);  // Allow motor to cool/settle
        
        // Reset motor drivers
        sm1_init();
        sm2_init();
    }
    
    if (error_flags & STEPPER_ERROR_POSITION) {
        // Reset position tracking
        sm1_reset_position();
        sm2_reset_position();
        
        // Re-home motors if necessary
        if (motor_ctrl.state == MA_ST_UNKNOWN) {
            ma_init_position();
        }
    }
    
    if (error_flags & STEPPER_ERROR_SPEED) {
        // Reset to safe speed
        sm1_set_speed(FIND_SPEED_HZ / 2);  // Half of find speed
        sm2_set_speed(FIND_SPEED_HZ / 2);
    }
    
    if (error_flags & STEPPER_ERROR_CURRENT) {
        // Reduce current to prevent overheating
        // Implementation depends on stepper driver
    }
    
    return ERROR_CODE_SUCCESS;
}

static error_code_t recover_from_position_error(error_code_t error) {
    uint8_t error_flags = (uint8_t)error;
    
    if (error_flags & POSITION_ERROR_SYNC) {
        // Resynchronize position tracking
        sm1_reset_position();
        sm2_reset_position();
        motor_ctrl.position = 0;
    }
    
    if (error_flags & POSITION_ERROR_OVERFLOW) {
        // Handle position counter overflow
        motor_ctrl.position = 0;
        sm1_reset_position();
        sm2_reset_position();
    }
    
    if (error_flags & POSITION_ERROR_SENSOR) {
        // Recalibrate position sensors
        if (ORG_SENSOR_IS_ON()) {
            motor_ctrl.position = 0;
            sm1_reset_position();
            sm2_reset_position();
        } else {
            // Move to find sensor
            sm1_run(DIR_CCW, FIND_SPEED_HZ);
            sm2_run(DIR_CCW, FIND_SPEED_HZ);
        }
    }
    
    if (error_flags & POSITION_ERROR_CALIBRATION) {
        // Perform full calibration sequence
        motor_ctrl.state = MA_ST_INITIALIZING;
        motor_ctrl.init_step = 0;
        ma_init_position();
    }
    
    return ERROR_CODE_SUCCESS;
}

// Register default recovery handlers
void error_recovery_register_defaults(void) {
    error_recovery_register_handler(ERR_DMA_ERROR, recover_from_dma_error, 1);
    error_recovery_register_handler(ERR_UART_ERROR, recover_from_uart_error, 2);
    error_recovery_register_handler(ERR_TIMER_ERROR, recover_from_timer_error, 3);
}

// Register additional recovery handlers
void error_recovery_register_extended(void) {
    error_recovery_register_handler(ERR_MOTOR_ERROR, recover_from_motor_error, 4);
    error_recovery_register_handler(ERR_EVENT_ERROR, recover_from_event_error, 5);
    error_recovery_register_handler(ERR_UNDERFLOW, recover_from_buffer_error, 6);
    error_recovery_register_handler(ERR_BUFFER_FULL, recover_from_buffer_error, 6);
    error_recovery_register_handler(ERR_BUFFER_OVERFLOW, recover_from_buffer_error, 6);
    error_recovery_register_handler(ERR_HARDWARE, recover_from_hardware_error, 7);
    error_recovery_register_handler(ERR_ADC_ERROR, recover_from_adc_error, 8);
    error_recovery_register_handler(ERR_PWM_ERROR, recover_from_pwm_error, 9);
    error_recovery_register_handler(ERR_SENSOR_ERROR, recover_from_sensor_error, 10);
    error_recovery_register_handler(ERR_STEPPER_ERROR, recover_from_stepper_error, 11);
    error_recovery_register_handler(ERR_POSITION_ERROR, recover_from_position_error, 12);
}
