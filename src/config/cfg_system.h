#ifndef CFG_SYSTEM_H
#define CFG_SYSTEM_H

// Build Configuration
#define BUILD_VERSION "1.0.0"
#define BUILD_DATE __DATE__
#define BUILD_TIME __TIME__

// Feature Flags
#define ENABLE_ADVANCED_DIAGNOSTICS 1
#define ENABLE_REMOTE_MONITORING 1
#define ENABLE_POWER_OPTIMIZATION 1
#define ENABLE_PREDICTIVE_MAINTENANCE 1

// System Validation
#define SYSTEM_VALID_SIGNATURE 0x55AA
#define SYSTEM_CONFIG_VERSION 0x0100

// Enhanced Memory Protection
#define STACK_GUARD_SIZE 32
#define HEAP_GUARD_SIZE 32
#define STACK_OVERFLOW_PATTERN 0xDEADBEEF
#define HEAP_GUARD_PATTERN 0xBAADF00D

// Memory Configuration
#define STACK_SIZE 2048
#define HEAP_SIZE 4096
#define MAX_TASKS 16

// Memory Configuration with Checks
#if (STACK_SIZE < 1024) || (STACK_SIZE > 8192)
    #error "Stack size must be between 1KB and 8KB"
#endif

#if (HEAP_SIZE < 2048) || (HEAP_SIZE > 16384)
    #error "Heap size must be between 2KB and 16KB"
#endif

// Resource Management
typedef struct {
    uint16_t signature;
    uint16_t version;
    uint8_t components_used;
    uint8_t sensors_used;
    uint8_t motors_used;
    uint8_t error_queue_count;
} system_resources_t;

// Debug Options
#ifdef DEBUG
    #define ENABLE_DEBUG_OUTPUT 1
    #define ENABLE_ASSERT_CHECKS 1
    #define ENABLE_PERFORMANCE_METRICS 1
#else
    #define ENABLE_DEBUG_OUTPUT 0
    #define ENABLE_ASSERT_CHECKS 0
    #define ENABLE_PERFORMANCE_METRICS 0
#endif

// Hardware Configuration
#define MCU_CLOCK_FREQUENCY 16000000UL
#define EXTERNAL_CRYSTAL_FREQUENCY 16000000UL

// Communication Configuration
#define USE_UART0 1
#define USE_UART1 1
#define USE_UART2 0
#define USE_UART3 1

// UART Configuration with Validation
#define UART_BUFFER_MULTIPLIER 4
#define MAX_PACKET_SIZE 256
#define UART_MIN_BUFFER_SIZE (MAX_PACKET_SIZE * UART_BUFFER_MULTIPLIER)

#if USE_UART0
    #define UART0_RX_BUFFER_SIZE UART_MIN_BUFFER_SIZE
    #define UART0_TX_BUFFER_SIZE UART_MIN_BUFFER_SIZE
#endif

#if USE_UART1
    #define UART1_RX_BUFFER_SIZE UART_MIN_BUFFER_SIZE
    #define UART1_TX_BUFFER_SIZE UART_MIN_BUFFER_SIZE
#endif

// Timer Configuration
#define USE_TIMER0 1
#define USE_TIMER1 1
#define USE_TIMER2 0
#define TIMER_PRESCALER 1
#define TIMER_COMPARE_VALUE 1

// Timer Validation
#define TIMER_VALID_PRESCALERS {1, 8, 64, 256, 1024}
#define TIMER_MIN_FREQ 100  // Minimum timer frequency in Hz
#define TIMER_MAX_FREQ 10000  // Maximum timer frequency in Hz

#if USE_TIMER1
    #if (MCU_CLOCK_FREQUENCY / (TIMER_PRESCALER * TIMER_COMPARE_VALUE)) > TIMER_MAX_FREQ
        #error "Timer1 frequency too high"
    #endif
    #if (MCU_CLOCK_FREQUENCY / (TIMER_PRESCALER * TIMER_COMPARE_VALUE)) < TIMER_MIN_FREQ
        #error "Timer1 frequency too low"
    #endif
#endif

// Include paths
#define INCLUDE_PATH_CORE "../core"
#define INCLUDE_PATH_DRIVERS "../drivers"
#define INCLUDE_PATH_MODULES "../modules"
#define INCLUDE_PATH_UTILS "../utils"

#endif // CFG_SYSTEM_H
