#ifndef CFG_SYSTEM_H
#define CFG_SYSTEM_H

#include "cfg_build.h"
#include "cfg_debug.h"

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

// Memory Configuration Validation
#if (STACK_SIZE < 1024) || (STACK_SIZE > 8192)
#error "Stack size must be between 1KB and 8KB"
#endif

#if (HEAP_SIZE < 2048) || (HEAP_SIZE > 16384)
#error "Heap size must be between 2KB and 16KB"
#endif

// Communication Configuration
#define USE_UART0 1
#define USE_UART1 1
#define USE_UART2 0
#define USE_UART3 1

// UART Buffer Configuration
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

// Communication Timeouts
#define PACKET_TIMEOUT_BASE_MS   50
#define PACKET_TIMEOUT_PER_BYTE_MS (PACKET_TIMEOUT_BASE_MS / 32)
#define PACKET_MAX_TIMEOUT_MS    500

// Calculate packet timeout based on size
#define CALC_PACKET_TIMEOUT(size) \
    ((size) * PACKET_TIMEOUT_PER_BYTE_MS + PACKET_TIMEOUT_BASE_MS)

// Validate timeout is within bounds
#define VALIDATE_PACKET_TIMEOUT(timeout) \
    ((timeout) < PACKET_MAX_TIMEOUT_MS ? (timeout) : PACKET_MAX_TIMEOUT_MS)

// Resource Management
typedef struct {
    uint16_t signature;
    uint16_t version;
    uint8_t components_used;
    uint8_t sensors_used;
    uint8_t motors_used;
    uint8_t error_queue_count;
} system_resources_t;

#endif // CFG_SYSTEM_H
