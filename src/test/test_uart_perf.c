#include "test_uart_perf.h"
#include "../drivers/mcu/uart/drv_uart.h"
#include "../drivers/mcu/dma/drv_dma.h"
#include "../utils/common/error_codes.h"
#include <stdio.h>
#include <string.h>

// Test buffer sizes
#define TEST_BUFFER_SMALL    32
#define TEST_BUFFER_MEDIUM   128
#define TEST_BUFFER_LARGE    512

// Test durations and iterations
#define TEST_DURATION_MS     1000
#define TEST_ITERATIONS      100

// Test data buffers
static uint8_t tx_buffer[TEST_BUFFER_LARGE];
static uint8_t rx_buffer[TEST_BUFFER_LARGE];
static volatile bool transfer_complete = false;

// Performance metrics
typedef struct {
    uint32_t throughput;          // Bytes per second
    float interrupts_per_byte;    // Average interrupts per byte
    uint32_t overflow_count;      // Buffer overflow count
    uint32_t error_count;         // Error count
    uint32_t memory_used;         // Total memory usage
    float cpu_usage;              // CPU usage percentage
    uint32_t latency_us;          // Average latency in microseconds
} perf_metrics_t;

// Callback functions
static void tx_complete_callback(void) {
    transfer_complete = true;
}

static void rx_complete_callback(void* data, uint32_t size) {
    memcpy(rx_buffer, data, size);
    transfer_complete = true;
}

// Test functions
static void test_uart_performance_mode(uart_port_t port, uint16_t buffer_size, bool use_dma, perf_metrics_t* metrics) {
    error_code_t err;
    uint32_t start_time, end_time;
    uint32_t total_bytes = 0;
    
    // Initialize test data
    for (uint16_t i = 0; i < buffer_size; i++) {
        tx_buffer[i] = i & 0xFF;
    }
    
    // Configure UART
    uart_config_t config = {
        .baud_rate = BAUD_115200,
        .data_bits = 8,
        .stop_bits = 1,
        .parity = 0,
        .rx_buffer_size = buffer_size,
        .tx_buffer_size = buffer_size,
        .flow_control = false,
        .tx_callback = tx_complete_callback,
        .rx_callback = rx_complete_callback
    };
    
    if (use_dma) {
        config.dma_flags = UART_FLAG_DMA_TX | UART_FLAG_DMA_RX;
        config.dma_priority = DMA_PRIORITY_HIGH;
    }
    
    // Initialize UART
    err = uart_init(port, &config);
    if (err != ERROR_CODE_SUCCESS) {
        printf("UART initialization failed: %d\n", err);
        return;
    }
    
    // Measure memory usage
    metrics->memory_used = sizeof(uart_config_t) + (2 * buffer_size);
    if (use_dma) {
        metrics->memory_used += (2 * buffer_size); // DMA buffers
    }
    
    // Performance test loop
    start_time = timer_get_ms();
    for (uint16_t i = 0; i < TEST_ITERATIONS; i++) {
        transfer_complete = false;
        
        // Send data
        err = uart_write(port, tx_buffer, buffer_size);
        if (err != ERROR_CODE_SUCCESS) {
            metrics->error_count++;
            continue;
        }
        
        // Wait for transfer completion
        uint32_t timeout = 100;
        while (!transfer_complete && timeout > 0) {
            _delay_ms(1);
            timeout--;
        }
        
        if (timeout == 0) {
            metrics->error_count++;
        } else {
            total_bytes += buffer_size;
        }
        
        // Check for overflow
        if (uart_rx_available(port) == buffer_size) {
            metrics->overflow_count++;
        }
    }
    end_time = timer_get_ms();
    
    // Calculate metrics
    uint32_t duration_ms = end_time - start_time;
    metrics->throughput = (total_bytes * 1000) / duration_ms; // Bytes per second
    
    // Get DMA status if enabled
    if (use_dma) {
        dma_status_t dma_status;
        dma_get_status(DMA_UART_TX_CHANNEL, &dma_status);
        metrics->interrupts_per_byte = (float)dma_status.transfers_complete / total_bytes;
    } else {
        // Estimate interrupt count for non-DMA mode (2 interrupts per byte - TX and RX)
        metrics->interrupts_per_byte = 2.0;
    }
    
    // Cleanup
    uart_deinit(port);
}

void test_uart_performance(void) {
    perf_metrics_t metrics_small, metrics_medium, metrics_large;
    perf_metrics_t metrics_small_dma, metrics_medium_dma, metrics_large_dma;
    
    printf("\n=== UART Driver Performance Test Suite ===\n\n");
    
    // Test with different buffer sizes
    printf("Testing small buffer (32 bytes)...\n");
    test_uart_performance_mode(UART_PORT_1, TEST_BUFFER_SMALL, false, &metrics_small);
    test_uart_performance_mode(UART_PORT_1, TEST_BUFFER_SMALL, true, &metrics_small_dma);
    
    printf("Testing medium buffer (128 bytes)...\n");
    test_uart_performance_mode(UART_PORT_1, TEST_BUFFER_MEDIUM, false, &metrics_medium);
    test_uart_performance_mode(UART_PORT_1, TEST_BUFFER_MEDIUM, true, &metrics_medium_dma);
    
    printf("Testing large buffer (512 bytes)...\n");
    test_uart_performance_mode(UART_PORT_1, TEST_BUFFER_LARGE, false, &metrics_large);
    test_uart_performance_mode(UART_PORT_1, TEST_BUFFER_LARGE, true, &metrics_large_dma);
    
    // Print results
    printf("\n=== Performance Test Results ===\n\n");
    
    printf("Small Buffer (32 bytes):\n");
    printf("Standard Mode:\n");
    printf("- Throughput: %.2f KB/s\n", metrics_small.throughput / 1024.0f);
    printf("- Interrupts per byte: %.2f\n", metrics_small.interrupts_per_byte);
    printf("- Memory used: %lu bytes\n", metrics_small.memory_used);
    printf("DMA Mode:\n");
    printf("- Throughput: %.2f KB/s\n", metrics_small_dma.throughput / 1024.0f);
    printf("- Interrupts per byte: %.2f\n", metrics_small_dma.interrupts_per_byte);
    printf("- Memory used: %lu bytes\n", metrics_small_dma.memory_used);
    
    printf("\nMedium Buffer (128 bytes):\n");
    printf("Standard Mode:\n");
    printf("- Throughput: %.2f KB/s\n", metrics_medium.throughput / 1024.0f);
    printf("- Interrupts per byte: %.2f\n", metrics_medium.interrupts_per_byte);
    printf("- Memory used: %lu bytes\n", metrics_medium.memory_used);
    printf("DMA Mode:\n");
    printf("- Throughput: %.2f KB/s\n", metrics_medium_dma.throughput / 1024.0f);
    printf("- Interrupts per byte: %.2f\n", metrics_medium_dma.interrupts_per_byte);
    printf("- Memory used: %lu bytes\n", metrics_medium_dma.memory_used);
    
    printf("\nLarge Buffer (512 bytes):\n");
    printf("Standard Mode:\n");
    printf("- Throughput: %.2f KB/s\n", metrics_large.throughput / 1024.0f);
    printf("- Interrupts per byte: %.2f\n", metrics_large.interrupts_per_byte);
    printf("- Memory used: %lu bytes\n", metrics_large.memory_used);
    printf("DMA Mode:\n");
    printf("- Throughput: %.2f KB/s\n", metrics_large_dma.throughput / 1024.0f);
    printf("- Interrupts per byte: %.2f\n", metrics_large_dma.interrupts_per_byte);
    printf("- Memory used: %lu bytes\n", metrics_large_dma.memory_used);
    
    printf("\nTest completed!\n");
}
