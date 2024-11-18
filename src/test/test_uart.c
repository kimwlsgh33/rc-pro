#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>
#include <stdio.h>

#include "../drivers/mcu/uart/drv_uart.h"
#include "../utils/common/error_codes.h"

// Test configuration
#define TEST_BUFFER_SIZE 128
#define TEST_STRING "Hello UART Test!"
#define TEST_DELAY_MS 100

// Test buffers
static uint8_t tx_buffer[TEST_BUFFER_SIZE];
static uint8_t rx_buffer[TEST_BUFFER_SIZE];
static volatile bool test_complete = false;

// Test results
static struct {
    uint32_t tests_run;
    uint32_t tests_passed;
    uint32_t tests_failed;
} test_stats = {0, 0, 0};

// Forward declarations
static void run_uart_tests(void);
static bool test_uart_init(void);
static bool test_uart_write(void);
static bool test_uart_read(void);
static bool test_uart_overflow(void);
static bool test_uart_error_handling(void);
static void print_test_results(void);

// Main test function
void test_uart_driver(void) {
    printf("\n=== UART Driver Test Suite ===\n");
    
    // Initialize test stats
    test_stats.tests_run = 0;
    test_stats.tests_passed = 0;
    test_stats.tests_failed = 0;
    
    // Run all tests
    run_uart_tests();
    
    // Print results
    print_test_results();
}

// Run all UART tests
static void run_uart_tests(void) {
    bool result;
    
    // Test 1: UART Initialization
    printf("\nTest 1: UART Initialization\n");
    result = test_uart_init();
    test_stats.tests_run++;
    if (result) {
        test_stats.tests_passed++;
        printf("PASS: UART initialization successful\n");
    } else {
        test_stats.tests_failed++;
        printf("FAIL: UART initialization failed\n");
    }
    
    // Test 2: UART Write
    printf("\nTest 2: UART Write Operation\n");
    result = test_uart_write();
    test_stats.tests_run++;
    if (result) {
        test_stats.tests_passed++;
        printf("PASS: UART write test successful\n");
    } else {
        test_stats.tests_failed++;
        printf("FAIL: UART write test failed\n");
    }
    
    // Test 3: UART Read
    printf("\nTest 3: UART Read Operation\n");
    result = test_uart_read();
    test_stats.tests_run++;
    if (result) {
        test_stats.tests_passed++;
        printf("PASS: UART read test successful\n");
    } else {
        test_stats.tests_failed++;
        printf("FAIL: UART read test failed\n");
    }
    
    // Test 4: Buffer Overflow
    printf("\nTest 4: Buffer Overflow Handling\n");
    result = test_uart_overflow();
    test_stats.tests_run++;
    if (result) {
        test_stats.tests_passed++;
        printf("PASS: Buffer overflow handling successful\n");
    } else {
        test_stats.tests_failed++;
        printf("FAIL: Buffer overflow handling failed\n");
    }
    
    // Test 5: Error Handling
    printf("\nTest 5: Error Handling\n");
    result = test_uart_error_handling();
    test_stats.tests_run++;
    if (result) {
        test_stats.tests_passed++;
        printf("PASS: Error handling successful\n");
    } else {
        test_stats.tests_failed++;
        printf("FAIL: Error handling failed\n");
    }
}

// Test UART initialization
static bool test_uart_init(void) {
    uart_config_t config = {
        .baud_rate = BAUD_115200,
        .data_bits = 8,
        .stop_bits = 1,
        .parity = 0,
        .rx_buffer_size = TEST_BUFFER_SIZE,
        .tx_buffer_size = TEST_BUFFER_SIZE,
        .flow_control = false
    };
    
    error_code_t err = uart_init(UART_PORT_1, &config);
    if (err != ERROR_CODE_SUCCESS) {
        printf("Error initializing UART: %d\n", err);
        return false;
    }
    
    // Verify UART status
    const uart_status_t* status = uart_get_status(UART_PORT_1);
    if (!status || status->state != 1) {
        printf("Error: UART not in active state\n");
        return false;
    }
    
    return true;
}

// Test UART write operation
static bool test_uart_write(void) {
    const char* test_data = TEST_STRING;
    error_code_t err = uart_write(UART_PORT_1, (const uint8_t*)test_data, strlen(test_data));
    
    if (err != ERROR_CODE_SUCCESS) {
        printf("Error writing to UART: %d\n", err);
        return false;
    }
    
    // Wait for transmission to complete
    _delay_ms(TEST_DELAY_MS);
    
    // Verify TX buffer is empty
    if (uart_tx_space(UART_PORT_1) != TEST_BUFFER_SIZE) {
        printf("Error: TX buffer not empty after transmission\n");
        return false;
    }
    
    return true;
}

// Test UART read operation
static bool test_uart_read(void) {
    uint16_t bytes_read;
    uint8_t read_data[TEST_BUFFER_SIZE];
    
    // Wait for data
    _delay_ms(TEST_DELAY_MS);
    
    error_code_t err = uart_read(UART_PORT_1, read_data, TEST_BUFFER_SIZE, &bytes_read);
    if (err != ERROR_CODE_SUCCESS) {
        printf("Error reading from UART: %d\n", err);
        return false;
    }
    
    // Verify received data
    if (bytes_read != strlen(TEST_STRING)) {
        printf("Error: Incorrect number of bytes read\n");
        return false;
    }
    
    return true;
}

// Test buffer overflow handling
static bool test_uart_overflow(void) {
    // Fill buffer with more data than it can hold
    uint8_t large_data[TEST_BUFFER_SIZE * 2];
    memset(large_data, 'A', sizeof(large_data));
    
    error_code_t err = uart_write(UART_PORT_1, large_data, sizeof(large_data));
    if (err != ERR_BUFFER_OVERFLOW) {
        printf("Error: Expected buffer overflow error\n");
        return false;
    }
    
    // Verify UART status shows overflow
    const uart_status_t* status = uart_get_status(UART_PORT_1);
    if (!status || status->overflow_count == 0) {
        printf("Error: Overflow not detected in status\n");
        return false;
    }
    
    return true;
}

// Test error handling
static bool test_uart_error_handling(void) {
    // Test invalid port
    error_code_t err = uart_write(UART_PORT_MAX, (uint8_t*)"test", 4);
    if (err != ERR_UART_INVALID_PORT) {
        printf("Error: Expected invalid port error\n");
        return false;
    }
    
    // Test NULL buffer
    err = uart_write(UART_PORT_1, NULL, 4);
    if (err != ERR_INVALID_PARAM) {
        printf("Error: Expected invalid parameter error\n");
        return false;
    }
    
    // Test zero length
    err = uart_write(UART_PORT_1, (uint8_t*)"test", 0);
    if (err != ERR_INVALID_PARAM) {
        printf("Error: Expected invalid parameter error\n");
        return false;
    }
    
    return true;
}

// Print test results
static void print_test_results(void) {
    printf("\n=== Test Results ===\n");
    printf("Tests Run: %lu\n", test_stats.tests_run);
    printf("Tests Passed: %lu\n", test_stats.tests_passed);
    printf("Tests Failed: %lu\n", test_stats.tests_failed);
    printf("Success Rate: %.1f%%\n", 
           (float)test_stats.tests_passed / test_stats.tests_run * 100);
}
