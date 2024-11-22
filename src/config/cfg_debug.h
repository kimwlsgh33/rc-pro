#ifndef CFG_DEBUG_H
#define CFG_DEBUG_H

#include <stdio.h>

// Debug Configuration
#ifdef DEBUG
#define DEBUG_LEVEL 2
#define ENABLE_DEBUG_OUTPUT 1
#define ENABLE_ASSERT_CHECKS 1
#define ENABLE_PERFORMANCE_METRICS 1
#define ENABLE_LOGGING 1
#define DEBUG_UART_BAUD 115200 // Debug UART baud rate
#define DEBUG_BUFFER_SIZE 128  // Debug message buffer size
#define DPRINTF(args...) printf(args)
#else
#define DEBUG_LEVEL 0
#define ENABLE_DEBUG_OUTPUT 0
#define ENABLE_ASSERT_CHECKS 0
#define ENABLE_PERFORMANCE_METRICS 0
#define ENABLE_LOGGING 0
#define DPRINTF(args...)
#endif

// Test Configuration
// #define TEST_SIM

#endif // CFG_DEBUG_H
