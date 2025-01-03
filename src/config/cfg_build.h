#ifndef CFG_BUILD_H
#define CFG_BUILD_H

// Version Information
#define FIRMWARE_VERSION_MAJOR 0
#define FIRMWARE_VERSION_MINOR 11
#define FIRMWARE_VERSION_PATCH 0
#define BUILD_VERSION_STRING "0.11.0"
#define BUILD_DATE __DATE__
#define BUILD_TIME __TIME__

// System Clock Configuration
#define SYSTEM_CLOCK_HZ     16000000UL   // 16MHz system clock
#define F_CPU               SYSTEM_CLOCK_HZ  // CPU frequency for AVR toolchain
#define EXTERNAL_CRYSTAL_FREQ_HZ SYSTEM_CLOCK_HZ
#define CFG_SYS_FREQ SYSTEM_CLOCK_HZ     // System frequency for validation

// Timer System Configuration
#define MAX_SYS_TIMER       16           // Maximum number of software timers
#define TIMER_MIN_FREQ      100          // Minimum timer frequency in Hz
#define TIMER_MAX_FREQ      100000       // Maximum timer frequency in Hz
#define TIMER_MIN_INTERVAL  1            // Minimum timer interval in ms
#define TIMER_MAX_INTERVAL  0xFFFFFFFF   // Maximum timer interval in ms

// Timer Hardware Configuration
#define TIMER_PRESCALER     64           // Timer1 prescaler value
#define TIMER_COMPARE_VALUE 250          // For 1ms interval (16MHz/64/250 = 1kHz)

// Safety Configuration
#define WATCHDOG_TIMEOUT    WDTO_250MS   // 250ms watchdog timeout
#define CRITICAL_TEMP_C     70           // Critical temperature in Celsius

// Error Handling
#define MAX_ERROR_HISTORY   16           // Number of errors to keep in history
#define ERROR_LOG_ENABLED   1            // Enable error logging

// Debug Configuration
#ifdef DEBUG
    #define DEBUG_UART_BAUD 115200      // Debug UART baud rate
    #define DEBUG_BUFFER_SIZE 128        // Debug message buffer size
#endif

#endif // CFG_BUILD_H
