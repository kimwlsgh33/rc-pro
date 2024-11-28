#ifndef LOGGING_H
#define LOGGING_H

#include "error_codes.h"

// Log Levels
typedef enum {
    LOG_LEVEL_NONE = 0,
    LOG_LEVEL_ERROR = 1,
    LOG_LEVEL_WARN = 2,
    LOG_LEVEL_INFO = 3,
    LOG_LEVEL_DEBUG = 4,
    LOG_LEVEL_VERBOSE = 5
} log_level_t;

// Log Categories
typedef enum {
    LOG_CAT_SYSTEM,
    LOG_CAT_MODULE,
    LOG_CAT_DRIVER,
    LOG_CAT_NETWORK,
    LOG_CAT_SAFETY,
    LOG_CAT_USER,
    LOG_CAT_CUSTOM
} log_category_t;

// Log Entry
typedef struct {
    log_level_t level;
    log_category_t category;
    uint32_t timestamp;
    uint16_t source_id;
    char message[128];
    error_code_t error_code;
} log_entry_t;

// Log Statistics
typedef struct {
    uint32_t total_entries;
    uint32_t error_entries;
    uint32_t warn_entries;
    uint32_t info_entries;
    uint32_t debug_entries;
    uint32_t dropped_entries;
} log_stats_t;

// Logging Configuration
typedef struct {
    log_level_t min_level;
    bool enable_timestamp;
    bool enable_categories;
    bool enable_source_id;
    bool enable_error_codes;
    void (*output_callback)(const log_entry_t* entry);
} log_config_t;

// Logging Interface
error_code_t log_init(const log_config_t* config);
error_code_t log_deinit(void);

// Logging Functions
#if ENABLE_LOGGING
    #define LOG_ERROR(cat, msg, ...) log_write(LOG_LEVEL_ERROR, cat, msg, ##__VA_ARGS__)
    #define LOG_WARN(cat, msg, ...)  log_write(LOG_LEVEL_WARN, cat, msg, ##__VA_ARGS__)
    #define LOG_INFO(cat, msg, ...)  log_write(LOG_LEVEL_INFO, cat, msg, ##__VA_ARGS__)
    #define LOG_DEBUG(cat, msg, ...) log_write(LOG_LEVEL_DEBUG, cat, msg, ##__VA_ARGS__)
    #define LOG_VERBOSE(cat, msg, ...) log_write(LOG_LEVEL_VERBOSE, cat, msg, ##__VA_ARGS__)
#else
    #define LOG_ERROR(cat, msg, ...)
    #define LOG_WARN(cat, msg, ...)
    #define LOG_INFO(cat, msg, ...)
    #define LOG_DEBUG(cat, msg, ...)
    #define LOG_VERBOSE(cat, msg, ...)
#endif

// Core Logging Functions
error_code_t log_write(log_level_t level, log_category_t category, const char* format, ...);
error_code_t log_write_error(log_category_t category, error_code_t error_code, const char* format, ...);

// Log Management
error_code_t log_set_level(log_level_t level);
error_code_t log_enable_category(log_category_t category);
error_code_t log_disable_category(log_category_t category);
error_code_t log_clear(void);

// Log Retrieval
const log_entry_t* log_get_last_entry(void);
const log_stats_t* log_get_statistics(void);
error_code_t log_get_entries(log_entry_t* entries, uint32_t* count);

// Log Configuration
error_code_t log_set_config(const log_config_t* config);
const log_config_t* log_get_config(void);

// Utility Functions
const char* log_level_to_string(log_level_t level);
const char* log_category_to_string(log_category_t category);

#endif // LOGGING_H
