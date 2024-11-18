#ifndef CUP_ROUTING_H
#define CUP_ROUTING_H

#include "types.h"
#include "cup_detection.h"

// Routing strategies
typedef enum {
    ROUTE_STRATEGY_OPTIMAL,
    ROUTE_STRATEGY_ENERGY_SAVING,
    ROUTE_STRATEGY_SPEED_PRIORITY,
    ROUTE_STRATEGY_LOAD_BALANCING
} routing_strategy_t;

// Route status
typedef enum {
    ROUTE_STATUS_IDLE,
    ROUTE_STATUS_IN_PROGRESS,
    ROUTE_STATUS_COMPLETED,
    ROUTE_STATUS_ERROR
} route_status_t;

// Route configuration
typedef struct {
    routing_strategy_t strategy;
    uint16_t max_queue_size;
    uint16_t path_timeout_ms;
    uint8_t retry_attempts;
} route_config_t;

// Route statistics
typedef struct {
    uint32_t total_cups_processed;
    uint32_t successful_routes;
    uint32_t failed_routes;
    uint16_t average_routing_time_ms;
    uint8_t current_queue_size;
} route_stats_t;

// Function declarations
void cr_init(void);
error_code_t cr_configure_routing(const route_config_t* config);
error_code_t cr_process_cup(const cup_info_t* cup_info);
route_status_t cr_get_route_status(void);
route_stats_t cr_get_statistics(void);
void cr_optimize_paths(void);
void cr_process(void);

#endif // CUP_ROUTING_H
