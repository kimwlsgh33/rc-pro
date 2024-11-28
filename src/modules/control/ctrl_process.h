#ifndef CTRL_PROCESS_H
#define CTRL_PROCESS_H

#include "../../config/cfg_build.h"
#include "../../utils/common/error_codes.h"

// Process States
typedef enum {
  PROCESS_STATE_IDLE,
  PROCESS_STATE_STARTING,
  PROCESS_STATE_RUNNING,
  PROCESS_STATE_PAUSING,
  PROCESS_STATE_PAUSED,
  PROCESS_STATE_STOPPING,
  PROCESS_STATE_ERROR
} process_state_t;

// Process Configuration
typedef struct {
  uint16_t startup_timeout_ms;
  uint16_t shutdown_timeout_ms;
  uint16_t heartbeat_interval_ms;
  uint8_t max_retries;
  bool auto_restart;
} process_config_t;

// Process Status
typedef struct {
  process_state_t state;
  uint32_t uptime_ms;
  uint32_t last_heartbeat_ms;
  uint8_t retry_count;
  uint32_t error_flags;
} process_status_t;

// Process Statistics
typedef struct {
  uint32_t total_starts;
  uint32_t clean_stops;
  uint32_t error_stops;
  uint32_t auto_restarts;
  uint32_t total_runtime_ms;
  float cpu_usage_percent;
  float memory_usage_percent;
} process_stats_t;

// Function Declarations
error_code_t process_init(const process_config_t *config);
error_code_t process_configure(const process_config_t *config);
error_code_t process_start(void);
error_code_t process_stop(void);
error_code_t process_pause(void);
error_code_t process_resume(void);
error_code_t process_emergency_stop(void);

// Status and Control
const process_status_t *process_get_status(void);
const process_stats_t *process_get_statistics(void);
error_code_t process_reset_error(void);

// System Processing
void process_heartbeat(void);
void process_process(void);

#endif // CTRL_PROCESS_H
