/*
 * RCIX.B2 main
 *
 * Created: 2023-02-02 오전 11:16:43
 * Author : tchan
 */

#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/wdt.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <util/atomic.h>
#include <util/delay.h>

#include "../config/cfg_debug.h"
#include "../config/cfg_hardware.h"
#include "../config/cfg_system.h"
#include "../drivers/mcu/timer/drv_timer.h"
#include "../drivers/mcu/uart/drv_uart.h"
#include "../modules/control/ctrl_motor.h"
#include "../modules/control/ctrl_process.h"
#include "../modules/control/ctrl_recycle.h"
#include "../platform/avr/plat_avr.h"
#include "../test/test_uart.h"
#include "../test/test_uart_perf.h"
#include "../utils/common/error_codes.h"
#include "app_main.h"

// Debug LED control
#define LED_ON()     _CLR(PORTB, 7)
#define LED_OFF()    _SET(PORTB, 7)
#define LED_TOGGLE() _XOR(PORTB, 7)

// Protocol definitions
#define DCP_MAX_PACKET_LEN  128
#define CHAR_CR             '\r'
#define CHAR_LF             '\n'
#define COMM_MAX_RETRIES    3
#define COMM_RETRY_DELAY_MS 10

// Buffer status flags
static volatile bool rx_buffer_full = false;
static volatile bool tx_buffer_busy = false;

// Error handling
#define CHECK_ERROR(x)                                                                             \
  do {                                                                                             \
    error_code_t err = (x);                                                                        \
    if (err != ERR_SUCCESS) {                                                                      \
      printf("Error at %s:%d - %d\n", __FILE__, __LINE__, err);                                    \
      return err;                                                                                  \
    }                                                                                              \
  } while (0)

typedef struct {
  uint8_t sv_mode;     // Sensor value send mode (0, 1)
  uint8_t st_mode;     // Status send mode (0, 1)
  uint8_t st_interval; // Status send interval (0..60)
  uint16_t mon_height; // Monitor height in mm (default: 500)
  char mark[4];        // Version mark ("RR01")
} app_config_t;

// Static variables
static volatile app_config_t app_config;
static volatile bool system_initialized = false;

// Secure command buffer implementation
typedef struct {
  char buffer[128];
  volatile uint16_t length;
  volatile bool in_use;
} secure_buffer_t;

static secure_buffer_t cmd_buffer;
static secure_buffer_t response_buffer;

// Initialize secure buffers
static void init_secure_buffers(void)
{
  memset(&cmd_buffer, 0, sizeof(cmd_buffer));
  memset(&response_buffer, 0, sizeof(response_buffer));
}

// Safe buffer access functions
static error_code_t secure_buffer_write(secure_buffer_t *buf, const char *data, size_t len)
{
  if (!buf || !data || len == 0) {
    return ERR_INVALID_PARAM;
  }

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    if (buf->in_use) {
      return ERR_BUSY;
    }

    if (len >= sizeof(buf->buffer)) {
      error_record(ERR_OVERFLOW);
      return ERR_OVERFLOW;
    }

    buf->in_use = true;
    memcpy(buf->buffer, data, len);
    buf->buffer[len] = '\0';
    buf->length      = len;
    buf->in_use      = false;
  }

  return ERR_SUCCESS;
}

static error_code_t secure_buffer_read(secure_buffer_t *buf, char *out, size_t max_len)
{
  if (!buf || !out || max_len == 0) {
    return ERR_INVALID_PARAM;
  }

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    if (buf->in_use) {
      return ERR_BUSY;
    }

    size_t copy_len = (buf->length < max_len) ? buf->length : max_len - 1;
    memcpy(out, buf->buffer, copy_len);
    out[copy_len] = '\0';
  }

  return ERR_SUCCESS;
}

static void secure_buffer_clear(secure_buffer_t *buf)
{
  if (!buf) {
    return;
  }

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    memset(buf->buffer, 0, sizeof(buf->buffer));
    buf->length = 0;
    buf->in_use = false;
  }
}

// Protocol buffers
static uint8_t rx_buff[DCP_MAX_PACKET_LEN];
static uint8_t rx_len;

// Function declarations
static error_code_t init_hardware(void);
static error_code_t init_peripherals(void);
static error_code_t load_configuration(void);
static error_code_t process_commands(void);
static error_code_t send_status_update(void);
static error_code_t handle_events(void);
static error_code_t mc_send_response(const char *res);
static error_code_t dcps_send_response(const char *packet, int len);
static int dcps_get_packet(char *out_buff);
static void error_handler(error_code_t err);

// Protocol handling functions
static error_code_t uart_send_string(uart_port_t port, const char *str)
{
  if (!str)
    return ERR_INVALID_PARAM;
  return uart_write(port, (const uint8_t *)str, strlen(str));
}

static error_code_t mc_send_response(const char *res)
{
  if (!res)
    return ERR_INVALID_PARAM;
  return uart_send_string(UART_PORT_1, res);
}

static error_code_t mc_send_response_with_retry(const char *res)
{
  error_code_t err;
  int retries = 0;

  while (retries < COMM_MAX_RETRIES) {
    err = mc_send_response(res);
    if (err == ERR_SUCCESS) {
      return ERR_SUCCESS;
    }
    _delay_ms(COMM_RETRY_DELAY_MS);
    retries++;
  }

  return ERR_TIMEOUT;
}

static error_code_t dcps_send_response(const char *packet, int len)
{
  if (!packet || len <= 0)
    return ERR_INVALID_PARAM;
  return uart_write(UART_PORT_2, (const uint8_t *)packet, len);
}

static error_code_t dcps_send_response_with_retry(const char *packet, int len)
{
  error_code_t err;
  int retries = 0;

  while (retries < COMM_MAX_RETRIES) {
    err = dcps_send_response(packet, len);
    if (err == ERR_SUCCESS) {
      return ERR_SUCCESS;
    }
    _delay_ms(COMM_RETRY_DELAY_MS);
    retries++;
  }

  return ERR_TIMEOUT;
}

static int dcps_get_packet(char *out_buff)
{
  if (!out_buff) {
    return -ERR_INVALID_PARAM;
  }

  static uint8_t rx_buff[DCP_MAX_PACKET_LEN];
  static uint16_t rx_len = 0;
  uint8_t c;

  while (uart_rx_available(UART_PORT_2) > 0) {
    if (uart_read_byte(UART_PORT_2, &c) == ERR_SUCCESS) {
      // Check for buffer space before adding byte
      if (rx_len < DCP_MAX_PACKET_LEN - 1) { // Leave space for null terminator
        rx_buff[rx_len++] = c;

        // Check for packet end
        if (c == CHAR_LF && rx_len >= 2 && rx_buff[rx_len - 2] == CHAR_CR) {
          // Validate packet size
          if (rx_len > sizeof(rx_buff)) {
            error_record(ERR_OVERFLOW);
            rx_len = 0;
            memset(rx_buff, 0, sizeof(rx_buff));
            return -ERR_OVERFLOW;
          }

          // Safe copy to output buffer
          size_t copy_len = rx_len;
          if (copy_len > DCP_MAX_PACKET_LEN - 1) {
            copy_len = DCP_MAX_PACKET_LEN - 1;
            error_record(ERR_OVERFLOW);
          }

          memcpy(out_buff, rx_buff, copy_len);
          out_buff[copy_len] = '\0'; // Ensure null termination

          // Clear and reset receive buffer
          int len = copy_len;
          rx_len  = 0;
          memset(rx_buff, 0, sizeof(rx_buff));

          return len;
        }
      } else {
        // Buffer overflow - reset and record error
        error_record(ERR_OVERFLOW);
        rx_len = 0;
        memset(rx_buff, 0, sizeof(rx_buff));
        return -ERR_OVERFLOW;
      }
    }
  }

  return -1; // No complete packet available
}

// Safe string formatting helper
static error_code_t safe_format_response(char *buffer, size_t size, const char *format, ...)
{
  if (!buffer || !format || size == 0) {
    return ERR_INVALID_PARAM;
  }

  va_list args;
  va_start(args, format);
  int result = vsnprintf(buffer, size, format, args);
  va_end(args);

  if (result < 0 || (size_t)result >= size) {
    error_record(ERR_OVERFLOW);
    buffer[size - 1] = '\0'; // Ensure null termination
    return ERR_OVERFLOW;
  }

  return ERR_SUCCESS;
}

// Event callbacks
void rm_ev_monitor_state(char st, int position)
{
  error_code_t err = safe_format_response(response_buffer.buffer, sizeof(response_buffer.buffer),
                                          "MP.ST=%c,%d", st, position);
  if (err != ERR_SUCCESS) {
    error_record(err);
    return;
  }

  err = mc_send_response_with_retry(response_buffer.buffer);
  if (err != ERR_SUCCESS) {
    error_record(ERR_TIMEOUT);
    printf("Error sending monitor state: %d\n", err);
  }
}

void rm_ev_gd_state(char st, int position)
{
  error_code_t err = safe_format_response(response_buffer.buffer, sizeof(response_buffer.buffer),
                                          "GD.ST=%c,%d", st, position);
  if (err != ERR_SUCCESS) {
    error_record(err);
    return;
  }

  err = mc_send_response_with_retry(response_buffer.buffer);
  if (err != ERR_SUCCESS) {
    error_record(ERR_TIMEOUT);
    printf("Error sending door state: %d\n", err);
  }
}

void rm_ev_completed(void)
{
  error_code_t err =
      safe_format_response(response_buffer.buffer, sizeof(response_buffer.buffer), "OP.COMPLETED");
  if (err != ERR_SUCCESS) {
    error_record(err);
    return;
  }

  err = mc_send_response_with_retry(response_buffer.buffer);
  if (err != ERR_SUCCESS) {
    error_record(ERR_TIMEOUT);
    printf("Error sending completion notification: %d\n", err);
  }
}

// Watchdog reset function
static inline void reset_watchdog(void) { wdt_reset(); }

// Configuration structure
typedef struct {
  uint8_t sv_mode;     // Sensor value send mode (0, 1)
  uint8_t st_mode;     // Status send mode (0, 1)
  uint8_t st_interval; // Status send interval (0..60)
  uint16_t mon_height; // Monitor height in mm (default: 500)
  char mark[4];        // Version mark ("RR01")
} app_config_t;

// Configuration functions
static error_code_t cfg_load(app_config_t *pc)
{
  if (!pc) {
    return ERR_INVALID_PARAM;
  }

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    eeprom_read_block(pc, (const void *)0, sizeof(app_config_t));
  }

  return ERR_SUCCESS;
}

static error_code_t cfg_save(const app_config_t *pc)
{
  if (!pc) {
    return ERR_INVALID_PARAM;
  }

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { eeprom_write_block(pc, (void *)0, sizeof(app_config_t)); }

  return ERR_SUCCESS;
}

static bool cfg_isValid(const app_config_t *pc)
{
  if (!pc) {
    return false;
  }

  // Check mode values
  if (pc->sv_mode > 1 || pc->st_mode > 1) {
    return false;
  }

  // Check interval range
  if (pc->st_interval > 60) {
    return false;
  }

  // Check monitor height
  if (pc->mon_height < 100 || pc->mon_height > 1000) {
    return false;
  }

  // Check version mark
  if (memcmp(pc->mark, "RR01", 4) != 0) {
    return false;
  }

  return true;
}

// Global error history
static error_history_t error_history;
static error_callback_t error_callbacks[8] = {NULL};
static uint8_t num_callbacks               = 0;

void error_handler(error_code_t err)
{
  error_info_t error_info = {
      .timestamp = timer_get_ms(),
      .code      = err,
      .severity  = ERROR_SEVERITY_ERROR // Default severity
  };

  // Determine error severity based on error code
  if ((err & ERR_CAT_MASK) == ERR_CAT_CRITICAL) {
    error_info.severity = ERROR_SEVERITY_CRITICAL;
  } else if ((err & ERR_CAT_MASK) == ERR_CAT_SYSTEM) {
    error_info.severity = ERROR_SEVERITY_FATAL;
  }

  // Store error in history
  error_history.errors[error_history.index] = error_info;
  error_history.index                       = (error_history.index + 1) % ERROR_HISTORY_SIZE;
  if (error_history.count < ERROR_HISTORY_SIZE) {
    error_history.count++;
  }

  // Notify all registered callbacks
  for (uint8_t i = 0; i < num_callbacks; i++) {
    if (error_callbacks[i]) {
      error_callbacks[i](&error_info);
    }
  }

  // Handle error based on severity
  switch (error_info.severity) {
  case ERROR_SEVERITY_INFO:
    // Log only
    break;

  case ERROR_SEVERITY_WARNING:
    LED_TOGGLE(); // Visual indication
    break;

  case ERROR_SEVERITY_ERROR:
    // Try to recover
    if (!attempt_error_recovery(err)) {
      error_info.severity = ERROR_SEVERITY_CRITICAL;
      goto critical_error;
    }
    break;

  case ERROR_SEVERITY_CRITICAL:
  critical_error:
    // Attempt safe shutdown
    safe_shutdown();
    // Fall through to fatal if shutdown fails

  case ERROR_SEVERITY_FATAL:
    // Fatal error - halt system
    cli(); // Disable interrupts
    LED_ON();
    while (1) {
      reset_watchdog();
      _delay_ms(100);
      LED_TOGGLE(); // Blink LED to indicate fatal error
    }
    break;
  }
}

// Register an error callback
error_code_t register_error_callback(error_callback_t callback)
{
  if (num_callbacks >= sizeof(error_callbacks) / sizeof(error_callbacks[0])) {
    return ERR_BUFFER_FULL;
  }
  error_callbacks[num_callbacks++] = callback;
  return ERR_SUCCESS;
}

// Attempt to recover from an error
static bool attempt_error_recovery(error_code_t err)
{
  switch (err & ERR_CAT_MASK) {
  case ERR_CAT_COMMUNICATION:
    return uart_error_recovery();

  case ERR_CAT_DRIVER:
    return dma_error_recovery();

  case ERR_CAT_PERIPHERAL:
    return peripheral_error_recovery();

  default:
    return false;
  }
}

// Perform safe system shutdown
static void safe_shutdown(void)
{
  // Disable all interrupts
  cli();

  // Stop all ongoing DMA transfers
  dma_stop_all_transfers();

  // Flush UART buffers
  uart_flush_all_ports();

  // Disable peripherals
  peripheral_shutdown();

  // Save critical data to EEPROM
  save_critical_data();

  // Enable watchdog with short timeout to force reset
  wdt_enable(WDTO_15MS);
  while (1)
    ;
}

int main(void)
{
  error_code_t err;

  // Initialize platform (disables watchdog and configures clock)
  err = plat_avr_init();
  if (err != ERR_SUCCESS) {
    error_handler(err);
    goto error;
  }

  // Initialize secure buffers
  init_secure_buffers();

  // Initialize hardware
  err = init_hardware();
  if (err != ERR_SUCCESS) {
    error_handler(err);
    return -1;
  }

  // Initialize peripherals
  err = init_peripherals();
  if (err != ERR_SUCCESS) {
    error_handler(err);
    return -1;
  }

// Run tests in debug mode
#ifdef DEBUG_MODE
  test_uart_driver();
  test_uart_performance();
#endif

  // Load configuration
  err = load_configuration();
  if (err != ERR_SUCCESS) {
    error_handler(err);
    return -1;
  }

  // Enable watchdog with 2s timeout
  plat_avr_watchdog_enable(WDTO_2S);

  // Enable interrupts - everything is initialized
  plat_avr_enable_interrupts();

  // Print startup message
  printf("Recycling Control System v%s\n", SYSTEM_VERSION);
  printf("Build: %s %s\n", BUILD_DATE, BUILD_TIME);

  // Initial LED state
  LED_OFF();

  // Set initial timers
  timer_set(timer_op, 500);
  timer_set(timer_status, app_config.st_interval * 1000UL);

  // First loop execution
  process_commands();
  us_process();
  rm_process();
  handle_events();

  // System is now initialized
  system_initialized = true;
  printf("System initialized successfully\n");

  // Main loop
  while (1) {
    // Reset watchdog
    reset_watchdog();

    // Process commands
    process_commands();

    // Process ultrasonic sensor
    us_process();

    // Process application
    rm_process();

    // Handle system events
    handle_events();

    // Status update
    if (timer_isfired(timer_status)) {
      timer_set(timer_status, app_config.st_interval * 1000UL);
      send_status_update();
    }

    // Heartbeat LED
    if (timer_isfired(timer_op)) {
      timer_set(timer_op, 500);
      LED_TOGGLE();

      // Send sensor value if enabled
      if (app_config.sv_mode && rm_is_sensor_updated()) {
        error_code_t err = safe_format_response(
            response_buffer.buffer, sizeof(response_buffer.buffer), "SS=%d", rm_get_sensorValue());
        if (err != ERR_SUCCESS) {
          error_record(err);
        } else {
          mc_send_response_with_retry(response_buffer.buffer);
        }
      }
    }
  }

error:
  // Error handling
  wdt_disable(); // Disable watchdog in error state
  LED_ON();      // Indicate error state
  while (1) {
    _delay_ms(1000);
    LED_TOGGLE(); // Blink LED in error state
  }

  return 0; // Never reached
}

static error_code_t init_hardware(void)
{
  // Configure system clock
  CLKPR = (1 << CLKPCE); // Enable clock prescaler change
  CLKPR = 0;             // Set prescaler to 1 (16MHz)

  // Configure LED pin
  DDRB |= (1 << 7); // Set PB7 as output
  LED_OFF();

  // Configure ultrasonic sensor control pin
  DDRE |= (1 << 4);  // Set PE4 as output
  PORTE |= (1 << 4); // Set PE4 high

  // Initialize timer system
  error_code_t status = timer_init();
  if (status != ERR_SUCCESS) {
    return status;
  }

  // Allocate system timer
  timer_id = timer_alloc();
  if (timer_id < 0) {
    error_record(ERR_TIMER_ALLOC);
    return ERR_TIMER_ALLOC;
  }

  // Configure system timer for 1ms tick
  status = timer_set(timer_id, 1000); // 1ms period
  if (status != ERR_SUCCESS) {
    timer_free(timer_id);
    timer_id = -1;
    return status;
  }

  // Initialize watchdog
  plat_avr_watchdog_enable(WDTO_2S);

  return ERR_SUCCESS;
}

static error_code_t init_peripherals(void)
{
  error_code_t status;

  // Initialize UART with error checking
  status = uart_init(UART_BAUD_115200);
  if (status != ERR_SUCCESS) {
    error_record(ERR_UART_INIT);
    return status;
  }

  // Initialize other peripherals with proper cleanup on failure
  status = adc_init();
  if (status != ERR_SUCCESS) {
    uart_deinit();
    error_record(ERR_ADC_INIT);
    return status;
  }

  status = spi_init();
  if (status != ERR_SUCCESS) {
    adc_deinit();
    uart_deinit();
    error_record(ERR_SPI_INIT);
    return status;
  }

  return ERR_SUCCESS;
}

static error_code_t load_configuration(void)
{
  error_code_t err;

  // Load configuration from EEPROM
  err = cfg_load(&app_config);
  if (err != ERR_SUCCESS) {
    error_record(ERR_SYS_CONFIG);
    printf("Error loading configuration: %d\n", err);
    goto set_defaults;
  }

  // Validate configuration
  if (!cfg_isValid(&app_config)) {
    error_record(ERR_SYS_CONFIG);
    printf("Invalid configuration, using defaults\n");
    goto set_defaults;
  }

  printf("Configuration loaded successfully\n");
  return ERR_SUCCESS;

set_defaults:
  // Set defaults if invalid
  app_config.sv_mode     = 0;
  app_config.st_mode     = 0;
  app_config.st_interval = 5;
  app_config.mon_height  = 500;
  memcpy(app_config.mark, "RR01", 4);

  // Save defaults
  err = cfg_save(&app_config);
  if (err != ERR_SUCCESS) {
    printf("Error saving default configuration: %d\n", err);
    return err;
  }

  printf("Default configuration saved\n");
  return ERR_SUCCESS;
}

static error_code_t process_commands(void)
{
  error_code_t err = ERR_SUCCESS;

  // Process device commands with enhanced bounds checking and validation
  if (cmd_buffer.buffer && sizeof(cmd_buffer.buffer) > 0) {
    // Clear buffer before use for security
    secure_buffer_clear(&cmd_buffer);

    int cmd_len = dcps_get_packet(cmd_buffer.buffer);
    if (cmd_len > 0) {
      // Strict bounds checking
      if (cmd_len >= sizeof(cmd_buffer.buffer)) {
        error_record(ERR_OVERFLOW);
        log_safety_event(SAFETY_EVENT_BUFFER_OVERFLOW);
        printf("Command buffer overflow detected\n");
        cmd_len = sizeof(cmd_buffer.buffer) - 1;
        err     = ERR_OVERFLOW;
      }

      // Validate command format and content
      if (!validate_command_format(cmd_buffer.buffer, cmd_len)) {
        error_record(ERR_INVALID_COMMAND);
        log_safety_event(SAFETY_EVENT_INVALID_COMMAND);
        printf("Invalid command format detected\n");
        err = ERR_INVALID_COMMAND;
        goto cleanup;
      }

      // Ensure null termination
      cmd_buffer.buffer[cmd_len] = '\0';

      // Log command for debugging with sanitization
      char sanitized_buffer[32];
      sanitize_command_log(cmd_buffer.buffer, sanitized_buffer, sizeof(sanitized_buffer));
      printf("Device command received: %s\n", sanitized_buffer);

      // Process command with error handling
      err = mc_processCommand(cmd_buffer.buffer, cmd_len);
      if (err != ERR_SUCCESS) {
        error_record(ERR_TIMEOUT);
        log_safety_event(SAFETY_EVENT_COMMAND_ERROR);
        printf("Error processing device command: %d\n", err);
      }
    }
  }

  // Process console commands with similar protections
  if (cmd_buffer.buffer && sizeof(cmd_buffer.buffer) > 0) {
    // Clear buffer before use
    secure_buffer_clear(&cmd_buffer);

    int cmd_len = cons_get_command_line(cmd_buffer.buffer);
    if (cmd_len > 0) {
      // Strict bounds checking
      if (cmd_len >= sizeof(cmd_buffer.buffer)) {
        error_record(ERR_OVERFLOW);
        log_safety_event(SAFETY_EVENT_BUFFER_OVERFLOW);
        printf("Command buffer overflow detected\n");
        cmd_len = sizeof(cmd_buffer.buffer) - 1;
        err     = ERR_OVERFLOW;
      }

      // Validate command format
      if (!validate_command_format(cmd_buffer.buffer, cmd_len)) {
        error_record(ERR_INVALID_COMMAND);
        log_safety_event(SAFETY_EVENT_INVALID_COMMAND);
        printf("Invalid command format detected\n");
        err = ERR_INVALID_COMMAND;
        goto cleanup;
      }

      // Ensure null termination
      cmd_buffer.buffer[cmd_len] = '\0';

      // Log sanitized command
      char sanitized_buffer[32];
      sanitize_command_log(cmd_buffer.buffer, sanitized_buffer, sizeof(sanitized_buffer));
      printf("Console command received: %s\n", sanitized_buffer);

      // Process command with error handling
      err = cons_processCommand(cmd_buffer.buffer, cmd_len);
      if (err != ERR_SUCCESS) {
        error_record(ERR_TIMEOUT);
        log_safety_event(SAFETY_EVENT_COMMAND_ERROR);
        printf("Error processing console command: %d\n", err);
      }
    }
  }

cleanup:
  // Secure cleanup - clear sensitive data
  secure_buffer_clear(&cmd_buffer);

  return err;
}

static bool validate_command_format(const char *cmd, size_t len)
{
  if (!cmd || len == 0)
    return false;

  // Check for control characters
  for (size_t i = 0; i < len; i++) {
    if (cmd[i] < 32 && cmd[i] != '\r' && cmd[i] != '\n' && cmd[i] != '\t') {
      return false;
    }
  }

  // Validate command structure (example)
  if (len < 3 || cmd[0] != '$')
    return false;

  return true;
}

static void sanitize_command_log(const char *cmd, char *out, size_t out_size)
{
  if (!cmd || !out || out_size == 0)
    return;

  size_t i, j;
  for (i = 0, j = 0; cmd[i] && j < out_size - 1; i++) {
    if (cmd[i] >= 32 && cmd[i] <= 126) { // Printable ASCII only
      out[j++] = cmd[i];
    } else {
      // Replace non-printable chars with '?'
      out[j++] = '?';
    }
  }
  out[j] = '\0';
}

static error_code_t send_status_update(void)
{
  error_code_t err;
  RmcState state;

  // Check if status updates are enabled
  if (!app_config.st_mode) {
    return ERR_SUCCESS;
  }

  // Get current system state
  state = rm_get_state();

  // Format status message
  error_code_t err = safe_format_response(
      response_buffer.buffer, sizeof(response_buffer.buffer), "ST=%c,%d,%c,%d,%d,%d,%d,%d,%d,%d",
      state.state, state.rstate, state.door, state.monotorPosition, state.plasticCups,
      state.paperCups, state.purifiedTankLevel, state.wasteTankLevel, state.errorCode,
      state.errorRoboSeq);
  if (err != ERR_SUCCESS) {
    error_record(err);
    return err;
  }

  // Send status update
  err = mc_send_response_with_retry(response_buffer.buffer);
  if (err != ERR_SUCCESS) {
    error_record(ERR_TIMEOUT);
    printf("Error sending status update: %d\n", err);
  }

  return err;
}

static error_code_t handle_events(void)
{
  error_code_t err;
  RcOpcEvent event;

  // Check for operation events
  if (rc_is_opc_event_updated()) {
    // Get event data
    event = rc_get_opcEvent();

    // Format event message
    error_code_t err = safe_format_response(response_buffer.buffer, sizeof(response_buffer.buffer),
                                            "OP.CUP=%d", event.cupMaterial);
    if (err != ERR_SUCCESS) {
      error_record(err);
      return err;
    }

    // Send event notification
    err = mc_send_response_with_retry(response_buffer.buffer);
    if (err != ERR_SUCCESS) {
      error_record(ERR_TIMEOUT);
      printf("Error sending event notification: %d\n", err);
    }
  }

  return err;
}

static error_code_t mc_processCommand(const char *cmdBuff, int cmdLen)
{
  if ((cmdLen <= 0) || (cmdBuff == 0))
    return ERR_INVALID_PARAM;

  printf("mc_processCommand: %s\n", cmdBuff);

  if (strcmp(cmdBuff, "ID?") == 0) {
    mc_send_response_with_retry("RC v1.0");
  } else if (strncmp(cmdBuff, "MP=", 3) == 0) {
    int height = atoi(cmdBuff + 3);
    if ((height < 0) || (height > app_config.mon_height)) {
      mc_send_response_with_retry("ERROR2"); // invalid parameter
      return ERR_INVALID_PARAM;
    } else {
      rm_mon_move_to(height);
      mc_send_response_with_retry("OK");
      return ERR_SUCCESS;
    }
  } else if (strcmp(cmdBuff, "START") == 0) {
    if (rm_start(0) == 0) {
      mc_send_response_with_retry("OK");
    } else {
      mc_send_response_with_retry("ERROR1");
    }
  } else if (strncmp(cmdBuff, "START.", 6) == 0) {
    int val = atoi(cmdBuff + 6);
    if ((val < 1) || (val > 6)) {
      mc_send_response_with_retry("ERROR2"); // invalid parameter
      return ERR_INVALID_PARAM;
    } else if (rm_start(val - 1) == 0) {
      mc_send_response_with_retry("OK");
    } else {
      mc_send_response_with_retry("ERROR1");
    }
  } else if (strcmp(cmdBuff, "STOP") == 0) {
    if (rm_stop() == 0) {
      mc_send_response_with_retry("OK");
    } else {
      mc_send_response_with_retry("ERROR1");
    }
  } else if (strcmp(cmdBuff, "RESET") == 0) {
    if (rm_reset_rc() == 0) {
      mc_send_response_with_retry("OK");
    } else {
      mc_send_response_with_retry("ERROR1");
    }
  } else if (strcmp(cmdBuff, "SV?") == 0) {
    int sv = rm_get_sensorValue();

    error_code_t err =
        safe_format_response(response_buffer.buffer, sizeof(response_buffer.buffer), "SV=%d", sv);
    if (err != ERR_SUCCESS) {
      error_record(err);
    } else {
      mc_send_response_with_retry(response_buffer.buffer);
    }
  } else if (strcmp(cmdBuff, "SV1") == 0) {
    app_config.sv_mode = 1;
    mc_send_response_with_retry("OK");
  } else if (strcmp(cmdBuff, "SV0") == 0) {
    app_config.sv_mode = 0;
    mc_send_response_with_retry("OK");
  } else if (strcmp(cmdBuff, "ST?") == 0) {
    send_rm_state();
  } else if (strcmp(cmdBuff, "ST0") == 0) {
    app_config.st_mode = 0;
    timer_clear(timer_status);
    mc_send_response_with_retry("OK");
  } else if (strncmp(cmdBuff, "ST1=", 4) == 0) {
    int v = atoi(cmdBuff + 4);
    if ((v < 0) || (v > 60)) {
      mc_send_response_with_retry("ERROR2"); // invalid parameter
      return ERR_INVALID_PARAM;
    } else {
      app_config.st_interval = v;
      app_config.st_mode     = 1;
      timer_set(timer_status, 1000);

      mc_send_response_with_retry("OK");
    }
  } else if (strcmp(cmdBuff, "GD.OPEN") == 0) {
    rm_gd_open();
    mc_send_response_with_retry("OK");
  } else if (strcmp(cmdBuff, "GD.OPEN2") == 0) {
    rm_gd_open2();
    mc_send_response_with_retry("OK");
  } else if (strcmp(cmdBuff, "GD.CLOSE") == 0) {
    rm_gd_close();
    mc_send_response_with_retry("OK");
  } else if (strcmp(cmdBuff, "GD.STOP") == 0) {
    rm_gd_stop();
    mc_send_response_with_retry("OK");
  } else if (strcmp(cmdBuff, "st") == 0) {
    rm_print_state();
  } else {
    printf("ERROR: UNKNOWN command\n");
  }

  return ERR_SUCCESS;
}

// cons_processCommand
static error_code_t cons_processCommand(const char *cmdBuff, int cmdLen)
{
  if ((cmdLen <= 0) || (cmdBuff == 0))
    return ERR_INVALID_PARAM;

  printf("cons_processCommand: %s\n", cmdBuff);

  if (strcmp(cmdBuff, "ID?") == 0) {
    DPRINTF("RC v1.0");
  } else if (strcmp(cmdBuff, "INIT") == 0) {
    DPRINTF("initializing..");
    rm_init_position();
  } else if (strncmp(cmdBuff, "MP=", 3) == 0) {
    int height = atoi(cmdBuff + 3);
    if ((height < 0) || (height > app_config.mon_height)) {
      DPRINTF("ERROR2"); // invalid parameter
      return ERR_INVALID_PARAM;
    } else {
      rm_mon_move_to(height);
      DPRINTF("OK");
      return ERR_SUCCESS;
    }
  } else if (strncmp(cmdBuff, "gd=", 3) == 0) {
    int val = atoi(cmdBuff + 3);
    DPRINTF("GD:move to %d\n", val);

    gd_move_to(val);
    return ERR_SUCCESS;
  } else if (strcmp(cmdBuff, "START") == 0) {
    if (rm_start(0) == 0) {
      DPRINTF("OK\n");
    } else {
      DPRINTF("ERROR1\n");
    }
  } else if (strcmp(cmdBuff, "STOP") == 0) {
    if (rm_stop() == 0) {
      DPRINTF("OK\n");
    } else {
      DPRINTF("ERROR1\n");
    }
  } else if (strcmp(cmdBuff, "SV?") == 0) {
    int sv = rm_get_sensorValue();

    error_code_t err =
        safe_format_response(response_buffer.buffer, sizeof(response_buffer.buffer), "SV=%d", sv);
    if (err != ERR_SUCCESS) {
      error_record(err);
    } else {
      DPRINTF(response_buffer.buffer);
    }
  } else if (strcmp(cmdBuff, "SV1") == 0) {
    app_config.sv_mode = 1;
    DPRINTF("OK");
  } else if (strcmp(cmdBuff, "SV0") == 0) {
    app_config.sv_mode = 0;
    DPRINTF("OK");
  } else if (strcmp(cmdBuff, "ST0") == 0) {
    app_config.st_mode = 0;
    timer_clear(timer_status);
    DPRINTF("OK");
  } else if (strncmp(cmdBuff, "ST1=", 4) == 0) {
    int v = atoi(cmdBuff + 4);
    if ((v < 0) || (v > 60)) {
      DPRINTF("ERROR2"); // invalid parameter
      return ERR_INVALID_PARAM;
    } else {
      app_config.st_interval = v;
      app_config.st_mode     = 1;
      timer_set(timer_status, 1000);

      DPRINTF("OK");
    }
  } else if (strcmp(cmdBuff, "st") == 0) {
    rm_print_state();
  } else if (strcmp(cmdBuff, "cfg.save") == 0) {
    cfg_save(&app_config);
    DPRINTF("Config saved\n");
  } else if (strcmp(cmdBuff, "cfg.print") == 0) {
    DPRINTF("Configuration : \n");
    DPRINTF(" - sv_mode = %d\n", app_config.sv_mode);
    DPRINTF(" - st_mode     = %d\n", app_config.st_mode);
    DPRINTF(" - st_interval = %d\n", app_config.st_interval);
    DPRINTF(" - mon_height  = %d\n", app_config.mon_height);
  } else if (strcmp(cmdBuff, "minit") == 0) {
    rm_init_position();
    DPRINTF("OK");
  } else if (strcmp(cmdBuff, "GD.OPEN") == 0) {
    rm_gd_open();
    DPRINTF("OK");
  } else if (strcmp(cmdBuff, "GD.OPEN2") == 0) {
    rm_gd_open2();
    DPRINTF("OK");
  } else if (strcmp(cmdBuff, "GD.CLOSE") == 0) {
    rm_gd_close();
    DPRINTF("OK");
  } else if (strcmp(cmdBuff, "GD.STOP") == 0) {
    rm_gd_stop();
    DPRINTF("OK");
  } else {
    printf("ERROR: UNKNOWN command\n");
  }

  return ERR_SUCCESS;
}

// send_rm_state
static error_code_t send_rm_state()
{
  RmcState rs = rm_get_state();

  error_code_t err = safe_format_response(
      response_buffer.buffer, sizeof(response_buffer.buffer), "ST=%c,%d,%c,%d,%d,%d,%d,%d,%d,%d",
      rs.state, rs.rstate, rs.door, rs.monotorPosition, rs.plasticCups, rs.paperCups,
      rs.purifiedTankLevel, rs.wasteTankLevel, rs.errorCode, rs.errorRoboSeq);
  if (err != ERR_SUCCESS) {
    error_record(err);
    return err;
  }

  return mc_send_response_with_retry(response_buffer.buffer);
}
