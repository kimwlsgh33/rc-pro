/**
 * @file error_codes.c
 * @brief Error handling system implementation
 */

#include "error_codes.h"
#include "../../app/app_main.h"
#include "../../config/cfg_debug.h"
#include "../../config/cfg_hardware.h"
#include "../../drivers/mcu/common/mcu_config.h"
#include "../../drivers/mcu/common/mcu_types.h"
#include "../../drivers/mcu/timer/drv_timer.h"
#include "../../drivers/mcu/uart/drv_uart.h"
#include "../../drivers/mcu/uart/drv_uart1.h"
#include "../../drivers/mcu/uart/drv_uart2.h"
#include "../../drivers/mcu/uart/drv_uart3.h"
#include "../../modules/safety/safe_monitor.h"
#include "../../platform/avr/plat_avr.h"
#include "error_recovery.h"
#include <avr/eeprom.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <util/atomic.h>

/* Constants */
#define SYSTEM_STATE_CRITICAL_FAILURE 1 /* System state for critical failure */

/* Static variables */
static volatile error_history_t error_history;
static volatile error_stats_t error_stats;
static volatile uint8_t last_error  = ERR_CAT_COMMON;
static volatile bool is_initialized = false;

/* Function prototypes */
static void error_escalate(error_record_t *record);
static uint8_t get_max_retries(error_code_t error_code);
static uint16_t get_error_context(void);
static bool should_attempt_recovery(error_code_t error_code);
static void system_critical_failure_handler(void);
static void safe_shutdown(void);
static error_code_t error_save_critical_data(void);
static inline int get_error_record_index(int current_index, int offset);
static inline error_category_t get_error_category(error_code_t error_code);
static inline void update_category_stats(error_category_t category);

/**
 * @brief Initializes the error handling system
 * @return Error code indicating success or failure
 */
error_code_t error_init(void)
{
  if (is_initialized) {
    return ERR_ALREADY_INITIALIZED;
  }

  /* Initialize error history with atomic protection */
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    memset((void *)&error_history, 0, sizeof(error_history_t));
    memset((void *)&error_stats, 0, sizeof(error_stats_t));

    is_initialized = true;
  }
  return ERR_SUCCESS;
}

/**
 * @brief Records an error in the error history
 * @param error_code Error code to record
 */
void error_record(error_code_t error_code)
{
  if (!is_initialized) {
    return;
  }

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    error_category_t category = get_error_category(error_code);
    update_category_stats(category);

    /* Update error statistics */
    error_stats.total_errors++;
    error_stats.last_error_time = timer_get_ms();

    /* Create new error record */
    // WARN: DO NOT CAST THIS TO void*
    error_record_t *record = &error_history.history[error_history.current_index];
    record->code           = error_code;
    record->timestamp      = error_stats.last_error_time;
    record->retry_count    = 0;
    record->resolved       = 0;
    record->severity       = GET_DEFAULT_SEVERITY(category);
    record->context        = get_error_context();

    /* Update history indices */
    error_history.current_index = (error_history.current_index + 1) % MAX_ERROR_HISTORY;
    if (error_history.total_errors < MAX_ERROR_HISTORY) {
      error_history.total_errors++;
    }

    last_error = error_code;

    /* Attempt recovery if appropriate */
    if (should_attempt_recovery(error_code)) {
      error_attempt_recovery(error_code);
    }
  }
}

/**
 * @brief Attempts to recover from an error
 * @param error_code Error code to recover from
 * @return true if recovery was successful, false otherwise
 */
uint8_t error_attempt_recovery(error_code_t error_code)
{
  uint8_t recovery_success = 0;

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    /* Find the most recent record of this error */
    for (int i = 0; i < error_history.total_errors; i++) {
      int idx                = get_error_record_index(error_history.current_index, i);
      error_record_t *record = &error_history.history[idx];

      if (record->code == error_code && !record->resolved) {
        record->retry_count++;

        /* Check if we've exceeded max retries for this category */
        if (record->retry_count > get_max_retries(error_code)) {
          error_escalate(record);
          return 0;
        }

        /* Attempt recovery using the error recovery system */
        error_code_t result = error_recovery_attempt(error_code);
        recovery_success    = (result == ERR_SUCCESS);

        record->resolved = recovery_success;
        if (recovery_success) {
          error_stats.recovered_errors++; // Update recovery statistics
        }
        break;
      }
    }
  }

  return recovery_success;
}

/**
 * @brief Helper function to validate initialization
 * @return Error code indicating success or failure
 */
static inline error_code_t validate_initialization(void)
{
  if (!is_initialized) {
    return ERR_NOT_INITIALIZED;
  }
  return ERR_SUCCESS;
}

/**
 * @brief Gets the last recorded error code
 * @return Last recorded error code
 */
error_code_t error_get_last(void)
{
  error_code_t status = validate_initialization();
  if (status != ERR_SUCCESS) {
    return status;
  }
  return last_error;
}

/**
 * @brief Clears the last recorded error code
 */
void error_clear(void)
{
  if (validate_initialization() != ERR_SUCCESS) {
    return;
  }
  last_error = ERR_CAT_COMMON;
}

/**
 * @brief Gets the error history
 * @return Pointer to error history structure
 */
volatile error_history_t *error_get_history(void)
{
  if (validate_initialization() != ERR_SUCCESS) {
    return NULL;
  }
  return &error_history;
}

/**
 * @brief Prints the error history
 */
void error_print_status(void)
{
  if (validate_initialization() != ERR_SUCCESS) {
    DPRINTF("Error system not initialized\n");
    return;
  }

  DPRINTF("Error History (Total: %d):\n", error_history.total_errors);

  for (int i = 0; i < error_history.total_errors; i++) {
    int idx                = get_error_record_index(error_history.current_index, i);
    error_record_t *record = &error_history.history[idx];

    DPRINTF("Error[%d]: Code=0x%02X, Time=%lu, Retries=%d, %s\n", i, record->code,
            record->timestamp, record->retry_count, record->resolved ? "Resolved" : "Unresolved");
  }
}

/**
 * @brief Escalates an error to critical failure
 * @param record Error record to escalate
 */
static void error_escalate(error_record_t *record)
{
  /* Escalate error handling based on severity and retry count */
  if (record->retry_count >= CRITICAL_RETRY_THRESHOLD) {
    /* Transition to critical error state */
    error_record(ERR_CRITICAL_FAILURE);

    /* Notify system of critical failure */
    system_critical_failure_handler();

    /* Attempt safe shutdown */
    safe_shutdown();
  }
}

/**
 * @brief Gets the maximum number of retries for a given error code
 * @param error_code Error code to check
 * @return Maximum number of retries allowed
 */
static uint8_t get_max_retries(error_code_t error_code)
{
  uint8_t category = error_get_category(error_code);

  switch (category) {
  case ERR_CAT_SENSOR:
  case ERR_CAT_MOTOR:
    return 3;
  case ERR_CAT_COMMUNICATION:
    return 5;
  case ERR_CAT_PERIPHERAL:
    return 4;
  case ERR_CAT_SYSTEM:
    return 1;
  case ERR_CAT_CRITICAL:
    return 0;
  default:
    return MAX_RECOVERY_ATTEMPTS;
  }
}

/**
 * @brief Gets the current system context for error logging
 * @return Current system context value
 */
static uint16_t get_error_context(void)
{
  uint16_t context = 0;

  /* Get current system state information */
  if (is_initialized) {
    context |= (error_history.total_errors & 0x00F0);
    context |= (error_get_category(last_error) & 0x000F);
  }

  return context;
}

/**
 * @brief Determines if recovery should be attempted for an error
 * @param error_code Error code to check
 * @return true if recovery should be attempted, false otherwise
 */
static bool should_attempt_recovery(error_code_t error_code)
{
  uint8_t category = error_get_category(error_code);

  /* Don't attempt recovery for system or critical errors */
  if (category == ERR_CAT_SYSTEM || category == ERR_CAT_CRITICAL) {
    return false;
  }

  /* Check if we've exceeded total errors threshold */
  if (error_history.total_errors >= MAX_ERROR_HISTORY) {
    return false;
  }

  return true;
}

/**
 * @brief Handles system critical failure by notifying all subsystems
 */
static void system_critical_failure_handler(void)
{
  /* Disable interrupts during critical failure handling */
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    /* Record critical failure state */
    last_error = SYSTEM_STATE_CRITICAL_FAILURE;

    /* Log the critical failure event */
    DPRINTF("CRITICAL FAILURE: System entering safe shutdown\n");

    /* Notify safety monitoring system */
    safety_trigger_emergency_stop();

    /* Stop all active operations */
    rm_stop();

    /* Update safety state through proper API */
    safety_trigger_critical(SAFETY_SYS_MECHANICAL);

    /* Disable non-essential peripherals */
    mcu_disable_peripheral(MCU_PERIPHERAL_ADC);
    mcu_disable_peripheral(MCU_PERIPHERAL_SPI);
  }
}

/**
 * @brief Performs a safe system shutdown
 *
 * This function:
 * 1. Disables interrupts
 * 2. Stops all DMA transfers
 * 3. Flushes UART buffers
 * 4. Disables non-essential peripherals
 * 5. Saves critical data
 * 6. Triggers watchdog reset
 */
static void safe_shutdown(void)
{
  /* Disable all interrupts */
  plat_avr_disable_interrupts();

  /* Stop all ongoing DMA transfers */
  error_code_t dma_status = dma_stop_all_transfers();
  if (dma_status != ERR_SUCCESS) {
    error_record(ERR_DMA_STOP_FAILED);
  }

  /* Flush all UART buffers */
  const uart_driver_t *driver = NULL;
  for (uart_port_t port = 0; port < UART_PORT_MAX; port++) {
    switch (port) {
    case UART_PORT_1:
      driver = uart1_get_driver();
      break;
    case UART_PORT_2:
      driver = uart2_get_driver();
      break;
    case UART_PORT_3:
      driver = uart3_get_driver();
      break;
    default:
      continue;
    }

    if (driver && driver->flush) {
      error_code_t uart_status = driver->flush();
      if (uart_status != ERR_SUCCESS) {
        error_record(ERR_UART_FLUSH_FAILED);
      }
    }
  }

  /* Disable non-essential peripherals */
  const mcu_peripheral_id_t peripherals[] = {MCU_PERIPHERAL_ADC, MCU_PERIPHERAL_SPI,
                                             MCU_PERIPHERAL_TIMER1, MCU_PERIPHERAL_TIMER2};

  for (size_t i = 0; i < sizeof(peripherals) / sizeof(peripherals[0]); i++) {
    error_code_t status = mcu_disable_peripheral(peripherals[i]);
    if (status != ERR_SUCCESS) {
      error_record(ERR_PERIPHERAL_DISABLE_FAILED);
    }
  }

  /* Save critical data to EEPROM */
  error_code_t save_status = error_save_critical_data();
  if (save_status != ERR_SUCCESS) {
    error_record(ERR_EEPROM_WRITE_FAILED);
  }

  /* Enable watchdog with short timeout to force reset */
  error_code_t wdt_status = plat_avr_watchdog_enable(WDTO_15MS);
  if (wdt_status != ERR_SUCCESS) {
    // If watchdog enable fails, try to force a reset through other means
    plat_avr_soft_reset();
  }

  /* Wait for watchdog reset */
  while (1) {
    plat_avr_watchdog_reset();
    __asm__ __volatile__("nop"); // Prevent optimization
  }
}

/**
 * @brief Save critical system data to EEPROM before shutdown
 *
 * This function saves critical system state to EEPROM to enable
 * post-mortem analysis after a system reset. The data saved includes:
 * - Last error code and timestamp
 * - System uptime
 * - Critical sensor readings
 * - System state flags
 *
 * @return error_code_t Error code indicating success or failure
 */
static error_code_t error_save_critical_data(void)
{
  error_code_t status  = ERR_SUCCESS;
  uint16_t eeprom_addr = EEPROM_CRITICAL_DATA_ADDR;

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    // Save error history
    for (uint8_t i = 0; i < error_history.total_errors && i < MAX_ERROR_HISTORY; i++) {
      status =
          plat_avr_eeprom_write(eeprom_addr, &error_history.history[i], sizeof(error_record_t));
      if (status != ERR_SUCCESS) {
        return status;
      }
      eeprom_addr += sizeof(error_record_t);
    }

    // Save system uptime
    uint32_t uptime = timer_get_ms();
    status          = plat_avr_eeprom_write(eeprom_addr, &uptime, sizeof(uptime));
    if (status != ERR_SUCCESS) {
      return status;
    }
    eeprom_addr += sizeof(uptime);

    // Save error statistics
    status = plat_avr_eeprom_write(eeprom_addr, &error_stats, sizeof(error_stats_t));
    if (status != ERR_SUCCESS) {
      return status;
    }
    eeprom_addr += sizeof(error_stats_t);

    // Save system state flags
    uint8_t system_flags = 0;
    system_flags |= (error_recovery_is_active() << 0);
    system_flags |= (safety_is_emergency_stopped() << 1);
    status = plat_avr_eeprom_write_byte(eeprom_addr, system_flags);
  }

  return status;
}

/**
 * @brief Helper function to get error record index
 * @param current_index Current index in error history
 * @param offset Offset from current index
 * @return Calculated error record index
 */
static inline int get_error_record_index(int current_index, int offset)
{
  return (current_index - 1 - offset + MAX_ERROR_HISTORY) % MAX_ERROR_HISTORY;
}

/**
 * @brief Helper function to handle error category operations
 * @param error_code Error code to check
 * @return Error category
 */
static inline error_category_t get_error_category(error_code_t error_code)
{
  error_category_t category = (error_code >> ERR_CAT_SHIFT) & ERR_CAT_MASK;
  if (category >= ERR_CAT_MAX) {
    category = ERR_CAT_COMMON;
  }
  return category;
}

/**
 * @brief Helper function to update category statistics
 * @param category Error category to update
 */
static inline void update_category_stats(error_category_t category)
{
  if (IS_VALID_ERROR_CATEGORY(category)) {
    error_category_stats_t *stats = &error_stats.category_stats[category >> ERR_CAT_SHIFT];

    /* Update basic counts */
    stats->count++;
    stats->last_time = timer_get_ms();

    /* Check if this is a consecutive error in this category */
    if (error_stats.last_error_time == 0 ||
        (stats->last_time - error_stats.last_error_time) < CONSECUTIVE_ERROR_THRESHOLD) {
      stats->consecutive++;
    } else {
      stats->consecutive = 1;
    }

    /* Check for consecutive error threshold */
    if (stats->consecutive >= CONSECUTIVE_ERROR_THRESHOLD) {
      /* Consider escalating the error severity */
      error_escalate(NULL);
    }
  }
}

/* Atomic helper functions */
static inline void atomic_increment_u8(volatile uint8_t *value)
{
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { (*value)++; }
}

static inline void atomic_increment_u32(volatile uint32_t *value)
{
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { (*value)++; }
}

static inline uint8_t atomic_load_u8(volatile uint8_t *value)
{
  uint8_t result;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { result = *value; }
  return result;
}

static inline void atomic_store_u8(volatile uint8_t *dest, uint8_t value)
{
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { *dest = value; }
}
