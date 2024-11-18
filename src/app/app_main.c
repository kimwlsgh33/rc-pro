/* #include "BuildOptions.h" */

#include "../config/cfg_build.h"
#include "../platform/common/plat_types.h"
#include <stdlib.h>
#include <string.h>
#include <util/delay.h>

#include "app_main.h"
#include "../drivers/mcu/uart/drv_uart3.h"
#include "../utils/common/error_codes.h"
#include <avr/io.h>
#include <util/delay.h>

#include "../modules/control/ctrl_power.h"
#include "../modules/safety/safe_maintenance.h"
#include "../modules/control/ctrl_network.h"
#include "../modules/ui/ui_interface.h"
#include "../modules/safety/safe_monitor.h"

#include "../drivers/mcu/drv_timer.h"
#include "../drivers/mcu/drv_uart.h"
#include "../utils/common/util_platform.h"

#include "../drivers/motor/drv_motor_a.h"
#include "../drivers/motor/drv_motor_b.h"
#include "../modules/control/ctrl_motor.h"
#include "../modules/control/ctrl_cup.h"
#include "../modules/control/ctrl_clean.h"
#include "../utils/debug/dbg_diagnostics.h"

RmcState rmcState;
static motor_params_t motor_status;
static route_stats_t routing_stats;
static cleaning_stats_t cleaning_stats;
static test_result_t last_test_result;

static int us_updated = 0;
static uint16_t us_value = 6000;

static int tid_op = -1;

static int is_mon_moving = 0;
extern void rm_ev_monitor_state(char st, int position);

static int is_gd_moving = 0;
extern void rm_ev_gd_state(char st, int position);

extern void rm_ev_completed();

error_code_t rm_init()
{
  memset(&rmcState, 0, sizeof(RmcState));
  rmcState.state = RMC_ST_UNKNOWN;

  error_init();
  pm_init();
  pwr_init();
  mon_init();  // Initialize monitoring module
  net_init();  // Initialize network communication
  ui_init();   // Initialize user interface
  cup_init();  // Initialize cup detection

  // Initialize new modules
  mc_init();
  cr_init();
  cc_init();
  diag_init();

  // Run initial diagnostics
  if (diag_run_test(DIAG_TEST_ALL) != ERROR_SUCCESS) {
    return ERROR_INITIALIZATION_FAILED;
  }

  // Initialize safety monitoring
  safety_init();
  safety_register_callback(safety_status_changed);

  if (rc_init() != 0) {
    error_record(ERR_SYS_INIT);
    return -1;
  }

  if (us_init() != 0) {
    error_record(ERR_SENSOR_TIMEOUT);
    pm_record_error(COMP_SENSOR);
    return -1;
  }

  if (gd_init() != 0) {
    error_record(ERR_DOOR_SENSOR);
    pm_record_error(COMP_DOOR);
    return -1;
  }

  if (ma_init() != 0) {
    error_record(ERR_MOTOR_STALL);
    pm_record_error(COMP_MOTOR);
    return -1;
  }

  tid_op = timer_alloc();
  if (tid_op < 0) {
    error_record(ERR_SYS_INIT);
    return -1;
  }

  timer_set(tid_op, 1000);

#ifdef TEST_SIM
  us_updated = 1;
  us_value = 1234;
#endif

  return 0;
}

int rm_init_position()
{
  if ((rmcState.state == RMC_ST_UNKNOWN) || (rmcState.state == RMC_ST_READY)) {
    rmcState.state = RMC_ST_INITIALIZING;
    gd_init_position();
    ma_init_position();
    return 0;
  } else {
    return -1;
  }
}

error_code_t rm_start(uint8_t ncup)
{
    if (rmcState.state != RMC_ST_READY) {
        error_record(ERR_INVALID_STATE);
        return ERR_INVALID_STATE;
    }
    
    error_code_t err = rc_start(ncup);
    if (err != ERROR_CODE_SUCCESS) {
        error_record(err);
        return err;
    }
    
    rmcState.state = RMC_ST_RUNNING;
    return ERROR_CODE_SUCCESS;
}

int rm_stop()
{
  rc_stop();

  ma_stop();
  gd_stop();

  return 0;
}

int rm_reset_rc()
{
  rc_reset();
  return 0;
}

RmcState rm_get_state() { return rmcState; }

int rm_mon_move_to(int height)
{
  ma_move_to(height);
  is_mon_moving = 1;
  return 0;
}

error_code_t rm_gd_open2(void)
{
    error_code_t err = gd_open2();
    if (err != ERROR_CODE_SUCCESS) {
        error_record(err);
        return err;
    }
    
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        is_gd_moving = 1;
    }
    return ERROR_CODE_SUCCESS;
}

error_code_t rm_gd_close(void)
{
    error_code_t err = gd_close();
    if (err != ERROR_CODE_SUCCESS) {
        error_record(err);
        return err;
    }
    
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        is_gd_moving = 1;
    }
    return ERROR_CODE_SUCCESS;
}

int rm_gd_stop()
{
  gd_stop();
  return 0;
}

int rm_is_sensor_updated() { return us_updated; }

uint16_t rm_get_sensorValue(void)
{
    uint16_t value;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        value = us_value;
        us_updated = 0;
    }
    return value;
}

int rm_rc_is_completed()
{
  if (rc_is_completed()) {
    return 1;
  }
  return 0;
}

void rm_process()
{
  static uint32_t last_activity_check = 0;
  static uint32_t last_status_print = 0;
  uint32_t current_time;

  ma_process();

  gd_process();

  us_process();

  rc_process();

  mon_process_cycle();  // Process monitoring
  net_process_cycle();  // Process network communication
  ui_process_cycle();   // Process user interface
  cup_process_cycle();  // Process cup detection
  safety_process_cycle();

  if (timer_isfired(tid_op)) {
    timer_set(tid_op, 1000);

    current_time = timer_get(tid_op);

    RcState st = rc_get_state();

    // Check for system activity
    if (st.roboSeq != rmcState.rstate ||
        st.plasticTrayCupNum != rmcState.plasticCups ||
        st.paperTrayCupNum != rmcState.paperCups ||
        st.purifiedTankLevel != rmcState.purifiedTankLevel ||
        st.wasteTankLevel != rmcState.wasteTankLevel) {
      pwr_process_event(PWR_EVENT_ACTIVITY);
    } else if (current_time - last_activity_check >= 1000) {
      // Check for inactivity every second
      pwr_process_event(PWR_EVENT_INACTIVITY);
      last_activity_check = current_time;
    }

    // Map RC error codes to our enhanced error system
    if (st.errorCode != 0) {
      if (st.errorCode & 0x01) {
        error_record(ERR_MOTOR_STALL);
        pm_record_error(COMP_MOTOR);
        ui_indicate_error();
      }
      if (st.errorCode & 0x02) {
        error_record(ERR_SENSOR_RANGE);
        pm_record_error(COMP_SENSOR);
        ui_indicate_error();
      }
      if (st.errorCode & 0x04) {
        error_record(ERR_DOOR_JAM);
        pm_record_error(COMP_DOOR);
        ui_indicate_error();
      }
    }

    // Record cycles for active components
    if (st.roboSeq != rmcState.rstate) {
      pm_record_cycle(COMP_MOTOR);
    }

    // Check tank level changes
    if (st.purifiedTankLevel != rmcState.purifiedTankLevel ||
        st.wasteTankLevel != rmcState.wasteTankLevel) {
      pm_record_cycle(COMP_TANK);
    }

    rmcState.rstate = st.roboSeq;
    rmcState.plasticCups = st.plasticTrayCupNum;
    rmcState.paperCups = st.paperTrayCupNum;
    rmcState.purifiedTankLevel = st.purifiedTankLevel;
    rmcState.wasteTankLevel = st.wasteTankLevel;
    rmcState.errorCode = st.errorCode;
    rmcState.errorRoboSeq = st.errorRoboSeq;
    rmcState.door = gd_get_door_state();
    rmcState.monotorPosition = ma_get_position();

    // Update UI based on system state changes
    if (st.roboSeq != rmcState.rstate ||
        st.plasticTrayCupNum != rmcState.plasticCups ||
        st.paperTrayCupNum != rmcState.paperCups ||
        st.purifiedTankLevel != rmcState.purifiedTankLevel ||
        st.wasteTankLevel != rmcState.wasteTankLevel ||
        st.errorCode != rmcState.errorCode) {
      
      // Update tank level indicator if levels changed
      if (st.purifiedTankLevel != rmcState.purifiedTankLevel ||
          st.wasteTankLevel != rmcState.wasteTankLevel) {
        ui_show_tank_levels();
        if (st.purifiedTankLevel == 2 || st.wasteTankLevel == 2) {
          ui_set_led(LED_TANK_LEVEL, LED_BLINK_FAST);
          ui_indicate_warning();
        } else {
          ui_set_led(LED_TANK_LEVEL, LED_ON);
        }
      }

      // Update status display
      ui_show_status();
    }

    // Check component health and update system state
    uint8_t worst_health = HEALTH_GOOD;
    for (int i = 1; i <= 4; i++) {
      uint8_t health = pm_get_health_status(i);
      if (health > worst_health) {
        worst_health = health;
      }
    }

    // If any component is in critical condition, move to error state
    if (worst_health == HEALTH_CRITICAL) {
      rmcState.state = RMC_ST_ERROR;
      pwr_process_event(PWR_EVENT_CRITICAL);
      ui_indicate_error();
    } else if (worst_health == HEALTH_WARNING) {
      pwr_process_event(PWR_EVENT_LOW_POWER);
      ui_indicate_warning();
    }

    // If system state changes, send status update
    if (st.roboSeq != rmcState.rstate ||
        st.plasticTrayCupNum != rmcState.plasticCups ||
        st.paperTrayCupNum != rmcState.paperCups ||
        st.purifiedTankLevel != rmcState.purifiedTankLevel ||
        st.wasteTankLevel != rmcState.wasteTankLevel ||
        st.errorCode != rmcState.errorCode) {
      net_send_status();  // Send status update over network
    }

    // Enhanced cup detection and sorting logic
    if (cup_is_present() && rmcState.state == RMC_ST_READY) {
      cup_detect_status_t detect_status = cup_start_detection();
      if (detect_status != CUP_DETECT_OK) {
        error_record(ERR_CUP_DETECT);
        rmcState.state = RMC_ST_ERROR;
      } else {
        rmcState.state = RMC_ST_RUNNING;
      }
    }

    // Update cup processing based on detection results
    if (rmcState.state == RMC_ST_RUNNING) {
      const cup_characteristics_t* cup_info = cup_get_characteristics();
      if (cup_info->material == CUP_TYPE_PLASTIC) {
        rc_set_target(RC_TARGET_PLASTIC);
      } else if (cup_info->material == CUP_TYPE_PAPER) {
        rc_set_target(RC_TARGET_PAPER);
      } else {
        // Invalid or unknown cup type
        error_record(ERR_INVALID_CUP);
        rmcState.state = RMC_ST_ERROR;
      }
    }

    // Check safety status before starting operation
    if (rmcState.state == RMC_ST_READY && 
        safety_get_system_status() != SAFETY_OK) {
        rmcState.state = RMC_ST_ERROR;
        error_record(ERR_SAFETY_CHECK);
    }

    // Don't proceed if safety critical or emergency stopped
    if (safety_is_emergency_stopped() || 
        safety_get_system_status() == SAFETY_CRITICAL) {
        rmcState.state = RMC_ST_ERROR;
        break;
    }
  }

  // Print monitoring status every minute
  if (current_time - last_status_print >= 60000) {
    mon_print_status();
    last_status_print = current_time;
  }

  switch (rmcState.state) {
  case RMC_ST_UNKNOWN:
    if ((gd_get_state() == GD_ST_READY) && (ma_get_state() == MA_ST_READY)) {
      if (us_updated) {
        rmcState.state = RMC_ST_READY;
        pwr_process_event(PWR_EVENT_ACTIVITY);
      }
    } else {
      if (rm_init_position() != 0) {
        error_record(ERR_SYS_INIT);
      }
    }
    break;

  case RMC_ST_INITIALIZING:
    if ((gd_get_state() == GD_ST_READY) && (ma_get_state() == MA_ST_READY)) {
      if (us_updated) {
        rmcState.state = RMC_ST_READY;
        pwr_process_event(PWR_EVENT_ACTIVITY);
      }
    }
    break;

  case RMC_ST_READY:
    break;

  case RMC_ST_RUNNING:
    if (rm_rc_is_completed()) {
      rmcState.state = RMC_ST_READY;
      rm_ev_completed();
      pwr_process_event(PWR_EVENT_ACTIVITY);
      ui_indicate_operation_complete();
    }
    break;

  case RMC_ST_ERROR:
    if (safety_get_system_status() == SAFETY_OK && 
        !safety_is_emergency_stopped()) {
        rmcState.state = RMC_ST_READY;
    }
    break;
  }

  if (is_mon_moving) {
    if (ma_get_state() == 'R') {
      rm_ev_monitor_state(ma_get_state(), ma_get_position());
      pm_record_cycle(COMP_MOTOR);
      pwr_process_event(PWR_EVENT_ACTIVITY);
      is_mon_moving = 0;
    } else if (ma_get_state() == 'E') {
      error_record(ERR_MOTOR_STALL);
      pm_record_error(COMP_MOTOR);
      is_mon_moving = 0;
    }
  }

  if (is_gd_moving) {
    if (gd_get_state() == GD_ST_READY) {
      rm_ev_gd_state(gd_get_door_state(), gd_get_position());
      pm_record_cycle(COMP_DOOR);
      pwr_process_event(PWR_EVENT_ACTIVITY);
      is_gd_moving = 0;
    } else if (gd_get_state() == GD_ST_ERROR) {
      error_record(ERR_DOOR_JAM);
      pm_record_error(COMP_DOOR);
      is_gd_moving = 0;
    }
  }
}

// Safety status callback
static void safety_status_changed(safety_system_t system, safety_status_t status)
{
    switch (status) {
        case SAFETY_WARNING:
            ui_indicate_warning();
            pwr_process_event(PWR_EVENT_WARNING);
            break;
            
        case SAFETY_CRITICAL:
            ui_indicate_error();
            pwr_process_event(PWR_EVENT_CRITICAL);
            rmcState.state = RMC_ST_ERROR;
            break;
            
        case SAFETY_OK:
            if (rmcState.state == RMC_ST_ERROR && 
                safety_get_system_status() == SAFETY_OK) {
                rmcState.state = RMC_ST_READY;
            }
            break;
    }
}

static void rm_update_ultra_sonic_value(uint16_t usValue)
{
  us_updated = 1;
  us_value = usValue;
}

void rm_print_state()
{
  char buffer[128];
  sprintf(buffer, "Recycle main st = %c\n", rmcState.state);
  for (char *p = buffer; *p; p++) {
    uart1_putchar(*p);
  }
  sprintf(buffer, "US: %s, v=%d\n", us_updated ? "ok" : "unknown", us_value);
  for (char *p = buffer; *p; p++) {
    uart1_putchar(*p);
  }
  rc_print_state();
  gd_print_state();
  ma_print_state();
  error_print_status();
  pm_print_status();
}

int us_init()
{
  DDRE |= (1 << 4);  // Set PE4 as output
  PORTE |= (1 << 4); // Set PE4 high

  uart3_init(BAUD_9600);
  uart3_set_ultrasonic_mode(true);
  
  us_updated = 0;
  us_value = 6000;
  
  return 0;
}

void us_process()
{
  if (uart3_is_ultrasonic_data_ready()) {
    uint16_t distance = uart3_get_ultrasonic_distance();
    rm_update_ultra_sonic_value(distance);
  }
}
