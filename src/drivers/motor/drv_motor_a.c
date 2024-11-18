/**
 * Peristaltic Pump Controller
 * using DM1, a simple DC motor controller with init
 */

#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>

#include "../../config/cfg_system.h"
#include "../../config/cfg_hardware.h"
#include "../../platform/common/plat_types.h"
#include "../../utils/common/error_codes.h"
#include "../../drivers/mcu/timer/drv_timer.h"
#include "common/drv_motor_common.h"
#include "drv_motor_a.h"

// ORG limit sensor : when motor runs CCW
#define ORG_SENSOR_IS_ON() ((PINB & 0x10) != 0x10)

// 800 ppr, 1cm /sec.
#define FIND_SPEED_HZ 400

static motor_control_t motor_ctrl;
static int tid_op = -1;
static int32_t pos2step = 160;

int ma_init()
{
  // limit pull-up
  // PORTH |= 0x20;
  PORTB |= 0x10; // PB4

  motor_ctrl.state = MA_ST_UNKNOWN;
  // motor_ctrl.state = MA_ST_READY;

  sm1_init();

  tid_op = timer_alloc();

  return 0;
}

char ma_get_state() { return motor_ctrl.state; }

/**
        Get current position in millis
*/
int ma_get_position() { return motor_ctrl.position; }

/**
        Get position to step conversion factor
*/
int ma_config_pos2step(int p2s);

/**
 * Initialize motor, find initial position
 */
int ma_init_position()
{
  if ((motor_ctrl.state == MA_ST_UNKNOWN) || (motor_ctrl.state == MA_ST_READY) ||
      (motor_ctrl.state == MA_ST_ERROR)) {
    motor_ctrl.state = MA_ST_INITIALIZING;
    motor_ctrl.init_step = 0;
    return 0;
  } else {
    return -1;
    // busy
  }
}

int ma_is_busy()
{
  if ((motor_ctrl.state == MA_ST_INITIALIZING) || (motor_ctrl.state == MA_ST_MOVING)) {
    return 1;
  } else {
    return 0;
  }
}

int ma_move_to(int position)
{
  DPRINTF("MA move to = %d\n", position);

  if (ma_is_busy()) {
    return -1; // BUSY
  }

  // distance mm to position in step
  int32 sm_pos = position * pos2step;

  motor_ctrl.state = MA_ST_MOVING;

  return sm1_move_to(sm_pos);
}

void ma_stop() { sm1_stop(); }

/*
void sy1_enable()
{
        sm2_enable();
}

void syd1_disable()
{
        sm2_disable();
}
*/

void ma_process()
{
  sm1_process();

  int pre_step = motor_ctrl.init_step;

  switch (motor_ctrl.state) {
  case MA_ST_INITIALIZING:
    if (motor_ctrl.init_step == 0) // unknown
    {
      if (ORG_SENSOR_IS_ON()) {
        sm1_run(DIR_CW, FIND_SPEED_HZ); // go far from org limit sensor
        motor_ctrl.init_step = 10;
      } else {
        sm1_run(DIR_CCW, FIND_SPEED_HZ); // go to find org limit sensor
        motor_ctrl.init_step = 1;
      }
    } else if (motor_ctrl.init_step == 1) // find unknown
    {
      if (ORG_SENSOR_IS_ON()) {
        sm1_stop();
        timer_set(tid_op, 100);
        motor_ctrl.init_step = 2;
      }
    } else if (motor_ctrl.init_step == 2) // find unknown
    {
      if (timer_isfired(tid_op)) {
        timer_clear(tid_op);
        sm1_run(DIR_CW, FIND_SPEED_HZ); // go far from org limit sensor
        motor_ctrl.init_step = 10;
      }
    } else if (motor_ctrl.init_step == 10) {
      if (!ORG_SENSOR_IS_ON()) {
        sm1_stop();
        timer_set(tid_op, 100);
        motor_ctrl.init_step = 11;
      }
    } else if (motor_ctrl.init_step == 11) // go find org limit sensor again
    {
      if (timer_isfired(tid_op)) {
        timer_clear(tid_op);
        sm1_run(DIR_CCW, FIND_SPEED_HZ);
        motor_ctrl.init_step = 20;
      }
    } else if (motor_ctrl.init_step == 20) // find org limit sensor : init position
    {
      if (ORG_SENSOR_IS_ON()) {
        sm1_stop();
        motor_ctrl.init_step = 0;

        motor_ctrl.position = 0;
        motor_ctrl.state = MA_ST_READY;

        sm1_reset_position();
      }
      if (timer_isfired(tid_op)) {
        timer_clear(tid_op);
        sm1_run(DIR_CCW, FIND_SPEED_HZ); // go far from org limit sensor
        motor_ctrl.init_step = 20;
      }
    }
    break;

  case MA_ST_MOVING:
    if (sm1_get_state() == SM_ST_READY) {
      DPRINTF("MA: moving -> ready\n");
      motor_ctrl.state = MA_ST_READY;

      motor_ctrl.position = sm1_get_position() / pos2step;
    }
    break;
  }
  // end-case

  if (pre_step != motor_ctrl.init_step) {
    DPRINTF("MA: step %d -> %d\n", pre_step, motor_ctrl.init_step);
  }
}

void ma_print_state()
{
  DPRINTF("MA st : %c, L1:%d\n", motor_ctrl.state, ORG_SENSOR_IS_ON() ? 1 : 0);
  DPRINTF(" + pos : %lu\n", motor_ctrl.position);
}
