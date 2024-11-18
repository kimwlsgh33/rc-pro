/**
 * Peristaltic Pump Controller
 * using DM1, a simple DC motor controller with init
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "../../config/cfg_system.h"
#include "../../config/cfg_hardware.h"
#include "../../platform/common/plat_types.h"
#include "../../utils/common/error_codes.h"
#include "../../drivers/mcu/timer/drv_timer.h"
#include "common/drv_motor_common.h"
#include "drv_motor_b.h"

// ORG limit sensor : when motor runs CCW
//#define	ORG_SENSOR_IS_ON()	((PINB & 0x10)!=0x10)
#define	ORG_SENSOR_IS_ON()	((PINH & 0x20)!=0x20)


static motor_control_t motor_ctrl;
static int tid_op = -1;
static char door_state = 'C';

// Position (may be in millimeters) to stepper pulse count conversion
// step = pos * POS_TO_STEP;
#define POS_TO_STEP		(int32_t)80
// 800 ppr, 0.5cm

// 800 ppr, 1cm /sec.
#define FIND_SPEED_HZ	400

static int open_position = 280;		// in millis
static int open_position2 = 300;

int gd_init()
{
	// limit pull-up
	//PORTB |= 0x10;
	PORTH |= 0x20;	// PH6

	motor_ctrl.state = GD_ST_UNKNOWN;
	//motor_ctrl.state = GD_ST_READY;

	sm2_init();

	tid_op = timer_alloc();

	return 0;
}

char gd_get_door_state()
{
	return door_state;
}

char gd_get_state()
{
	return motor_ctrl.state;
}


int gd_get_position()
{
	return motor_ctrl.position;
	//return motor_ctrl.state;
}


/**
 * Initialize motor, find initial position
 */
int gd_init_position()
{
	if ((motor_ctrl.state == GD_ST_UNKNOWN)
		|| (motor_ctrl.state == GD_ST_READY)
		|| (motor_ctrl.state == GD_ST_ERROR))
	{
		motor_ctrl.state = GD_ST_INITIALIZING;
		motor_ctrl.init_step = 0;
		return 0;
	}
	else
	{
		return -1;
		// busy
	}
}

int gd_is_busy()
{
	if ((motor_ctrl.state == GD_ST_INITIALIZING)
		|| (motor_ctrl.state == GD_ST_MOVING))
	{
		return 1;
	}
	else
	{
		return 0;
	}
}


int gd_move_to(int position)
{
	DPRINTF("MA move to = %d\n", position);

	if (gd_is_busy())
	{
		return -1;	// BUSY
	}

	// distance mm to position in step
	int32 sm_pos = position * POS_TO_STEP;

	motor_ctrl.state = GD_ST_MOVING;

	return sm2_move_to(sm_pos);
}

void gd_stop()
{
	sm2_stop();
}

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

void gd_process()
{
	sm2_process();
	
	int pre_step = motor_ctrl.init_step;

	switch (motor_ctrl.state)
	{
	case GD_ST_INITIALIZING:
		if (motor_ctrl.init_step == 0)		// unknown
		{
			if (ORG_SENSOR_IS_ON())
			{
				sm2_run(DIR_CW, FIND_SPEED_HZ);		// go far from org limit sensor
				motor_ctrl.init_step = 10;
			}
			else
			{
				sm2_run(DIR_CCW, FIND_SPEED_HZ);		// go to find org limit sensor
				motor_ctrl.init_step = 1;
			}
		}
		else if (motor_ctrl.init_step == 1)		// find unknown
		{
			if (ORG_SENSOR_IS_ON())
			{
				sm2_stop();
				timer_set(tid_op, 100);
				motor_ctrl.init_step = 2;
			}
		}
		else if (motor_ctrl.init_step == 2)		// find unknown
		{
			if (timer_isfired(tid_op))
			{
				timer_clear(tid_op);
				sm2_run(DIR_CW, FIND_SPEED_HZ);		// go far from org limit sensor
				motor_ctrl.init_step = 10;
			}
		}
		else if (motor_ctrl.init_step == 10)		// find org limit sensor - out of init position
		{
			if (!ORG_SENSOR_IS_ON())
			{
				sm2_stop();
				timer_set(tid_op, 100);
				motor_ctrl.init_step = 11;
			}
		}
		else if (motor_ctrl.init_step == 11)		// go find org limit sensor again
		{
			if (timer_isfired(tid_op))
			{
				timer_clear(tid_op);
				sm2_run(DIR_CCW, FIND_SPEED_HZ);
				motor_ctrl.init_step = 20;
			}
		}
		else if (motor_ctrl.init_step == 20)		// find org limit sensor : init position
		{
			if (ORG_SENSOR_IS_ON())
			{
				sm2_stop();
				motor_ctrl.init_step = 0;

				motor_ctrl.position = 0;
				motor_ctrl.state = GD_ST_READY;
				
				sm2_reset_position();
			}
			if (timer_isfired(tid_op))
			{
				timer_clear(tid_op);
				sm2_run(DIR_CCW, FIND_SPEED_HZ);		// go far from org limit sensor
				motor_ctrl.init_step = 20;
			}
		}
		break;

	case GD_ST_MOVING:
		if (sm2_get_state() == SM_ST_READY)
		{
			DPRINTF("GD: moving -> ready\n");
			motor_ctrl.position = sm2_get_position() / POS_TO_STEP;
			motor_ctrl.state = GD_ST_READY;

			if (door_state == 'o') door_state = 'O';
			else if (door_state == 'c') door_state = 'C';
		}
		break;
	}
	//end-case

	if (pre_step != motor_ctrl.init_step)
	{
		DPRINTF("GD: step %d -> %d\n", pre_step, motor_ctrl.init_step);
	}
}
//-----------------------------------------------------------------------------

int gd_is_open()
{
	if (motor_ctrl.state == GD_ST_READY)
	{
		if (motor_ctrl.position == open_position) return 1;
		else return 0;
	}
	return 0;
}

int gd_is_closed()
{
	if (motor_ctrl.state == GD_ST_READY)
	{
		if (motor_ctrl.position == 0) return 1;
		else return 0;
	}
	return 0;
}

int gd_open()
{
	door_state = 'o';
	return gd_move_to(open_position);
}

int gd_open2()
{
	door_state = 'o';
	return gd_move_to(open_position2);
}

int gd_close()
{
	door_state = 'c';
	return gd_move_to(0);
}

void gd_print_state()
{
	DPRINTF("GD st : %c, L1:%d\n", motor_ctrl.state, ORG_SENSOR_IS_ON()?1:0);
	DPRINTF(" + pos : %ld\n", motor_ctrl.position);
}
