/*
 *	Stepper Motor Driver
 *
 *	using TIMER4
 */
#include "BuildOptions.h"

#include <stdio.h>				// for debugg.
#include <string.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>


#include "types.h"
#include "putil.h"

#include "SM2.h"

#define		MMIN_SPEED		200			// hz
#define		MMAX_SPEED		800			// hz		// 10 sec.

//#define		MMAX_SPEED		3200			// hz		// 10 sec.
//#define		MMAX_SPEED		2400			// hz		// 10 sec.
//#define		MMAX_SPEED		1600			// hz		// 14 sec.

#define		MDT				50			// speed change MDT times per sec.
#define		MDELTA_SPEED	20			// in hz

#define		SM_MIN_HZ	MMIN_SPEED
#define 	SM_MAX_HZ	MMAX_SPEED

// motor control port operation
// PK6 : CW/CCW
// PK7 : Enable
// PH3(OC4A) : Pulse

#define SM_PULSE_ON()	//		_SET(PORTH, 3)
#define SM_PULSE_OFF()	//		_CLR(PORTH, 3)

#define SM_DIR_CCW()				_SET(PORTK, 2)
#define SM_DIR_CW()			_CLR(PORTK, 2)
/*
#define SM_DIR_CW()				_SET(PORTK, 6)
#define SM_DIR_CCW()			_CLR(PORTK, 6)
*/
#define SM_ENABLE()				_SET(PORTK, 7)
#define SM_DISABLE()			_CLR(PORTK, 7)


// internal step motor control functions
static void sm_calcSpeedPlan(uint32 dist);
static void sm_calcNextPlan();
static void sm_prepare_next_speed(int spd);
static uint16 sm_getOcrForSpeed(uint32 hz);

static void mi_start();
static void mi_stop();

static SMCtrl mc;

int sm2_init()
{
	// PK4,5,6,7
	DDRK |= 0xF0;

	// PH3
	DDRH |= 0x08;

	//SM_DECAY_OFF();
	//SM_MODE_HALF();

	SM_DIR_CW();
	SM_ENABLE();

	mc.min_speed = MMIN_SPEED;
	mc.max_speed = MMAX_SPEED;
	mc.astate = 0;		// dummy

	mc.state = SM_ST_READY;

	return 0;
}

int sm2_get_state()
{
	return mc.state;
}

int sm2_is_busy()
{
	if (SM_ST_READY) return 0;
	else return 1;
}

// current position in pulse count
int32 sm2_get_position()
{
	return mc.cpPos;
}

/*
	Run with given speed in hz.
*/
int sm2_run(int dir, int speed)
{
	if (mc.state == SM_ST_READY)
	{
		DPRINTF("SM2: run %d\n", speed);

		mc.astate = 0;			// init accel state
		mc.speed = speed;
		mc.dir = dir;

		int next_spd = mc.min_speed;
		if (next_spd > mc.speed) next_spd = mc.speed;		// target speed.

		sm_prepare_next_speed(next_spd);

		if (mc.dir == DIR_CW)	SM_DIR_CW();
		else					SM_DIR_CCW();

		SM_ENABLE();

		mi_start();

		mc.state = SM_ST_RUNNING;

		return 0;
	}
	else
	{
		// error.. running
		return -1;
	}
}

/**
	go to position
*/
int sm2_move_to(int32_t targetPos)
{
	int32_t dist = targetPos - mc.cpPos;

	DPRINTF("move to : cp = %ld, dist=%ld\n", mc.cpPos, dist);

	if (dist == 0)	return 1;
	else if (dist > 0)	mc.dir = DIR_CW;
	else
	{
		mc.dir = DIR_CCW;
		dist = -dist;
	}

	sm_calcSpeedPlan(dist);

	// m_nextSpeed, m_nextPos, m_nextOCR
	sm_calcNextPlan();

	if (mc.dir == DIR_CW)	SM_DIR_CW();
	else					SM_DIR_CCW();

	DPRINTF(" -> start\n");

	SM_ENABLE();

	mi_start();

	mc.state = SM_ST_MOVING;

	return 0;
}

void sm2_stop()
{
	if (mc.state == SM_ST_MOVING)
	{
		mc.state = SM_ST_READY;
	
		SM_PULSE_OFF();

		mi_stop();

		//SM_DISABLE();
	}
	else if (mc.state == SM_ST_RUNNING)
	{
		// temporary

		mc.state = SM_ST_READY;
		
		SM_PULSE_OFF();

		mi_stop();

		/*
		SM_DISABLE();

		mc.state = SM_ST_STOPPING;
		*/
	}

	DPRINTF("SM2: stop\n");
}

int sm2_reset_position()
{
	if (mc.state == SM_ST_READY)
	{
		mc.cpPos = 0;
		return 0;
	}
	
	return -1;
}


/**
	motor enable(power)
*/
void sm2_enable()
{
	SM_ENABLE();
}

/**
	motor disable(power)
*/
void sm2_disable()
{
	SM_DISABLE();
}

void sm2_process()
{
	if (mc.state == SM_ST_RUNNING)
	{
		if (mc.mi_upFlag == 0)
		{
			// calc. next speed for running mode, until reach to target speed

			// TODO: only when (curr_speed < mc.speed)
			if (mc.m_nextSpeed < mc.speed)
			{			
				int next_spd = mc.m_nextSpeed + MDELTA_SPEED;
				if (next_spd > mc.speed) next_spd = mc.speed;		// target speed.

				sm_prepare_next_speed(next_spd);

				mc.mi_upFlag = 1;
			}
			// else running at target speed
		}
	}
	else if (mc.state == SM_ST_STOPPING)
	{
		if (mc.mi_upFlag == 0)
		{
			// calc. next speed for running mode, until reach to target speed

			// TODO: only when (curr_speed < mc.speed)
			if (mc.m_nextSpeed >=  mc.min_speed)
			{
				int next_spd = mc.m_nextSpeed - MDELTA_SPEED;
				if (next_spd < mc.min_speed)
				{
					// stop now.
					mi_stop();
					//SM_DISABLE();
					mc.state = SM_ST_READY;
				}
				else
				{
					sm_prepare_next_speed(next_spd);
					//mc.m_nextPos = 0;		// to update now

					mc.mi_upFlag = 1;
				}
			}
			// else running at target speed
		}
	}
	else if (mc.state == SM_ST_MOVING)
	{
		if (mc.mi_upFlag == 0)
		{
			if (mc.m_nextSpeed == 0)
			{
				DPRINTF(" SM2: moving -> stopped \n");
				mc.state = SM_ST_READY;
			}
			else
			{
				// m_nextSpeed, m_nextPos.
				if (mc.astate == 4)		// no more update
				{
					// stop after update
					mc.mi_stopFlag = 1;

					//mc.m_nextOCR = sm_getOcrForSpeed(mc.m_nextSpeed);
					//mc.mi_upFlag = 1;
				}
				else
				{
					sm_calcNextPlan();

					//mc.m_nextOCR = sm_getOcrForSpeed(mc.m_nextSpeed);
					mc.mi_upFlag = 1;

					if (mc.astate == 4)
					{
						// stop after update
						mc.mi_stopFlag = 1;
					}

				}

			}
		}
	}
}

void sm2_print_state()
{
	printf("st:%c, pos:%ld\n", sm2_get_state(), sm2_get_position());
}

//-----------------------------------------------------------------------------
// motor with timer 4.
//
/**
	Prepare for next speed for run mode
	update m_nextSpeed, m_nextPos, m_nextOCR
*/
static inline void sm_prepare_next_speed(int spd)
{
	DPRINTF("sm1- spd : %d\n", spd);
	mc.m_nextSpeed = spd;
	mc.m_nextPos = mc.m_nextSpeed / MDT;

	mc.m_nextOCR = sm_getOcrForSpeed(mc.m_nextSpeed);
}

/**
	Using curr pos. target.

	Calculate 
	- max_speed
	- m_pb : de-accel point.
*/
static void sm_calcSpeedPlan(uint32 dist)
{
	if (dist == 0) return;

	int32 m_speed = MMIN_SPEED;

	mc.min_speed = MMIN_SPEED;
	mc.max_speed = MMIN_SPEED;
	mc.m_pb = 0;			// de-accel point
	mc.m_dist = dist;
	mc.astate = 0;
	
	while (1)
	{
		int td = m_speed / MDT;		// 100 ms moving distance.
		if (td == 0) td = 1;

		if (dist > td * 3)
		{
			dist = dist - td*2;

			mc.m_pb += td;				// deaccel point.
			m_speed += MDELTA_SPEED;
			mc.max_speed = m_speed;

			if (mc.max_speed >= MMAX_SPEED) break;
		}
		else
		{
			break;
		}
	}

	mc.m_pb = mc.m_dist - mc.m_pb;

	DPRINTF("SM2: max_speed=%ld, pb=%ld\n", mc.max_speed, mc.m_pb);
}


/**
	From current state,	calculate next speed and position to update

	- update m_nextSpeed, m_nextPos, m_nextOCR
*/
static void sm_calcNextPlan()
{
	if (mc.astate == 0)			// first move.
	{
		mc.m_prevPos = 0;
		mc.m_nextSpeed = mc.min_speed;		// start speed.

		if (mc.m_nextSpeed >= mc.max_speed)
		{
			mc.m_nextPos = mc.m_pb;							// same with dist
			mc.astate = 2;
		}
		else
		{
			mc.m_nextPos = mc.m_nextSpeed / MDT;			// same with dist
			if (mc.m_nextPos == 0) mc.m_nextPos = 1;
			mc.astate = 1;
		}
	}
	else if (mc.astate == 1)	// accel area.
	{
		mc.m_prevPos = mc.m_prevPos + mc.m_nextPos;

		mc.m_nextSpeed += MDELTA_SPEED;

		if (mc.m_nextSpeed >= mc.max_speed)
		{
			mc.m_nextPos = (mc.m_pb - mc.m_prevPos);		// dist
			if (mc.m_nextPos == 0) mc.m_nextPos = 1;
			mc.astate = 2;
		}
		else
		{
			mc.m_nextPos = mc.m_nextSpeed / MDT;		// dist
			if (mc.m_nextPos == 0) mc.m_nextPos = 1;
		}
	}
	else if (mc.astate == 2)	// max, steady speed area.
	{
		mc.m_prevPos = mc.m_prevPos + mc.m_nextPos;
		mc.m_nextSpeed -= MDELTA_SPEED;

		if (mc.m_nextSpeed <= mc.min_speed)
		{
			mc.m_nextSpeed = mc.min_speed;
			mc.m_nextPos = (mc.m_dist - mc.m_prevPos);		// dist

			mc.astate = 4;
		}
		else
		{
			mc.m_nextPos = mc.m_nextSpeed / MDT;		// dist
			if (mc.m_nextPos == 0) mc.m_nextPos = 1;
			mc.astate = 3;
		}
	}
	else if (mc.astate == 3)	// de-accel area
	{
		mc.m_prevPos = mc.m_prevPos + mc.m_nextPos;
		mc.m_nextSpeed -= MDELTA_SPEED;
		
		if (mc.m_nextSpeed <= mc.min_speed)
		{
			mc.m_nextSpeed = mc.min_speed;
			mc.m_nextPos = (mc.m_dist - mc.m_prevPos);		// dist
			mc.astate = 4;
		}
		else
		{
			mc.m_nextPos = mc.m_nextSpeed / MDT;		// dist
			if (mc.m_nextPos == 0) mc.m_nextPos = 1;
			mc.astate = 3;
		}
	}
	else if (mc.astate == 4)
	{
		mc.m_nextSpeed = 0;
		mc.m_nextPos = 0;	// dist
	}

	mc.m_nextOCR = sm_getOcrForSpeed(mc.m_nextSpeed);

	//DPRINTF("- ast:%d, spd:%ld, p=%ld, d=%ld\n", mc.astate, mc.m_nextSpeed, mc.m_prevPos, mc.m_nextPos);
}


/**
	Speed(hz) to OCR value
*/
static uint16 sm_getOcrForSpeed(uint32 hz)
{
	//if (hz < SM_MIN_HZ) hz = SM_MIN_HZ;
	//else if (hz > SM_MAX_HZ) hz = SM_MAX_HZ;

	uint16 ocr = F_CPU / 256 / (hz*2);

	return ocr;
}


/**
	start move accel/deaccel.
	with..
		m_nextOCR <- m_nextSpeed
		m_nextPos
*/
static void mi_start()
{
	// first set next speed update position
	mc.mi_dpos = 0;
	mc.mi_nextPos = mc.m_nextPos;

	OCR4A = mc.m_nextOCR;

	mc.mi_upFlag = 0;
	mc.mi_stopFlag = 0;

	TCNT4H = 0;
	TCNT4L = 0;
	TCCR4A = 0x40;		// Normal mode.  WGMn[3-0] = b0100.
		// COM1A1,A0 = b00, normal port operation.(CTC mode)
	
	//TCCR4A = 0x40;		// OC1A pulse mode ?

	// clock.
	//TCCR4B = 0x09;		// clk_io/1
	TCCR4B = 0x0C;		// CSn2/1/0 = b100, clk_io/256, CTC mode.

	TIMSK4 |= (1<<OCIE4A);
}

/**
	Motor Stop
*/
static void mi_stop()
{
	TCCR4B = 0x08; // No Clock Source(Timer/Counter Stopped)

	TIMSK4 &= ~(1<<OCIE4A);
}



ISR(TIMER4_COMPA_vect)
{
	static volatile int pst = 0;

	if (pst == 1)
	{
		//SM_PULSE_OFF();

		pst = 0;
		mc.mi_dpos ++;
		mc.cpPos += mc.dir;

		if (mc.mi_dpos >= mc.mi_nextPos)
		{
			if (mc.mi_upFlag == 1)
			{
				// update speed and target position
				OCR4A = mc.m_nextOCR;

				mc.mi_dpos = 0;
				mc.mi_nextPos = mc.m_nextPos;		// as distance from prev.
				mc.mi_upFlag = 0;
			}
			else if (mc.mi_stopFlag == 1) 
			{
				mc.state = SM_ST_READY;
				TCCR4B = 0x18;	
				TIMSK4 |= (1<<OCIE4A);
				
				//SM_DISABLE();
			}
		}
	}
	else
	{
		//SM_PULSE_ON();
		pst = 1;
	}
}
