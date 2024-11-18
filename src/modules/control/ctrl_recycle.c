/**
	RCP: Recycle Device Control Protocol

	- automatic query, update current state of recycler.
*/
#include "BuildOptions.h"

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "types.h"
#include "timer.h"

#include "RC.h"
#include "RCP.h"

#define	STX_H	0xFF
#define	STX_L	0xAA
//-----------------------------------------------------------------------------

//internal state
#define ST_UNKNOWN		'u'
#define ST_CHECKING		'c'
#define ST_READY		'R'
#define ST_RUNNING		'r'
//#define ST_QUERY		'q'

static int state;
static int completed;
static int tid_op;
static RcState st;
static int opcEventUpdated;		
static RcOpcEvent opcEvent;

#ifdef TEST_SIM
static int sim_state = 0;
static int tid_sim;
static int sim_ncup = 0;

void rc_sim_process();
void rc_print_status();

#endif

int rc_init()
{
	rcp_init();
	
	memset(&st, 0, sizeof(RcState));

	state = ST_UNKNOWN;
	tid_op = timer_alloc();
	timer_set(tid_op, 3000);			// start check in 3 seconds

	opcEventUpdated = 0;
#ifdef TEST_SIM
	tid_sim = timer_alloc();
#endif

	return 0;
}

RcState rc_get_state()
{
	return st;
}

int rc_is_completed()
{
	return completed;
}
/*
	emergency stop
*/
int rc_stop()
{
	uint8_t cmd[1] = {0x01};
	return rcp_send_command(cmd, 1);
}

/*
	Start operation
*/
int rc_start(uint8_t ncup)
{
#ifdef TEST_SIM
	sim_state = 1;
	sim_ncup = ncup+1;
	timer_set(tid_sim, 1000);
#endif

	state = ST_RUNNING;
	completed = 0;

	uint8_t cmd[2] = {0x02};
	cmd[1] = ncup;
	return rcp_send_command(cmd, 2);
}

void rc_reset_state()
{	
	st.errorCode = 0;
	st.errorRoboSeq = 0;
	st.remainCupNum = 0;
	st.purifiedTankLevel = 0;
	st.wasteTankLevel = 0;

	// 6 fields
	st.plasticTray1Num = 0;
	st.plasticTray2Num = 0;
	st.plasticTray3Num = 0;
	st.plasticTray4Num = 0;
	st.plasticTray5Num = 0;
	st.plasticTray6Num = 0;
	
	st.paperTray1Num = 0;
	st.paperTray2Num = 0;
	st.paperTray3Num = 0;
	st.paperTray4Num = 0;
	st.paperTray5Num = 0;
	st.paperTray6Num = 0;

	st.plasticTrayCupNum = 0;	
	st.paperTrayCupNum = 0;
	
	st.cupHeightL = 0;
	st.cupHeightH = 0;
}

// send reset command..
// - re
int rc_reset()
{
	rc_reset_state();
	
	uint8_t cmd[2] = {0x00};
	return rcp_send_command(cmd, 1);
}


/*
	Start sub-operation

	subCmd :
		1	- rotate plastic tray
		2	- rotate paper tray
		3	- inject hot water
		4	- inject air
		5	- run drain pump
*/
int rc_start_sub(uint8_t subCmd)
{
	uint8_t cmd[2] = {0x03};
	cmd[1] = subCmd;
	return rcp_send_command(cmd, 2);
}
/*
	Stop sub-operation
*/
int rc_stop_sub()
{
	uint8_t cmd[1] = {0x04};
	return rcp_send_command(cmd, 1);
}

/*
	Query tray state
*/
int rc_query_tray_state()
{
	uint8_t cmd[1] = {0x11};
	return rcp_send_command(cmd, 1);	
}

/*
	Query water tank state
*/
int rc_query_water_state()
{
	uint8_t cmd[1] = {0x12};
	return rcp_send_command(cmd, 1);
}



/*
	Query overall state
*/
int rc_query_state()
{
	uint8_t cmd[1] = {0x10};
	return rcp_send_command(cmd, 1);
}

int rc_is_opc_event_updated()
{
	return opcEventUpdated;
}

RcOpcEvent rc_get_opcEvent()
{
	opcEventUpdated = 0;
	return opcEvent;
}

/*
uint8_t roboSeq = 0;
uint8_t remainCupNum = 0;
uint8_t plasticTrayCupNum = 0;
uint8_t paperTrayCupNum = 0;

// 0: low, 1:normal, 2:full
uint8_t purifiedTankState = 0;
uint8_t wasteTankState = 0;

uint8_t errorRoboSeq = 0;
uint8_t errorCode = 0;
*/
static uint8_t rxBuff[DCP_MAX_PACKET_LEN];
static int rxLen;

#define ERROR_ROBO_SEQ		0x01
#define ERROR_TRAY_FULL		0x02
#define ERROR_WATER_FULL	0x04

void rc_process()
{
	uint8_t cmd;
	uint8_t i;

	// comm. control - rx ctrl message and proces it.
	if ((rxLen=rcp_get_packet(rxBuff)) > 0)
	{
		cmd = rxBuff[0];
		switch (cmd)
		{
		case 0x10 :	// res. operation state
			DPRINTF("RC.OP st len = %d\n", rxLen);
			if (rxLen != 20) {
				DPRINTF(" --> garbage discard\n");
				break;
			}
			i=1;
			// 0 : no data
			i++;
			st.roboSeq = rxBuff[i++];
			st.remainCupNum = rxBuff[i++];
			st.purifiedTankLevel = rxBuff[i++];
			st.wasteTankLevel = rxBuff[i++];

			// 6 fields
			st.plasticTray1Num = rxBuff[i++];
			st.plasticTray2Num = rxBuff[i++];
			st.plasticTray3Num = rxBuff[i++];
			st.plasticTray4Num = rxBuff[i++];
			st.plasticTray5Num = rxBuff[i++];
			st.plasticTray6Num = rxBuff[i++];
			
			st.paperTray1Num = rxBuff[i++];
			st.paperTray2Num = rxBuff[i++];
			st.paperTray3Num = rxBuff[i++];
			st.paperTray4Num = rxBuff[i++];
			st.paperTray5Num = rxBuff[i++];
			st.paperTray6Num = rxBuff[i++];

			st.plasticTrayCupNum = 0;
			st.plasticTrayCupNum += st.plasticTray1Num;
			st.plasticTrayCupNum += st.plasticTray2Num;
			st.plasticTrayCupNum += st.plasticTray3Num;
			st.plasticTrayCupNum += st.plasticTray4Num;
			st.plasticTrayCupNum += st.plasticTray5Num;
			st.plasticTrayCupNum += st.plasticTray6Num;
			
			st.paperTrayCupNum = 0;
			st.paperTrayCupNum += st.paperTray1Num;
			st.paperTrayCupNum += st.paperTray2Num;
			st.paperTrayCupNum += st.paperTray3Num;
			st.paperTrayCupNum += st.paperTray4Num;
			st.paperTrayCupNum += st.paperTray5Num;
			st.paperTrayCupNum += st.paperTray6Num;
			
			st.cupHeightL = rxBuff[i++];
			st.cupHeightH = rxBuff[i++];

			if (state == ST_CHECKING)
			{
				state = ST_READY;
				timer_set(tid_op, 1000);
			}
			break;

		case 0x20 :	// ev. operation error
			DPRINTF("RC.OP ev");
			if (rxLen != 20) {
				DPRINTF(" --> garbage discard\n");
				break;
			}
			i=1;
			st.errorCode = rxBuff[i++];			// event
			st.errorRoboSeq = rxBuff[i++];
			st.remainCupNum = rxBuff[i++];
			st.purifiedTankLevel = rxBuff[i++];
			st.wasteTankLevel = rxBuff[i++];

			// 6 fields
			st.plasticTray1Num = rxBuff[i++];
			st.plasticTray2Num = rxBuff[i++];
			st.plasticTray3Num = rxBuff[i++];
			st.plasticTray4Num = rxBuff[i++];
			st.plasticTray5Num = rxBuff[i++];
			st.plasticTray6Num = rxBuff[i++];
						
			st.paperTray1Num = rxBuff[i++];
			st.paperTray2Num = rxBuff[i++];
			st.paperTray3Num = rxBuff[i++];
			st.paperTray4Num = rxBuff[i++];
			st.paperTray5Num = rxBuff[i++];
			st.paperTray6Num = rxBuff[i++];

			st.plasticTrayCupNum = 0;
			st.plasticTrayCupNum += st.plasticTray1Num;
			st.plasticTrayCupNum += st.plasticTray2Num;
			st.plasticTrayCupNum += st.plasticTray3Num;
			st.plasticTrayCupNum += st.plasticTray4Num;
			st.plasticTrayCupNum += st.plasticTray5Num;
			st.plasticTrayCupNum += st.plasticTray6Num;
						
			st.paperTrayCupNum = 0;
			st.paperTrayCupNum += st.paperTray1Num;
			st.paperTrayCupNum += st.paperTray2Num;
			st.paperTrayCupNum += st.paperTray3Num;
			st.paperTrayCupNum += st.paperTray4Num;
			st.paperTrayCupNum += st.paperTray5Num;
			st.paperTrayCupNum += st.paperTray6Num;
						
			st.cupHeightL = rxBuff[i++];
			st.cupHeightH = rxBuff[i++];
			break;
			
		case 0x21 :	// ev. op complete event
			{
				opcEvent.cupMaterial = rxBuff[1];
				opcEvent.cupTray = 0;

				opcEventUpdated = 1;
				DPRINTF("RC:ev cupMaterial = %d\n", opcEvent.cupMaterial);
				DPRINTF("RC:ev cupTray     = %d\n", opcEvent.cupTray);
			}
			break;
		case 0x22 :	// ev. operation complete, ready
			// make it ready
			DPRINTF("RC: ready - complete\n");
			if (state == ST_RUNNING)
			{
				completed = 1;
				state = ST_READY;
			}
			break;
		default:
			DPRINTF("RC: unknown %d, len=%d\n", cmd, rxLen);
			break;
		}
		//end switch
	}

	switch (state)
	{
	case ST_UNKNOWN: 
		if (timer_isfired(tid_op))
		{
			//DPRINTF("RC:u\n");
			rc_query_state();
			state = ST_CHECKING;
			timer_set(tid_op, 2000);
		}
		break;
	case ST_CHECKING:
		if (timer_isfired(tid_op))
		{
			//DPRINTF("RC:f\n");
			timer_set(tid_op, 2000);
			state = ST_UNKNOWN;
#ifdef TEST_SIM
			state = ST_READY;
#endif
		}
		break;
	case ST_READY:
	case ST_RUNNING:
		// do nothing..
		if (timer_isfired(tid_op))
		{
			DPRINTF("RC:q\n");
			rc_query_state();
			timer_set(tid_op, 2000);
		}
		break;
	}

#ifdef TEST_SIM
	rc_sim_process();
#endif
}

void rc_print_state()
{
	DPRINTF("RC: st = %c\n", state);
	DPRINTF(" + roboSeq           = %d\n", st.roboSeq);
	DPRINTF(" + remainCupNum      = %d\n", st.remainCupNum);
	DPRINTF(" + plasticTrayCupNum = %d\n", st.plasticTrayCupNum);
	DPRINTF(" + paperTrayCupNum   = %d\n", st.paperTrayCupNum);
	DPRINTF(" + purifiedTankLevel = %d\n", st.purifiedTankLevel);
	DPRINTF(" + wasteTankLevel    = %d\n", st.wasteTankLevel);
	DPRINTF(" + errorCode         = %d\n", st.errorCode);
	DPRINTF(" + errorRoboSeq      = %d\n", st.errorRoboSeq);
	DPRINTF(" + plasticTray1Num   = %d\n", st.plasticTray1Num);
	DPRINTF(" + plasticTray2Num   = %d\n", st.plasticTray2Num);
	DPRINTF(" + plasticTray3Num   = %d\n", st.plasticTray3Num);
	DPRINTF(" + plasticTray4Num   = %d\n", st.plasticTray4Num);
	DPRINTF(" + plasticTray5Num   = %d\n", st.plasticTray5Num);
	DPRINTF(" + plasticTray6Num   = %d\n", st.plasticTray6Num);
	DPRINTF(" + paperTray1Num     = %d\n", st.paperTray1Num);
	DPRINTF(" + paperTray2Num     = %d\n", st.paperTray2Num);
	DPRINTF(" + paperTray3Num     = %d\n", st.paperTray3Num);
	DPRINTF(" + paperTray4Num     = %d\n", st.paperTray4Num);
	DPRINTF(" + paperTray5Num     = %d\n", st.paperTray5Num);
	DPRINTF(" + paperTray6Num     = %d\n", st.paperTray6Num);
}

#ifdef TEST_SIM
void rc_sim_process()
{
	if (timer_isfired(tid_sim))
	{
		timer_set(tid_sim, 1000);

		if (sim_state == 1)
		{
			if (sim_ncup == 0)
			{
				timer_clear(tid_sim);
				sim_state = 0;

				DPRINTF("RC: sim ready - complete\n");
				completed = 1;
				state = ST_READY;
			}
			else if (sim_ncup > 0)
			{
				sim_ncup -= 1;
				opcEvent.cupMaterial = sim_ncup % 2;
				opcEvent.cupTray = 0;

				opcEventUpdated = 1;
				DPRINTF("RC:sim ev cupMaterial = %d\n", opcEvent.cupMaterial);
				DPRINTF("RC:sim ev cupTray     = %d\n", opcEvent.cupTray);
			}
		}
	}
}
#endif