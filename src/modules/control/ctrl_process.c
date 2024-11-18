/**
	RCP: Recycle Device Control Protocol
*/
#include "BuildOptions.h"

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <util/delay.h>

#include "types.h"
#include "putil.h"
#include "uart2.h"
#include "timer.h"

#include "RCP.h"

#define RCP_GETCHAR(pc)	uart2_getchar(pc)
#define RCP_PUTCHAR(c)	uart2_putchar(c)

#define	STX_H	0xFF
#define	STX_L	0xAA

#define MAX_RX_BUFFER_SIZE 256

//-----------------------------------------------------------------------------

//static uint8_t	myid = '1';			// '1'..n

static uint8_t rxBuff[MAX_RX_BUFFER_SIZE];
static uint8_t rxLen;
static uint8_t rxState;
static uint8_t rxPacketLength;

int rcp_send_packet(uint8_t *packet, int len);

int rcp_init()
{
	//myid = bid;

	uart2_init(BAUD_115200);

	rxState = 0;

	return 0;
}

/**
	Get rx message and parse.
	packet format : {STX_H, STX_L, Length, Cmd, DataN, CS}
*/
int rcp_get_packet(uint8_t *out_buff)
{
	uint8_t c;

	if (RCP_GETCHAR(&c) == 1)
	{
		switch (rxState)
		{
		case 0:
			if (c == STX_H)
			{
				rxLen = 0;
				rxBuff[rxLen++] = c;
				rxState = 1;
			}
			break;
		case 1:
			if (c == STX_L)
			{
				if (rxLen >= MAX_RX_BUFFER_SIZE - 1) {
					rxState = 0;
					return -1;
				}
				rxBuff[rxLen++] = c;
				rxState = 10;
			}
			else
			{
				rxState = 0;
			}
			break;
		case 10:
			if (rxLen >= MAX_RX_BUFFER_SIZE - 1) {
				rxState = 0;
				return -1;
			}
			rxBuff[rxLen++] = c;
			rxPacketLength = c;
			// Validate packet length
			if (rxPacketLength > MAX_RX_BUFFER_SIZE - 5) { // STX_H + STX_L + LEN + CS
				rxState = 0;
				return -1;
			}
			rxState = 20;
			break;

		case 20:	// receive only rxPacketLength
			if (rxPacketLength > 0)
			{
				if (rxLen >= MAX_RX_BUFFER_SIZE - 1) {
					rxState = 0;
					return -1;
				}
				rxBuff[rxLen++] = c;
				rxPacketLength--;

				if (rxPacketLength == 0)
				{
					rxState = 90;
				}
			}
			break;
		case 90:	// wait checksum
			if (rxLen >= MAX_RX_BUFFER_SIZE) {
				rxState = 0;
				return -1;
			}
			rxBuff[rxLen++] = c;
			// check checksum
#if 0
			DPRINTF("RCP : rx :");
			for (int i=0; i<rxLen; i++)
			{				
				DPRINTF("%02X,", rxBuff[i]);
			}
			DPRINTF("\n");
#endif
			uint8_t cs = 0;
			for (int i=3; i<rxLen-1; i++)
			{
				cs += rxBuff[i];
			}

			if (cs == c)
			{
				// only {cmd, DataN} and length
				int len = rxLen-4;
				memcpy(out_buff, rxBuff+3, len);

				DPRINTF("RC: -> ok\n");
				rxState = 0;
				
				return len;
			}
			else
			{
				DPRINTF("RCP:rx cs fail : %02x,%02x\n", cs, c);
			}
			rxState = 0;

			break;
		}
		// end switch
	}

	return -1;
}

/**
	Make command packet and send it, from {Cmd, DataN}
*/

static uint8_t txBuff[16];

int rcp_send_command(uint8_t *cmd, int len)
{
	int txLen = 0;

	txBuff[txLen++] = STX_H;
	txBuff[txLen++] = STX_L;
	txBuff[txLen++] = (uint8_t)len;

	for (int i=0; i<len; i++)
	{
		txBuff[txLen++] = cmd[i];
	}
	uint8_t cs = 0;
	for (int i=3; i<txLen; i++)
	{
		cs += txBuff[i];
	}
	txBuff[txLen++] = cs;

	rcp_send_packet(txBuff, txLen);

	return 0;
}

int rcp_send_packet(uint8_t *packet, int len)
{
	int i;

	for (i=0; i<len; i++)
	{
		RCP_PUTCHAR(packet[i]);
	}
	
#if 0
	DPRINTF("RCP : tx :");
	for (int i=0; i<len; i++)
	{		
		DPRINTF("%02x", packet[i]);
	}
	DPRINTF("\n");
#endif
	return 0;
}
