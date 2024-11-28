/**
 * RCP: Recycle Device Control Protocol
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <util/delay.h>

/* Macros */
#define RCP_GETCHAR(pc)     uart2_getchar(pc)
#define RCP_PUTCHAR(c)      uart2_putchar(c)

#define STX_H               0xFF
#define STX_L               0xAA
#define MAX_RX_BUFFER_SIZE  256
#define PACKET_OVERHEAD     5    /* STX_H + STX_L + LEN + CS */
#define MAX_PAYLOAD_SIZE    (MAX_RX_BUFFER_SIZE - PACKET_OVERHEAD)

/* Function prototypes */
static int rcp_send_packet(uint8_t *packet, int len);
static int rcp_send_command(uint8_t *cmd, int len);

/* Static variables */
static uint8_t rxBuff[MAX_RX_BUFFER_SIZE];
static uint8_t rxLen;
static uint8_t rxState;
static uint8_t rxPacketLength;
static uint8_t txBuff[16];

/**
 * Initialize RCP module.
 *
 * @return 0 on success, -1 on failure
 */
int rcp_init(void)
{
    uart2_init(BAUD_115200);
    rxState = 0;
    return 0;
}

/**
 * Get rx message and parse.
 * Packet format: {STX_H, STX_L, Length, Cmd, DataN, CS}
 *
 * @param out_buff Output buffer to store the received packet
 * @return Length of the received packet, -1 on failure
 */
int rcp_get_packet(uint8_t *out_buff)
{
    if (out_buff == NULL) {
        return -1;
    }

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
            if (rxPacketLength > MAX_PAYLOAD_SIZE) {
                rxState = 0;
                return -1;
            }
            rxState = 20;
            break;

        case 20:    // receive only rxPacketLength
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
        case 90:    // wait checksum
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
 * Make command packet and send it, from {Cmd, DataN}
 *
 * @param cmd Command packet to send
 * @param len Length of the command packet
 * @return 0 on success, -1 on failure
 */
int rcp_send_command(uint8_t *cmd, int len)
{
    if (cmd == NULL || len <= 0) {
        return -1;
    }

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

/**
 * Send packet over UART.
 *
 * @param packet Packet to send
 * @param len Length of the packet
 * @return 0 on success, -1 on failure
 */
static int rcp_send_packet(uint8_t *packet, int len)
{
    if (packet == NULL || len <= 0) {
        return -1;
    }

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
