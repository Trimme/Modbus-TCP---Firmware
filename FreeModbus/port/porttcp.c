/*
 * porttcp.c
 *
 *  Created on: 16 Dec 2019
 *      Author: David
 */
/* ----------------------- System includes ----------------------------------*/
#include <stdio.h>
#include <string.h>
#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"
/* ----------------------- Wiznet includes ----------------------------------*/
#include "socket.h"

#define MB_TCP_DEFAULT_PORT  502          /* TCP listening port. */
#define MB_TCP_BUF_SIZE      2048         /* Must hold a complete Modbus TCP frame. */


static UCHAR    ucTCPRequestFrame[MB_TCP_BUF_SIZE]; // Receive buffer
static USHORT   ucTCPRequestLen;

static UCHAR    ucTCPResponseFrame[MB_TCP_BUF_SIZE]; // Transmit buffer
static USHORT   ucTCPResponseLen;

BOOL bFrameSent = FALSE;   // Send response flag

BOOL xMBTCPPortInit (USHORT usTCPPort)
{
	SOCKET sn;
	sn = 0;
	if (getSn_SR(sn) == SOCK_CLOSED) {
		socket(sn, Sn_MR_TCP, usTCPPort, 0x00);  // open socket
	}
	if (getSn_SR(sn) == SOCK_INIT ) {
		listen(sn); // listen
		return TRUE;
	}

	return FALSE;
}

BOOL xMBTCPPortGetRequest (UCHAR **ppucMBTCPFrame, USHORT *usTCPLength)
{
	*ppucMBTCPFrame = (uint8_t *) & ucTCPRequestFrame [0];
	*usTCPLength = ucTCPRequestLen;
	/* Reset the buffer. */
	ucTCPRequestLen = 0;

	return TRUE;
}


BOOL xMBTCPPortSendResponse (const UCHAR *pucMBTCPFrame, USHORT usTCPLength)
{
	memcpy (ucTCPResponseFrame, pucMBTCPFrame, usTCPLength);
	ucTCPResponseLen = usTCPLength;
	bFrameSent = TRUE; // W5500 transmits data by

	return bFrameSent;
}

void modbus_tcps(uint8_t sn, uint16_t port)
{
	switch(getSn_SR(sn)) {    // Get socket status
		case SOCK_CLOSED:     // Socket is in closed state
			socket(sn, Sn_MR_TCP, port, 0x00);  //Open socket
			break;
		case SOCK_INIT :  // Socket is in initialized state
			listen(sn);  // Listen
			break;
		case SOCK_ESTABLISHED :   // Socket is in connected state
			if (getSn_IR(sn) & Sn_IR_CON) {
				setSn_IR(sn, Sn_IR_CON);
			}
			ucTCPRequestLen = recv(sn, ucTCPRequestFrame, MB_TCP_BUF_SIZE); // W5500 receives data
			xMBPortEventPost(EV_FRAME_RECEIVED);  // Send EV_FRAME_RECEIVED event to drive the state machine in eMBpoll() function
			eMBPoll();   // Process EV_FRAME_RECEIVED event
			eMBPoll();   // Handle EV_EXECUTE event
			if (bFrameSent) {
				bFrameSent = FALSE;
				// W5500 sends Modbus response packet
				send(sn, ucTCPResponseFrame, ucTCPResponseLen);
			}
			break;
		case SOCK_CLOSE_WAIT :   //Socket is waiting to close
			disconnect(sn); // Close the connection
			break;
		default:
			break;
   }
}

void vMBTCPPortClose (void)
{
};

void vMBTCPPortDisable (void)
{
};
