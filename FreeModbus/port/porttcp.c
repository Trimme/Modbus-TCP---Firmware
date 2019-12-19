/*
* FreeModbus Libary: Wiznet W55500 Port
* Copyright (C) 2006 Christian Walter <wolti@sil.at>
*
* This library is free software; you can redistribute it and/or
* modify it under the terms of the GNU Lesser General Public
* License as published by the Free Software Foundation; either
* version 2.1 of the License, or (at your option) any later version.
*
* This library is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
* Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public
* License along with this library; if not, write to the Free Software
* Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*
* File: $Id: porttcp.c,v 1.0 2019/12/16 dasi Exp $
*/

/* ----------------------- System includes ----------------------------------*/
#include <stdio.h>
#include <string.h>

/* ----------------------- Platform includes --------------------------------*/
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
	*ppucMBTCPFrame = &ucTCPRequestFrame[0];
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
