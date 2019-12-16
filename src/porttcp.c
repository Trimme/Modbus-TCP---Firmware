/*
 * porttcp.c
 *
 *  Created on: 16 Dec 2019
 *      Author: David
 */


BOOL xMBTCPPortInit (USHORT usTCPPort)
{
    SOCKET sn;
    sn = 0;
    if (getSn_SR (sn) == SOCK_CLOSED)
    {
       socket (sn, Sn_MR_TCP, usTCPPort, 0x00);  // open socket
    }
    if (getSn_SR (sn) == SOCK_INIT )
    {
     listen (sn); // listen
     return TRUE;
    }
    return FALSE;
}

BOOL xMBTCPPortGetRequest (UCHAR ** ppucMBTCPFrame, USHORT * usTCPLength)
{
    * ppucMBTCPFrame = (uint8_t *) & ucTCPRequestFrame [0];
    * usTCPLength = ucTCPRequestLen;
    / * Reset the buffer. * /
    UcTCPRequestLen = 0;
    return TRUE;
}


XMBTCPPortSendResponse BOOL (UCHAR * const pucMBTCPFrame, USHORT usTCPLength)
{
      the memcpy (ucTCPResponseFrame, pucMBTCPFrame, usTCPLength);
      ucTCPResponseLen = usTCPLength;
      bFrameSent = TRUE; // W5500 transmits data by
      return bFrameSent;
}


void vMBTCPPortClose (void)
{
};

void vMBTCPPortDisable (void)
{
};
