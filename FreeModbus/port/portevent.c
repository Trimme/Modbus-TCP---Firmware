/*
 * portevent.c
 *
 *  Created on: 16 Dec 2019
 *      Author: David
 */


/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"

/* ----------------------- Variables ----------------------------------------*/
static eMBEventType   eQueuedEvent;
static BOOL           xEventInQueue;

/* ----------------------- Start implementation -----------------------------*/
BOOL xMBPortEventInit(void)
{
    xEventInQueue = FALSE;
    return TRUE;
}

BOOL xMBPortEventPost(eMBEventType eEvent)
{
  // Event flag update
  xEventInQueue = TRUE;
  // Set event flag
  eQueuedEvent = eEvent;
  return TRUE;
}

BOOL xMBPortEventGet(eMBEventType * eEvent)
{
  BOOL xEventHappened = FALSE;

  // If there is an event update
  if(xEventInQueue) {
    // Get event
    *eEvent = eQueuedEvent;
    xEventInQueue = FALSE;
    xEventHappened = TRUE;
  }

  return xEventHappened;
}
