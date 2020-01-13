/*
===============================================================================
 Name        : uart.h
 Author      : $(author)
 Version     :
 Copyright   : $(copyright)
 Description : Modbus TCP Implementation for NXP LPC176xx and Wiznet W5500
===============================================================================
*/

/* ------------------------ Defines --------------------------------------- */

/* UART Selection */
#define UART_SELECTION LPC_UART2
#define IRQ_SELECTION UART2_IRQn
#define HANDLER_NAME UART2_IRQHandler

/* UART Tx/Rx Ring Buffers */
STATIC RINGBUFF_T txring, rxring;

/* UART Ring Buffer Sizes */
#define UART_SRB_SIZE 128
#define UART_RRB_SIZE 32

/* ------------------------ Function declarations ------------------------- */

void UART_Init(void);
void HANDLER_NAME(void);
