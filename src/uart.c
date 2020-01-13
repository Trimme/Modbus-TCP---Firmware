/*
===============================================================================
 Name        : uart.c
 Author      : $(author)
 Version     :
 Copyright   : $(copyright)
 Description : Modbus TCP Implementation for NXP LPC176xx and Wiznet W5500
===============================================================================
*/

/* ------------------------ System Includes ------------------------------- */
#include <stdio.h>

/* ------------------------ Project Includes ------------------------------ */
#include "chip.h"
#include "uart.h"

/* ------------------------ Private Variables------------------------------ */

/* UART Tx/Rx buffers */
static uint8_t rxbuff[UART_RRB_SIZE], txbuff[UART_SRB_SIZE];

/* ------------------------ Function Definitions--------------------------- */
void UART_Init(void)
{
    /* Init Pins */
    // IOCON P0.10 TXD2 (func1), no pull
    Chip_IOCON_PinMux(LPC_IOCON, 0, 10, IOCON_MODE_INACT, IOCON_FUNC1);
    // IOCON P0.11 RXD2 (func1), no pull
    Chip_IOCON_PinMux(LPC_IOCON, 0, 11, IOCON_MODE_INACT, IOCON_FUNC1);

    /* UART Init */
    Chip_UART_Init(UART_SELECTION);
    Chip_UART_SetBaud(UART_SELECTION, 57600);
    Chip_UART_ConfigData(UART_SELECTION, (UART_LCR_WLEN8 | UART_LCR_SBS_1BIT));
    Chip_UART_SetupFIFOS(UART_SELECTION, (UART_FCR_FIFO_EN | UART_FCR_TRG_LEV2));
    Chip_UART_TXEnable(UART_SELECTION);

    /* Init Ring Buffers */
    RingBuffer_Init(&rxring, rxbuff, 1, UART_RRB_SIZE);
    RingBuffer_Init(&txring, txbuff, 1, UART_SRB_SIZE);

    /* Reset and enable FIFOs, FIFO trigger level 3 (14 chars) */
    Chip_UART_SetupFIFOS(UART_SELECTION, (UART_FCR_FIFO_EN | UART_FCR_RX_RS |
                         UART_FCR_TX_RS | UART_FCR_TRG_LEV1));

    /* Enable receive data and line status interrupt */
    Chip_UART_IntEnable(UART_SELECTION, (UART_IER_RBRINT | UART_IER_RLSINT));

    /* preemption = 1, sub-priority = 1 */
    NVIC_SetPriority(IRQ_SELECTION, 1);
    NVIC_EnableIRQ(IRQ_SELECTION);
}

void HANDLER_NAME(void)
{
	Chip_UART_IRQRBHandler(UART_SELECTION, &rxring, &txring);
}
