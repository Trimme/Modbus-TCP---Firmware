/*
===============================================================================
 Name        : Modbus_TCP-Firmware.c
 Author      : $(author)
 Version     :
 Copyright   : $(copyright)
 Description : main definition
===============================================================================
*/

#include "chip.h"
#include <cr_section_macros.h>
#include <stdio.h>
//#include "stdutils.h"

// TODO: insert other definitions and declarations here

/* SSP */
#define BUFFER_SIZE (0x100)
#define LPC_SSP LPC_SSP1
#define LPC_GPDMA_SSP_TX GPDMA_CONN_SSP1_Tx
#define LPC_GPDMA_SSP_RX GPDMA_CONN_SSP1_Rx

static SSP_ConfigFormat ssp_format;
static volatile uint8_t ssp_dma_txf_completed = 0;
static volatile uint8_t ssp_dma_rxf_completed = 0;

/* UART Selection */
#define UART_SELECTION LPC_UART2
#define IRQ_SELECTION UART2_IRQn
#define HANDLER_NAME UART2_IRQHandler

/* Tx/Rx ring buffers */
STATIC RINGBUFF_T txring, rxring;

/* Ring buffer sizes */
#define UART_SRB_SIZE 128
#define UART_RRB_SIZE 32

/* Tx/Rx buffers */
static uint8_t rxbuff[UART_RRB_SIZE], txbuff[UART_SRB_SIZE];

/* Function declarations */
void GPIO_Init(void);
void UART_Init(void);
void SSP_Init(void);

int _write(int iFileHandle, char *pcBuffer, int iLength);
void _delay_ms(uint16_t ms);
void HANDLER_NAME(void);

int main(void) {

	// Read clock settings and update SystemCoreClock variable
    SystemCoreClockUpdate();
    // Init GPIO
    GPIO_Init();
    // Init UART
    UART_Init();
    printf("UART init done\r\n");
    // Init SSP
    SSP_Init();
	printf("SSP init done\r\n");

    _delay_ms(500);
    printf("Sending SPI frame: [10101101]\r\n");
    Chip_SSP_SendFrame(LPC_SSP, 0b10101101);
    printf("Done.\r\n");

    _delay_ms(1000);
    printf("UART Deactivated, entering blinking mode...\r\n");
    _delay_ms(500);

    NVIC_DisableIRQ(IRQ_SELECTION);
    Chip_UART_DeInit(UART_SELECTION);

    // Force the counter to be placed into memory
    volatile static int i = 0 ;
    // Enter an infinite loop, just incrementing a counter
    while(1) {

    	Chip_GPIO_SetPinState(LPC_GPIO, 1, 18, true);
    	Chip_GPIO_SetPinState(LPC_GPIO, 1, 23, false);

    	_delay_ms(250);

    	Chip_GPIO_SetPinState(LPC_GPIO, 1, 20, true);
    	Chip_GPIO_SetPinState(LPC_GPIO, 1, 18, false);

    	_delay_ms(250);

    	Chip_GPIO_SetPinState(LPC_GPIO, 1, 21, true);
    	Chip_GPIO_SetPinState(LPC_GPIO, 1, 20, false);

    	_delay_ms(250);

    	Chip_GPIO_SetPinState(LPC_GPIO, 1, 23, true);
    	Chip_GPIO_SetPinState(LPC_GPIO, 1, 21, false);

    	_delay_ms(250);

        i++ ;
    }
    return 0;
}

void SSP_Init(void){

	/* Init Pins */
	Chip_IOCON_PinMux(LPC_IOCON, 0, 6, IOCON_MODE_INACT, IOCON_FUNC2); // SSEL 1
	Chip_IOCON_PinMux(LPC_IOCON, 0, 7, IOCON_MODE_INACT, IOCON_FUNC2); // SCLK1
	Chip_IOCON_PinMux(LPC_IOCON, 0, 8, IOCON_MODE_INACT, IOCON_FUNC2); // MISO1
	Chip_IOCON_PinMux(LPC_IOCON, 0, 9, IOCON_MODE_INACT, IOCON_FUNC2); // MOSI1

	/* SSP Init */
	Chip_SSP_Init(LPC_SSP);
	ssp_format.frameFormat = SSP_FRAMEFORMAT_SPI;
	ssp_format.bits = SSP_BITS_8;
	ssp_format.clockMode = SSP_CLOCK_MODE0;
	Chip_SSP_SetFormat(LPC_SSP, ssp_format.bits, ssp_format.frameFormat, ssp_format.clockMode);
	Chip_SSP_Enable(LPC_SSP);

}

void GPIO_Init(void){

    /* Init GPIO */
    Chip_GPIO_Init(LPC_GPIO);
    Chip_GPIO_SetPinDIROutput(LPC_GPIO, 1, 18);
    Chip_GPIO_SetPinDIROutput(LPC_GPIO, 1, 20);
    Chip_GPIO_SetPinDIROutput(LPC_GPIO, 1, 21);
    Chip_GPIO_SetPinDIROutput(LPC_GPIO, 1, 23);

}
void UART_Init(void){

    /* Init Pins */
    Chip_IOCON_PinMux(LPC_IOCON, 0, 10, IOCON_MODE_INACT, IOCON_FUNC1); // IOCON P0.10 TXD2 (func1), no pull
    Chip_IOCON_PinMux(LPC_IOCON, 0, 11, IOCON_MODE_INACT, IOCON_FUNC1); // IOCON P0.11 RXD2 (func1), no pull

    /* UART Init */
    Chip_UART_Init(UART_SELECTION);
    Chip_UART_SetBaud(UART_SELECTION, 115200);
    Chip_UART_ConfigData(UART_SELECTION, (UART_LCR_WLEN8 | UART_LCR_SBS_1BIT));
    Chip_UART_SetupFIFOS(UART_SELECTION, (UART_FCR_FIFO_EN | UART_FCR_TRG_LEV2));
    Chip_UART_TXEnable(UART_SELECTION);

    /* Init ring buffers */
    RingBuffer_Init(&rxring, rxbuff, 1, UART_RRB_SIZE);
    RingBuffer_Init(&txring, txbuff, 1, UART_SRB_SIZE);

    /* Reset and enable FIFOs, FIFO trigger level 3 (14 chars) */
    Chip_UART_SetupFIFOS(UART_SELECTION, (UART_FCR_FIFO_EN | UART_FCR_RX_RS | UART_FCR_TX_RS | UART_FCR_TRG_LEV3));

    /* Enable receive data and line status interrupt */
    Chip_UART_IntEnable(UART_SELECTION, (UART_IER_RBRINT | UART_IER_RLSINT));

    /* preemption = 1, sub-priority = 1 */
    NVIC_SetPriority(IRQ_SELECTION, 1);
    NVIC_EnableIRQ(IRQ_SELECTION);

}

void HANDLER_NAME(void){

	Chip_UART_IRQRBHandler(UART_SELECTION, &rxring, &txring);

}

int _write(int iFileHandle, char *pcBuffer, int iLength){

	int ret;

	ret = Chip_UART_SendRB(UART_SELECTION, &txring, (const uint8_t *) pcBuffer, iLength);

	return ret;
}

void _delay_ms(uint16_t ms){

	uint16_t delay;
	volatile uint32_t i;

	for(delay = ms; delay > 0; delay--){
		for(i = 10000; i > 0; i--);
	}

}
