/*
===============================================================================
 Name        : ssp_spi.c
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

/* ------------------------ Peripheral Includes --------------------------- */
#include "ssp_spi.h"

/* ------------------------ Function Definitions--------------------------- */

void SSP_Init(void)
{
	/* Init Pins */
	Chip_IOCON_PinMux(LPC_IOCON, 0, 7, IOCON_MODE_INACT, IOCON_FUNC2); // SCK1
	Chip_IOCON_PinMux(LPC_IOCON, 0, 8, IOCON_MODE_INACT, IOCON_FUNC2); // MISO1
	Chip_IOCON_PinMux(LPC_IOCON, 0, 9, IOCON_MODE_INACT, IOCON_FUNC2); // MOSI1

	Chip_IOCON_PinMux(LPC_IOCON, 0, 6, IOCON_MODE_PULLUP, IOCON_FUNC0);
	Chip_GPIO_SetPinDIROutput(LPC_GPIO, 0, 6); // SSEL1

	Chip_IOCON_PinMux(LPC_IOCON, 2, 13, IOCON_MODE_PULLUP, IOCON_FUNC0);
	Chip_GPIO_SetPinDIROutput(LPC_GPIO, 2, 13); // N_RESET

	/* SSP Init */
	Chip_SSP_Init(LPC_SSP);
	Chip_SSP_SetFormat(LPC_SSP, SSP_BITS_8, SSP_FRAMEFORMAT_SPI, SSP_CLOCK_MODE0);
	Chip_SSP_SetMaster(LPC_SSP, true);
	Chip_SSP_SetBitRate(LPC_SSP, 20000000);
	Chip_SSP_Enable(LPC_SSP);
}
