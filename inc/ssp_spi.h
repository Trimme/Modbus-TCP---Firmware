/*
===============================================================================
 Name        : ssp_spi.h
 Author      : $(author)
 Version     :
 Copyright   : $(copyright)
 Description : Modbus TCP Implementation for NXP LPC176xx and Wiznet W5500
===============================================================================
*/

/* ------------------------ Defines --------------------------------------- */

#define BUFFER_SIZE (0x100)
#define LPC_SSP LPC_SSP1

/* ------------------------ Function declarations ------------------------- */
void SSP_Init(void);
