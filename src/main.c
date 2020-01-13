/*
===============================================================================
 Name        : Modbus_TCP-Firmware.c
 Author      : $(author)
 Version     :
 Copyright   : $(copyright)
 Description : Modbus TCP Implementation for NXP LPC176xx and Wiznet W5500
===============================================================================
*/

/* ------------------------ System Includes ------------------------------- */
#include <assert.h>
#include <cr_section_macros.h>
#include <stdio.h>
#include <string.h>

/* ------------------------ Project Includes ------------------------------ */
#include "chip.h"
#include "main.h"

/* ------------------------ Wiznet Includes ------------------------------- */
#include "wizchip_conf.h"
#include "socket.h"
#include "loopback.h"

/* ------------------------ Modbus Includes ------------------------------- */
#include "mb.h"
#include "mbutils.h"

/* ------------------------ Peripheral Includes --------------------------- */
#include "modbus_tcp.h"
#include "ssp_spi.h"
#include "uart.h"
#include "wiznet.h"

/* ------------------------ Private Variables ----------------------------- */

/* Modbus Register Buffers */
static uint8_t ucRegDiscreteBuf[(REG_DISCRETE_SIZE + 7) / 8];
static uint8_t ucRegCoilsBuf[(REG_COILS_SIZE + 7) / 8];
static uint16_t usRegInputBuf[REG_INPUT_NREGS];
static uint16_t usRegHoldingBuf[REG_HOLDING_NREGS];

/* Modbus Register Start Addresses */
//static uint16_t usRegDiscreteStart = REG_DISCRETE_START;
//static uint16_t usRegCoilsStart = REG_COILS_START;
static uint16_t usRegInputStart = REG_INPUT_START;
static uint16_t usRegHoldingStart = REG_HOLDING_START;

/* Data Unit for UART Demo Function */
uint8_t serial_data[5] = {0};


/*
===============================================================================
--------------------------- MAIN PROGRAM --------------------------------------
===============================================================================
*/

int main(void) {

	usRegInputBuf[0] = 64;
    /* Read clock settings and update SystemCoreClock variable */
	SystemCoreClockUpdate();

    /* Initialize GPIO pins for LEDs */
	GPIO_Init();

    /* Initialize UART */
	UART_Init();

    /* Initialize SSP in SPI mode */
	SSP_Init();

    /* Initialize W5500 */
	W5500_Init();
	_delay_ms(3);

    /* Configure Net */
	Net_Conf();
	_delay_ms(3);

    /* Display Net Configuration */
	Display_Net_Conf();
	_delay_ms(500);

    /* Modbus initialization */
	if (eMBTCPInit(MB_TCP_PORT_USE_DEFAULT) != MB_ENOERR) {
		printf("ERROR: Modbus TCP initialization failed (eMBTCPInit)\r\n");
		Error_Handler();
	};

	_delay_ms(50);

	if (eMBEnable() != MB_ENOERR) {
		printf("ERROR: Modbus TCP not enabled (eMBEnable)\r\n");
		Error_Handler();
	};

	while (1) {
    	modbus_tcps(2, 502);
    	data_poll();

    	/*
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
		*/
	}

	return 0;
}

void GPIO_Init(void)
{
    /* Init GPIO */
	Chip_GPIO_Init(LPC_GPIO);

    /* Set all pins as outputs in low state */
	Chip_GPIO_SetPortDIR(LPC_GPIO, 0, 0xFFFFFFFF, true);
	Chip_GPIO_SetPortDIR(LPC_GPIO, 1, 0xFFFFFFFF, true);
	Chip_GPIO_SetPortDIR(LPC_GPIO, 2, 0xFFFFFFFF, true);
	Chip_GPIO_SetPortDIR(LPC_GPIO, 3, 0xFFFFFFFF, true);
	Chip_GPIO_SetPortDIR(LPC_GPIO, 4, 0xFFFFFFFF, true);
	Chip_GPIO_SetPortOutLow(LPC_GPIO, 0, 0xFFFFFFFF);
	Chip_GPIO_SetPortOutLow(LPC_GPIO, 1, 0xFFFFFFFF);
	Chip_GPIO_SetPortOutLow(LPC_GPIO, 2, 0xFFFFFFFF);
	Chip_GPIO_SetPortOutLow(LPC_GPIO, 3, 0xFFFFFFFF);
	Chip_GPIO_SetPortOutLow(LPC_GPIO, 4, 0xFFFFFFFF);

    /* Debug LED */
	Chip_GPIO_SetPinDIROutput(LPC_GPIO, 1, 15);
	Chip_GPIO_SetPinOutLow(LPC_GPIO, 1, 15);

    /* Wiznet Reset */
	Chip_GPIO_SetPinDIROutput(LPC_GPIO, 2, 13);
	Chip_GPIO_SetPinOutHigh(LPC_GPIO, 2, 13);
}


int _write(int iFileHandle, char *pcBuffer, int iLength)
{
	_delay_ms(50);

	int ret;

	ret = Chip_UART_SendRB(UART_SELECTION, &txring, (const uint8_t *) pcBuffer, iLength);

	return ret;
}

void data_poll(void)
{
	static uint16_t addr;
	static uint8_t code;
	static uint16_t data;
	static uint16_t idx;
	static uint16_t bit;

	Chip_UART_ReadRB(UART_SELECTION, &rxring, serial_data, 5);

	if(serial_data[0] != 0 || serial_data[1] != 0){

		addr = ((uint16_t)serial_data[0] << 8) | serial_data[1];

		code = serial_data[2];

		data = ((uint16_t)serial_data[3] << 8 | serial_data[4]);

		//printf("[ %x %x ] [ %x ] [ %x %x ]\r\n", serial_data[0], serial_data[1], serial_data[2], serial_data[3], serial_data[4]);
		printf("Received address: %05d \r\n", addr);
		printf("Received code: %d \r\n", code);
		printf("Received data: %d \r\n", data);


		if((addr >= REG_COILS_START) && (addr <= REG_COILS_START + REG_COILS_SIZE)){

			addr = addr - REG_COILS_START;

			idx = addr / 8;

			bit = addr - (idx * 8);

			switch(code){

				case 0:
					printf("Value of requested Coil: [ %d ] \r\n", xMBUtilGetBits(ucRegCoilsBuf, addr, 1));
					break;

				case 1:
					printf("Use code 10 or 11 to manipulate the value of a Coil. \r\n");
					break;

				case 10:
					ucRegCoilsBuf[idx] &=~ (1 << bit);
					printf("Requested Coil set to 0 \r\n");
					break;

				case 11:
					ucRegCoilsBuf[idx] |= (1 << bit);
					printf("Requested Coil set to 1 \r\n");
					break;

				default:
					printf("Invalid Code. \r\n");
					break;
			}

		}
		else if((addr >= REG_DISCRETE_START) && (addr <= REG_DISCRETE_START + REG_DISCRETE_SIZE)){

			addr = addr - REG_DISCRETE_START;

			idx = addr / 8;

			bit = addr - (idx * 8);

			switch(code){

				case 0:
					printf("Value of requested Discrete Input: [ %d ] \r\n", xMBUtilGetBits(ucRegDiscreteBuf, addr, 1));
					break;

				case 1:
					printf("Use code 10 or 11 to manipulate the value of a Discrete Input. \r\n");
					break;

				case 10:
					ucRegDiscreteBuf[idx] &=~ (1 << bit);
					printf("Requested Discrete Input set to 0 \r\n");
					break;

				case 11:
					ucRegDiscreteBuf[idx] |= (1 << bit);
					printf("Requested Discrete Input set to 1 \r\n");
					break;

				default:
					printf("Invalid Code. \r\n");
					break;
			}

		}
		else if((addr >= REG_INPUT_START) && (addr <= REG_INPUT_START + REG_INPUT_NREGS)){

			addr = addr - REG_INPUT_START;

			switch(code){

				case 0:
					printf("Value in requested Input Register: [ %d ] \r\n", usRegInputBuf[addr]);
					break;

				case 1:
					usRegInputBuf[addr] = data;
					printf("Value put in requested Input Register. \r\n");
					break;

				default:
					printf("Invalid Code. \r\n");
					break;
			}

		}
		else if((addr >= REG_HOLDING_START) && (addr <= REG_HOLDING_START + REG_HOLDING_NREGS)){

			addr = addr - REG_HOLDING_START;

			switch(code){

				case 0:
					printf("Value in requested Holding Register: [ %d ] \r\n", usRegHoldingBuf[addr]);
					break;

				case 1:
					usRegHoldingBuf[addr] = data;
					printf("Value put in requested Holding Register. \r\n");
					break;

				default:
					printf("Invalid Code. \r\n");
					break;
			}

		}
		else{

			printf("Address not in any defined address-space. \r\n");

		}


	}

	_delay_ms(100);

	serial_data[0] = 0;
	serial_data[1] = 0;
	serial_data[2] = 0;
	serial_data[3] = 0;
	serial_data[4] = 0;

}


void _delay_ms(uint16_t ms)
{
	for (uint16_t i = ms; i > 0; i--) {
		for (uint32_t j = 10000; j > 0; j--);
	}
}

//void _delay_ms(uint16_t ms)
//{
//	uint16_t delay;
//	volatile uint32_t i;
//
//	for(delay = ms; delay > 0; delay--){
//		for(i = 10000; i > 0; i--);
//	}
//}

void Error_Handler(void)
{
}

eMBErrorCode eMBRegInputCB(UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs)
{
	eMBErrorCode eStatus = MB_ENOERR;
	int iRegIndex;

    // Query if it is in the register range
	if ((usAddress >= REG_INPUT_START) &&
        (usAddress + usNRegs <= REG_INPUT_START + REG_INPUT_NREGS)) {
		iRegIndex = (int)(usAddress - usRegInputStart);
		while (usNRegs > 0) {
			*pucRegBuffer++ = (unsigned char)(usRegInputBuf[iRegIndex] >> 8);
			*pucRegBuffer++ = (unsigned char)(usRegInputBuf[iRegIndex] & 0xFF);
			iRegIndex++;
			usNRegs--;
		}
	}
	else {
		eStatus = MB_ENOREG;
	}

	return eStatus;
}

eMBErrorCode eMBRegHoldingCB(UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs,
                             eMBRegisterMode eMode)
{
	eMBErrorCode eStatus = MB_ENOERR;
	int iRegIndex;

	if ((usAddress >= REG_HOLDING_START) &&
        (usAddress + usNRegs <= REG_HOLDING_START + REG_HOLDING_NREGS)) {
		iRegIndex = (int)(usAddress - usRegHoldingStart);
		switch (eMode) {
			case MB_REG_READ:
				while(usNRegs > 0) {
					*pucRegBuffer++ = (unsigned char)(usRegHoldingBuf[iRegIndex] >> 8);
					*pucRegBuffer++ = (unsigned char)(usRegHoldingBuf[iRegIndex] & 0xFF);
					iRegIndex++;
					usNRegs--;
				}
				break;

			case MB_REG_WRITE:
				while(usNRegs > 0) {
					usRegHoldingBuf[iRegIndex] = *pucRegBuffer++ << 8;
					usRegHoldingBuf[iRegIndex] |= *pucRegBuffer++;
					iRegIndex++;
					usNRegs--;
				}
				break;
		}
	}
	else {
		eStatus = MB_ENOREG;
	}

	return eStatus;
}

eMBErrorCode eMBRegCoilsCB(UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNCoils,
                           eMBRegisterMode eMode)
{
	eMBErrorCode eStatus = MB_ENOERR;
	short iNCoils = (short)usNCoils;
	unsigned short usBitOffset;

	if ((usAddress >= REG_COILS_START) &&
        (usAddress + usNCoils <= REG_COILS_START + REG_COILS_SIZE)) {
		usBitOffset = (unsigned short)(usAddress - REG_COILS_START);
		switch (eMode) {
			case MB_REG_READ:
				while (iNCoils > 0) {
					*pucRegBuffer++ = xMBUtilGetBits(ucRegCoilsBuf, usBitOffset,
                                                     (unsigned char)(iNCoils > 8 ? 8 : iNCoils));
					iNCoils -= 8;
					usBitOffset += 8;
				}
				break;

			case MB_REG_WRITE:
				while (iNCoils > 0) {
					xMBUtilSetBits(ucRegCoilsBuf, usBitOffset,
                                   (unsigned char)(iNCoils > 8 ? 8 : iNCoils),
                                   *pucRegBuffer++ );
					iNCoils -= 8;
					usBitOffset += 8;
				}
				break;
		}
	}
	else {
		eStatus = MB_ENOREG;
	}

	return eStatus;
}

eMBErrorCode eMBRegDiscreteCB(UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNDiscrete)
{
	eMBErrorCode eStatus = MB_ENOERR;
	short iNDiscrete = ( short )usNDiscrete;
	unsigned short usBitOffset;

	if ((usAddress >= REG_DISCRETE_START) &&
        (usAddress + usNDiscrete <= REG_DISCRETE_START + REG_DISCRETE_SIZE)) {
		usBitOffset = (unsigned short)(usAddress - REG_DISCRETE_START);

		while(iNDiscrete > 0) {
			*pucRegBuffer++ = xMBUtilGetBits(ucRegDiscreteBuf, usBitOffset,
                                             (unsigned char)(iNDiscrete > 8 ? 8 : iNDiscrete));
			iNDiscrete -= 8;
			usBitOffset += 8;
		}
	}
	else {
		eStatus = MB_ENOREG;
	}

	return eStatus;
}
