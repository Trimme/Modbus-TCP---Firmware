/*
===============================================================================
 Name        : Modbus_TCP-Firmware
 Author      : Simon Liljelind (s.liljelind@gmail.com)
               David Skånehult (david.skanehult@gmail.com)

 Version     : 1.0
 Copyright   : $(copyright)
 Description : Modbus TCP Implementation for NXP LPC176xx and Wiznet W5500.

               Uses the following libraries:
               NXP LPCOpen              http://www.nxp.com
               FreeMODBUS               https://github.com/cwalter-at/freemodbus
               Wiznet ioLibrary Driver  https://github.com/Wiznet/ioLibrary_Driver

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

/* ------------------------ Public Variables ------------------------------ */

/* Modbus Register Buffers */
uint8_t ucRegDiscreteBuf[(REG_DISCRETE_SIZE + 7) / 8] = {0};
uint8_t ucRegCoilsBuf[(REG_COILS_SIZE + 7) / 8] = {0};
uint16_t usRegInputBuf[REG_INPUT_NREGS] = {0};
uint16_t usRegHoldingBuf[REG_HOLDING_NREGS] = {0};

/* Modbus Register Start Addresses */
uint16_t usRegInputStart = REG_INPUT_START;
uint16_t usRegHoldingStart = REG_HOLDING_START;

/* UART Tx/Rx Ring Buffers */
RINGBUFF_T rxring;
RINGBUFF_T txring;

/* ------------------------ Private Variables ----------------------------- */

/* Data Unit for UART Demo Function */
uint8_t serial_data[5] = {0};

/*
===============================================================================
--------------------------- MAIN PROGRAM --------------------------------------
===============================================================================
*/

int main(void) {

    /* Read clock settings and update SystemCoreClock variable */
	SystemCoreClockUpdate();

    /* Initialize GPIO pins */
	GPIO_Init();

    /* Initialize UART */
	UART_Init();

    /* Initialize SSP in SPI mode */
	SSP_Init();

    /* Initialize W5500 */
	W5500_Init();

    /* Configure Net */
	Net_Conf();
	_delay_ms(5000);

	/* Display Net Configuration */
	Display_Net_Conf();

    /* Initialize Modbus TCP */
	if (eMBTCPInit(MB_TCP_PORT_USE_DEFAULT) != MB_ENOERR) {
		printf("ERROR: Modbus TCP initialization failed (eMBTCPInit)\r\n");
		Error_Handler();
	};

	if (eMBEnable() != MB_ENOERR) {
		printf("ERROR: Modbus TCP not enabled (eMBEnable)\r\n");
		Error_Handler();
	};

	while (1) {

		modbus_tcps(2, 502);
//		data_poll();

	}

	return 0;
}

void GPIO_Init(void)
{
    /* Init GPIO */
	Chip_GPIO_Init(LPC_GPIO);

    /* Set pins as outputs in low state (Except UART and SPI)*/
	Chip_GPIO_SetPortDIR(LPC_GPIO, 0, 0xF81FFFFF, true);
	Chip_GPIO_SetPortDIR(LPC_GPIO, 1, 0xFFFFFFFF, true);
	Chip_GPIO_SetPortDIR(LPC_GPIO, 2, 0xFFFFFFFF, true);
	Chip_GPIO_SetPortDIR(LPC_GPIO, 3, 0xFFFFFFFF, true);
	Chip_GPIO_SetPortDIR(LPC_GPIO, 4, 0xFFFFFFFF, true);
	Chip_GPIO_SetPortOutLow(LPC_GPIO, 0, 0xF81FFFFF);
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

	/* Clock Out Enable */
//	Chip_IOCON_PinMux(LPC_IOCON, 1, 27, IOCON_MODE_INACT, IOCON_FUNC1); // CLKOUT
//	Chip_Clock_EnableCLKOUT();
}

void data_poll(void)
{
	static uint16_t addr = 0;
	static uint8_t code = 0;
	static uint16_t data = 0;
	static uint16_t idx = 0;
	static uint16_t bit = 0;

	Chip_UART_ReadRB(UART_SELECTION, &rxring, serial_data, 5);

	if (serial_data[0] != 0 || serial_data[1] != 0) {
		addr = ((uint16_t)serial_data[0] << 8) | serial_data[1];
		code = serial_data[2];
		data = ((uint16_t)serial_data[3] << 8 | serial_data[4]);

		printf("Received address: %05d \r\n", addr);
		printf("Received code: %d \r\n", code);
		printf("Received data: %d \r\n", data);

		if ((addr >= REG_COILS_START) && (addr <= REG_COILS_START + REG_COILS_SIZE)) {
			addr = addr - REG_COILS_START;
			idx = addr / 8;
			bit = addr - (idx * 8);

			switch(code) {
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
		else if ((addr >= REG_DISCRETE_START) && (addr <= REG_DISCRETE_START + REG_DISCRETE_SIZE)) {

			addr = addr - REG_DISCRETE_START;
			idx = addr / 8;
			bit = addr - (idx * 8);

			switch(code) {
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
		else if ((addr >= REG_INPUT_START) && (addr <= REG_INPUT_START + REG_INPUT_NREGS)) {

			addr = addr - REG_INPUT_START;

			switch(code) {
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
		else if ((addr >= REG_HOLDING_START) && (addr <= REG_HOLDING_START + REG_HOLDING_NREGS)) {

			addr = addr - REG_HOLDING_START;

			switch(code) {
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
		else {
			printf("Address not in any defined address-space. \r\n");
		}
	}

//	_delay_ms(100);

	serial_data[0] = 0;
	serial_data[1] = 0;
	serial_data[2] = 0;
	serial_data[3] = 0;
	serial_data[4] = 0;
}

int _write(int iFileHandle, char *pcBuffer, int iLength)
{
	int ret = 0;

	ret = Chip_UART_SendRB(UART_SELECTION, &txring, (const uint8_t *) pcBuffer, iLength);
	_delay_ms(50);

	return ret;
}

void _delay_ms(uint16_t ms)
{
	for (uint16_t i = ms; i > 0; i--) {
		for (volatile uint32_t j = 11875; j > 0; j--);
	}
}

void Error_Handler(void)
{
}
