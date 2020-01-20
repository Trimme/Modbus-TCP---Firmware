/*
===============================================================================
 Name        : modbus_tcp.c
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

/* ------------------------ Modbus Includes ------------------------------- */
#include "mb.h"
#include "mbutils.h"

/* ------------------------ Peripheral Includes --------------------------- */
#include "modbus_tcp.h"

/* ------------------------ Function Definitions--------------------------- */

eMBErrorCode eMBRegInputCB(UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs)
{
	eMBErrorCode eStatus = MB_ENOERR;
	int iRegIndex;

    // Query if address is in the register range
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

	// Query if address is in the register range
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

	// Query if address is in the register range
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

	// Query if address is in the register range
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
