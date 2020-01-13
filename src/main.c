/*
===============================================================================
 Name        : Modbus_TCP-Firmware.c
 Author      : $(author)
 Version     :
 Copyright   : $(copyright)
 Description : Modbus TCP Implementation for NXP LPC176xx and Wiznet W5500
===============================================================================
*/

/* ------------------------ System includes ------------------------------- */
#include <assert.h>
#include <cr_section_macros.h>
#include <stdio.h>
#include <string.h>

/* ------------------------ Project includes ------------------------------ */
#include "chip.h"

/* ------------------------ Wiznet includes ------------------------------- */
#include "wizchip_conf.h"
#include "socket.h"
#include "loopback.h"

/* ------------------------ Modbus includes ------------------------------- */
#include "mb.h"
#include "mbutils.h"

/* ------------------------ Utilities includes ---------------------------- */
#include "stdutils.h"
#include "uart.h"
#include "ssp_spi.h"
// TODO: insert other definitions and declarations here

/* ------------------------ Defines --------------------------------------- */
/* SSP */
//#define BUFFER_SIZE (0x100)
//#define LPC_SSP LPC_SSP1

/* UART Selection */
//#define UART_SELECTION LPC_UART2
//#define IRQ_SELECTION UART2_IRQn
//#define HANDLER_NAME UART2_IRQHandler

/* UART Tx/Rx ring buffers */
//STATIC RINGBUFF_T txring, rxring;

/* UART Ring buffer sizes */
//#define UART_SRB_SIZE 128
//#define UART_RRB_SIZE 32

/* UART Tx/Rx buffers */
//static uint8_t rxbuff[UART_RRB_SIZE], txbuff[UART_SRB_SIZE];

/*
#define TICKRATE_HZ1 (1000)
#define TICKRATE_HZ2 (1)
volatile uint32_t msTicks;
*/

/* FreeModbus stuff */
#define REG_DISCRETE_START    10001                // Start address of discrete inputs
#define REG_DISCRETE_SIZE     15                    // Number of discrete inputs

#define REG_COILS_START       00001                // Coil start address
#define REG_COILS_SIZE        9                     // Number of coils

#define REG_INPUT_START       30001                // Input register start address
#define REG_INPUT_NREGS       10                    // Number of input registers

#define REG_HOLDING_START     40001                // Holding register start address
#define REG_HOLDING_NREGS     1                     // Number of holding registers

/* Discrete Inputs */
#define DISCRETE_WARN          10001
#define DISCRETE_ALARM         10002
#define DISCRETE_TEMP_WARN     10003
#define DISCRETE_DIG1          10004
#define DISCRETE_DIG2          10005
#define DISCRETE_DIG3          10006
#define DISCRETE_DIG4          10007
#define DISCRETE_DIG5          10008
#define DISCRETE_DIG6          10009
#define DISCRETE_DIG7          10010
#define DISCRETE_DIG8          10011
#define DISCRETE_DIG9          10012
#define DISCRETE_DIG10         10013
#define DISCRETE_DIG11         10014
#define DISCRETE_DIG12         10015

/* Coils */
#define COIL_START             00001
#define COIL_QUICKSTOP         00002
#define COIL_REVERSE           00003
#define COIL_EN_SWASHREG       00004
#define COIL_EN_PRESSREG       00005
#define COIL_DIG1              00006
#define COIL_DIG2              00007
#define COIL_DIG3              00008
#define COIL_DIG4              00009

/* Input Registers */
#define INPUT_REG_PUMPCRNT     30001
#define INPUT_REG_SWASHANGLE   30002
#define INPUT_REG_HIPRESS      30003
#define INPUT_REG_PT100        30004
#define INPUT_REG_ANALOG1      30005
#define INPUT_REG_ANALOG2      30006
#define INPUT_REG_ANALOG3      30007
#define INPUT_REG_ANALOG4      30008
#define INPUT_REG_ANALOG5      30009
#define INPUT_REG_ANALOG6      30010

/* Holding Registers */
#define HOLD_SETPOINT          40001

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

/* WizNet stuff */
wiz_NetInfo gWIZNETINFO = {.mac = {0x9b, 0x52, 0x9d, 0x41, 0xfc, 0x7c}, // MAC address
                           .ip = {192, 168, 1, 31},                     // IP address
                           .sn = {255, 255, 255, 0},                    // Subnet mask
                           .dns = {8, 8, 8, 8},                         // DNS address
                           .gw = {192, 168, 1, 1},                      // Gateway address
                           .dhcp = NETINFO_STATIC};

/* Other */
uint8_t serial_data[5] = {0};

/* Function declarations */
void GPIO_Init(void);
//void UART_Init(void);
//void SSP_Init(void);
void W5500_Init(void);

void data_poll(void);

void TCP_Testing(void);

static void Net_Conf(void);
static void Display_Net_Conf(void);
void wizchip_select(void);
void wizchip_deselect(void);
void wizchip_write(uint8_t wb);
uint8_t wizchip_read(void);

int _write(int iFileHandle, char *pcBuffer, int iLength);
void _delay_ms(uint16_t ms);

int main(void) {

	usRegInputBuf[0] = 64;
	// Read clock settings and update SystemCoreClock variable
    SystemCoreClockUpdate();

    // Initialize GPIO pins for LEDs
    GPIO_Init();

    // Initialize UART
    UART_Init();

    // Initialize SSP in SPI mode
    SSP_Init();

    // Initialize W5500
    W5500_Init();
    _delay_ms(3);

    // Configure net
    Net_Conf();
    _delay_ms(3);

    // Display configuration
    Display_Net_Conf();
    _delay_ms(500);


    // Modbus initialization
    if (eMBTCPInit(MB_TCP_PORT_USE_DEFAULT) != MB_ENOERR) {
    	printf("Modbus TCP initialization failed.\r\n");
	};

    printf("Modbus TCP initialized.\r\n");
    _delay_ms(50);

    eMBEnable();
    // TODO: Code here

    // Force the counter to be placed into memory
    volatile static int i = 0 ;
    // Enter an infinite loop, just incrementing a counter
    while(1) {

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

        i++ ;
    }
    return 0;
}


void W5500_Init(void)
{
	//uint8_t tmp;
	uint8_t memsize[2][8] = {{2, 2, 2, 2, 2, 2, 2, 2}, {2, 2, 2, 2, 2, 2, 2, 2}};

	Chip_GPIO_SetPinState(LPC_GPIO, 0, 6, true); // SSEL


	Chip_GPIO_SetPinState(LPC_GPIO, 2, 13, false);
	_delay_ms(250);
	Chip_GPIO_SetPinState(LPC_GPIO, 2, 13, true);
	_delay_ms(750);


	reg_wizchip_cs_cbfunc(wizchip_select, wizchip_deselect);
	reg_wizchip_spi_cbfunc(wizchip_read, wizchip_write);


	/* wizchip initialization */
	if(ctlwizchip(CW_INIT_WIZCHIP, (void*) memsize) == -1){
		printf("WIZCHIP initialization failed.");
	}

	ctlwizchip(CW_RESET_PHY, 0);
}

//void SSP_Init(void)
//{
//	/* Init Pins */
//	Chip_IOCON_PinMux(LPC_IOCON, 0, 7, IOCON_MODE_INACT, IOCON_FUNC2); // SCK1
//	Chip_IOCON_PinMux(LPC_IOCON, 0, 8, IOCON_MODE_INACT, IOCON_FUNC2); // MISO1
//	Chip_IOCON_PinMux(LPC_IOCON, 0, 9, IOCON_MODE_INACT, IOCON_FUNC2); // MOSI1
//
//	Chip_IOCON_PinMux(LPC_IOCON, 0, 6, IOCON_MODE_PULLUP, IOCON_FUNC0);
//	Chip_GPIO_SetPinDIROutput(LPC_GPIO, 0, 6); // SSEL1
//
//	Chip_IOCON_PinMux(LPC_IOCON, 2, 13, IOCON_MODE_PULLUP, IOCON_FUNC0);
//	Chip_GPIO_SetPinDIROutput(LPC_GPIO, 2, 13); // N_RESET
//
//	/* SSP Init */
//	Chip_SSP_Init(LPC_SSP);
//	Chip_SSP_SetFormat(LPC_SSP, SSP_BITS_8, SSP_FRAMEFORMAT_SPI, SSP_CLOCK_MODE0);
//	Chip_SSP_SetMaster(LPC_SSP, true);
//	Chip_SSP_SetBitRate(LPC_SSP, 20000000);
//	Chip_SSP_Enable(LPC_SSP);
//}

void GPIO_Init(void)
{
    /* Init GPIO */
	Chip_GPIO_Init(LPC_GPIO);
    /* Set all pins as outputs in low state */
//	Chip_GPIO_SetPortDIR(LPC_GPIO, 0, 0xFFFFFFFF, true);
//	Chip_GPIO_SetPortDIR(LPC_GPIO, 1, 0xFFFFFFFF, true);
//	Chip_GPIO_SetPortDIR(LPC_GPIO, 2, 0xFFFFFFFF, true);
//	Chip_GPIO_SetPortDIR(LPC_GPIO, 3, 0xFFFFFFFF, true);
//	Chip_GPIO_SetPortDIR(LPC_GPIO, 4, 0xFFFFFFFF, true);
//	Chip_GPIO_SetPortOutLow(LPC_GPIO, 0, 0xFFFFFFFF);
//	Chip_GPIO_SetPortOutLow(LPC_GPIO, 1, 0xFFFFFFFF);
//	Chip_GPIO_SetPortOutLow(LPC_GPIO, 2, 0xFFFFFFFF);
//	Chip_GPIO_SetPortOutLow(LPC_GPIO, 3, 0xFFFFFFFF);
//	Chip_GPIO_SetPortOutLow(LPC_GPIO, 4, 0xFFFFFFFF);

	Chip_GPIO_SetPinDIROutput(LPC_GPIO, 1, 15);
	Chip_GPIO_SetPinOutHigh(LPC_GPIO, 1, 15);
//	Chip_GPIO_SetPinDIROutput(LPC_GPIO, 1, 20);
//	Chip_GPIO_SetPinDIROutput(LPC_GPIO, 1, 21);
//	Chip_GPIO_SetPinDIROutput(LPC_GPIO, 1, 23);
}

//void UART_Init(void)
//{
//    /* Init Pins */
//    Chip_IOCON_PinMux(LPC_IOCON, 0, 10, IOCON_MODE_INACT, IOCON_FUNC1); // IOCON P0.10 TXD2 (func1), no pull
//    Chip_IOCON_PinMux(LPC_IOCON, 0, 11, IOCON_MODE_INACT, IOCON_FUNC1); // IOCON P0.11 RXD2 (func1), no pull
//
//    /* UART Init */
//    Chip_UART_Init(UART_SELECTION);
//    Chip_UART_SetBaud(UART_SELECTION, 57600);
//    Chip_UART_ConfigData(UART_SELECTION, (UART_LCR_WLEN8 | UART_LCR_SBS_1BIT));
//    Chip_UART_SetupFIFOS(UART_SELECTION, (UART_FCR_FIFO_EN | UART_FCR_TRG_LEV2));
//    Chip_UART_TXEnable(UART_SELECTION);
//
//    /* Init ring buffers */
//    RingBuffer_Init(&rxring, rxbuff, 1, UART_RRB_SIZE);
//    RingBuffer_Init(&txring, txbuff, 1, UART_SRB_SIZE);
//
//    /* Reset and enable FIFOs, FIFO trigger level 3 (14 chars) */
//    Chip_UART_SetupFIFOS(UART_SELECTION, (UART_FCR_FIFO_EN | UART_FCR_RX_RS | UART_FCR_TX_RS | UART_FCR_TRG_LEV1));
//
//    /* Enable receive data and line status interrupt */
//    Chip_UART_IntEnable(UART_SELECTION, (UART_IER_RBRINT | UART_IER_RLSINT));
//
//    /* preemption = 1, sub-priority = 1 */
//    NVIC_SetPriority(IRQ_SELECTION, 1);
//    NVIC_EnableIRQ(IRQ_SELECTION);
//}

static void Net_Conf(void)
{
	/* wizchip netconf */
	ctlnetwork(CN_SET_NETINFO, (void*) &gWIZNETINFO);
}

static void Display_Net_Conf(void)
{
	uint8_t tmpstr[6] = {0,};
	wiz_NetInfo gWIZNETINFO;

	ctlnetwork(CN_GET_NETINFO, (void*) &gWIZNETINFO);

	// Display Network Information
	ctlwizchip(CW_GET_ID,(void*)tmpstr);

	if (gWIZNETINFO.dhcp == NETINFO_DHCP) {
		printf("\r\n===== %s NET CONF : DHCP =====\r\n",(char*)tmpstr);
	}
	else {
		printf("\r\n===== %s NET CONF : Static =====\r\n",(char*)tmpstr);
	}
	printf(" MAC : %02X:%02X:%02X:%02X:%02X:%02X\r\n", gWIZNETINFO.mac[0], gWIZNETINFO.mac[1], gWIZNETINFO.mac[2], gWIZNETINFO.mac[3], gWIZNETINFO.mac[4], gWIZNETINFO.mac[5]);
	printf(" IP : %d.%d.%d.%d\r\n", gWIZNETINFO.ip[0], gWIZNETINFO.ip[1], gWIZNETINFO.ip[2], gWIZNETINFO.ip[3]);
	printf(" GW : %d.%d.%d.%d\r\n", gWIZNETINFO.gw[0], gWIZNETINFO.gw[1], gWIZNETINFO.gw[2], gWIZNETINFO.gw[3]);
	printf(" SN : %d.%d.%d.%d\r\n", gWIZNETINFO.sn[0], gWIZNETINFO.sn[1], gWIZNETINFO.sn[2], gWIZNETINFO.sn[3]);
	printf("===================================\r\n");
}

void wizchip_select(void)
{
	Chip_GPIO_SetPinState(LPC_GPIO, 0, 6, false); // SSEL
}

void wizchip_deselect()
{
	Chip_GPIO_SetPinState(LPC_GPIO, 0, 6, true); // SSEL
}

void wizchip_write(uint8_t wb)
{
	Chip_SSP_WriteFrames_Blocking(LPC_SSP, &wb, 1);

}

uint8_t wizchip_read(void)
{
	uint8_t rb;

	Chip_SSP_ReadFrames_Blocking(LPC_SSP, &rb, 1);

	return rb;
}


int _write(int iFileHandle, char *pcBuffer, int iLength)
{
	_delay_ms(50);

	int ret;

	ret = Chip_UART_SendRB(UART_SELECTION, &txring, (const uint8_t *) pcBuffer, iLength);

	return ret;
}

void data_poll(void){

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
	uint16_t delay;
	volatile uint32_t i;

	for(delay = ms; delay > 0; delay--){
		for(i = 10000; i > 0; i--);
	}
}

eMBErrorCode eMBRegInputCB(UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs)
{
	eMBErrorCode eStatus = MB_ENOERR;
	int iRegIndex;

    // Query if it is in the register range
    // To avoid warnings, modify to signed integer
	if((usAddress >= REG_INPUT_START)
       && (usAddress + usNRegs <= REG_INPUT_START + REG_INPUT_NREGS)) {
		iRegIndex = (int)(usAddress - usRegInputStart);
		while(usNRegs > 0) {
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

	if((usAddress >= REG_HOLDING_START)
       && (usAddress + usNRegs <= REG_HOLDING_START + REG_HOLDING_NREGS)) {
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

	if((usAddress >= REG_COILS_START) &&
		(usAddress + usNCoils <= REG_COILS_START + REG_COILS_SIZE)) {
		usBitOffset = (unsigned short)(usAddress - REG_COILS_START);
		switch (eMode) {
			case MB_REG_READ:
				while( iNCoils > 0 ) {
					*pucRegBuffer++ = xMBUtilGetBits(ucRegCoilsBuf, usBitOffset,
                                                     (unsigned char)(iNCoils > 8 ? 8 : iNCoils));
					iNCoils -= 8;
					usBitOffset += 8;
				}
				break;

			case MB_REG_WRITE:
				while( iNCoils > 0 ) {
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

	if((usAddress >= REG_DISCRETE_START) &&
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

//void LinkProcessData(void)
//{
//	// input process data (from masters point of view)
//	usRegInputBuf[0] = u32Heartbeat;
//	usRegInputBuf[2] = u16ADCData;
//	usRegInputBuf[4] = u16DataIn;
//
//	// output process data (from masters point of view)
//	u16DataOut = usRegHoldingBuf[ 4];
//	u8LedRed = usRegHoldingBuf[ 8];
//	u8LedGreen = usRegHoldingBuf[12];
//	u8LedBlue = usRegHoldingBuf[16];
//}
