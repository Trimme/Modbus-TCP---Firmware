/*
===============================================================================
 Name        : Modbus_TCP-Firmware.c
 Author      : $(author)
 Version     :
 Copyright   : $(copyright)
 Description : main definition
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
// TODO: insert other definitions and declarations here

/* ------------------------ Defines ----------------------------- */
/* SSP */
#define BUFFER_SIZE (0x100)
#define LPC_SSP LPC_SSP1

/* UART Selection */
#define UART_SELECTION LPC_UART2
#define IRQ_SELECTION UART2_IRQn
#define HANDLER_NAME UART2_IRQHandler

/* UART Tx/Rx ring buffers */
STATIC RINGBUFF_T txring, rxring;

/* UART Ring buffer sizes */
#define UART_SRB_SIZE 128
#define UART_RRB_SIZE 32

/* UART Tx/Rx buffers */
static uint8_t rxbuff[UART_RRB_SIZE], txbuff[UART_SRB_SIZE];

/*
#define TICKRATE_HZ1 (1000)
#define TICKRATE_HZ2 (1)
volatile uint32_t msTicks;
*/

/* FreeModbus stuff */
/* Discrete Inputs */
uint16_t discrete_inputs;
//discrete_dig1
//discrete_dig2
//discrete_dig3
//discrete_dig4
//discrete_dig5
//discrete_dig6
//discrete_dig7
//discrete_dig8
//discrete_dig9
//discrete_dig10
//discrete_dig11
//discrete_dig12
//discrete_warn
//discrete_alarm
//discrete_temp_warn
//
///* Coils */
uint16_t coils;
//coil_start
//coil_quickstop
//coil_reverse
//coil_en_swashreg
//coil_en_pressreg
//coil_dig1
//coil_dig2
//coil_dig3
//coil_dig4
//
///* Input Registers */
//input_reg_pumpcrnt
//input_reg_swashangle
//input_reg_hipress
//input_reg_pt100
//input_reg_analog1
//input_reg_analog2
//input_reg_analog3
//input_reg_analog4
//input_reg_analog5
//input_reg_analog6

/* Holding Registers */
//hold_setpoint


#define REG_DISCRETE_START    0x0000                // Start address of discrete inputs
#define REG_DISCRETE_SIZE     15                    // Number of discrete inputs

#define REG_COILS_START       0x1000                // Coil start address
#define REG_COILS_SIZE        9                     // Number of coils

#define REG_INPUT_START       0x2000                // Input register start address
#define REG_INPUT_NREGS       10                    // Number of input registers

#define REG_HOLDING_START     0x3000                // Holding register start address
#define REG_HOLDING_NREGS     1                     // Number of holding registers

// Input register content
uint16_t usRegInputBuf[REG_INPUT_NREGS] = {0,};
// Input register start address
uint16_t usRegInputStart = REG_INPUT_START;
// Holding register content
uint16_t usRegHoldingBuf[REG_HOLDING_NREGS] = {0,};
// Holding register start address
uint16_t usRegHoldingStart = REG_HOLDING_START;
// Coil status
uint8_t ucRegCoilsBuf[2] = {0,};
// Discrete input status
uint8_t ucRegDiscreteBuf[2] = {0,};

/* WizNet stuff */
wiz_NetInfo gWIZNETINFO = {.mac = {0x9b, 0x52, 0x9d, 0x41, 0xfc, 0x7c}, // MAC address
                           .ip = {192, 168, 1, 31},                     // IP address
                           .sn = {255, 255, 255, 0},                    // Subnet mask
                           .dns = {8, 8, 8, 8},                         // DNS address
                           .gw = {192, 168, 1, 1},                      // Gateway address
                           .dhcp = NETINFO_STATIC};

/* Function declarations */
void GPIO_Init(void);
void UART_Init(void);
void SSP_Init(void);
void W5500_Init(void);
void TCP_Testing(void);

static void Net_Conf(void);
static void Display_Net_Conf(void);
void wizchip_select(void);
void wizchip_deselect(void);
void wizchip_write(uint8_t wb);
uint8_t wizchip_read(void);

int _write(int iFileHandle, char *pcBuffer, int iLength);
void _delay_ms(uint16_t ms);
void HANDLER_NAME(void);

int main(void) {

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

//    TCP_Testing();

    printf("Testing over. Please reset.\r\n");

    if (eMBTCPInit(MB_TCP_PORT_USE_DEFAULT) != MB_ENOERR) {
    	printf("Modbus TCP initilization failed.\r\n");
	};
    eMBEnable();
    // TODO: Code here

    // Force the counter to be placed into memory
    volatile static int i = 0 ;
    // Enter an infinite loop, just incrementing a counter
    while(1) {

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

//void TCP_Testing(void){
//
//	int32_t temp;
//	uint8_t temp2 = 0;
//    uint8_t dest_ip[4] = {192, 168, 1, 32};
//
//    if(socket(3, Sn_MR_TCP, 5000, 0) != 3){
//    	printf("Socket error.\r\n");
//    	return;
//    }
//    else {
//    	printf("Socket 3 opened. TCP. 5000.\r\n");
//    }
//
//    _delay_ms(2000);
//
//    /*
//    int8_t temp = connect(3, dest_ip, 49983);
//    if(temp != SOCK_OK){
//    	printf("Connect error. Return: %d\r\n", temp);
//    }
//    else{
//    	printf("Connected to TCP Server. 49983.\r\n");
//    }
//	*/
//
//    if(listen(3) != SOCK_OK){
//    	printf("Listen error.\r\n");
//    	return;
//    }
//    else {
//    	printf("Listening on TCP port 5000.\r\n");
//    }
//
//    _delay_ms(50);
//
//    while(temp2 != SOCK_ESTABLISHED){
//    	getsockopt(3, SO_STATUS, &temp2);
//    }
//    printf("Connection established.\r\n");
//
//    _delay_ms(50);
//
//    while(1){
//
//        temp = recv(3, ethBuf0, 2048);
//
//        for(int i = 0; i < temp; i++){
//        	printf("%c", ethBuf0[i]);
//        }
//        printf("\r\n");
//
//        _delay_ms(50);
//
//        if(ethBuf0[0] == 'X'){
//
//        	sprintf((char*)ethBuf0, "Shutting down...\r\n");
//        	send(3, ethBuf0, 18);
//        	break;
//        }
//
//        //sprintf((char*)ethBuf0, "Meddelande mottaget\r\n");
//
//        send(3, ethBuf0, temp); // Echo received message
//
//        temp = 0;
//
//    }
//
//    close(3); // Close socket
//}

void W5500_Init(void)
{
	//uint8_t tmp;
	uint8_t memsize[2][8] = {{2, 2, 2, 2, 2, 2, 2, 2}, {2, 2, 2, 2, 2, 2, 2, 2}};

	Chip_GPIO_SetPinState(LPC_GPIO, 0, 18, true); // SSEL


	Chip_GPIO_SetPinState(LPC_GPIO, 0, 1, false);
	_delay_ms(250);
	Chip_GPIO_SetPinState(LPC_GPIO, 0, 1, true);
	_delay_ms(750);


	reg_wizchip_cs_cbfunc(wizchip_select, wizchip_deselect);
	reg_wizchip_spi_cbfunc(wizchip_read, wizchip_write);


	/* wizchip initialization */
	if(ctlwizchip(CW_INIT_WIZCHIP, (void*) memsize) == -1){
		printf("WIZCHIP initialization failed.");
	}

	ctlwizchip(CW_RESET_PHY, 0);
}

void SSP_Init(void)
{
	/* Init Pins */
	Chip_IOCON_PinMux(LPC_IOCON, 0, 7, IOCON_MODE_INACT, IOCON_FUNC2); // SCK1
	Chip_IOCON_PinMux(LPC_IOCON, 0, 8, IOCON_MODE_INACT, IOCON_FUNC2); // MISO1
	Chip_IOCON_PinMux(LPC_IOCON, 0, 9, IOCON_MODE_INACT, IOCON_FUNC2); // MOSI1

	Chip_IOCON_PinMux(LPC_IOCON, 0, 18, IOCON_MODE_PULLUP, IOCON_FUNC0);
	Chip_GPIO_SetPinDIROutput(LPC_GPIO, 0, 18); // SSEL1

	Chip_IOCON_PinMux(LPC_IOCON, 0, 1, IOCON_MODE_PULLUP, IOCON_FUNC0);
	Chip_GPIO_SetPinDIROutput(LPC_GPIO, 0, 1); // N_RESET

	/* SSP Init */
	Chip_SSP_Init(LPC_SSP);
	Chip_SSP_SetFormat(LPC_SSP, SSP_BITS_8, SSP_FRAMEFORMAT_SPI, SSP_CLOCK_MODE0);
	Chip_SSP_SetMaster(LPC_SSP, true);
	Chip_SSP_SetBitRate(LPC_SSP, 20000000);
	Chip_SSP_Enable(LPC_SSP);
}

void GPIO_Init(void)
{
    /* Init GPIO */
    Chip_GPIO_Init(LPC_GPIO);
    Chip_GPIO_SetPinDIROutput(LPC_GPIO, 1, 18);
    Chip_GPIO_SetPinDIROutput(LPC_GPIO, 1, 20);
    Chip_GPIO_SetPinDIROutput(LPC_GPIO, 1, 21);
    Chip_GPIO_SetPinDIROutput(LPC_GPIO, 1, 23);
}

void UART_Init(void)
{
    /* Init Pins */
    Chip_IOCON_PinMux(LPC_IOCON, 0, 10, IOCON_MODE_INACT, IOCON_FUNC1); // IOCON P0.10 TXD2 (func1), no pull
    Chip_IOCON_PinMux(LPC_IOCON, 0, 11, IOCON_MODE_INACT, IOCON_FUNC1); // IOCON P0.11 RXD2 (func1), no pull

    /* UART Init */
    Chip_UART_Init(UART_SELECTION);
    Chip_UART_SetBaud(UART_SELECTION, 57600);
    Chip_UART_ConfigData(UART_SELECTION, (UART_LCR_WLEN8 | UART_LCR_SBS_1BIT));
    Chip_UART_SetupFIFOS(UART_SELECTION, (UART_FCR_FIFO_EN | UART_FCR_TRG_LEV2));
    Chip_UART_TXEnable(UART_SELECTION);

    /* Init ring buffers */
    RingBuffer_Init(&rxring, rxbuff, 1, UART_RRB_SIZE);
    RingBuffer_Init(&txring, txbuff, 1, UART_SRB_SIZE);

    /* Reset and enable FIFOs, FIFO trigger level 3 (14 chars) */
    Chip_UART_SetupFIFOS(UART_SELECTION, (UART_FCR_FIFO_EN | UART_FCR_RX_RS | UART_FCR_TX_RS | UART_FCR_TRG_LEV1));

    /* Enable receive data and line status interrupt */
    Chip_UART_IntEnable(UART_SELECTION, (UART_IER_RBRINT | UART_IER_RLSINT));

    /* preemption = 1, sub-priority = 1 */
    NVIC_SetPriority(IRQ_SELECTION, 1);
    NVIC_EnableIRQ(IRQ_SELECTION);
}

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
	Chip_GPIO_SetPinState(LPC_GPIO, 0, 18, false); // SSEL
}

void wizchip_deselect()
{
	Chip_GPIO_SetPinState(LPC_GPIO, 0, 18, true); // SSEL
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

void HANDLER_NAME(void)
{
	Chip_UART_IRQRBHandler(UART_SELECTION, &rxring, &txring);
}

int _write(int iFileHandle, char *pcBuffer, int iLength)
{
	_delay_ms(50);

	int ret;

	ret = Chip_UART_SendRB(UART_SELECTION, &txring, (const uint8_t *) pcBuffer, iLength);

	return ret;
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
	if(((int16_t) usAddress >= REG_INPUT_START) \
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

	if(((int16_t)usAddress >= REG_HOLDING_START) \
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

	if(((int16_t)usAddress >= REG_COILS_START) &&
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

	if(((int16_t)usAddress >= REG_DISCRETE_START) &&
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
