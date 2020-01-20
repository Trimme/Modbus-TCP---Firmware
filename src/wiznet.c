/*
===============================================================================
 Name        : wiznet.c
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
#include "main.h"

/* ------------------------ Wiznet Includes ------------------------------- */
#include "wizchip_conf.h"
#include "socket.h"
#include "loopback.h"

/* ------------------------ Peripheral Includes --------------------------- */
#include "ssp_spi.h"
#include "wiznet.h"

/* ------------------------ Private Variables ----------------------------- */
wiz_NetInfo gWIZNETINFO = {.mac = {0x9b, 0x52, 0x9d, 0x41, 0xfc, 0x7c}, // MAC address
                           .ip = {192, 168, 1, 31},                     // IP address
                           .sn = {255, 255, 255, 0},                    // Subnet mask
                           .dns = {8, 8, 8, 8},                         // DNS address
                           .gw = {192, 168, 1, 1},                      // Gateway address
                           .dhcp = NETINFO_STATIC};


/* ------------------------ Function Definitions--------------------------- */
void W5500_Init(void)
{
	uint8_t memsize[2][8] = {{2, 2, 2, 2, 2, 2, 2, 2}, {2, 2, 2, 2, 2, 2, 2, 2}};

	Chip_GPIO_SetPinState(LPC_GPIO, 0, 6, true); // SSEL

	Chip_GPIO_SetPinState(LPC_GPIO, 2, 13, false);
	_delay_ms(250);
	Chip_GPIO_SetPinState(LPC_GPIO, 2, 13, true);
	_delay_ms(750);

	reg_wizchip_cs_cbfunc(wizchip_select, wizchip_deselect);
	reg_wizchip_spi_cbfunc(wizchip_read, wizchip_write);


	/* Wizchip Initialization */
	if (ctlwizchip(CW_INIT_WIZCHIP, (void*) memsize) == -1) {
		printf("WIZCHIP initialization failed.");
	}

	ctlwizchip(CW_RESET_PHY, 0);
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
	uint8_t rb = 0;

	Chip_SSP_ReadFrames_Blocking(LPC_SSP, &rb, 1);

	return rb;
}

void Net_Conf(void)
{
	/* Wizchip Netconf */
	ctlnetwork(CN_SET_NETINFO, (void*) &gWIZNETINFO);
}

void Display_Net_Conf(void)
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
