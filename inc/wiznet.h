/*
===============================================================================
 Name        : wiznet.h
 Author      : $(author)
 Version     :
 Copyright   : $(copyright)
 Description : Modbus TCP Implementation for NXP LPC176xx and Wiznet W5500
===============================================================================
*/

/* ------------------------ Function declarations ------------------------- */

void W5500_Init(void);
void wizchip_select(void);
void wizchip_deselect(void);
void wizchip_write(uint8_t wb);
uint8_t wizchip_read(void);
void Net_Conf(void);
void Display_Net_Conf(void);
