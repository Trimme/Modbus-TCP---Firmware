/*
===============================================================================
 Name        : main.h
 Author      : $(author)
 Version     :
 Copyright   : $(copyright)
 Description : Modbus TCP Implementation for NXP LPC176xx and Wiznet W5500
===============================================================================
*/


/* ------------------------ Function Declarations ------------------------- */
void GPIO_Init(void);
int _write(int iFileHandle, char *pcBuffer, int iLength);
void _delay_ms(uint16_t ms);
void data_poll(void);
void Error_Handler(void);
