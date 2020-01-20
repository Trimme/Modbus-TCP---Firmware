/*
===============================================================================
 Name        : main.h
 Author      : $(author)
 Version     :
 Copyright   : $(copyright)
 Description : Modbus TCP Implementation for NXP LPC176xx and Wiznet W5500
===============================================================================
*/

#ifndef _MAIN_H_
#define	_MAIN_H_

/* ------------------------ Function Declarations ------------------------- */
int main(void);
void GPIO_Init(void);
void data_poll(void);
int _write(int iFileHandle, char *pcBuffer, int iLength);
void _delay_ms(uint16_t ms);
void Error_Handler(void);

#endif /* _MAIN_H_ */
