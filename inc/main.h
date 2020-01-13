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
void GPIO_Init(void);
void _delay_ms(uint16_t ms);
int _write(int iFileHandle, char *pcBuffer, int iLength);
void data_poll(void);
void Error_Handler(void);
int main(void);

#endif /* _MAIN_H_ */
