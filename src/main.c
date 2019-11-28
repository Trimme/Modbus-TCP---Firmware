/*
===============================================================================
 Name        : Modbus_TCP-Firmware.c
 Author      : $(author)
 Version     :
 Copyright   : $(copyright)
 Description : main definition
===============================================================================
*/

#include "chip.h"
#include <cr_section_macros.h>

// TODO: insert other include files here

// TODO: insert other definitions and declarations here
#define PULL_UP 00
#define REPEATR 01
#define NO_PULL 10
#define PULL_DN 11

#define FUNC0 00
#define FUNC1 01
#define FUNC2 10
#define FUNC3 11

void _delay_ms(uint16_t ms);

int main(void) {


	// Read clock settings and update SystemCoreClock variable
    SystemCoreClockUpdate();


    // TODO: insert code here

    /* Init GPIO */
    Chip_GPIO_Init(LPC_GPIO);
    Chip_GPIO_SetPinDIROutput(LPC_GPIO, 1, 18);
    Chip_GPIO_SetPinDIROutput(LPC_GPIO, 1, 20);
    Chip_GPIO_SetPinDIROutput(LPC_GPIO, 1, 21);
    Chip_GPIO_SetPinDIROutput(LPC_GPIO, 1, 23);

    /* Init UART */
    Chip_IOCON_PinMux(LPC_IOCON, 0, 10, NO_PULL, FUNC1); // IOCON P0.10 TXD2 (func1), no pull
    Chip_IOCON_PinMux(LPC_IOCON, 0, 11, NO_PULL, FUNC1); // IOCON P0.11 RXD2 (func1), no pull
    Chip_UART_Init(LPC_UART2);
    Chip_UART_SetBaud(LPC_UART2, 115200);


    // Force the counter to be placed into memory
    volatile static int i = 0 ;
    // Enter an infinite loop, just incrementing a counter
    while(1) {

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

        i++ ;
    }
    return 0 ;
}

void _delay_ms(uint16_t ms){

	uint16_t delay;
	volatile uint32_t i;

	for(delay = ms; delay > 0; delay--){
		for(i = 10000; i > 0; i--);
	}

}
