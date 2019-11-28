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
#include "stdutils.h"

// TODO: insert other include files here
#include "gpio_17xx_40xx.h"

// TODO: insert other definitions and declarations here
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
