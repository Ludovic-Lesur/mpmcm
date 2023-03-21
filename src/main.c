/*
 * main.c
 *
 *  Created on: 6 mar. 2023
 *      Author: Ludo
 */

#include "gpio.h"
#include "mapping.h"
#include "types.h"

/*** MAIN function ***/

/* MAIN FUNCTION.
 * @param:	None.
 * @return:	None.
 */
int main(void) {
	// Local variables.
	uint32_t i = 0;
	// Init GPIOs.
	GPIO_init();
	GPIO_configure(&GPIO_LED, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	// Blink LED.
	while (1) {
		GPIO_toggle(&GPIO_LED);
		for (i=0 ; i<100000 ; i++);
	}
	return 0;
}

