/*
 * main.c
 *
 *  Created on: 6 mar. 2023
 *      Author: Ludo
 */

#include "error.h"
#include "gpio.h"
#include "mapping.h"
#include "rcc.h"
#include "types.h"

/*** MAIN function ***/

/* MAIN FUNCTION.
 * @param:	None.
 * @return:	None.
 */
int main(void) {
	// Local variables.
	RCC_status_t rcc_status = RCC_SUCCESS;
	uint32_t i = 0;
	// Init GPIOs.
	GPIO_init();
	// Init clock.
	rcc_status = RCC_init();
	RCC_error_check();
	// Blink LED.
	GPIO_configure(&GPIO_LED_RED, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	while (1) {
		GPIO_toggle(&GPIO_LED_RED);
		for (i=0 ; i<1000000 ; i++);
	}
	return 0;
}

