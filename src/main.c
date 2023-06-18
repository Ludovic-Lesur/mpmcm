/*
 * main.c
 *
 *  Created on: 6 mar. 2023
 *      Author: Ludo
 */

#include "adc.h"
#include "error.h"
#include "exti.h"
#include "gpio.h"
#include "iwdg.h"
#include "lptim.h"
#include "mapping.h"
#include "mode.h"
#include "nvic.h"
#include "pwr.h"
#include "rcc.h"
#include "tim.h"
#include "types.h"

/*** MAIN local functions ***/

static void _MPMCM_init_hw(void) {
	// Local variables.
	ADC_status_t adc_status = ADC_SUCCESS;
#ifndef DEBUG
	IWDG_status_t iwdg_status = IWDG_SUCCESS;
#endif
	// Init error stack
	ERROR_stack_init();
	// Init memory.
	NVIC_init();
	// Init GPIOs.
	GPIO_init();
	EXTI_init();
	// Init clock and power modules.
	RCC_init();
	PWR_init();
	// Start oscillators.
	RCC_enable_lsi();
	RCC_enable_lse();
	// Start independent watchdog.
#ifndef DEBUG
	iwdg_status = IWDG_init();
	IWDG_error_check();
#endif
	IWDG_reload();
	// Init peripherals.
	LPTIM1_init();
	TIM6_init();
	adc_status = ADC_init();
	ADC_error_check();
}

/*** MAIN function ***/

/* MAIN FUNCTION.
 * @param:	None.
 * @return:	None.
 */
int main(void) {
	// Init board.
	_MPMCM_init_hw();
	// Local variables.
	ADC_status_t adc_status = ADC_SUCCESS;
	// Init LED.
	GPIO_configure(&GPIO_LED_RED, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_configure(&GPIO_LED_GREEN, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_configure(&GPIO_LED_BLUE, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_write(&GPIO_LED_RED, 1);
	GPIO_write(&GPIO_LED_GREEN, 1);
	GPIO_write(&GPIO_LED_BLUE, 1);
	// Start ADC.
	adc_status = ADC_start();
	ADC_error_check();
	// Blink LED.
	while (1) {
		GPIO_toggle(&GPIO_LED_BLUE);
		LPTIM1_delay_milliseconds(1000, LPTIM_DELAY_MODE_ACTIVE);
		IWDG_reload();
	}
	return 0;
}

