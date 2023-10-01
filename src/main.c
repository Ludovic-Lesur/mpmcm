/*
 * main.c
 *
 *  Created on: 6 mar. 2023
 *      Author: Ludo
 */

// Peripherals.
#include "adc.h"
#include "error.h"
#include "exti.h"
#include "gpio.h"
#include "iwdg.h"
#include "lptim.h"
#include "lpuart.h"
#include "mapping.h"
#include "nvic.h"
#include "pwr.h"
#include "rcc.h"
#include "rtc.h"
#include "tim.h"
#include "types.h"
// Components.
#include "measure.h"
// Applicative.
#include "at_bus.h"
#include "mode.h"

/*** MAIN local functions ***/

/*******************************************************************/
static void _MPMCM_init_hw(void) {
	// Local variables.
	MEASURE_status_t measure_status = MEASURE_SUCCESS;
	RCC_status_t rcc_status = RCC_SUCCESS;
	RTC_status_t rtc_status = RTC_SUCCESS;
#ifndef DEBUG
	IWDG_status_t iwdg_status = IWDG_SUCCESS;
#endif
	// Init error stack
	ERROR_stack_init();
	// Init memory.
	NVIC_init();
	// Init power module and clock tree.
	PWR_init();
	RCC_init();
	// Init GPIOs.
	GPIO_init();
	EXTI_init();
#ifndef DEBUG
	// Start independent watchdog.
	iwdg_status = IWDG_init();
	IWDG_stack_error();
	IWDG_reload();
#endif
	// High speed oscillator.
	rcc_status = RCC_switch_to_pll();
	RCC_stack_error();
	// Init RTC.
	rtc_status = RTC_init();
	RTC_stack_error();
	// Init delay timer.
	LPTIM1_init();
	// Init components.
	POWER_init();
	measure_status = MEASURE_init();
	MEASURE_stack_error();
	// Init AT BUS interface.
	AT_BUS_init();
}

/*** MAIN function ***/

/*******************************************************************/
int main(void) {
	// Init board.
	_MPMCM_init_hw();
	// Local variables.
	MEASURE_status_t measure_status = MEASURE_SUCCESS;
	// Main loop.
	while (1) {
		// Enter sleep mode.
		IWDG_reload();
		PWR_enter_sleep_mode();
		IWDG_reload();
		// Process measure.
		measure_status = MEASURE_process();
		MEASURE_stack_error();
		// Check RTC flag.
		if (RTC_get_wakeup_timer_flag() != 0) {
			// Clear flag.
			RTC_clear_wakeup_timer_flag();
			// Call measure tick.
			measure_status = MEASURE_tick();
			MEASURE_stack_error();
#ifdef HIGH_SPEED_LOG
			AT_BUS_high_speed_log();
#endif
		}
		// Process AT bus link.
		AT_BUS_task();
	}
	return 0;
}

