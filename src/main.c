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
#include "lpuart.h"
#include "mapping.h"
#include "mode.h"
#include "nvic.h"
#include "pwr.h"
#include "rcc.h"
#include "rtc.h"
#include "tim.h"
#include "types.h"
// Applicative.
#include "at_bus.h"
#include "measure.h"

/*** MAIN local functions ***/

static void _MPMCM_init_hw(void) {
	// Local variables.
	MEASURE_status_t measure_status = MEASURE_SUCCESS;
	RTC_status_t rtc_status = RTC_SUCCESS;
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
	// Reset RTC.
	RTC_reset();
	// Start oscillators.
	RCC_enable_lsi();
	RCC_enable_lse();
	// Start independent watchdog.
#ifndef DEBUG
	iwdg_status = IWDG_init();
	IWDG_error_check();
#endif
	IWDG_reload();
	// RTC.
	rtc_status = RTC_init();
	RTC_error_check();
	// Init peripherals.
	LPTIM1_init();
	LPUART1_init(0x1C);
	// Init LBUS layer.
	LBUS_init(0x1C);
	// Init mains measurements block.
	measure_status = MEASURE_init();
	MEASURE_error_check();
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
	MEASURE_status_t measure_status = MEASURE_SUCCESS;
	// Main loop.
	while (1) {
		// Enter sleep mode.
		__asm volatile ("wfi");
		// Process measure.
		measure_status = MEASURE_process();
		MEASURE_error_check();
		// Check RTC flag.
		if (RTC_get_wakeup_timer_flag() != 0) {
			// Clear flag.
			RTC_clear_wakeup_timer_flag();
#ifdef HIGH_SPEED_LOG
			AT_BUS_high_speed_log();
#endif
		}
		// Process AT bus link.
		AT_BUS_task();
		// Reload watchdog.
		IWDG_reload();
	}
	return 0;
}

