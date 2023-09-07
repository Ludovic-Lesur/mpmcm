/*
 * lptim.c
 *
 *  Created on: 18 jun. 2023
 *      Author: Ludo
 */

#include "lptim.h"

#include "exti.h"
#include "iwdg.h"
#include "lptim_reg.h"
#include "nvic.h"
#include "pwr.h"
#include "rcc.h"
#include "rcc_reg.h"
#include "types.h"

/*** LPTIM local macros ***/

#define LPTIM_TIMEOUT_COUNT			1000000

#define LPTIM_ARR_MAX_VALUE			0xFFFF

#define LPTIM_CLOCK_FREQUENCY_HZ	(RCC_LSE_FREQUENCY_HZ >> 3)

#define LPTIM_DELAY_MS_MIN			2
#define LPTIM_DELAY_MS_MAX			((LPTIM_ARR_MAX_VALUE * 1000) / (LPTIM_CLOCK_FREQUENCY_HZ))

/*** LPTIM local global variables ***/

static volatile uint8_t lptim_wake_up = 0;

/*** LPTIM local functions ***/

/*******************************************************************/
void __attribute__((optimize("-O0"))) LPTIM1_IRQHandler(void) {
	// Check flag.
	if (((LPTIM1 -> ISR) & (0b1 << 1)) != 0) {
		// Set local flag.
		if (((LPTIM1 -> IER) & (0b1 << 1)) != 0) {
			lptim_wake_up = 1;
		}
		// Clear flag.
		LPTIM1 -> ICR |= (0b1 << 1);
	}
	EXTI_clear_flag(EXTI_LINE_LPTIM1);
}

/*** LPTIM functions ***/

/*******************************************************************/
void LPTIM1_init(void) {
	// Select clock source.
	RCC -> CCIPR |= (0b11 << 18); // LPTIMSEL='11' (LSE clock selected).
	// Enable peripheral clock.
	RCC -> APB1ENR1 |= (0b1 << 31); // LPTIM1EN='1'.
	// Configure peripheral.
	LPTIM1 -> CFGR |= (0b011 << 9); // Prescaler = 8.
	// Enable LPTIM EXTI line.
	LPTIM1 -> IER |= (0b1 << 1); // ARRMIE='1'.
	EXTI_configure_line(EXTI_LINE_LPTIM1, EXTI_TRIGGER_RISING_EDGE);
}

/*******************************************************************/
LPTIM_status_t LPTIM1_delay_milliseconds(uint32_t delay_ms, LPTIM_delay_mode_t delay_mode) {
	// Local variables.
	LPTIM_status_t status = LPTIM_SUCCESS;
	uint32_t arr = 0;
	// Check delay.
	if (delay_ms > LPTIM_DELAY_MS_MAX) {
		status = LPTIM_ERROR_DELAY_OVERFLOW;
		goto errors;
	}
	if (delay_ms < LPTIM_DELAY_MS_MIN) {
		status = LPTIM_ERROR_DELAY_UNDERFLOW;
		goto errors;
	}
	// Enable timer.
	LPTIM1 -> CR |= (0b1 << 0); // Enable LPTIM1 (ENABLE='1').
	// Reset counter and flags.
	LPTIM1 -> CNT &= 0xFFFF0000;
	LPTIM1 -> ICR |= (0b1111111 << 0);
	// Compute ARR value.
	arr = (LPTIM1 -> ARR);
	arr &= 0xFFFF0000;
	arr |= ((((delay_ms - 1) * LPTIM_CLOCK_FREQUENCY_HZ) / (1000)) & 0x0000FFFF);
	LPTIM1 -> ARR = arr;
	// Start timer.
	lptim_wake_up = 0;
	LPTIM1 -> CR |= (0b1 << 1); // SNGSTRT='1'.
	// Perform delay with the selected waiting mode.
	switch (delay_mode) {
	case LPTIM_DELAY_MODE_ACTIVE:
		// Active loop.
		while (((LPTIM1 -> ISR) & (0b1 << 1)) == 0) {
			IWDG_reload();
		}
		break;
	case LPTIM_DELAY_MODE_SLEEP:
		// Enable interrupt.
		NVIC_enable_interrupt(NVIC_INTERRUPT_LPTIM1, NVIC_PRIORITY_LPTIM1);
		// Enter sleep mode.
		while (lptim_wake_up == 0) {
			PWR_enter_sleep_mode();
			IWDG_reload();
		}
		// Disable interrupt.
		NVIC_disable_interrupt(NVIC_INTERRUPT_LPTIM1);
		break;
		// Disable interrupt.
		NVIC_disable_interrupt(NVIC_INTERRUPT_LPTIM1);
		break;
	default:
		status = LPTIM_ERROR_DELAY_MODE;
		goto errors;
	}
errors:
	// Clear flag.
	LPTIM1 -> ICR |= (0b1 << 1);
	// Disable timer.
	LPTIM1 -> CR &= ~(0b1 << 0); // Disable LPTIM1 (ENABLE='0').
	return status;
}
