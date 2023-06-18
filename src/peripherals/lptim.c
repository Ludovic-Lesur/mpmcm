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
#include "rcc.h"
#include "rcc_reg.h"
#include "types.h"

/*** LPTIM local macros ***/

#define LPTIM_TIMEOUT_COUNT		1000000
#define LPTIM_DELAY_MS_MIN		1
#define LPTIM_DELAY_MS_MAX		55000

/*** LPTIM local global variables ***/

static uint32_t lptim_clock_frequency_hz = 0;
static volatile uint8_t lptim_wake_up = 0;

/*** LPTIM local functions ***/

/* LPTIM INTERRUPT HANDLER.
 * @param:	None.
 * @return:	None.
 */
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

/* WRITE ARR REGISTER.
 * @param arr_value:	ARR register value to write.
 * @return status:		Function execution status.
 */
static LPTIM_status_t _LPTIM1_write_arr(uint32_t arr_value) {
	// Local variables.
	LPTIM_status_t status = LPTIM_SUCCESS;
	uint32_t loop_count = 0;
	// Reset bits.
	LPTIM1 -> ICR |= (0b1 << 4);
	LPTIM1 -> ARR &= 0xFFFF0000;
	while (((LPTIM1 -> ISR) & (0b1 << 4)) == 0) {
		// Wait for ARROK='1' or timeout.
		loop_count++;
		if (loop_count > LPTIM_TIMEOUT_COUNT) {
			status = LPTIM_ERROR_WRITE_ARR;
			goto errors;
		}
	}
	// Write new value.
	LPTIM1 -> ICR |= (0b1 << 4);
	LPTIM1 -> ARR |= arr_value;
	loop_count = 0;
	while (((LPTIM1 -> ISR) & (0b1 << 4)) == 0) {
		// Wait for ARROK='1' or timeout.
		loop_count++;
		if (loop_count > LPTIM_TIMEOUT_COUNT) {
			status = LPTIM_ERROR_WRITE_ARR;
			goto errors;
		}
	}
errors:
	return status;
}

/*** LPTIM functions ***/

/* INIT LPTIM FOR DELAY OPERATION.
 * @param:	None.
 * @return:	None.
 */
void LPTIM1_init(void) {
	// Configure clock source.
	RCC -> CCIPR |= (0b11 << 18); // LPTIMSEL='01' (LSE clock selected).
	lptim_clock_frequency_hz = (RCC_LSE_FREQUENCY_HZ >> 5);
	// Enable peripheral clock.
	RCC -> APB1ENR1 |= (0b1 << 31); // LPTIM1EN='1'.
	// Configure peripheral.
	LPTIM1 -> CFGR |= (0b101 << 9); // Prescaler = 32.
	// Enable LPTIM EXTI line.
	LPTIM1 -> IER |= (0b1 << 1); // ARRMIE='1'.
	EXTI_configure_line(EXTI_LINE_LPTIM1, EXTI_TRIGGER_RISING_EDGE);
	// Set interrupt priority.
	NVIC_set_priority(NVIC_INTERRUPT_LPTIM1, 1);
}

/* DELAY FUNCTION.
 * @param delay_ms:		Number of milliseconds to wait.
 * @param delay_mode:	Delay waiting mode.
 * @return status:		Function execution status.
 */
LPTIM_status_t LPTIM1_delay_milliseconds(uint32_t delay_ms, LPTIM_delay_mode_t delay_mode) {
	// Local variables.
	LPTIM_status_t status = LPTIM_SUCCESS;
	uint32_t arr = 0;
	// Check delay.
	if ((delay_ms > LPTIM_DELAY_MS_MAX) || (delay_ms > (IWDG_REFRESH_PERIOD_SECONDS * 1000))) {
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
	arr = ((delay_ms * lptim_clock_frequency_hz) / (1000)) & 0x0000FFFF;
	status = _LPTIM1_write_arr(arr);
	if (status != LPTIM_SUCCESS) goto errors;
	// Perform delay with the selected mode.
	switch (delay_mode) {
	case LPTIM_DELAY_MODE_ACTIVE:
		// Start timer.
		LPTIM1 -> CR |= (0b1 << 1); // SNGSTRT='1'.
		// Wait for flag.
		while (((LPTIM1 -> ISR) & (0b1 << 1)) == 0);
		// Clear flag.
		LPTIM1 -> ICR |= (0b1 << 1);
		break;
	case LPTIM_DELAY_MODE_STOP:
		// Enable interrupt.
		NVIC_enable_interrupt(NVIC_INTERRUPT_LPTIM1);
		lptim_wake_up = 0;
		// Start timer.
		LPTIM1 -> CR |= (0b1 << 1); // SNGSTRT='1'.
		// Wait for interrupt.
		while (lptim_wake_up == 0) {
			__asm volatile ("wfi"); // Wait For Interrupt core instruction.
		}
		NVIC_disable_interrupt(NVIC_INTERRUPT_LPTIM1);
		break;
	default:
		status = LPTIM_ERROR_DELAY_MODE;
		break;
	}
errors:
	// Disable timer.
	LPTIM1 -> CR &= ~(0b1 << 0); // Disable LPTIM1 (ENABLE='0').
	return status;
}
