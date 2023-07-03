/*
 * rtc.c
 *
 *  Created on: 25 oct. 2020
 *      Author: Ludo
 */

#include "rtc.h"

#include "exti.h"
#include "nvic.h"
#include "rcc_reg.h"
#include "rtc_reg.h"
#include "types.h"

/*** RTC local macros ***/

#define RTC_INIT_TIMEOUT_COUNT		1000
#define RTC_WAKEUP_TIMER_DELAY_MAX	65536
// RTC wake-up timer period.
// Warning: this value must be lower than the watchdog period = 25s.
#define RTC_WAKEUP_PERIOD_SECONDS	1

/*** RTC local global variables ***/

static volatile uint8_t rtc_wakeup_timer_flag = 0;

/*** RTC local functions ***/

/* RTC INTERRUPT HANDLER.
 * @param:	None.
 * @return:	None.
 */
void __attribute__((optimize("-O0"))) RTC_WKUP_IRQHandler(void) {
	// Wake-up timer interrupt.
	if (((RTC -> SR) & (0b1 << 2)) != 0) {
		// Set local flag.
		if (((RTC -> CR) & (0b1 << 14)) != 0) {
			rtc_wakeup_timer_flag = 1;
		}
		// Clear flags.
		RTC -> SCR |= (0b1 << 2); // WUTF='0'.
		EXTI_clear_flag(EXTI_LINE_RTC_WAKEUP_TIMER);
	}
}

/* ENTER INITIALIZATION MODE TO ENABLE RTC REGISTERS UPDATE.
 * @param:			None.
 * @return status:	Function execution status.
 */
static RTC_status_t __attribute__((optimize("-O0"))) _RTC_enter_initialization_mode(void) {
	// Local variables.
	RTC_status_t status = RTC_SUCCESS;
	uint32_t loop_count = 0;
	// Enter key.
	RTC -> WPR = 0xCA;
	RTC -> WPR = 0x53;
	RTC -> ICSR |= (0b1 << 7); // INIT='1'.
	// Wait for initialization mode.
	while (((RTC -> ICSR) & (0b1 << 6)) == 0) {
		// Wait for INITF='1' or timeout.
		loop_count++;
		if (loop_count > RTC_INIT_TIMEOUT_COUNT) {
			status = RTC_ERROR_INITIALIZATION_MODE;
			break;
		}
	}
	return status;
}

/* EXIT INITIALIZATION MODE TO PROTECT RTC REGISTERS.
 * @param:	None.
 * @return:	None.
 */
static void __attribute__((optimize("-O0"))) _RTC_exit_initialization_mode(void) {
	RTC -> ICSR &= ~(0b1 << 7); // INIT='0'.
}

/* START RTC WAKE-UP TIMER.
 * @param delay_seconds:	Delay in seconds.
 * @return status:			Function execution status.
 */
RTC_status_t _RTC_start_wakeup_timer(void) {
	// Local variables.
	RTC_status_t status = RTC_SUCCESS;
	uint32_t loop_count = 0;
	// Check parameter.
	if (RTC_WAKEUP_PERIOD_SECONDS > RTC_WAKEUP_TIMER_DELAY_MAX) {
		status = RTC_ERROR_WAKEUP_TIMER_DELAY;
		goto errors;
	}
	// Check if timer is not already running.
	if (((RTC -> CR) & (0b1 << 10)) != 0) {
		status = RTC_ERROR_WAKEUP_TIMER_RUNNING;
		goto errors;
	}
	// Enable RTC and register access.
	status = _RTC_enter_initialization_mode();
	if (status != RTC_SUCCESS) goto errors;
	// Disable interrupt.
	RTC -> CR &= ~(0b1 << 14); // WUTE='0'.
	// Poll WUTWF flag before accessing reload register.
	while (((RTC -> ICSR) & (0b1 << 2)) == 0) {
		// Wait for WUTWF='1' or timeout.
		loop_count++;
		if (loop_count > RTC_INIT_TIMEOUT_COUNT) {
			status = RTC_ERROR_WAKEUP_TIMER_REGISTER_ACCESS;
			goto errors;
		}
	}
	// Configure wake-up timer.
	RTC -> WUTR = (RTC_WAKEUP_PERIOD_SECONDS - 1);
	// Clear flags.
	RTC -> SCR |= (0b1 << 2); // WUTF='0'.
	EXTI_clear_flag(EXTI_LINE_RTC_WAKEUP_TIMER);
	// Enable interrupt.
	RTC -> CR |= (0b1 << 14); // WUTIE='1'.
	// Start timer.
	RTC -> CR |= (0b1 << 10); // Enable wake-up timer.
errors:
	_RTC_exit_initialization_mode();
	return status;
}

/*** RTC functions ***/

/* RESET RTC PERIPHERAL.
 * @param:	None.
 * @return:	None.
 */
void __attribute__((optimize("-O0"))) RTC_reset(void) {
	// Local variables.
	uint8_t j = 0;
	// Reset RTC peripheral.
	RCC -> BDCR |= (0b1 << 16); // BDRST='1'.
	for (j=0 ; j<100 ; j++);
	RCC -> BDCR &= ~(0b1 << 16); // BDRST='0'.
}

/* INIT HARDWARE RTC PERIPHERAL.
 * @param:			None.
 * @return status:	Function execution status.
 */
RTC_status_t __attribute__((optimize("-O0"))) RTC_init(void) {
	// Local variables.
	RTC_status_t status = RTC_SUCCESS;
	// Use LSE.
	RCC -> BDCR |= (0b01 << 8); // RTCSEL='01'.
	// Enable RTC and register access.
	RCC -> BDCR |= (0b1 << 15); // RTCEN='1'.
	// Switch to LSI if RTC failed to enter initialization mode.
	status = _RTC_enter_initialization_mode();
	if (status != RTC_SUCCESS) {
		// Try using LSI.
		RTC_reset();
		RCC -> BDCR &= ~(0b11 << 8); // Reset bits.
		RCC -> BDCR |= (0b10 << 16); // RTCSEL='10'.
		RCC -> BDCR |= (0b1 << 15); // RTCEN='1'.
		status = _RTC_enter_initialization_mode();
		if (status != RTC_SUCCESS) goto errors;
	}
	// Compute prescaler for 32.768kHz quartz.
	RTC -> PRER = (127 << 16) | (255 << 0);
	// Force registers reset.
	RTC -> CR = 0;
	RTC -> ALRMAR = 0;
	RTC -> ALRMBR = 0;
	// Bypass shadow registers.
	RTC -> CR |= (0b1 << 5); // BYPSHAD='1'.
	// Configure wake-up timer.
	RTC -> CR |= (0b100 << 0); // Wake-up timer clocked by RTC clock (1Hz).
	_RTC_exit_initialization_mode();
	// Configure EXTI line.
	EXTI_configure_line(EXTI_LINE_RTC_WAKEUP_TIMER, EXTI_TRIGGER_RISING_EDGE);
	// Disable interrupt and clear all flags.
	RTC -> CR &= ~(0b1 << 14);
	RTC -> SCR |= 0x0000003F;
	EXTI_clear_flag(EXTI_LINE_RTC_WAKEUP_TIMER);
	// Set interrupt priority.
	NVIC_set_priority(NVIC_INTERRUPT_RTC_WKUP, NVIC_PRIORITY_RTC_WKUP);
	NVIC_enable_interrupt(NVIC_INTERRUPT_RTC_WKUP);
	// Start wakeup timer.
	status = _RTC_start_wakeup_timer();
errors:
	return status;
}

/* RETURN THE CURRENT ALARM INTERRUPT STATUS.
 * @param:	None.
 * @return:	1 if the RTC interrupt occured, 0 otherwise.
 */
volatile uint8_t RTC_get_wakeup_timer_flag(void) {
	return rtc_wakeup_timer_flag;
}

/* CLEAR ALARM A INTERRUPT FLAG.
 * @param:	None.
 * @return:	None.
 */
void RTC_clear_wakeup_timer_flag(void) {
	// Clear flag.
	RTC -> SCR |= (0b1 << 2); // WUTF='0'.
	EXTI_clear_flag(EXTI_LINE_RTC_WAKEUP_TIMER);
	rtc_wakeup_timer_flag = 0;
}
