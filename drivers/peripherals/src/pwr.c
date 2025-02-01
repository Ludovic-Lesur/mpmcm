/*
 * pwr.c
 *
 *  Created on: 18 jun. 2023
 *      Author: Ludo
 */

#include "pwr.h"

#include "pwr_reg.h"
#include "rcc_reg.h"
#include "scb_reg.h"

/*** PWR functions ***/

/*******************************************************************/
void PWR_init(void) {
	// Enable power interface clock.
	RCC -> APB1ENR1 |= (0b1 << 28); // PWREN='1'.
	// Unlock back-up registers (DBP bit).
	PWR -> CR1 |= (0b1 << 8);
}

/*******************************************************************/
void PWR_enter_sleep_mode(void) {
	// Enter stop mode.
	SCB -> SCR &= ~(0b1 << 2); // SLEEPDEEP='0'.
	 // Wait For Interrupt core instruction.
	__asm volatile ("wfi");
}

/*******************************************************************/
void PWR_enter_stop_mode_1(void) {
	// Select low power mode.
	PWR -> CR1 &= ~(0b111 << 0); // Reset bits 0-2.
	PWR -> CR1 |= (0b001 << 0); // LPMS='001'.
	// Enter stop mode.
	SCB -> SCR |= (0b1 << 2); // SLEEPDEEP='1'.
	__asm volatile ("wfi"); // Wait For Interrupt core instruction.
}

/*******************************************************************/
void PWR_software_reset(void) {
	// Trigger software reset.
	SCB -> AIRCR = 0x05FA0000 | ((SCB -> AIRCR) & 0x0000FFFF) | (0b1 << 2);
}
