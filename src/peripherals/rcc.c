/*
 * rcc.c
 *
 *  Created on: 16 jun. 2023
 *      Author: Ludo
 */

#include "rcc.h"

#include "error.h"
#include "gpio.h"
#include "mapping.h"
#include "nvic.h"
#include "pwr.h"
#include "rcc_reg.h"
#include "types.h"

/*** RCC macros ***/

//#define RCC_MCO
#define RCC_TIMEOUT_COUNT	1000000

/*** RCC local global variables ***/

#ifdef RCC_MCO
static const GPIO_pin_t GPIO_MCO = (GPIO_pin_t) {GPIOA, 0, 8, 0}; // AF0 = MCO.
#endif

/*** RCC local functions ***/

/* RCC INTERRUPT HANDLER.
 * @param:	None.
 * @return:	None.
 */
void RCC_IRQHandler(void) {
	// Clear all flags.
	RCC -> CICR |= (0b11 << 0);
}

/*** RCC functions ***/

/* INIT MCU CLOCK TREE.
 * @param:	None.
 * @return:	Function execution status.
 */
RCC_status_t RCC_init(void) {
	// Local variables.
	RCC_status_t status = RCC_SUCCESS;
	uint32_t loop_count = 0;
	uint8_t hse_ready = 1;
	// Turn TCXO on.
	GPIO_configure(&GPIO_TCXO_POWER_ENABLE, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_write(&GPIO_TCXO_POWER_ENABLE, 1);
#ifdef RCC_MCO
	// Configure MCO to output (SYSCLK / 16).
	GPIO_configure(&GPIO_MCO, GPIO_MODE_ALTERNATE_FUNCTION, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_HIGH, GPIO_PULL_NONE);
	RCC -> CFGR &= 0x80FFFFFF;
	RCC -> CFGR |= (0b100 << 28) | (0b0001 << 24);
#endif
	// No prescaler.
	RCC -> CFGR &= ~(0b111 << 11);
	RCC -> CFGR &= ~(0b111 << 8);
	RCC -> CFGR &= ~(0b1111 << 4);
	// Turn HSE on.
	RCC -> CR |= (0b1 << 18); // External clock mode.
	RCC -> CR |= (0b1 << 16);
	// Wait for HSE to be ready.
	while (((RCC -> CR) & (0b1 << 17)) == 0) {
		// Wait for HSERDY='1' or timeout.
		loop_count++;
		if (loop_count > RCC_TIMEOUT_COUNT) {
			// Store error in stack.
			ERROR_stack_add(ERROR_BASE_RCC + RCC_ERROR_HSE_READY);
			// Turn TCXO off.
			GPIO_write(&GPIO_TCXO_POWER_ENABLE, 0);
			RCC -> CR &= ~(0b1 << 16); // Disable HSE (HSEON='0').
			// Reset flag and exit.
			hse_ready = 0;
			break;
		}
	}
	// Configure PLL.
	// Clock entry = HSE 16MHz.
	// M=2 -> VCO input clock = 8MHz.
	// N=30 -> VCO output clock = 240MHz.
	// R=2 -> SYSCLK = 120MHz.
	// P=30 -> ADCCLK = 8MHz.
	RCC -> PLLCFGR = 0;
	RCC -> PLLCFGR |= ((hse_ready == 0) ? (0b10 << 0) : (0b11 << 0));
	RCC -> PLLCFGR |= (30 << 27) | (0 << 25) | (30 << 8) | (1 << 4);
	RCC -> PLLCFGR |= (0b1 << 24) | (0b1 << 16);
	// Turn PLL on.
	RCC -> CR |= (0b1 << 24);
	// Wait for PLL to be ready.
	loop_count = 0;
	while (((RCC -> CR) & (0b1 << 25)) == 0) {
		// Wait for PLLRDY='1' or timeout.
		if (loop_count > RCC_TIMEOUT_COUNT) {
			// Turn TCXO off.
			GPIO_write(&GPIO_TCXO_POWER_ENABLE, 0);
			RCC -> CR &= ~(0b1 << 16); // Disable HSE (HSEON='0').
			// Exit.
			status = RCC_ERROR_PLL_READY;
			goto errors;
		}
	}
	// Switch SYSCLK.
	RCC -> CFGR |= (0b11 << 0); // Use PLL as system clock (SW='11').
	// Wait for clock switch.
	loop_count = 0;
	while (((RCC -> CFGR) & (0b11 << 2)) != (0b11 << 2)) {
		// Wait for SWS='10' or timeout.
		loop_count++;
		if (loop_count > RCC_TIMEOUT_COUNT) {
			// Turn PLL off.
			RCC -> CR &= ~(0b1 << 24); // Disable PLL (PLLON='0').
			// Turn TCXO off.
			GPIO_write(&GPIO_TCXO_POWER_ENABLE, 0);
			RCC -> CR &= ~(0b1 << 16); // Disable HSE (HSEON='0').
			// Exit.
			status = RCC_ERROR_PLL_SWITCH;
			goto errors;
		}
	}
	// Disable HSI.
	RCC -> CR &= ~(0b1 << 8); // HSION='0'.
errors:
	return status;
}

/* ENABLE INTERNAL LOW SPEED OSCILLATOR (38kHz INTERNAL RC).
 * @param:	None.
 * @return:	None.
 */
void RCC_enable_lsi(void) {
	// Enable LSI.
	RCC -> CSR |= (0b1 << 0); // LSION='1'.
	// Enable interrupt.
	RCC -> CIER |= (0b1 << 0);
	NVIC_enable_interrupt(NVIC_INTERRUPT_RCC);
	// Wait for LSI to be stable.
	while (((RCC -> CSR) & (0b1 << 1)) == 0) {
		__asm volatile ("wfi"); // Wait For Interrupt core instruction.
	}
	NVIC_disable_interrupt(NVIC_INTERRUPT_RCC);
}

/* ENABLE EXTERNAL LOW SPEED OSCILLATOR (32.768kHz QUARTZ).
 * @param:	None.
 * @return:	None.
 */
void RCC_enable_lse(void) {
	// Enable LSE (32.768kHz crystal).
	RCC -> BDCR |= (0b1 << 0); // LSEON='1'.
	// Enable interrupt.
	RCC -> CIER |= (0b1 << 1);
	NVIC_enable_interrupt(NVIC_INTERRUPT_RCC);
	// Wait for LSE to be stable.
	while (((RCC -> BDCR) & (0b1 << 1)) == 0) {
		__asm volatile ("wfi"); // Wait For Interrupt core instruction.
	}
	NVIC_disable_interrupt(NVIC_INTERRUPT_RCC);
}
