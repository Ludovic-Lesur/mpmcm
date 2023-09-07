/*
 * rcc.c
 *
 *  Created on: 16 jun. 2023
 *      Author: Ludo
 */

#include "rcc.h"

#include "error.h"
#include "flash.h"
#include "gpio.h"
#include "mapping.h"
#include "nvic.h"
#include "pwr.h"
#include "rcc_reg.h"
#include "types.h"

/*** RCC local macros ***/

//#define RCC_MCO
#define RCC_TIMEOUT_COUNT	1000000

/*** RCC local functions ***/

/*******************************************************************/
void __attribute__((optimize("-O0"))) RCC_IRQHandler(void) {
	// Clear all flags.
	RCC -> CICR |= (0b11 << 0);
}

/*******************************************************************/
void _RCC_enable_lsi(void) {
	// Enable LSI.
	RCC -> CSR |= (0b1 << 0); // LSION='1'.
	// Enable interrupt.
	RCC -> CIER |= (0b1 << 0);
	NVIC_enable_interrupt(NVIC_INTERRUPT_RCC, NVIC_PRIORITY_RCC);
	// Wait for LSI to be stable.
	while (((RCC -> CSR) & (0b1 << 1)) == 0) {
		PWR_enter_sleep_mode();
	}
	NVIC_disable_interrupt(NVIC_INTERRUPT_RCC);
}

/*******************************************************************/
void _RCC_enable_lse(void) {
	// Enable LSE (32.768kHz crystal).
	RCC -> BDCR |= (0b1 << 0); // LSEON='1'.
	// Enable interrupt.
	RCC -> CIER |= (0b1 << 1);
	NVIC_enable_interrupt(NVIC_INTERRUPT_RCC, NVIC_PRIORITY_RCC);
	// Wait for LSE to be stable.
	while (((RCC -> BDCR) & (0b1 << 1)) == 0) {
		PWR_enter_sleep_mode();
	}
	NVIC_disable_interrupt(NVIC_INTERRUPT_RCC);
}

/*** RCC functions ***/

/*******************************************************************/
void __attribute__((optimize("-O0"))) RCC_init(void) {
	// Local variables.
	uint8_t i = 0;
	// Reset backup domain.
	RCC -> BDCR |= (0b1 << 16); // BDRST='1'.
	for (i=0 ; i<100 ; i++);
	RCC -> BDCR &= ~(0b1 << 16); // BDRST='0'.
	// Enable low speed oscillators.
	_RCC_enable_lsi();
	_RCC_enable_lse();
}

/*******************************************************************/
RCC_status_t RCC_switch_to_pll(void) {
	// Local variables.
	RCC_status_t status = RCC_SUCCESS;
	FLASH_status_t flash_status = FLASH_SUCCESS;
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
	// Clock entry = HSE or HSI 16MHz.
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
	// Update FLASH latency.
	flash_status = FLASH_set_latency(4);
	FLASH_exit_error(RCC_ERROR_BASE_FLASH);
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
