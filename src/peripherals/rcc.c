/*
 * rcc.c
 *
 *  Created on: 16 jun. 2023
 *      Author: Ludo
 */

#include "rcc.h"

#include "error.h"
#include "gpio.h"
#include "flash.h"
#include "mapping.h"
#include "nvic.h"
#include "power.h"
#include "pwr.h"
#include "rcc_reg.h"
#include "tim.h"
#include "types.h"

/*** RCC local macros ***/

#define RCC_TIMEOUT_COUNT				1000000

#define RCC_LSI_FREQUENCY_DEFAULT_HZ	38000
#define RCC_LSI_FREQUENCY_MIN_HZ		26000
#define RCC_LSI_FREQUENCY_MAX_HZ		56000

#define RCC_HSI_FREQUENCY_DEFAULT_HZ	16000000
#define RCC_HSI_FREQUENCY_MIN_HZ		15040000
#define RCC_HSI_FREQUENCY_MAX_HZ		16960000

#define RCC_HSE_FREQUENCY_HZ			16000000

#define RCC_PLL_FREQUENCY_DEFAULT_HZ	128000000

/*** RCC local structures ***/

/*******************************************************************/
typedef struct {
	RCC_clock_t sysclk_source;
	RCC_clock_t pll_source;
	uint32_t clock_frequency[RCC_CLOCK_LAST];
} RCC_context_t;

/*** RCC local global variables ***/

static RCC_context_t rcc_ctx;

/*** RCC local functions ***/

/*******************************************************************/
void __attribute__((optimize("-O0"))) RCC_IRQHandler(void) {
	// Clear flags.
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
	// Init context.
	rcc_ctx.clock_frequency[RCC_CLOCK_LSI] = RCC_LSI_FREQUENCY_DEFAULT_HZ;
	rcc_ctx.clock_frequency[RCC_CLOCK_LSE] = RCC_LSE_FREQUENCY_HZ;
	rcc_ctx.clock_frequency[RCC_CLOCK_HSI] = RCC_HSI_FREQUENCY_DEFAULT_HZ;
	rcc_ctx.clock_frequency[RCC_CLOCK_HSE] = RCC_HSE_FREQUENCY_HZ;
	rcc_ctx.clock_frequency[RCC_CLOCK_PLL] = RCC_PLL_FREQUENCY_DEFAULT_HZ;
	// Update system clock.
	rcc_ctx.sysclk_source = RCC_CLOCK_HSI;
	rcc_ctx.pll_source = RCC_CLOCK_HSI;
	rcc_ctx.clock_frequency[RCC_CLOCK_SYSTEM] = rcc_ctx.clock_frequency[rcc_ctx.sysclk_source];
	// Reset backup domain.
	RCC -> BDCR |= (0b1 << 16); // BDRST='1'.
	for (i=0 ; i<100 ; i++);
	RCC -> BDCR &= ~(0b1 << 16); // BDRST='0'.
	// Enable low speed oscillators.
	_RCC_enable_lsi();
	_RCC_enable_lse();
}

/*******************************************************************/
RCC_status_t RCC_switch_to_hsi(void) {
	// Local variables.
	RCC_status_t status = RCC_SUCCESS;
	FLASH_status_t flash_status = FLASH_SUCCESS;
	POWER_status_t power_status = POWER_SUCCESS;
	uint32_t reg_cfgr = 0;
	uint32_t loop_count = 0;
	// Check current clock source.
	if (rcc_ctx.sysclk_source == RCC_CLOCK_HSI) goto errors;
	// Enable HSI.
	RCC -> CR |= (0b1 << 8); // Enable HSI (HSI16ON='1').
	// Wait for HSI to be stable.
	while (((RCC -> CR) & (0b1 << 10)) == 0) {
		// Wait for HSIRDYF='1' or timeout.
		loop_count++;
		if (loop_count > RCC_TIMEOUT_COUNT) {
			status = RCC_ERROR_HSI_READY;
			goto errors;
		}
	}
	// Switch SYSCLK.
	reg_cfgr = (RCC -> CFGR);
	reg_cfgr &= ~(0b11 << 0); // Reset bits 0-1.
	reg_cfgr |= (0b01 << 0); // Use HSI as system clock (SW='01').
	RCC -> CFGR = reg_cfgr;
	// Wait for clock switch.
	loop_count = 0;
	while (((RCC -> CFGR) & (0b11 << 2)) != (0b01 << 2)) {
		// Wait for SWS='01' or timeout.
		loop_count++;
		if (loop_count > RCC_TIMEOUT_COUNT) {
			status = RCC_ERROR_HSI_SWITCH;
			goto errors;
		}
	}
	// Update clocks context.
	rcc_ctx.sysclk_source = RCC_CLOCK_HSI;
	// Disable PLL and HSE.
	RCC -> CR &= ~(0b1 << 24); // PLLON='0'.
	RCC -> CR &= ~(0b1 << 16); // HSEON='0'.
	// Set flash latency.
	flash_status = FLASH_set_latency(1);
	FLASH_exit_error(RCC_ERROR_BASE_FLASH);
	// Turn TCXO off.
	power_status = POWER_disable(POWER_DOMAIN_MCU_TCXO);
	POWER_exit_error(RCC_ERROR_BASE_POWER);
errors:
	// Update system clock.
	rcc_ctx.clock_frequency[RCC_CLOCK_SYSTEM] = rcc_ctx.clock_frequency[rcc_ctx.sysclk_source];
	return status;
}

/*******************************************************************/
RCC_status_t RCC_switch_to_pll(void) {
	// Local variables.
	RCC_status_t status = RCC_SUCCESS;
	FLASH_status_t flash_status = FLASH_SUCCESS;
	POWER_status_t power_status = POWER_SUCCESS;
	uint32_t loop_count = 0;
	// Check current clock source.
	if (rcc_ctx.sysclk_source == RCC_CLOCK_PLL) goto errors;
	// Turn TCXO on.
	power_status = POWER_enable(POWER_DOMAIN_MCU_TCXO, LPTIM_DELAY_MODE_ACTIVE);
	POWER_exit_error(RCC_ERROR_BASE_POWER);
	// No prescaler.
	RCC -> CFGR &= ~(0b111 << 11);
	RCC -> CFGR &= ~(0b111 << 8);
	RCC -> CFGR &= ~(0b1111 << 4);
	// Turn HSE on.
	RCC -> CR |= (0b1 << 18); // External clock mode.
	RCC -> CR |= (0b1 << 16);
	// Use HSE by default.
	rcc_ctx.pll_source = RCC_CLOCK_HSE;
	// Wait for HSE to be ready.
	while (((RCC -> CR) & (0b1 << 17)) == 0) {
		// Wait for HSERDY='1' or timeout.
		loop_count++;
		if (loop_count > RCC_TIMEOUT_COUNT) {
			// Store error in stack.
			ERROR_stack_add(ERROR_BASE_RCC + RCC_ERROR_HSE_READY);
			// Disable HSE.
			RCC -> CR &= ~(0b1 << 16); // HSEON='0'.
			// Turn TCXO off.
			power_status = POWER_disable(POWER_DOMAIN_MCU_TCXO);
			POWER_stack_error();
			// Reset source.
			rcc_ctx.pll_source = RCC_CLOCK_HSI;
			break;
		}
	}
	// Configure PLL.
	// Clock entry = HSE 16MHz or HSI 16MHz.
	// M=2 -> VCO input clock = 8MHz.
	// N=30 -> VCO output clock = 240MHz.
	// R=2 -> SYSCLK = 120MHz.
	// P=30 -> ADCCLK = 8MHz.
	RCC -> PLLCFGR = 0;
	RCC -> PLLCFGR |= ((rcc_ctx.pll_source == RCC_CLOCK_HSI) ? (0b10 << 0) : (0b11 << 0));
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
			power_status = POWER_disable(POWER_DOMAIN_MCU_TCXO);
			POWER_stack_error();
			// Disable PLL and HSE.
			RCC -> CR &= ~(0b1 << 24); // PLLON='0'.
			RCC -> CR &= ~(0b1 << 16); // HSEON='0'.
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
			// Disable PLL and HSE.
			RCC -> CR &= ~(0b1 << 24); // PLLON='0'.
			RCC -> CR &= ~(0b1 << 16); // HSEON='0'.
			// Turn TCXO off.
			power_status = POWER_disable(POWER_DOMAIN_MCU_TCXO);
			POWER_stack_error();
			// Exit.
			status = RCC_ERROR_PLL_SWITCH;
			goto errors;
		}
	}
#ifndef LINKY_TIC_ENABLE
	// Disable HSI.
	if (rcc_ctx.pll_source != RCC_CLOCK_HSI) {
		RCC -> CR &= ~(0b1 << 8); // HSION='0'.
	}
#endif
	// Update clocks context.
	rcc_ctx.sysclk_source = RCC_CLOCK_PLL;
	rcc_ctx.clock_frequency[RCC_CLOCK_PLL] = (rcc_ctx.clock_frequency[rcc_ctx.pll_source] * 30) / (4);
errors:
	// Update system clock.
	rcc_ctx.clock_frequency[RCC_CLOCK_SYSTEM] = rcc_ctx.clock_frequency[rcc_ctx.sysclk_source];
	return status;
}

/*******************************************************************/
RCC_status_t RCC_calibrate(void) {
	// Local variables.
	RCC_status_t status = RCC_SUCCESS;
	TIM_status_t tim17_status = TIM_SUCCESS;
	uint16_t ref_clock_pulse_count = 0;
	uint16_t mco_pulse_count = 0;
	uint64_t temp_u64 = 0;
	uint32_t clock_frequency_hz = 0;
	uint8_t lse_status = 0;
	// Init measurement timer.
	TIM17_init();
	GPIO_configure(&GPIO_MCO, GPIO_MODE_ALTERNATE_FUNCTION, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_HIGH, GPIO_PULL_NONE);
	// Check LSE status.
	status = RCC_get_status(RCC_CLOCK_LSE, &lse_status);
	if (status != RCC_SUCCESS) goto errors;
	// HSI calibration is not possible without LSE.
	if (lse_status == 0) goto lsi_calibration;
	// Connect MCO to LSE clock.
	RCC -> CFGR &= ~(0x7F << 24);
	RCC -> CFGR |= (0b0111 << 24);
	// Perform measurement.
	tim17_status = TIM17_mco_capture(&ref_clock_pulse_count, &mco_pulse_count);
	TIM17_stack_error();
	// Compute HSI frequency.
	temp_u64 = ((uint64_t) RCC_LSE_FREQUENCY_HZ * (uint64_t) ref_clock_pulse_count);
	clock_frequency_hz = (uint32_t) ((temp_u64) / ((uint64_t) mco_pulse_count));
	// Check value.
	if ((tim17_status != TIM_SUCCESS) || (clock_frequency_hz < RCC_HSI_FREQUENCY_MIN_HZ) || (clock_frequency_hz > RCC_HSI_FREQUENCY_MAX_HZ)) {
		// Set to default value if out of expected range
		clock_frequency_hz = RCC_HSI_FREQUENCY_DEFAULT_HZ;
		ERROR_stack_add(ERROR_BASE_RCC + RCC_ERROR_HSI_CALIBRATION);
	}
	// Update local data.
	rcc_ctx.clock_frequency[RCC_CLOCK_HSI] = clock_frequency_hz;
	rcc_ctx.clock_frequency[RCC_CLOCK_PLL] = (rcc_ctx.clock_frequency[rcc_ctx.pll_source] * 30) / (4);
lsi_calibration:
	// Connect MCO to LSI clock.
	RCC -> CFGR &= ~(0x7F << 24);
	RCC -> CFGR |= (0b0110 << 24);
	// Perform measurement.
	tim17_status = TIM17_mco_capture(&ref_clock_pulse_count, &mco_pulse_count);
	TIM17_stack_error();
	// Compute LSI frequency.
	temp_u64 = ((uint64_t) rcc_ctx.clock_frequency[RCC_CLOCK_HSI] * (uint64_t) mco_pulse_count);
	clock_frequency_hz = (uint32_t) ((temp_u64) / ((uint64_t) ref_clock_pulse_count));
	// Check value.
	if ((tim17_status != TIM_SUCCESS) || (clock_frequency_hz < RCC_LSI_FREQUENCY_MIN_HZ) || (clock_frequency_hz > RCC_LSI_FREQUENCY_MAX_HZ)) {
		// Set to default value if out of expected range
		clock_frequency_hz = RCC_LSI_FREQUENCY_DEFAULT_HZ;
		ERROR_stack_add(ERROR_BASE_RCC + RCC_ERROR_LSI_CALIBRATION);
	}
	// Update local data.
	rcc_ctx.clock_frequency[RCC_CLOCK_LSI] = clock_frequency_hz;
errors:
	// Release timer.
	TIM17_de_init();
	// Update system clock.
	rcc_ctx.clock_frequency[RCC_CLOCK_SYSTEM] = rcc_ctx.clock_frequency[rcc_ctx.sysclk_source];
	return status;
}

/*******************************************************************/
RCC_status_t RCC_get_frequency_hz(RCC_clock_t clock, uint32_t* frequency_hz) {
	// Local variables.
	RCC_status_t status = RCC_SUCCESS;
	// Check parameters.
	if (clock >= RCC_CLOCK_LAST) {
		status = RCC_ERROR_CLOCK;
		goto errors;
	}
	if (frequency_hz == NULL) {
		status = RCC_ERROR_NULL_PARAMETER;
		goto errors;
	}
	// Read frequency.
	(*frequency_hz) = rcc_ctx.clock_frequency[clock];
errors:
	return status;
}

/*******************************************************************/
RCC_status_t RCC_get_status(RCC_clock_t clock, uint8_t* clock_is_ready) {
	// Local variables.
	RCC_status_t status = RCC_SUCCESS;
	// Check parameters.
	if (clock_is_ready == NULL) {
		status = RCC_ERROR_NULL_PARAMETER;
		goto errors;
	}
	// Check clock.
	switch (clock) {
	case RCC_CLOCK_LSI:
		(*clock_is_ready) = (((RCC -> CSR) >> 1) & 0b1);
		break;
	case RCC_CLOCK_LSE:
		(*clock_is_ready) = (((RCC -> BDCR) >> 1) & 0b1);
		break;
	case RCC_CLOCK_HSI:
		(*clock_is_ready) = (((RCC -> CR) >> 10) & 0b1);
		break;
	case RCC_CLOCK_HSE:
		(*clock_is_ready) = (((RCC -> CR) >> 17) & 0b1);
		break;
	case RCC_CLOCK_PLL:
		(*clock_is_ready) = (((RCC -> CR) >> 25) & 0b1);
		break;
	case RCC_CLOCK_SYSTEM:
		(*clock_is_ready) = 1;
		break;
	default:
		status = RCC_ERROR_CLOCK;
		goto errors;
	}
errors:
	return status;
}
