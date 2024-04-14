/*
 * tim.c
 *
 *  Created on: Jun 17, 2023
 *      Author: ludo
 */

#include "tim.h"

#include "adc.h"
#include "gpio.h"
#include "mapping.h"
#include "nvic.h"
#include "rcc.h"
#include "rcc_reg.h"
#include "tim_reg.h"
#include "types.h"

/*** TIM local macros ***/

#define TIM_TIMEOUT_COUNT				10000000

#define TIM4_CLOCK_HZ					10000
#define TIM4_NUMBER_OF_CHANNELS			4
#define TIM4_NUMBER_OF_USED_CHANNELS	3

#define TIM17_INPUT_CAPTURE_PRESCALER	8

/*** TIM local structures ***/

/*******************************************************************/
typedef struct {
	volatile uint16_t ccr1_start;
	volatile uint16_t ccr1_end;
	volatile uint16_t capture_count;
	volatile uint8_t capture_done;
} TIM17_context_t;

/*** TIM local global variables ***/

static const uint8_t TIM4_LED_CHANNELS[TIM4_NUMBER_OF_USED_CHANNELS] = {
	TIM4_CHANNEL_LED_RED,
	TIM4_CHANNEL_LED_GREEN,
	TIM4_CHANNEL_LED_BLUE
};
static TIM17_context_t tim17_ctx;

/*** TIM local functions ***/

/*******************************************************************/
void __attribute__((optimize("-O0"))) TIM1_TRG_COM_DIR_IDX_TIM17_IRQHandler(void) {
	// TI1 interrupt.
	if (((TIM17 -> SR) & (0b1 << 1)) != 0) {
		// Update flags.
		if (((TIM17 -> DIER) & (0b1 << 1)) != 0) {
			// Check count.
			if (tim17_ctx.capture_count == 0) {
				// Store start value.
				tim17_ctx.ccr1_start = (TIM17 -> CCR1);
				tim17_ctx.capture_count++;
			}
			else {
				// Check rollover.
				if ((TIM17 -> CCR1) > tim17_ctx.ccr1_end) {
					// Store new value.
					tim17_ctx.ccr1_end = (TIM17 -> CCR1);
					tim17_ctx.capture_count++;
				}
				else {
					// Capture complete.
					tim17_ctx.capture_done = 1;
				}
			}
		}
		TIM17 -> SR &= ~(0b1 << 1);
	}
}

/*******************************************************************/
void __attribute__((optimize("-O0"))) TIM4_IRQHandler(void) {
	// Check flag.
	if (((TIM4 -> SR) & (0b1 << 0)) != 0) {
		// Clear flag.
		TIM4 -> SR &= ~(0b1 << 0);
	}
}

/*** TIM functions ***/

/*******************************************************************/
TIM_status_t TIM2_init(uint32_t sampling_frequency_hz) {
	// Local variables.
	TIM_status_t status = TIM_SUCCESS;
	RCC_status_t rcc_status = RCC_SUCCESS;
	uint32_t tim2_clock_hz = 0;
	// Get clock source frequency.
	rcc_status = RCC_get_frequency_hz(RCC_CLOCK_SYSTEM, &tim2_clock_hz);
	RCC_exit_error(TIM_ERROR_BASE_RCC);
	// Enable peripheral clock.
	RCC -> APB1ENR1 |= (0b1 << 0); // TIM2EN='1'.
	// Set prescaler.
	TIM2 -> PSC = ((tim2_clock_hz / sampling_frequency_hz) - 1);
	TIM2 -> ARR = 0xFFFFFFFF;
	// Configure CH1 in input capture mode with prescaler (2 zero cross edges per period).
	TIM2 -> CCMR1 |= (0b01 << 2) | (0b01 << 0);
	// Enable DMA request.
	TIM2 -> DIER |= (0b1 << 9);
	// Generate event to update registers.
	TIM2 -> EGR |= (0b1 << 0); // UG='1'.
	// Configure GPIO.
	GPIO_configure(&GPIO_ACV_FREQUENCY, GPIO_MODE_ALTERNATE_FUNCTION, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
errors:
	return status;
}

/*******************************************************************/
void TIM2_de_init(void) {
	// ReLease GPIO.
	GPIO_configure(&GPIO_ACV_FREQUENCY, GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	// Disable peripheral clock.
	RCC -> APB1ENR1 &= ~(0b1 << 0); // TIM2EN='0'.
}

/*******************************************************************/
void TIM2_start(void) {
	// Reset counter.
	TIM2 -> CNT = 0;
	// Enable channel and counter.
	TIM2 -> CCER |= (0b1 << 0);
	TIM2 -> CR1 |= (0b1 << 0);
}

/*******************************************************************/
void TIM2_stop(void) {
	// Disable counter and channel.
	TIM2 -> CR1 &= ~(0b1 << 0);
	TIM2 -> CCER &= ~(0b1 << 0);
}

/*******************************************************************/
TIM_status_t TIM4_init(void) {
	// Local variables.
	TIM_status_t status = TIM_SUCCESS;
	RCC_status_t rcc_status = RCC_SUCCESS;
	uint32_t tim4_clock_hz = 0;
	uint8_t idx = 0;
	// Get clock source frequency.
	rcc_status = RCC_get_frequency_hz(RCC_CLOCK_SYSTEM, &tim4_clock_hz);
	RCC_exit_error(TIM_ERROR_BASE_RCC);
	// Enable peripheral clock.
	RCC -> APB1ENR1 |= (0b1 << 2); // TIM4EN='1'.
	// Set prescaler.
	TIM4 -> PSC = ((tim4_clock_hz / TIM4_CLOCK_HZ) - 1);
	// One pulse mode.
	TIM4 -> ARR = 0;
	TIM4 -> CR1 |= (0b1 << 3) | (0b1 << 7);
	// Configure channels 1-4 in PWM mode 2 (OCxM='0111', OCxFE='1' and OCxPE='1').
	TIM4 -> CCMR1 |= (0b111 << 12) | (0b11 << 10) | (0b111 << 4) | (0b11 << 2);
	TIM4 -> CCMR2 |= (0b111 << 12) | (0b11 << 10) | (0b111 << 4) | (0b11 << 2);
	// Polarity inverted.
	for (idx=0 ; idx<TIM4_NUMBER_OF_USED_CHANNELS ; idx++) {
		TIM4 -> CCER |= (0b1 << ((TIM4_LED_CHANNELS[idx] << 2) + 1));
		TIM4 -> CCRx[TIM4_LED_CHANNELS[idx]] = 1;
	}
	// Enable update interrupt.
	TIM4 -> DIER |= (0b1 << 0); // UIE='1'.
	NVIC_enable_interrupt(NVIC_INTERRUPT_TIM4, NVIC_PRIORITY_TIM4);
	// Generate event to update registers.
	TIM4 -> EGR |= (0b1 << 0); // UG='1'.
errors:
	return status;
}

/*******************************************************************/
void TIM4_single_pulse(uint32_t pulse_duration_ms, TIM4_channel_mask_t led_color) {
	// Local variables.
	uint8_t idx = 0;
	// Reset counter.
	TIM4 -> CR1 &= ~(0b1 << 0);
	TIM4 -> CNT = 0;
	// Check parameter.
	if (pulse_duration_ms == 0) return;
	// Set pulse length.
	TIM4 -> ARR = ((pulse_duration_ms * TIM4_CLOCK_HZ) / (1000));
	// Enable required channels.
	for (idx=0 ; idx<TIM4_NUMBER_OF_CHANNELS ; idx++) {
		// Check channel bit.
		if ((led_color & (0b1 << idx)) != 0) {
			TIM4 -> CCER |= 0b1 << (idx << 2);
		}
		else {
			TIM4 -> CCER &= ~(0b1 << (idx << 2));
		}
	}
	// Generate event to update registers.
	TIM4 -> EGR |= (0b1 << 0); // UG='1'.
	// Start counter.
	TIM4 -> CR1 |= (0b1 << 0); // CEN='1'.
}

/*******************************************************************/
uint8_t TIM4_is_single_pulse_done(void) {
	// Local variables.
	uint8_t pulse_done = (((TIM4 -> CR1) & (0b1 << 0)) == 0) ? 1 : 0;
	return pulse_done;
}

/*******************************************************************/
TIM_status_t TIM6_init(void) {
	// Local variables.
	TIM_status_t status = TIM_SUCCESS;
	RCC_status_t rcc_status = RCC_SUCCESS;
	uint32_t tim6_clock_hz = 0;
	// Get clock source frequency.
	rcc_status = RCC_get_frequency_hz(RCC_CLOCK_SYSTEM, &tim6_clock_hz);
	RCC_exit_error(TIM_ERROR_BASE_RCC);
	// Enable peripheral clock.
	RCC -> APB1ENR1 |= (0b1 << 4); // TIM6EN='1'.
	// Configure peripheral.
	TIM6 -> CR1 &= ~(0b1 << 0); // CEN='0'.
	TIM6 -> CNT = 0;
	TIM6 -> DIER &= ~(0b1 << 0); // Disable interrupt (UIE='0').
	TIM6 -> SR &= ~(0b1 << 0); // UIF='0'.
	// Set PSC and ARR registers to reach 2ms.
	TIM6 -> PSC = ((tim6_clock_hz / 1000000) - 1); // TIM6 input clock set to 1MHz.
	TIM6 -> ARR = ADC_SAMPLING_PERIOD_US;
	// Generate event to update registers.
	TIM6 -> EGR |= (0b1 << 0); // UG='1'.
	TIM6 -> SR &= ~(0b1 << 0); // UIF='0'.
	// Enable trigger output on update.
	TIM6 -> CR2 |= (0b010 << 4);
errors:
	return status;
}

/*******************************************************************/
void TIM6_de_init(void) {
	// Disable peripheral clock.
	RCC -> APB1ENR1 &= ~(0b1 << 4); // TIM6EN='0'.
}

/*******************************************************************/
void TIM6_start(void) {
	// Reset counter.
	TIM6 -> CNT = 0;
	// Start timer.
	TIM6 -> CR1 |= (0b1 << 0); // CEN='1'.
}

/*******************************************************************/
void TIM6_stop(void) {
	// Stop timer.
	TIM6 -> CR1 &= ~(0b1 << 0); // CEN='0'.
	// Reset counter.
	TIM6 -> CNT = 0;
}

/*******************************************************************/
void TIM17_init(void) {
	// Enable peripheral clock.
	RCC -> APB2ENR |= (0b1 << 18); // TIM17EN='1'.
	// Configure timer.
	// Channel input on TI1.
	// Capture done every 8 edges.
	// CH1 mapped on MCO.
	TIM17 -> CCMR1 |= (0b01 << 0) | (0b11 << 2);
	TIM17 -> TISEL |= (0b0010 << 0);
	// Enable interrupt.
	TIM17 -> DIER |= (0b1 << 1); // CC1IE='1'.
	// Generate event to update registers.
	TIM17 -> EGR |= (0b1 << 0); // UG='1'.
}

/*******************************************************************/
void TIM17_de_init(void) {
	// Disable timer.
	TIM17 -> CR1 &= ~(0b1 << 0); // CEN='0'.
	// Disable peripheral clock.
	RCC -> APB2ENR &= ~(0b1 << 18); // TIM17EN='0'.
}

/*******************************************************************/
TIM_status_t TIM17_mco_capture(uint16_t* ref_clock_pulse_count, uint16_t* mco_pulse_count) {
	// Local variables.
	TIM_status_t status = TIM_SUCCESS;
	uint32_t loop_count = 0;
	// Check parameters.
	if ((ref_clock_pulse_count == NULL) || (mco_pulse_count == NULL)) {
		status = TIM_ERROR_NULL_PARAMETER;
		goto errors;
	}
	// Reset timer context.
	tim17_ctx.ccr1_start = 0;
	tim17_ctx.ccr1_end = 0;
	tim17_ctx.capture_count = 0;
	tim17_ctx.capture_done = 0;
	// Reset counter.
	TIM17 -> CNT = 0;
	TIM17 -> CCR1 = 0;
	// Enable interrupt.
	TIM17 -> SR &= 0xFFFFFD5C; // Clear all flags.
	NVIC_enable_interrupt(NVIC_INTERRUPT_TIM1_TRG_COM_DIR_IDX_TIM17, NVIC_PRIORITY_TIM17);
	// Enable TIM17 peripheral.
	TIM17 -> CR1 |= (0b1 << 0); // CEN='1'.
	TIM17 -> CCER |= (0b1 << 0); // CC1E='1'.
	// Wait for capture to complete.
	while (tim17_ctx.capture_done == 0) {
		// Manage timeout.
		loop_count++;
		if (loop_count > TIM_TIMEOUT_COUNT) {
			status = TIM_ERROR_CAPTURE_TIMEOUT;
			goto errors;
		}
	}
	// Update results.
	(*ref_clock_pulse_count) = (tim17_ctx.ccr1_end - tim17_ctx.ccr1_start);
	(*mco_pulse_count) = (TIM17_INPUT_CAPTURE_PRESCALER * (tim17_ctx.capture_count - 1));
errors:
	// Disable interrupt.
	NVIC_disable_interrupt(NVIC_INTERRUPT_TIM1_TRG_COM_DIR_IDX_TIM17);
	// Stop counter.
	TIM17 -> CR1 &= ~(0b1 << 0); // CEN='0'.
	TIM17 -> CCER &= ~(0b1 << 0); // CC1E='0'.
	return status;
}
