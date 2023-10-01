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
#include "rcc.h"
#include "rcc_reg.h"
#include "tim_reg.h"
#include "types.h"

/*** TIM local macros ***/

#define TIM4_CLOCK_KHZ					10
#define TIM4_NUMBER_OF_CHANNELS			4
#define TIM4_NUMBER_OF_USED_CHANNELS	3

/*** TIM local global variables ***/

static const uint8_t TIM4_LED_CHANNELS[TIM4_NUMBER_OF_USED_CHANNELS] = {
	TIM4_CHANNEL_LED_RED,
	TIM4_CHANNEL_LED_GREEN,
	TIM4_CHANNEL_LED_BLUE
};

/*** TIM functions ***/

/*******************************************************************/
void TIM2_init(uint32_t sampling_frequency_hz) {
	// Enable peripheral clock.
	RCC -> APB1ENR1 |= (0b1 << 0); // TIM2EN='1'.
	// Set prescaler.
	TIM2 -> PSC = (((RCC_SYSCLK_FREQUENCY_KHZ * 1000) / (sampling_frequency_hz)) - 1);
	TIM2 -> ARR = 0xFFFFFFFF;
	// Configure CH1 in input capture mode with prescaler (2 zero cross edges per period).
	TIM2 -> CCMR1 |= (0b01 << 2) | (0b01 << 0);
	// Enable DMA request.
	TIM2 -> DIER |= (0b1 << 9);
	// Generate event to update registers.
	TIM2 -> EGR |= (0b1 << 0); // UG='1'.
	// Configure GPIO.
	GPIO_configure(&GPIO_ACV_FREQUENCY, GPIO_MODE_ALTERNATE_FUNCTION, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
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
void TIM4_init(void) {
	// Local variables.
	uint8_t idx = 0;
	// Enable peripheral clock.
	RCC -> APB1ENR1 |= (0b1 << 2); // TIM4EN='1'.
	// Set prescaler.
	TIM4 -> PSC = ((RCC_SYSCLK_FREQUENCY_KHZ / TIM4_CLOCK_KHZ) - 1);
	// One pulse mode.
	TIM4 -> ARR = 0xFFFF;
	TIM4 -> CR1 |= (0b1 << 3) | (0b1 << 7);
	// Configure channels 1-4 in PWM mode 2 (OCxM='0111', OCxFE='1' and OCxPE='1').
	TIM4 -> CCMR1 |= (0b111 << 12) | (0b11 << 10) | (0b111 << 4) | (0b11 << 2);
	TIM4 -> CCMR2 |= (0b111 << 12) | (0b11 << 10) | (0b111 << 4) | (0b11 << 2);
	// Enable required channels.
	for (idx=0 ; idx<TIM4_NUMBER_OF_USED_CHANNELS ; idx++) {
		TIM4 -> CCER |= (0b1 << (TIM4_LED_CHANNELS[idx] << 2));
	}
	// Generate event to update registers.
	TIM4 -> EGR |= (0b1 << 0); // UG='1'.
}

/*******************************************************************/
void TIM4_single_pulse(uint32_t pulse_duration_ms, TIM4_channel_mask_t led_color) {
	// Local variables.
	uint8_t idx = 0;
	// Stop counter.
	TIM4 -> CR1 &= ~(0b1 << 0);
	// Reset counter.
	TIM4 -> CNT = 0;
	// Check parameter.
	if (pulse_duration_ms == 0) return;
	// Configure channels.
	for (idx=0 ; idx<TIM4_NUMBER_OF_CHANNELS ; idx++) {
		// Set pulse length.
		TIM4 -> CCRx[idx] = ((led_color & (0b1 << idx)) != 0) ? (pulse_duration_ms * TIM4_CLOCK_KHZ) : 0;
	}
	// Generate event to update registers.
	TIM4 -> EGR |= (0b1 << 0); // UG='1'.
	// Start counter.
	TIM4 -> CR1 |= (0b1 << 0); // CEN='1'.
}

/*******************************************************************/
void TIM6_init(void) {
	// Enable peripheral clock.
	RCC -> APB1ENR1 |= (0b1 << 4); // TIM6EN='1'.
	// Configure peripheral.
	TIM6 -> CR1 &= ~(0b1 << 0); // CEN='0'.
	TIM6 -> CNT = 0;
	TIM6 -> DIER &= ~(0b1 << 0); // Disable interrupt (UIE='0').
	TIM6 -> SR &= ~(0b1 << 0); // UIF='0'.
	// Set PSC and ARR registers to reach 2ms.
	TIM6 -> PSC = (((RCC_SYSCLK_FREQUENCY_KHZ) / 1000) - 1); // TIM6 input clock set to 1MHz.
	TIM6 -> ARR = ADC_SAMPLING_PERIOD_US;
	// Generate event to update registers.
	TIM6 -> EGR |= (0b1 << 0); // UG='1'.
	TIM6 -> SR &= ~(0b1 << 0); // UIF='0'.
	// Enable trigger output on update.
	TIM6 -> CR2 |= (0b010 << 4);
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
