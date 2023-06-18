/*
 * tim.c
 *
 *  Created on: Jun 17, 2023
 *      Author: ludo
 */

#include "tim.h"

#include "adc.h"
#include "rcc.h"
#include "rcc_reg.h"
#include "tim_reg.h"

/*** TIM functions ***/

/* INIT TIM6 FOR ADC TRIGGER OPERATION.
 * @param:	None.
 * @return:	None.
 */
void TIM6_init(void) {
	// Enable peripheral clock.
	RCC -> APB1ENR1 |= (0b1 << 4); // TIM6EN='1'.
	// Configure peripheral.
	TIM6 -> CR1 &= ~(0b1 << 0); // CEN='0'.
	TIM6 -> CNT = 0;
	TIM6 -> DIER &= ~(0b1 << 0); // Disable interrupt (UIE='0').
	TIM6 -> SR &= ~(0b1 << 0); // UIF='0'.
	// Set PSC and ARR registers to reach 2ms.
	TIM6 -> PSC = (((RCC_SYSCLK_KHZ) / 1000) - 1); // TIM6 input clock set to 1MHz.
	TIM6 -> ARR = ADC_SAMPLING_PERIOD_US;
	// Generate event to update registers.
	TIM6 -> EGR |= (0b1 << 0); // UG='1'.
	TIM6 -> SR &= ~(0b1 << 0); // UIF='0'.
	// Enable trigger output on update.
	TIM6 -> CR2 |= (0b010 << 4);
}

/* START TIM6.
 * @param:	None.
 * @return:	None.
 */
void TIM6_start(void) {
	// Reset counter.
	TIM6 -> CNT = 0;
	// Start timer.
	TIM6 -> CR1 |= (0b1 << 0); // CEN='1'.
}

/* START TIM6.
 * @param:	None.
 * @return:	None.
 */
void TIM6_stop(void) {
	// Stop timer.
	TIM6 -> CR1 &= ~(0b1 << 0); // CEN='0'.
	// Reset counter.
	TIM6 -> CNT = 0;
}
