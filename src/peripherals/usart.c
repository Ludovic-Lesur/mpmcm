/*
 * usart.c
 *
 *  Created on: 09 apr. 2024
 *      Author: Ludo
 */

#include "usart.h"

#include "exti.h"
#include "gpio.h"
#include "mapping.h"
#include "nvic.h"
#include "rcc.h"
#include "rcc_reg.h"
#include "types.h"
#include "usart_reg.h"

/*** USART local macros ***/

#define USART_TIMEOUT_COUNT		100000

/*** USART local global variables ***/

static USART_character_match_irq_cb_t usart_cm_irq_callback = NULL;

/*** USART local functions ***/

/*******************************************************************/
void __attribute__((optimize("-O0"))) USART2_IRQHandler(void) {
	// Character match interrupt.
	if (((USART2 -> ISR) & (0b1 << 17)) != 0) {
		// Notify upper layer.
		if (usart_cm_irq_callback != NULL) {
			usart_cm_irq_callback();
		}
		// Clear CM flag.
		USART2 -> ICR |= (0b1 << 17);
	}
	// Parity error interrupt.
	if (((USART2 -> ISR) & (0b1 << 0)) != 0) {
		// Clear PE flag.
		USART2 -> ICR |= (0b1 << 0);
	}
	// Overrun error interrupt.
	if (((USART2 -> ISR) & (0b1 << 3)) != 0) {
		// Clear ORE flag.
		USART2 -> ICR |= (0b1 << 3);
	}
	EXTI_clear_flag(EXTI_LINE_USART2);
}

/*** USART functions ***/

/*******************************************************************/
USART_status_t USART2_init(uint32_t baud_rate, char_t character_match, USART_character_match_irq_cb_t irq_callback) {
	// Local variables.
	USART_status_t status = USART_SUCCESS;
	RCC_status_t rcc_status = RCC_SUCCESS;
	uint32_t usart_clock_hz = 0;
	uint32_t brr = 0;
	// Get clock source frequency.
	rcc_status = RCC_get_frequency_hz(RCC_CLOCK_HSI, &usart_clock_hz);
	RCC_exit_error(USART_ERROR_BASE_RCC);
	// Select LSE as peripheral clock.
	RCC -> CCIPR &= ~(0b11 << 2); // Reset bits 2-3.
	RCC -> CCIPR |= (0b10 << 2); // USART2SEL='10'.
	// Enable peripheral clock.
	RCC -> APB1ENR1 |= (0b1 << 17); // USART2EN='1'.
	RCC -> APB1SMENR1 |= (0b1 << 17); // Enable clock in sleep mode.
	// Configure peripheral.
	USART2 -> CR1 |= (0b1 << 10); // Parity enabled (PCE='1' and PS='0').
	USART2 -> CR3 |= (0b1 << 12); // No overrun detection (OVRDIS='1').
	brr = (usart_clock_hz / baud_rate);
	USART2 -> BRR = (brr & 0x000FFFFF); // BRR = (fCK)/(baud rate).
	// Configure character match interrupt and DMA.
	USART2 -> CR2 |= (character_match << 24); // LF character used to trigger CM interrupt (works as is because even parity of LF is 0).
	USART2 -> CR3 |= (0b1 << 6); // Transfer is performed after each RXNE event.
	USART2 -> CR1 |= (0b1 << 14) | (0b1 << 8); // Enable CM and PE interrupts (PEIE='1' and CMIE='1').
	// Enable peripheral.
	USART2 -> CR1 |= (0b1 << 0); // UE='1'.
	// Register callback.
	usart_cm_irq_callback = irq_callback;
errors:
	return status;
}

/*******************************************************************/
void USART2_de_init(void) {
	// Disable receiver.
	USART2_disable_rx();
	// Disable peripheral.
	USART2 -> CR1 &= ~(0b1 << 0); // UE='0'.
	// Disable peripheral clock.
	RCC -> APB1ENR1 &= ~(0b1 << 17); // USART2EN='0'.
}

/*******************************************************************/
void USART2_enable_rx(void) {
	// Enable receiver.
	USART2 -> CR1 |= (0b1 << 2); // RE='1'.
	// Enable alternate function.
	GPIO_configure(&GPIO_USART2_TX, GPIO_MODE_ALTERNATE_FUNCTION, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_configure(&GPIO_USART2_RX, GPIO_MODE_ALTERNATE_FUNCTION, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	// Clear flag and enable interrupt.
	USART2 -> RQR |= (0b1 << 3);
	NVIC_enable_interrupt(NVIC_INTERRUPT_USART2, NVIC_PRIORITY_USART2);
}

/*******************************************************************/
void USART2_disable_rx(void) {
	// Disable interrupt.
	NVIC_disable_interrupt(NVIC_INTERRUPT_USART2);
	// Disable USART alternate function.
	GPIO_configure(&GPIO_USART2_TX, GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_configure(&GPIO_USART2_RX, GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	// Disable receiver.
	USART2 -> CR1 &= ~(0b1 << 2); // RE='0'.
}
