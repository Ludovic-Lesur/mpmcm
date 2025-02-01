/*
 * nvic.c
 *
 *  Created on: 17 jun. 2023
 *      Author: Ludo
 */

#include "nvic.h"

#include "nvic_reg.h"
#include "scb_reg.h"
#include "types.h"

/*** NVIC local global variables ***/

extern uint32_t __Vectors;

/*** NVIC local functions ***/

/*******************************************************************/
static void _NVIC_set_priority(NVIC_interrupt_t irq_index, uint8_t priority) {
	// Reset bits.
	NVIC -> IPR[(irq_index >> 2)] &= ~(0xFF << (8 * (irq_index % 4)));
	// Set priority.
	NVIC -> IPR[(irq_index >> 2)] |= (priority << (8 * (irq_index % 4)));
}

/*** NVIC functions ***/

/*******************************************************************/
void NVIC_init(void) {
	// Init vector table address.
	SCB -> VTOR = (uint32_t) &__Vectors;
}

/*******************************************************************/
void NVIC_enable_interrupt(NVIC_interrupt_t irq_index, uint8_t priority) {
	// Set priority.
	_NVIC_set_priority(irq_index, priority);
	// Enable interrupt.
	NVIC -> ISER[irq_index >> 5] = (0b1 << (irq_index % 32));
}

/*******************************************************************/
void NVIC_disable_interrupt(NVIC_interrupt_t irq_index) {
	// Disable interrupt.
	NVIC -> ICER[irq_index >> 5] = (0b1 << (irq_index % 32));
}
