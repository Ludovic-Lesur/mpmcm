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

/*** NVIC functions ***/

/* INIT VECTOR TABLE ADDRESS.
 * @param:	None.
 * @return:	None.
 */
void NVIC_init(void) {
	SCB -> VTOR = (uint32_t) &__Vectors;
}

/* ENABLE AN INTERRUPT LINE.
 * @param irq_index:	Interrupt number (use enum defined in 'nvic.h').
 * @return:			None.
 */
void NVIC_enable_interrupt(NVIC_interrupt_t irq_index) {
	NVIC -> ISER[irq_index >> 5] = (0b1 << (irq_index % 32));
}

/* DISABLE AN INTERRUPT LINE.
 * @param irq_index:	Interrupt number (use enum defined in 'nvic.h').
 * @return:			None.
 */
void NVIC_disable_interrupt(NVIC_interrupt_t irq_index) {
	NVIC -> ICER[irq_index >> 5] = (0b1 << (irq_index % 32));
}

/* SET THE PRIORITY OF AN INTERRUPT LINE.
 * @param irq_index:	Interrupt number (use enum defined in 'nvic.h').
 * @param priority:	Interrupt priority (0 to 255).
 */
void NVIC_set_priority(NVIC_interrupt_t irq_index, uint8_t priority) {
	// Check parameter.
	if ((priority >= NVIC_PRIORITY_MAX) && (priority <= NVIC_PRIORITY_MIN)) {
		// Reset bits.
		NVIC -> IPR[(irq_index >> 2)] &= ~(0xFF << (8 * (irq_index % 4)));
		// Set priority.
		NVIC -> IPR[(irq_index >> 2)] |= (priority << (8 * (irq_index % 4)));
	}
}
