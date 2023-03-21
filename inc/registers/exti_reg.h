/*
 * exti_reg.h
 *
 *  Created on: 9 mar. 2023
 *      Author: Ludo
 */

#ifndef __EXTI_REG_H__
#define __EXTI_REG_H__

/*** EXTI registers ***/

typedef struct {
	volatile uint32_t IMR1;    	// EXTI interrupt mask register 1.
	volatile uint32_t EMR1;    	// EXTI event mask register 1.
	volatile uint32_t RTSR1;    // EXTI rising edge trigger selection register 1.
	volatile uint32_t FTSR1;    // EXTI falling edge trigger selection register 1.
	volatile uint32_t SWIER1;	// EXTI software interrupt event register 1.
	volatile uint32_t PR1;    	// EXTI pending register 1.
	uint32_t RESERVED[2];		// Reserved 0x18-0x1C.
	volatile uint32_t IMR2;    	// EXTI interrupt mask register 2.
	volatile uint32_t EMR2;    	// EXTI event mask register 2.
	volatile uint32_t RTSR2;    // EXTI rising edge trigger selection register 2.
	volatile uint32_t FTSR2;    // EXTI falling edge trigger selection register 2.
	volatile uint32_t SWIER2;	// EXTI software interrupt event register 2.
	volatile uint32_t PR2;    	// EXTI pending register 2.
} EXTI_registers_t;

/*** EXTI base address ***/

#define EXTI	((EXTI_registers_t*) ((uint32_t) 0x40010400))

#endif /* __EXTI_REG_H__ */
