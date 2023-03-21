/*
 * nvic_reg.h
 *
 *  Created on: 31 dec. 2022
 *      Author: Ludo
 */

#ifndef __NVIC_REG_H__
#define __NVIC_REG_H__

#include "types.h"

/*** NVIC registers ***/

typedef struct {
	volatile uint32_t ISER[3];		// Interrupt set-enable registers 0 to 7.
	uint32_t RESERVED0[29];			// Reserved 0xE000E10C.
	volatile uint32_t ICER[3];		// Interrupt clear-enable registers 0 to 7.
	uint32_t RESERVED1[29];			// Reserved 0xE000E18C.
	volatile uint32_t ISPR[3];		// Interrupt set-pending registers 0 to 7.
	uint32_t RESERVED2[29];			// Reserved 0xE000E20C.
	volatile uint32_t ICPR[3];    	// Interrupt clear-pending registers 0 to 7.
	uint32_t RESERVED3[29];			// Reserved 0xE000E29C.
	volatile uint32_t IABR[3];		// Interrupt active bit registers 0 to 7.
	uint32_t RESERVED4[61];			// Reserved 0xE000E31C.
	volatile uint32_t IPR[21];		// Interrupt priority registers 0 to 59.
	uint32_t RESERVED5[683];		// Reserved 0xE000E504.
	volatile uint32_t STIR;    		// Interrupt software trigger register.
} NVIC_registers_t;

/*** NVIC base address ***/

#define NVIC	((NVIC_registers_t*) ((uint32_t) 0xE000E100))

#endif /* __NVIC_REG_H__ */
