/*
 * fpu_reg.h
 *
 *  Created on: 25 apr. 2024
 *      Author: Ludo
 */

#ifndef __FPU_REG_H__
#define __FPU_REG_H__

#include "types.h"

/*** FPU REG macros ***/

// Peripheral base address.
#define FPU		((FPU_registers_t*) ((uint32_t) 0xE000ED88))

/*** FPU REG structures ***/

/*!******************************************************************
 * \enum FPU_registers_t
 * \brief FPU registers map.
 *******************************************************************/
typedef struct {
	volatile uint32_t CPACR;			// FPU co-processor access control register.
	volatile uint32_t RESERVED0[106];	// Reserved.
	volatile uint32_t FPCCR;			// FPU context control register.
	volatile uint32_t FPCAR;			// FPU context address register.
	volatile uint32_t FPDSCR;			// FPU default status control register.
	volatile uint32_t FPSCR;			// FPU status control register.
} FPU_registers_t;

#endif /* __FPU_REG_H__ */
