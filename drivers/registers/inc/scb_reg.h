/*
 * scb_reg.h
 *
 *  Created on: 31 dec. 2022
 *      Author: Ludo
 */

#ifndef __SCB_REG_H__
#define __SCB_REG_H__

#include "types.h"

/*** SCB REG macros ***/

// Peripheral base address.
#define SCB		((SCB_registers_t*) ((uint32_t) 0xE000ED00))

/*** SCB REG structures ***/

/*!******************************************************************
 * \enum SCB_registers_t
 * \brief SCB registers map.
 *******************************************************************/
typedef struct {
	volatile uint32_t CPUID;	// SCB CPU ID base register.
	volatile uint32_t ICSR;		// SCB interrupt control and state register.
	volatile uint32_t VTOR;		// SCB vector table offset register.
	volatile uint32_t AIRCR;	// SCB application interrupt and reset control register.
	volatile uint32_t SCR;		// SCB system control register.
	volatile uint32_t CCR;		// SCB configuration and control register.
	volatile uint32_t SHPR1;	// SCB system handler priority register 1.
	volatile uint32_t SHPR2;	// SCB system handler priority register 2.
	volatile uint32_t SHPR3;	// SCB system handler priority register 3.
	volatile uint32_t SHCRS;	// SCB system handler control and state register.
	volatile uint32_t CFSR;		// SCB configurable fault status register.
	volatile uint32_t HFSR;		// SCB hard fault status register.
	volatile uint32_t RESERVED;	// Reserved 0x30.
	volatile uint32_t MMAR;		// SCB memory manage fault address register.
	volatile uint32_t BFAR;		// SCB bus fault address register.
	volatile uint32_t AFSR;		// SCB bus fault address register.
} SCB_registers_t;

#endif /* __SCB_REG_H__ */
