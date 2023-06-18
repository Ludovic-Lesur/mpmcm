/*
 * syscfg_reg.h
 *
 *  Created on: 18 jun. 2023
 *      Author: Ludo
 */

#ifndef __SYSCFG_REG_H__
#define __SYSCFG_REG_H__

#include "types.h"

/*** SYSCFG registers ***/

typedef struct {
	volatile uint32_t MEMRMP;    	// SYSCFG memory remap register.
	volatile uint32_t CFGR1;    	// SYSCFG configuration register 1.
	volatile uint32_t EXTICR[4];  	// SYSCFG external interrupt configuration registers 1-4.
	volatile uint32_t SCSR;   		// SYSCFG CCM SRAM control and status register.
	volatile uint32_t CFGR2;   		// SYSCFG configuration register 2.
	volatile uint32_t SWPR;   		// SYSCFG CCM SRAM write protection register.
	volatile uint32_t SKR;   		// SYSCFG CCM SRAM write key register.
} SYSCFG_registers_t;

/*** SYSCFG base address ***/

#define SYSCFG	((SYSCFG_registers_t*) ((uint32_t) 0x40010000))

#endif /* __SYSCFG_REG_H__ */
