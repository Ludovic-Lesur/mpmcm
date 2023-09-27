/*
 * pwr_reg.h
 *
 *  Created on: 09 mar 2023
 *      Author: Ludo
 */

#ifndef __PWR_REG_H__
#define __PWR_REG_H__

#include "types.h"

/*** PWR REG macros ***/

// Peripheral base address.
#define PWR		((PWR_registers_t*) ((uint32_t) 0x40007000))

/*** PWR REG structures ***/

/*!******************************************************************
 * \enum PWR_registers_t
 * \brief PWR registers map.
 *******************************************************************/
typedef struct {
	volatile uint32_t CR1;				// PWR control register 1.
	volatile uint32_t CR2;				// PWR control register 2.
	volatile uint32_t CR3;				// PWR control register 3.
	volatile uint32_t CR4;				// PWR control register 4.
	volatile uint32_t SR1;				// PWR status register 1.
	volatile uint32_t SR2;				// PWR status register 2.
	volatile uint32_t SCR;				// PWR status clear register.
	volatile uint32_t RESERVED0;		// Reserved 0x1C.
	volatile uint32_t PUCRA;			// PWR port A pull-up control register.
	volatile uint32_t PDCRA;			// PWR port A pull-down control register.
	volatile uint32_t PUCRB;			// PWR port B pull-up control register.
	volatile uint32_t PDCRB;			// PWR port B pull-down control register.
	volatile uint32_t PUCRC;			// PWR port C pull-up control register.
	volatile uint32_t PDCRC;			// PWR port C pull-down control register.
	volatile uint32_t PUCRD;			// PWR port D pull-up control register.
	volatile uint32_t PDCRD;			// PWR port D pull-down control register.
	volatile uint32_t PUCRE;			// PWR port E pull-up control register.
	volatile uint32_t PDCRE;			// PWR port E pull-down control register.
	volatile uint32_t PUCRF;			// PWR port F pull-up control register.
	volatile uint32_t PDCRF;			// PWR port F pull-down control register.
	volatile uint32_t PUCRG;			// PWR port G pull-up control register.
	volatile uint32_t PDCRG;			// PWR port G pull-down control register.
	volatile uint32_t RESERVED1[10];	// Reserved 0x58-0x7C.
	volatile uint32_t CR5;				// PWR control register 5.
} PWR_registers_t;

#endif /* __PWR_REG_H__ */
