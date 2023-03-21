/*
 * lpuart_reg.h
 *
 *  Created on: 9 mar. 2023
 *      Author: Ludo
 */

#ifndef __LPUART_REG_H__
#define __LPUART_REG_H__

#include "types.h"

/*** LPUART registers ***/

typedef struct {
	volatile uint32_t CR1;    		// LPUART control register 1.
	volatile uint32_t CR2;   		// LPUART control register 2.
	volatile uint32_t CR3;  		// LPUART control register 3.
	volatile uint32_t BRR;    		// LPUART baud rate register.
	volatile uint32_t RESERVED0[2];	// Reserved 0x10-0x14.
	volatile uint32_t RQR;      	// LPUART request register.
	volatile uint32_t ISR;      	// LPUART interrupt and status register.
	volatile uint32_t ICR;    		// LPUART interrupt flag clear register.
	volatile uint32_t RDR;     		// LPUART receive data register.
	volatile uint32_t TDR;   		// LPUART transmit data register.
	volatile uint32_t PRESC;   		// LPUART prescaler register.
} LPUART_registers_t;

/*** LPUART base address ***/

#define LPUART1	((LPUART_registers_t*) ((uint32_t) 0x40008000))

#endif /* __LPUART_REG_H__ */
