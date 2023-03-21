/*
 * usart_reg.h
 *
 *  Created on: 31 dec. 2022
 *      Author: Ludo
 */

#ifndef __USART_REG_H__
#define __USART_REG_H__

#include "types.h"

/*** USARTx registers ***/

typedef struct {
	volatile uint32_t CR1;    	// USART control register 1.
	volatile uint32_t CR2;    	// USART control register 2.
	volatile uint32_t CR3;    	// USART control register 3.
	volatile uint32_t BRR;    	// USART baud rate register.
	volatile uint32_t GTPR;    	// USART guard time and prescaler register.
	volatile uint32_t RTOR;    	// USART receiver timeout register.
	volatile uint32_t RQR;    	// USART request register.
	volatile uint32_t ISR;    	// USART interrupt and status register.
	volatile uint32_t ICR;    	// USART interrupt flag clear register.
	volatile uint32_t RDR;    	// USART receive data register.
	volatile uint32_t TDR;    	// USART transmit data register.
	volatile uint32_t PRESC;	// USART prescaler register.
} USART_registers_t;

/*** USARTx base addresses ***/

#define USART1	((USART_registers_t*) ((uint32_t) 0x40013800))
#define USART2	((USART_registers_t*) ((uint32_t) 0x40004400))
#define USART3	((USART_registers_t*) ((uint32_t) 0x40004800))
#define UART4	((USART_registers_t*) ((uint32_t) 0x40004C00))
#if (defined MCU_CATEGORY_3) || (MCU_CATEGORY_4)
#define UART5	((USART_registers_t*) ((uint32_t) 0x40005000))
#endif

#endif /* __USART_REG_H__ */
