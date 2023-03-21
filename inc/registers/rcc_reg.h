/*
 * rcc_reg.h
 *
 *  Created on: 31 dec. 2022
 *      Author: Ludo
 */

#ifndef __RCC_REG_H__
#define __RCC_REG_H__

#include "types.h"

/*** RCC registers ***/

typedef struct {
	volatile uint32_t CR;			// RCC clock control register.
	volatile uint32_t ICSCR;    	// RCC internal clock sources calibration register.
	volatile uint32_t CFGR;      	// RCC clock configuration register.
	volatile uint32_t PLLCFGR;      // RCC PLL configuration register.
	uint32_t RESERVED0[2]; 			// Reserved 0x10.
	volatile uint32_t CIER;			// RCC interrupt enable register.
	volatile uint32_t CIFR;			// RCC interrupt flag register.
	volatile uint32_t CICR;			// RCC interrupt clear register.
	uint32_t RESERVED1; 			// Reserved 0x24.
	volatile uint32_t AHB1RSTR;  	// RCC AHB1 peripheral reset register.
	volatile uint32_t AHB2RSTR;  	// RCC AHB2 peripheral reset register.
	volatile uint32_t AHB3RSTR;  	// RCC AHB3 peripheral reset register.
	uint32_t RESERVED2; 			// Reserved 0x34.
	volatile uint32_t APB1RSTR1; 	// RCC APB1 peripheral reset register 1.
	volatile uint32_t APB1RSTR2; 	// RCC APB1 peripheral reset register 2.
	volatile uint32_t APB2RSTR; 	// RCC APB2 peripheral reset register.
	uint32_t RESERVED3; 			// Reserved 0x44.
	volatile uint32_t AHB1ENR;  	// RCC AHB1 peripheral clock register.
	volatile uint32_t AHB2ENR;  	// RCC AHB2 peripheral clock register.
	volatile uint32_t AHB3ENR; 		// RCC AHB3 peripheral clock register.
	uint32_t RESERVED4; 			// Reserved 0x54.
	volatile uint32_t APB1ENR1;  	// RCC APB1 peripheral clock enable register 1.
	volatile uint32_t APB1ENR2;  	// RCC APB1 peripheral clock enable register 2.
	volatile uint32_t APB2ENR;  	// RCC APB2 peripheral clock enable register.
	uint32_t RESERVED5; 			// Reserved 0x64.
	volatile uint32_t AHB1SMENR;	// RCC AHB1 peripheral clock enable in low power mode register.
	volatile uint32_t AHB2SMENR; 	// RCC AHB2 peripheral clock enable in low power mode register.
	volatile uint32_t AHB3SMENR; 	// RCC AHB3 peripheral clock enable in low power mode register.
	uint32_t RESERVED6; 			// Reserved 0x74.
	volatile uint32_t APB1SMENR1;	// RCC APB1 peripheral clock enable in low power mode register 1.
	volatile uint32_t APB1SMENR2;	// RCC APB1 peripheral clock enable in low power mode register 2.
	volatile uint32_t APB2SMENR; 	// RCC APB2 peripheral clock enable in low power mode register.
	uint32_t RESERVED7; 			// Reserved 0x84.
	volatile uint32_t CCIPR;     	// RCC peripheral independent clock configuration register 1.
	uint32_t RESERVED8;				// Reserved 0x8C.
	volatile uint32_t BDCR;      	// RCC RTC domain control register.
	volatile uint32_t CSR;      	// RCC control and status register.
	volatile uint32_t CRRCR;		// RCC clock recovery RC register.
	volatile uint32_t CCIPR2;		// RCC peripheral independent clock configuration register 2.
} RCC_registers_t;

/*** RCC base address ***/

#define RCC		((RCC_registers_t*) ((uint32_t) 0x40021000))

#endif /* __RCC_REG_H__ */
