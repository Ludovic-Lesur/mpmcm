/*
 * dmamux_reg.h
 *
 *  Created on: 18 jun. 2023
 *      Author: Ludo
 */

#ifndef __DMAMUX_REG_H__
#define __DMAMUX_REG_H__

#include "types.h"

/*** DMAMUX REG macros ***/

// Peripheral base address.
#define DMAMUX	((DMAMUX_registers_t*) ((uint32_t) 0x40020800))

/*** DMAMUX REG structures ***/

/*!******************************************************************
 * \enum DMAMUX_registers_t
 * \brief DMAMUX registers map.
 *******************************************************************/
typedef struct {
	volatile uint32_t CxCR[16];			// DMAMUX request line multiplexer line x configuration register.
	volatile uint32_t RESERVED0[16];	// Reserved 0x40-0x7C.
	volatile uint32_t CSR;
	volatile uint32_t CCFR;
	volatile uint32_t RESERVED1[30];	// Reserved 0x88-0xFC.
	volatile uint32_t RGxCR[4];
	volatile uint32_t RESERVED2[12];	// Reserved 0x110-0x13C.
	volatile uint32_t RGSR;
	volatile uint32_t RGCFR;
} DMAMUX_registers_t;

#endif /* __DMAMUX_REG_H__ */
