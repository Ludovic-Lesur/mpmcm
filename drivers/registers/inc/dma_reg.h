/*
 * dma_reg.h
 *
 *  Created on: 18 jun. 2023
 *      Author: Ludo
 */

#ifndef __DMA_REG_H__
#define __DMA_REG_H__

#include "types.h"

/*** DMA REG macros ***/

// Peripherals base address.
#define DMA1	((DMA_registers_t*) ((uint32_t) 0x40020000))
#define DMA2	((DMA_registers_t*) ((uint32_t) 0x40020400))

/*** DMA REG structures ***/

/*!******************************************************************
 * \enum DMA_channel_registers_t
 * \brief DMA channel registers map.
 *******************************************************************/
typedef struct {
	volatile uint32_t CCR;						// DMA channel x configuration register.
	volatile uint32_t CNDTR;					// DMA channel x number of data register.
	volatile uint32_t CPAR;						// DMA channel x peripheral address register.
	volatile uint32_t CMAR;    					// DMA channel x memory address register.
	volatile uint32_t RESERVED;					// Reserved.
} DMA_channel_registers_t;

/*!******************************************************************
 * \enum DMA_registers_t
 * \brief DMA registers map.
 *******************************************************************/
typedef struct {
	volatile uint32_t ISR;						// DMA interrupt status register.
	volatile uint32_t IFCR;						// DMA interrupt flag clear register.
	volatile DMA_channel_registers_t CHx[8];	// DMA channel x registers.
} DMA_registers_t;

#endif /* __DMA_REG_H__ */
