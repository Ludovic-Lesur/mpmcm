/*
 * tim_reg.h
 *
 *  Created on: 31 dec. 2022
 *      Author: Ludo
 */

#ifndef __TIM_REG_H__
#define __TIM_REG_H__

#include "types.h"

/*** TIM REG macros ***/

// Peripherals base address.
#define TIM1	((TIM_registers_t*) ((uint32_t) 0x40012C00))
#define TIM2	((TIM_registers_t*) ((uint32_t) 0x40000000))
#define TIM3	((TIM_registers_t*) ((uint32_t) 0x40000400))
#define TIM4	((TIM_registers_t*) ((uint32_t) 0x40000800))
#ifdef MCU_CATEGORY_3
#define TIM5	((TIM_registers_t*) ((uint32_t) 0x40000C00))
#endif
#define TIM6	((TIM_registers_t*) ((uint32_t) 0x40001000))
#define TIM7	((TIM_registers_t*) ((uint32_t) 0x40001400))
#define TIM8	((TIM_registers_t*) ((uint32_t) 0x40013400))
#define TIM15	((TIM_registers_t*) ((uint32_t) 0x40014000))
#define TIM16	((TIM_registers_t*) ((uint32_t) 0x40014400))
#define TIM17	((TIM_registers_t*) ((uint32_t) 0x40014800))
#if (defined MCU_CATEGORY_3) || (defined MCU_CATEGORY_4)
#define TIM20	((TIM_registers_t*) ((uint32_t) 0x40015000))
#endif

/*** TIM REG structures ***/

/*!******************************************************************
 * \enum TIM_registers_t
 * \brief TIM registers map.
 *******************************************************************/
typedef struct {
	volatile uint32_t CR1;    			// TIM control register 1.
	volatile uint32_t CR2;    			// TIM control register 2.
	volatile uint32_t SMCR;    			// TIM slave mode controler register.
	volatile uint32_t DIER;    			// TIM DMA interrupt enable register.
	volatile uint32_t SR;    			// TIM status register.
	volatile uint32_t EGR;    			// TIM event generation register.
	volatile uint32_t CCMR1;    		// TIM capture/compare mode register 1.
	volatile uint32_t CCMR2;    		// TIM capture/compare mode register 2.
	volatile uint32_t CCER;    			// TIM capture/compare enable register.
	volatile uint32_t CNT;    			// TIM counter register.
	volatile uint32_t PSC;    			// TIM prescaler register.
	volatile uint32_t ARR;    			// TIM auto-reload register.
	volatile uint32_t RCR;    			// TIM repetition counter register.
	volatile uint32_t CCR1;    			// TIM capture/compare register 1.
	volatile uint32_t CCR2;    			// TIM capture/compare register 2.
	volatile uint32_t CCR3;    			// TIM capture/compare register 3.
	volatile uint32_t CCR4;    			// TIM capture/compare register 4.
	volatile uint32_t BDTR;    			// TIM break and dead-time register.
	volatile uint32_t CCR5;    			// TIM capture/compare register 5.
	volatile uint32_t CCR6;    			// TIM capture/compare register 6.
	volatile uint32_t CCMR3;    		// TIM capture/compare mode register 3.
	volatile uint32_t DTR2;   			// TIM timer deadline register.
	volatile uint32_t ECR;    			// TIM encoder control register.
	volatile uint32_t TISEL;    		// TIM Input selection register.
	volatile uint32_t AF1;				// TIM Alternate function option register 1.
	volatile uint32_t AF2;				// TIM Alternate function option register 2.
	volatile uint32_t OR;				// TIM Option register.
	volatile uint32_t RESERVED[220];	// Reserved 0x68.
	volatile uint32_t DCR;				// TIM DMA control register.
	volatile uint32_t DMAR;				// TIM DMA address for full transfer register.
} TIM_registers_t;

#endif /* __TIM_REG_H__ */
