/*
 * exti_reg.h
 *
 *  Created on: 09 mar. 2023
 *      Author: Ludo
 */

#ifndef __EXTI_REG_H__
#define __EXTI_REG_H__

/*** EXTI REG macros ***/

// Peripheral base address.
#define EXTI	((EXTI_registers_t*) ((uint32_t) 0x40010400))

/*** EXTI REG structures ***/

/*!******************************************************************
 * \enum EXTI_register_pattern_t
 * \brief EXTI registers map pattern.
 *******************************************************************/
typedef struct {
	volatile uint32_t IMR;    		// EXTI interrupt mask register.
	volatile uint32_t EMR;    		// EXTI event mask register.
	volatile uint32_t RTSR;    		// EXTI rising edge trigger selection register.
	volatile uint32_t FTSR;    		// EXTI falling edge trigger selection register.
	volatile uint32_t SWIER;		// EXTI software interrupt event register.
	volatile uint32_t PR;    		// EXTI pending register.
	volatile uint32_t RESERVED[2];	// Reserved.
} EXTI_register_pattern_t;

/*!******************************************************************
 * \enum EXTI_registers_t
 * \brief EXTI registers map.
 *******************************************************************/
typedef struct {
	volatile EXTI_register_pattern_t EXTIx[2];
} EXTI_registers_t;

#endif /* __EXTI_REG_H__ */
