/*
 * gpio_reg.h
 *
 *  Created on: 31 dec. 2022
 *      Author: Ludo
 */

#ifndef __GPIO_REG_H__
#define __GPIO_REG_H__

#include "types.h"

/*** GPIO REG macros ***/

// Peripherals base address.
#define GPIOA	((GPIO_registers_t*) ((uint32_t) 0x48000000))
#define GPIOB	((GPIO_registers_t*) ((uint32_t) 0x48000400))
#define GPIOC	((GPIO_registers_t*) ((uint32_t) 0x48000800))
#define GPIOD	((GPIO_registers_t*) ((uint32_t) 0x48000C00))
#define GPIOE	((GPIO_registers_t*) ((uint32_t) 0x48001000))
#define GPIOF	((GPIO_registers_t*) ((uint32_t) 0x48001400))
#define GPIOG	((GPIO_registers_t*) ((uint32_t) 0x48001800))

/*** GPIO REG structures ***/

/*!******************************************************************
 * \enum GPIO_registers_t
 * \brief GPIO registers map.
 *******************************************************************/
typedef struct {
	volatile uint32_t MODER;	// GPIO port mode register.
	volatile uint32_t OTYPER;	// GPIO port output type register.
	volatile uint32_t OSPEEDR;	// GPIO port output speed register.
	volatile uint32_t PUPDR;	// GPIO port pull-up/pull-down register.
	volatile uint32_t IDR;		// GPIO port input data register.
	volatile uint32_t ODR;		// GPIO port output data register.
	volatile uint32_t BSRR;		// GPIO port bit set/reset low register.
	volatile uint32_t LCKR;		// GPIO port configuration lock register.
	volatile uint32_t AFRL;		// GPIO alternate function low register.
	volatile uint32_t AFRH;		// GPIO alternate function high register.
	volatile uint32_t BRR;		// GPIO port bit reset register.
} GPIO_registers_t;

#endif /* __GPIO_REG_H__ */
