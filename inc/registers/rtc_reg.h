/*
 * rtc_reg.h
 *
 *  Created on: 2 jul. 2023
 *      Author: Ludo
 */

#ifndef __RTC_REG_H__
#define __RTC_REG_H__

#include "types.h"

/*** RTC REG macros ***/

// Peripheral base address.
#define RTC		((RTC_registers_t*) ((uint32_t) 0x40002800))

/*** RTC REG structures ***/

/*!******************************************************************
 * \enum RTC_registers_t
 * \brief RTC registers map.
 *******************************************************************/
typedef struct {
	volatile uint32_t TR;			// RTC time register.
	volatile uint32_t DR;			// RTC date register.
	volatile uint32_t SSR;			// RTC sub-seconds register.
	volatile uint32_t ICSR;			// RTC initialization control and status register.
	volatile uint32_t PRER;			// RTC prescaler register.
	volatile uint32_t WUTR;			// RTC write unlock register.
	volatile uint32_t CR;			// RTC control register.
	volatile uint32_t RESERVED0[2];	// Reserved 0x1C-0x20.
	volatile uint32_t WPR;			// RTC write protect register.
	volatile uint32_t CALR;			// RTC calibration register.
	volatile uint32_t SHIFTR;		// RTC shift control register.
	volatile uint32_t TSTR;			// RTC timestamp time register.
	volatile uint32_t TSDR;			// RTC timestamp date register.
	volatile uint32_t TSSSR;		// RTC timestamp sub-seconds register.
	volatile uint32_t RESERVED1;	// Reserved 0x3C.
	volatile uint32_t ALRMAR;		// RTC alarm A register.
	volatile uint32_t ALRMASSR;		// RTC alarm A sub-seconds register.
	volatile uint32_t ALRMBR;		// RTC alarm B register.
	volatile uint32_t ALRMBSSR;		// RTC alarm B sub-seconds register.
	volatile uint32_t SR;			// RTC status register.
	volatile uint32_t MISR;			// RTC masked interrupt and status register.
	volatile uint32_t RESERVED2;	// Reserved 0x5C.
	volatile uint32_t SCR;			// RTC status clear register.
} RTC_registers_t;

#endif /* __RTC_REG_H__ */
