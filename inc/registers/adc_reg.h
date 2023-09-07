/*
 * adc_reg.h
 *
 *  Created on: 31 dec. 2022
 *      Author: Ludo
 */

#ifndef __ADC_REG_H__
#define __ADC_REG_H__

#include "types.h"

/*** ADC REG macros ***/

// Peripherals base address.
#define ADC1		((ADC_registers_t*) ((uint32_t) 0x50000000))
#define ADC2		((ADC_registers_t*) ((uint32_t) 0x50000100))
#define ADCCR12		((ADCCR_registers_t*) ((uint32_t) 0x50000300))
#if (defined MCU_CATEGORY_3) || (defined MCU_CATEGORY_4)
#define ADC3		((ADC_registers_t*) ((uint32_t) 0x50000400))
#endif
#ifdef MCU_CATEGORY_3
#define ADC4		((ADC_registers_t*) ((uint32_t) 0x50000500))
#define ADC5		((ADC_registers_t*) ((uint32_t) 0x50000600))
#endif
#if (defined MCU_CATEGORY_3) || (defined MCU_CATEGORY_4)
#define ADCCR345	((ADCCR_registers_t*) ((uint32_t) 0x50000700))
#endif

/*** ADC REG structures ***/

/*!******************************************************************
 * \enum ADC_registers_t
 * \brief ADC registers map.
 *******************************************************************/
typedef struct {
	volatile uint32_t ISR;    			// ADC interrupt and status register.
	volatile uint32_t IER;				// ADC interrupt enable register.
	volatile uint32_t CR;    			// ADC control register.
	volatile uint32_t CFGR;    			// ADC configuration register 1.
	volatile uint32_t CFGR2;			// ADC configuration register 2.
	volatile uint32_t SMPR1;   			// ADC sample time register 1.
	volatile uint32_t SMPR2;    		// ADC sample time register 2.
	volatile uint32_t RESERVED0;		// Reserved 0x1C.
	volatile uint32_t TR1;				// ADC watchdog threshold register 1.
	volatile uint32_t TR2;				// ADC watchdog threshold register 2.
	volatile uint32_t TR3;				// ADC watchdog threshold register 3.
	volatile uint32_t RESERVED1;		// Reserved 0x2C.
	union {
		struct {
			volatile uint32_t SQR1;		// ADC regular sequence register 1.
			volatile uint32_t SQR2; 	// ADC regular sequence register 2.
			volatile uint32_t SQR3;		// ADC regular sequence register 3.
			volatile uint32_t SQR4;		// ADC regular sequence register 4.
		};
		volatile uint32_t SQR[4];		// ADC regular sequence registers 1 to 4.
	};
	volatile uint32_t DR;    			// ADC regular data register.
	volatile uint32_t RESERVED2[2];		// Reserved 0x44 - 0x48.
	volatile uint32_t JSQR;				// ADC injected sequence register.
	volatile uint32_t RESERVED3[4];		// Reserved 0x50 - 0x5C.
	union {
		struct {
			volatile uint32_t OFR1;		// ADC data offset register 1.
			volatile uint32_t OFR2;		// ADC data offset register 2.
			volatile uint32_t OFR3;		// ADC data offset register 3.
			volatile uint32_t OFR4;		// ADC data offset register 4.
		};
		volatile uint32_t OFR[4];		// ADC data offset registers 1 to 4.
	};
	volatile uint32_t RESERVED4[4];		// Reserved 0x70 - 0x7C.
	union {
		struct {
			volatile uint32_t JDR1;		// ADC injected data register 1.
			volatile uint32_t JDR2;		// ADC injected data register 2.
			volatile uint32_t JDR3;		// ADC injected data register 3.
			volatile uint32_t JDR4;		// ADC injected data register 4.
		};
		volatile uint32_t JQR[4];		// ADC injected data registers 1 to 4.
	};

	volatile uint32_t RESERVED5[4];		// Reserved 0x90 - 0x9C.
	volatile uint32_t AWD2CR;    		// ADC analog watchdog 2 configuration register.
	volatile uint32_t AWD3CR;    		// ADC analog watchdog 3 configuration register.
	volatile uint32_t RESERVED6[2];		// Reserved 0xA8 - 0xAC.
	volatile uint32_t DIFSEL;			// ADC differential mode selection register.
	volatile uint32_t CALFACT;			// ADC calibration factors registers.
	volatile uint32_t RESERVED7[2];		// Reserved 0xB8 - 0xBC.
	volatile uint32_t GCOMP;			// ADC gain compensation register.
} ADC_registers_t;

/*!******************************************************************
 * \enum ADCCR_registers_t
 * \brief ADC common registers map.
 *******************************************************************/
typedef struct {
	volatile uint32_t CSR;    	// ADC common status register.
	volatile uint32_t RESERVED;	// Reserved 0x04.
	volatile uint32_t CCR;    	// ADC common control register.
	volatile uint32_t CDR;    	// ADC common regular data register.
} ADCCR_registers_t;

#endif /* __ADC_REG_H__ */
