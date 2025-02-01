/*
 * flash_reg.h
 *
 *  Created on: 31 dec. 2022
 *      Author: Ludo
 */

#ifndef __FLASH_REG_H__
#define __FLASH_REG_H__

#include "types.h"

/*** FLASH REG macros ***/

// Peripheral base address.
#define FLASH	((FLASH_registers_t*) ((uint32_t) 0x40022000))

/*** FLASH REG structures ***/

/*!******************************************************************
 * \enum FLASH_registers_t
 * \brief FLASH registers map.
 *******************************************************************/
typedef struct {
	volatile uint32_t ACR;				// FLASH interface access control register.
	volatile uint32_t PDKEYR;			// FLASH interface power down key register.
	volatile uint32_t KEYR;				// FLASH interface key register.
	volatile uint32_t OPTKEYR;			// FLASH interface option key register.
	volatile uint32_t SR;    			// FLASH interface status register.
	volatile uint32_t CR;    			// FLASH interface control register.
	volatile uint32_t ECCR;				// FLASH interface ECC register.
	volatile uint32_t RESERVED0;		// Reserved 0x1C.
	volatile uint32_t OPTR;				// FLASH interface option register.
	volatile uint32_t PCROP1SR;			// FLASH interface PCROP1 start address register.
	volatile uint32_t PCROP1ER;			// FLASH interface PCROP1 end address register.
	volatile uint32_t WRP1AR;			// FLASH interface bank 1 WRP area A address register.
	volatile uint32_t WRP1BR;			// FLASH interface bank 1 WRP area B address register.
#ifdef MCU_CATEGORY_3
	volatile uint32_t RESERVED1[4];		// Reserved 0x34 - 0x40.
	volatile uint32_t PCROP2SR;			// FLASH interface PCROP2 start address register.
	volatile uint32_t PCROP2ER;			// FLASH interface PCROP2 end address register.
	volatile uint32_t WRP2AR;			// FLASH interface bank 2 WRP area A address register.
	volatile uint32_t WRP2BR;			// FLASH interface bank 2 WRP area B address register.
	volatile uint32_t RESERVED2[7];		// Reserved 0x54 - 0x6C.
#else
	volatile uint32_t RESERVED1[15];	// Reserved 0x34 - 0x6C.
#endif
	volatile uint32_t SEC1R;			// FLASH interface securable area bank 1 register.
#ifdef MCU_CATEGORY_3
	volatile uint32_t SEC2R;			// FLASH interface securable area bank 2 register.
#else
	volatile uint32_t RESERVED3;		// Reserved 0x74.
#endif
} FLASH_registers_t;

#endif /* __FLASH_REG_H__ */
