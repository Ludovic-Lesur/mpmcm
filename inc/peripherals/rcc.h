/*
 * rcc.h
 *
 *  Created on: 16 jun. 2023
 *      Author: Ludo
 */

#ifndef __RCC_H__
#define __RCC_H__

#include "flash.h"
#include "types.h"

/*** RCC macros ***/

#define RCC_LSI_FREQUENCY_HZ		38000
#define RCC_LSE_FREQUENCY_HZ		32768
#define RCC_HSI_FREQUENCY_KHZ		16000
#define RCC_SYSCLK_FREQUENCY_KHZ	120000

/*** RCC structures ***/

/*!******************************************************************
 * \enum RCC_status_t
 * \brief RCC driver error codes.
 *******************************************************************/
typedef enum {
	// Driver errors.
	RCC_SUCCESS = 0,
	RCC_ERROR_HSE_READY,
	RCC_ERROR_PLL_READY,
	RCC_ERROR_PLL_SWITCH,
	// Low level drivers errors.
	RCC_ERROR_BASE_FLASH = 0x0100,
	// Last base value.
	RCC_ERROR_BASE_LAST = (RCC_ERROR_BASE_FLASH + FLASH_ERROR_BASE_LAST)
} RCC_status_t;

/*** RCC functions ***/

/*!******************************************************************
 * \fn void RCC_init(void)
 * \brief Init MCU default clock tree.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void RCC_init(void);

/*!******************************************************************
 * \fn RCC_status_t RCC_switch_to_pll(void)
 * \brief Switch system clock to 120MHz PLL.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
RCC_status_t RCC_switch_to_pll(void);

/*******************************************************************/
#define RCC_exit_error(error_base) { if (rcc_status != RCC_SUCCESS) { status = (error_base + rcc_status); goto errors; } }

/*******************************************************************/
#define RCC_stack_error(void) { if (rcc_status != RCC_SUCCESS) { ERROR_stack_add(ERROR_BASE_RCC + rcc_status); } }

/*******************************************************************/
#define RCC_stack_exit_error(error_code) { if (rcc_status != RCC_SUCCESS) { ERROR_stack_add(ERROR_BASE_RCC + rcc_status); status = error_code; goto errors; } }

#endif /* __RCC_H__ */
