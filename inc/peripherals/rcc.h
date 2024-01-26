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

#define RCC_LSE_FREQUENCY_HZ	32768

/*** RCC structures ***/

/*!******************************************************************
 * \enum RCC_status_t
 * \brief RCC driver error codes.
 *******************************************************************/
typedef enum {
	// Driver errors.
	RCC_SUCCESS = 0,
	RCC_ERROR_NULL_PARAMETER,
	RCC_ERROR_CLOCK,
	RCC_ERROR_HSI_READY,
	RCC_ERROR_HSI_SWITCH,
	RCC_ERROR_HSE_READY,
	RCC_ERROR_PLL_READY,
	RCC_ERROR_PLL_SWITCH,
	RCC_ERROR_LSE_READY,
	RCC_ERROR_HSI_CALIBRATION,
	RCC_ERROR_LSI_CALIBRATION,
	// Low level drivers errors.
	RCC_ERROR_BASE_FLASH = 0x0100,
	// Last base value.
	RCC_ERROR_BASE_LAST = (RCC_ERROR_BASE_FLASH + FLASH_ERROR_BASE_LAST)
} RCC_status_t;

/*!******************************************************************
 * \enum RCC_clock_t
 * \brief RCC clocks list.
 *******************************************************************/
typedef enum {
	RCC_CLOCK_LSI = 0,
	RCC_CLOCK_LSE,
	RCC_CLOCK_HSI,
	RCC_CLOCK_HSE,
	RCC_CLOCK_PLL,
	RCC_CLOCK_SYSTEM,
	RCC_CLOCK_LAST
} RCC_clock_t;

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

/*!******************************************************************
 * \fn RCC_status_t RCC_calibrate(void)
 * \brief Measure internal oscillators frequency with external reference clock.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
RCC_status_t RCC_calibrate(void);

/*!******************************************************************
 * \fn RCC_status_t RCC_get_frequency_hz(RCC_clock_t clock, uint32_t* frequency_hz)
 * \brief Get clock frequency.
 * \param[in]  	clock: Clock to read.
 * \param[out] 	frequency_hz: Pointer to the clock frequency in Hz.
 * \retval		Function execution status.
 *******************************************************************/
RCC_status_t RCC_get_frequency_hz(RCC_clock_t clock, uint32_t* frequency_hz);

/*!******************************************************************
 * \fn RCC_status_t RCC_get_status(RCC_clock_t clock, uint32_t* clock_is_ready
 * \brief Get clock status.
 * \param[in]  	clock: Clock to read.
 * \param[out] 	clock_is_ready: Pointer to the clock status (1 if the clock is running correctly, 0 otherwise).
 * \retval		Function execution status.
 *******************************************************************/
RCC_status_t RCC_get_status(RCC_clock_t clock, uint8_t* clock_is_ready);

/*******************************************************************/
#define RCC_exit_error(error_base) { if (rcc_status != RCC_SUCCESS) { status = (error_base + rcc_status); goto errors; } }

/*******************************************************************/
#define RCC_stack_error(void) { if (rcc_status != RCC_SUCCESS) { ERROR_stack_add(ERROR_BASE_RCC + rcc_status); } }

/*******************************************************************/
#define RCC_stack_exit_error(error_code) { if (rcc_status != RCC_SUCCESS) { ERROR_stack_add(ERROR_BASE_RCC + rcc_status); status = error_code; goto errors; } }

#endif /* __RCC_H__ */
