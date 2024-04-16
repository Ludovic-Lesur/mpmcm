/*
 * tic.h
 *
 *  Created on: 09 apr. 2024
 *      Author: Ludo
 */

#ifndef __TIC_H__
#define __TIC_H__

#include "data.h"
#include "led.h"
#include "math_custom.h"
#include "mode.h"
#include "power.h"
#include "types.h"
#include "usart.h"

/*** TIC macros ***/

#define TIC_SAMPLING_PERIOD_DEFAULT_SECONDS		10

/*** TIC structures ***/

/*!******************************************************************
 * \enum TIC_status_t
 * \brief TIC driver error codes.
 *******************************************************************/
typedef enum {
	// Driver errors.
	TIC_SUCCESS = 0,
	TIC_ERROR_NULL_PARAMETER,
	TIC_ERROR_STATE,
	TIC_ERROR_DATA_INDEX,
	TIC_ERROR_SAMPLING_PERIOD_UNDERFLOW,
	TIC_ERROR_SAMPLING_PERIOD_OVERFLOW,
	// Low level drivers errors.
	TIC_ERROR_BASE_USART2 = 0x0100,
	TIC_ERROR_BASE_MATH = (TIC_ERROR_BASE_USART2 + USART_ERROR_BASE_LAST),
	TIC_ERROR_BASE_POWER = (TIC_ERROR_BASE_MATH + MATH_ERROR_BASE_LAST),
	TIC_ERROR_BASE_LED = (TIC_ERROR_BASE_POWER + POWER_ERROR_BASE_LAST),
	// Last base value.
	TIC_ERROR_BASE_LAST = (TIC_ERROR_BASE_LED + LED_ERROR_BASE_LAST)
} TIC_status_t;

/*!******************************************************************
 * \enum TIC_state_t
 * \brief TIC states list.
 *******************************************************************/
typedef enum {
	TIC_STATE_OFF = 0,
	TIC_STATE_ACTIVE,
	TIC_STATE_LAST
} TIC_state_t;

/*** TIC functions ***/

/*!******************************************************************
 * \fn TIC_status_t TIC_init(void)
 * \brief Init TIC driver.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
TIC_status_t TIC_init(void);

#ifdef LINKY_TIC_ENABLE
/*!******************************************************************
 * \fn TIC_status_t TIC_process(void)
 * \brief Process TIC driver.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
TIC_status_t TIC_process(void);
#endif

/*!******************************************************************
 * \fn TIC_state_t TIC_get_state(void)
 * \brief Read TIC driver state.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		TIC driver state.
 *******************************************************************/
TIC_state_t TIC_get_state(void);

/*!******************************************************************
 * \fn TIC_status_t TIC_set_sampling_period(uint32_t period_seconds)
 * \brief Set TIC data sampling period.
 * \param[in]  	period_seconds: Sampling period in seconds.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
TIC_status_t TIC_set_sampling_period(uint32_t period_seconds);

#ifdef LINKY_TIC_ENABLE
/*!******************************************************************
 * \fn TIC_status_t TIC_tick_second(void)
 * \brief Function to call every second.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void TIC_tick_second(void);
#endif

/*!******************************************************************
 * \fn TIC_status_t TIC_get_detect_flag(uint8_t* linky_tic_connected)
 * \brief Get Linky TIC detect flag.
 * \param[in]  	none
 * \param[out] 	linky_tic_connected: Pointer to the Linky TIC detection flag.
 * \retval		Function execution status.
 *******************************************************************/
TIC_status_t TIC_get_detect_flag(uint8_t* linky_tic_connected);

/*!******************************************************************
 * \fn TIC_status_t TIC_get_channel_run_data(DATA_run_channel_t* channel_run_data)
 * \brief Get run data.
 * \param[in]  	none
 * \param[out] 	channel_run_data: Pointer to the TIC channel run data.
 * \retval		Function execution status.
 *******************************************************************/
TIC_status_t TIC_get_channel_run_data(DATA_run_channel_t* channel_run_data);

/*!******************************************************************
 * \fn TIC_status_t TIC_get_channel_accumulated_data(TIC_accumulated_data_t* channel_accumulated_data)
 * \brief Get accumulated data.
 * \param[in]  	none
 * \param[out] 	channel_accumulated_data: Pointer to the TIC channel accumulated data.
 * \retval		Function execution status.
 *******************************************************************/
TIC_status_t TIC_get_channel_accumulated_data(DATA_accumulated_channel_t* channel_accumulated_data);

/*******************************************************************/
#define TIC_exit_error(error_base) { if (tic_status != TIC_SUCCESS) { status = error_base + tic_status; goto errors; } }

/*******************************************************************/
#define TIC_stack_error() { if (tic_status != TIC_SUCCESS) { ERROR_stack_add(ERROR_BASE_TIC + tic_status); } }

/*******************************************************************/
#define TIC_stack_exit_error(error_code) { if (tic_status != TIC_SUCCESS) { ERROR_stack_add(ERROR_BASE_TIC + tic_status); status = error_code; goto errors; }

#endif /* __TIC_H__ */
