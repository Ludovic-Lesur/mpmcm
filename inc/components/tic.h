/*
 * tic.h
 *
 *  Created on: 09 apr. 2024
 *      Author: Ludo
 */

#ifndef __TIC_H__
#define __TIC_H__

#include "math_custom.h"
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
	// Last base value.
	TIC_ERROR_BASE_LAST = 0x0100
} TIC_status_t;

/*!******************************************************************
 * \enum TIC_data_index_t
 * \brief TIC single data list.
 *******************************************************************/
typedef enum {
	TIC_DATA_INDEX_APPARENT_POWER_MVA = 0,
	TIC_DATA_INDEX_LAST
} TIC_data_index_t;

/*!******************************************************************
 * \struct TIC_data_t
 * \brief Single run data structure.
 *******************************************************************/
typedef int32_t TIC_data_t;

/*!******************************************************************
 * \struct TIC_accumulated_data_t
 * \brief Single data result structure.
 *******************************************************************/
typedef struct {
	int32_t min;
	int32_t max;
	int32_t rolling_mean;
	uint32_t number_of_samples;
} TIC_accumulated_data_t;

/*** TIC functions ***/

/*!******************************************************************
 * \fn TIC_status_t TIC_init(void)
 * \brief Init TIC driver.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
TIC_status_t TIC_init(void);

/*!******************************************************************
 * \fn TIC_status_t TIC_set_sampling_period(uint32_t period_seconds)
 * \brief Set TIC data sampling period.
 * \param[in]  	period_seconds: Sampling period in seconds.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
TIC_status_t TIC_set_sampling_period(uint32_t period_seconds);

/*!******************************************************************
 * \fn TIC_status_t TIC_tick_second(void)
 * \brief Function to call every second.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
TIC_status_t TIC_tick_second(void);

/*!******************************************************************
 * \fn TIC_status_t TIC_get_detect_flag(uint8_t* linky_tic_connected)
 * \brief Get Linky TIC detect flag.
 * \param[in]  	none
 * \param[out] 	linky_tic_connected: Pointer to the Linky TIC detection flag.
 * \retval		Function execution status.
 *******************************************************************/
TIC_status_t TIC_get_detect_flag(uint8_t* linky_tic_connected);

/*!******************************************************************
 * \fn TIC_status_t TIC_get_run_data(TIC_data_index_t data_index, TIC_data_t* run_data)
 * \brief Get run data.
 * \param[in]  	data_index: Data to read.
 * \param[out] 	run_data: Pointer to the run data.
 * \retval		Function execution status.
 *******************************************************************/
TIC_status_t TIC_get_run_data(TIC_data_index_t data_index, TIC_data_t* run_data);

/*!******************************************************************
 * \fn TIC_status_t TIC_get_accumulated_data(TIC_data_index_t data_index, TIC_accumulated_data_t* accumulated_data)
 * \brief Get accumulated data.
 * \param[in]  	data_index: Data to read.
 * \param[out] 	accumulated_data: Pointer to the accumulated data.
 * \retval		Function execution status.
 *******************************************************************/
TIC_status_t TIC_get_accumulated_data(TIC_data_index_t data_index, TIC_accumulated_data_t* accumulated_data);

/*******************************************************************/
#define TIC_exit_error(error_base) { if (tic_status != TIC_SUCCESS) { status = error_base + tic_status; goto errors; } }

/*******************************************************************/
#define TIC_stack_error() { if (tic_status != TIC_SUCCESS) { ERROR_stack_add(ERROR_BASE_TIC + tic_status); } }

/*******************************************************************/
#define TIC_stack_exit_error(error_code) { if (tic_status != TIC_SUCCESS) { ERROR_stack_add(ERROR_BASE_TIC + tic_status); status = error_code; goto errors; }

#endif /* __TIC_H__ */
