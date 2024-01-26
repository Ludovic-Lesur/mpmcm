/*
 * measure.h
 *
 *  Created on: 18 jun. 2023
 *      Author: Ludo
 */

#ifndef __MEASURE_H__
#define __MEASURE_H__

#include "adc.h"
#include "dma.h"
#include "led.h"
#include "math_custom.h"
#include "power.h"
#include "tim.h"
#include "types.h"

/*** MEASURE structures ***/

/*!******************************************************************
 * \enum MEASURE_status_t
 * \brief MEASURE driver error codes.
 *******************************************************************/
typedef enum {
	// Driver errors.
	MEASURE_SUCCESS = 0,
	MEASURE_ERROR_NULL_PARAMETER,
	MEASURE_ERROR_STATE,
	MEASURE_ERROR_ZERO_CROSS_TIMEOUT,
	MEASURE_ERROR_DATA_TYPE,
	MEASURE_ERROR_AC_CHANNEL,
	// Low level drivers errors.
	MEASURE_ERROR_BASE_ADC = 0x0100,
	MEASURE_ERROR_BASE_TIM2 = (MEASURE_ERROR_BASE_ADC + ADC_ERROR_BASE_LAST),
	MEASURE_ERROR_BASE_TIM6 = (MEASURE_ERROR_BASE_TIM2 + TIM_ERROR_BASE_LAST),
	MEASURE_ERROR_BASE_MATH = (MEASURE_ERROR_BASE_TIM6 + TIM_ERROR_BASE_LAST),
	MEASURE_ERROR_BASE_POWER = (MEASURE_ERROR_BASE_MATH + MATH_ERROR_BASE_LAST),
	MEASURE_ERROR_BASE_LED = (MEASURE_ERROR_BASE_POWER + POWER_ERROR_BASE_LAST),
	// Last base value.
	MEASURE_ERROR_BASE_LAST = (MEASURE_ERROR_BASE_LED + LED_ERROR_BASE_LAST)
} MEASURE_status_t;

/*!******************************************************************
 * \enum MEASURE_data_type_t
 * \brief MEASURE single data list.
 *******************************************************************/
typedef enum {
	MEASURE_DATA_TYPE_MAINS_FREQUENCY = 0,
	MEASURE_DATA_TYPE_LAST
} MEASURE_data_type_t;

/*!******************************************************************
 * \struct MEASURE_channel_data_t
 * \brief Single channel run data structure.
 *******************************************************************/
typedef struct {
	int32_t value;
	uint32_t number_of_samples;
} MEASURE_data_t;

/*!******************************************************************
 * \struct MEASURE_channel_data_t
 * \brief Single channel run data structure.
 *******************************************************************/
typedef struct {
	MEASURE_data_t active_power_mw;
	MEASURE_data_t rms_voltage_mv;
	MEASURE_data_t rms_current_ma;
	MEASURE_data_t apparent_power_mva;
	MEASURE_data_t power_factor;
} MEASURE_channel_data_t;

/*!******************************************************************
 * \struct MEASURE_accumulated_data_t
 * \brief Single data result structure.
 *******************************************************************/
typedef struct {
	int32_t min;
	int32_t max;
	int32_t rolling_mean;
	uint32_t number_of_samples;
} MEASURE_accumulated_data_t;

/*!******************************************************************
 * \struct MEASURE_channel_accumulated_data_t
 * \brief Single channel accumulated data structure.
 *******************************************************************/
typedef struct {
	MEASURE_accumulated_data_t active_power_mw;
	MEASURE_accumulated_data_t rms_voltage_mv;
	MEASURE_accumulated_data_t rms_current_ma;
	MEASURE_accumulated_data_t apparent_power_mva;
	MEASURE_accumulated_data_t power_factor;
	int32_t active_energy_mwh;
	int32_t apparent_energy_mvah;
} MEASURE_channel_accumulated_data_t;

/*** MEASURE functions ***/

/*!******************************************************************
 * \fn MEASURE_status_t MEASURE_init(void)
 * \brief Init MEASURE driver.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
MEASURE_status_t MEASURE_init(void);

/*!******************************************************************
 * \fn MEASURE_status_t MEASURE_tick_second(void)
 * \brief Function to call every second.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
MEASURE_status_t MEASURE_tick_second(void);

/*!******************************************************************
 * \fn MEASURE_status_t MEASURE_get_probe_detect_flag(uint8_t ac_channel_index, uint8_t* current_probe_connected)
 * \brief Get AC channel detect flag.
 * \param[in]  	ac_channel_index: AC channel index to read.
 * \param[out] 	current_probe_connected: Pointer to the current probe detection flag.
 * \retval		Function execution status.
 *******************************************************************/
MEASURE_status_t MEASURE_get_probe_detect_flag(uint8_t ac_channel_index, uint8_t* current_probe_connected);

/*!******************************************************************
 * \fn MEASURE_status_t MEASURE_get_mains_detect_flag(uint8_t mains_voltage_detected)
 * \brief Get mains voltage detect flag.
 * \param[in]  	none
 * \param[out] 	mains_voltage_detected: Pointer to the mains voltage detection flag.
 * \retval		Function execution status.
 *******************************************************************/
MEASURE_status_t MEASURE_get_mains_detect_flag(uint8_t* mains_voltage_detected);

/*!******************************************************************
 * \fn MEASURE_status_t MEASURE_get_run_data(MEASURE_data_type_t data_type, MEASURE_data_t* run_data)
 * \brief Get run data.
 * \param[in]  	data_type: Data to read.
 * \param[out] 	data: Pointer to the run data.
 * \retval		Function execution status.
 *******************************************************************/
MEASURE_status_t MEASURE_get_run_data(MEASURE_data_type_t data_type, MEASURE_data_t* run_data);

/*!******************************************************************
 * \fn MEASURE_status_t MEASURE_get_accumulated_data(MEASURE_data_type_t data_type, MEASURE_accumulated_data_t* accumulated_data)
 * \brief Get accumulated data.
 * \param[in]  	data_type: Data to read.
 * \param[out] 	data: Pointer to the accumulated data.
 * \retval		Function execution status.
 *******************************************************************/
MEASURE_status_t MEASURE_get_accumulated_data(MEASURE_data_type_t data_type, MEASURE_accumulated_data_t* accumulated_data);

/*!******************************************************************
 * \fn MEASURE_status_t MEASURE_get_channel_run_data(uint8_t ac_channel, MEASURE_channel_data_t* ac_channel_run_data)
 * \brief Get AC channel run data.
 * \param[in]  	ac_channel_index: AC channel index to read.
 * \param[out] 	ac_channel_data: Pointer to the channel results.
 * \retval		Function execution status.
 *******************************************************************/
MEASURE_status_t MEASURE_get_channel_run_data(uint8_t ac_channel, MEASURE_channel_data_t* ac_channel_run_data);

/*!******************************************************************
 * \fn MEASURE_status_t MEASURE_get_channel_accumulated_data(uint8_t ac_channel, MEASURE_channel_accumulated_data_t* ac_channel_accumulated_data)
 * \brief Get AC channel accumulated data.
 * \param[in]  	ac_channel_index: AC channel index to read.
 * \param[out] 	ac_channel_data: Pointer to the channel results.
 * \retval		Function execution status.
 *******************************************************************/
MEASURE_status_t MEASURE_get_channel_accumulated_data(uint8_t ac_channel, MEASURE_channel_accumulated_data_t* ac_channel_accumulated_data);

/*******************************************************************/
#define MEASURE_exit_error(error_base) { if (measure_status != MEASURE_SUCCESS) { status = error_base + measure_status; goto errors; } }

/*******************************************************************/
#define MEASURE_stack_error() { if (measure_status != MEASURE_SUCCESS) { ERROR_stack_add(ERROR_BASE_MEASURE + measure_status); } }

/*******************************************************************/
#define MEASURE_stack_exit_error(error_code) { if (measure_status != MEASURE_SUCCESS) { ERROR_stack_add(ERROR_BASE_MEASURE + measure_status); status = error_code; goto errors; }

#endif /* __MEASURE_H__ */
