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
#include "power.h"
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
	MEASURE_ERROR_AC_LINE_INDEX,
	// Low level drivers errors.
	MEASURE_ERROR_BASE_ADC = 0x0100,
	MEASURE_ERROR_BASE_POWER = (MEASURE_ERROR_BASE_ADC + ADC_ERROR_BASE_LAST),
	// Last base value.
	MEASURE_ERROR_BASE_LAST = (MEASURE_ERROR_BASE_POWER + POWER_ERROR_BASE_LAST)
} MEASURE_status_t;

/*!******************************************************************
 * \enum MEASURE_channel_run_data_t
 * \brief Single channel instantaneous results structure.
 *******************************************************************/
typedef struct {
	int32_t active_power_mw;
	int32_t rms_voltage_mv;
	int32_t rms_current_ma;
	int32_t apparent_power_mva;
	int32_t power_factor;
	uint32_t number_of_samples;
} MEASURE_channel_run_data_t;

/*!******************************************************************
 * \enum MEASURE_accumulated_data_t
 * \brief Single data result structure.
 *******************************************************************/
typedef struct {
	int32_t min;
	int32_t max;
	int32_t rolling_mean;
	uint32_t number_of_samples;
} MEASURE_accumulated_data_t;

/*!******************************************************************
 * \enum MEASURE_channel_accumulated_data_t
 * \brief Single channel accumulated results structure.
 *******************************************************************/
typedef struct {
	MEASURE_accumulated_data_t active_power_mw;
	MEASURE_accumulated_data_t rms_voltage_mv;
	MEASURE_accumulated_data_t rms_current_ma;
	MEASURE_accumulated_data_t apparent_power_mva;
	MEASURE_accumulated_data_t power_factor;
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
 * \fn MEASURE_status_t MEASURE_process(void)
 * \brief Process MEASURE driver.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
MEASURE_status_t MEASURE_process(void);

/*!******************************************************************
 * \fn MEASURE_status_t MEASURE_tick(void)
 * \brief Function to call every second.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
MEASURE_status_t MEASURE_tick(void);

/*!******************************************************************
 * \fn MEASURE_status_t MEASURE_get_detect_flag(uint8_t ac_channel_index, uint8_t* current_sensor_connected)
 * \brief Get AC channel detect flag.
 * \param[in]  	ac_channel_index: AC channel index to read.
 * \param[out] 	current_sensor_connected: Pointer to the current sensor detection flag.
 * \retval		Function execution status.
 *******************************************************************/
MEASURE_status_t MEASURE_get_detect_flag(uint8_t ac_channel_index, uint8_t* current_sensor_connected);

/*!******************************************************************
 * \fn MEASURE_status_t MEASURE_get_run_data(uint8_t ac_channel_index, MEASURE_channel_run_data_t* ac_channel_data)
 * \brief Get AC channel instantaneous data.
 * \param[in]  	ac_channel_index: AC channel index to read.
 * \param[out] 	ac_channel_data: Pointer to the channel results.
 * \retval		Function execution status.
 *******************************************************************/
MEASURE_status_t MEASURE_get_run_data(uint8_t ac_channel_index, MEASURE_channel_run_data_t* ac_channel_data);

/*!******************************************************************
 * \fn MEASURE_status_t MEASURE_get_accumulated_data(uint8_t ac_channel_index, MEASURE_channel_accumulated_data_t* ac_channel_data)
 * \brief Get AC channel accumulated data.
 * \param[in]  	ac_channel_index: AC channel index to read.
 * \param[out] 	ac_channel_data: Pointer to the channel results.
 * \retval		Function execution status.
 *******************************************************************/
MEASURE_status_t MEASURE_get_accumulated_data(uint8_t ac_channel_index, MEASURE_channel_accumulated_data_t* ac_channel_data);

/*******************************************************************/
#define MEASURE_exit_error(error_base) { if (measure_status != MEASURE_SUCCESS) { status = error_base + measure_status; goto errors; } }

/*******************************************************************/
#define MEASURE_stack_error() { if (measure_status != MEASURE_SUCCESS) { ERROR_stack_add(ERROR_BASE_MEASURE + measure_status); } }

/*******************************************************************/
#define MEASURE_stack_exit_error(error_code) { if (measure_status != MEASURE_SUCCESS) { ERROR_stack_add(ERROR_BASE_MEASURE + measure_status); status = error_code; goto errors; }

#endif /* __MEASURE_H__ */
