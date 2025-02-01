/*
 * adc.h
 *
 *  Created on: 17 jun. 2023
 *      Author: Ludo
 */

#ifndef __ADC_H__
#define __ADC_H__

#include "lptim.h"
#include "types.h"

/*** ADC macros ***/

#define ADC_VREF_MV					2500
#define ADC_RESOLUTION_BITS			12
#define ADC_FULL_SCALE				((1 << ADC_RESOLUTION_BITS) - 1)

#define ADC_NUMBER_OF_ACI_CHANNELS	4
#define ADC_SAMPLING_PERIOD_US		200

/*** ADC structures ***/

/*!******************************************************************
 * \enum ADC_status_t
 * \brief ADC driver error codes.
 *******************************************************************/
typedef enum {
	// Driver errors.
	ADC_SUCCESS = 0,
	ADC_ERROR_NULL_PARAMETER,
	ADC_ERROR_DISABLE_TIMEOUT,
	ADC_ERROR_CALIBRATION,
	ADC_ERROR_READY_TIMEOUT,
	// Low level drivers errors.
	ADC_ERROR_BASE_LPTIM1 = 0x0100,
	// Last base value.
	ADC_ERROR_BASE_LAST = (ADC_ERROR_BASE_LPTIM1 + LPTIM_ERROR_BASE_LAST)
} ADC_status_t;

/*** ADC functions ***/

/*!******************************************************************
 * \fn ADC_status_t ADC_init(void)
 * \brief Init ADC peripheral.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
ADC_status_t ADC_init(void);

/*!******************************************************************
 * \fn ADC_status_t ADC_de_init(void)
 * \brief Release ADC peripheral.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
ADC_status_t ADC_de_init(void);

/*!******************************************************************
 * \fn ADC_status_t ADC_start(void)
 * \brief Start ADC continuous conversions.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
ADC_status_t ADC_start(void);

/*!******************************************************************
 * \fn ADC_status_t ADC_stop(void)
 * \brief Stop ADC continuous conversions.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
ADC_status_t ADC_stop(void);

/*******************************************************************/
#define ADC_exit_error(error_base) { if (adc_status != ADC_SUCCESS) { status = (error_base + adc_status); goto errors; } }

/*******************************************************************/
#define ADC_stack_error(void) { if (adc1_status != ADC_SUCCESS) { ERROR_stack_add(ERROR_BASE_ADC + adc_status); } }

/*******************************************************************/
#define ADC_stack_exit_error(error_code) { if (adc_status != ADC_SUCCESS) { ERROR_stack_add(ERROR_BASE_ADC + adc_status); status = error_code; goto errors; }

#endif /* __ADC_H__ */
