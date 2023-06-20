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

#define ADC_NUMBER_OF_ACI_CHANNELS	4
#define ADC_SAMPLING_PERIOD_US		200

/*** ADC structures ***/

typedef enum {
	ADC_SUCCESS = 0,
	ADC_ERROR_NULL_PARAMETER,
	ADC_ERROR_CALIBRATION,
	ADC_ERROR_READY,
	ADC_ERROR_BASE_LPTIM1 = 0x0100,
	ADC_ERROR_BASE_LAST = (ADC_ERROR_BASE_LPTIM1 + LPTIM_ERROR_BASE_LAST)
} ADC_status_t;

/*** ADC functions ***/

ADC_status_t ADC_init(void);

ADC_status_t ADC_start(void);
ADC_status_t ADC_stop(void);

#define ADC_status_check(error_base) { if (adc_status != ADC_SUCCESS) { status = error_base + adc_status; goto errors; }}
#define ADC_error_check() { ERROR_status_check(adc_status, ADC_SUCCESS, ERROR_BASE_ADC); }
#define ADC_error_check_print() { ERROR_status_check_print(adc_status, ADC_SUCCESS, ERROR_BASE_ADC); }

#endif /* __ADC_H__ */
