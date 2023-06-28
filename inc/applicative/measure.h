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
#include "types.h"

/*** MEASURE structures ***/

typedef enum {
	MEASURE_SUCCESS = 0,
	MEASURE_ERROR_NULL_PARAMETER,
	MEASURE_ERROR_STATE,
	MEASURE_ERROR_ZERO_CROSS_TIMEOUT,
	MEASURE_ERROR_AC_LINE_INDEX,
	MEASURE_ERROR_BASE_ADC = 0x0100,
	MEASURE_ERROR_BASE_LAST = (MEASURE_ERROR_BASE_ADC + ADC_ERROR_BASE_LAST)
} MEASURE_status_t;

typedef struct {
	int32_t min;
	int32_t max;
	int32_t rolling_mean;
	uint32_t number_of_samples;
} MEASURE_result_t;

typedef struct {
	MEASURE_result_t active_power;
	MEASURE_result_t rms_voltage;
	MEASURE_result_t rms_current;
	MEASURE_result_t apparent_power;
	MEASURE_result_t power_factor;
} MEASURE_channel_result_t;

/*** MEASURE functions ***/

MEASURE_status_t MEASURE_init(void);
MEASURE_status_t MEASURE_task(void);

MEASURE_status_t MEASURE_get_ac_channel_data(uint8_t ac_channel_index, MEASURE_channel_result_t* ac_channel_data);

void MEASURE_increment_zero_cross_count(void);
void MEASURE_set_dma_transfer_end_flag(void);

#define MEASURE_status_check(error_base) { if (measure_status != MEASURE_SUCCESS) { status = error_base + measure_status; goto errors; }}
#define MEASURE_error_check() { ERROR_status_check(measure_status, MEASURE_SUCCESS, ERROR_BASE_MEASURE); }
#define MEASURE_error_check_print() { ERROR_status_check_print(measure_status, MEASURE_SUCCESS, ERROR_BASE_MEASURE); }

#endif /* __MEASURE_H__ */
