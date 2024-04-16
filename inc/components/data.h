/*
 * data.h
 *
 *  Created on: 16 apr. 2024
 *      Author: Ludo
 */

#ifndef __DATA_H__
#define __DATA_H__

#include "types.h"

/*** DATA macros ***/

#define DATA_SECONDS_PER_HOUR	3600

/*** DATA structures ***/

/*!******************************************************************
 * \struct MEASURE_data_t
 * \brief Single run data structure.
 *******************************************************************/
typedef struct {
	int32_t value;
	uint32_t number_of_samples;
} DATA_run_t;

/*!******************************************************************
 * \struct DATA_accumulated_t
 * \brief Single accumulated data structure.
 *******************************************************************/
typedef struct {
	int32_t min;
	int32_t max;
	int32_t rolling_mean;
	uint32_t number_of_samples;
} DATA_accumulated_t;

/*!******************************************************************
 * \struct DATA_run_channel_t
 * \brief Single channel run data structure.
 *******************************************************************/
typedef struct {
	DATA_run_t active_power_mw;
	DATA_run_t rms_voltage_mv;
	DATA_run_t rms_current_ma;
	DATA_run_t apparent_power_mva;
	DATA_run_t power_factor;
} DATA_run_channel_t;

/*!******************************************************************
 * \struct DATA_accumulated_channel_t
 * \brief Single channel accumulated data structure.
 *******************************************************************/
typedef struct {
	DATA_accumulated_t active_power_mw;
	DATA_accumulated_t rms_voltage_mv;
	DATA_accumulated_t rms_current_ma;
	DATA_accumulated_t apparent_power_mva;
	DATA_accumulated_t power_factor;
	int32_t active_energy_mwh;
	int32_t apparent_energy_mvah;
} DATA_accumulated_channel_t;

/*** DATA functions ***/

/*******************************************************************/
#define DATA_reset_run(source) { \
	source.value = 0; \
	source.number_of_samples = 0; \
}

/*******************************************************************/
#define DATA_reset_run_channel(channel) { \
	DATA_reset_run(channel.active_power_mw); \
	DATA_reset_run(channel.rms_voltage_mv); \
	DATA_reset_run(channel.rms_current_ma); \
	DATA_reset_run(channel.apparent_power_mva); \
	DATA_reset_run(channel.power_factor); \
}

/*******************************************************************/
#define DATA_reset_accumulated(result) { \
	result.min = 2147483647; \
	result.max = 0; \
	result.rolling_mean = 0; \
	result.number_of_samples = 0; \
}

/*******************************************************************/
#define DATA_reset_accumulated_channel(channel) { \
	DATA_reset_accumulated(channel.active_power_mw); \
	DATA_reset_accumulated(channel.rms_voltage_mv); \
	DATA_reset_accumulated(channel.rms_current_ma); \
	DATA_reset_accumulated(channel.apparent_power_mva); \
	DATA_reset_accumulated(channel.power_factor); \
	channel.active_energy_mwh = 0; \
	channel.apparent_energy_mvah = 0; \
}

/*******************************************************************/
#define DATA_copy_run(source, destination) { \
	destination.value = source.value; \
	destination.number_of_samples = source.number_of_samples; \
}

/*******************************************************************/
#define DATA_copy_run_channel(source, destination) { \
	DATA_copy_run(source.active_power_mw, destination.active_power_mw); \
	DATA_copy_run(source.rms_voltage_mv, destination.rms_voltage_mv); \
	DATA_copy_run(source.rms_current_ma, destination.rms_current_ma); \
	DATA_copy_run(source.apparent_power_mva, destination.apparent_power_mva); \
	DATA_copy_run(source.power_factor, destination.power_factor); \
}

/*******************************************************************/
#define DATA_copy_accumulated(source, destination) { \
	destination.min = source.min; \
	destination.max = source.max; \
	destination.rolling_mean = source.rolling_mean; \
	destination.number_of_samples = source.number_of_samples; \
}

/*******************************************************************/
#define DATA_copy_accumulated_channel(source, destination) { \
	DATA_copy_accumulated(source.active_power_mw, destination.active_power_mw); \
	DATA_copy_accumulated(source.rms_voltage_mv, destination.rms_voltage_mv); \
	DATA_copy_accumulated(source.rms_current_ma, destination.rms_current_ma); \
	DATA_copy_accumulated(source.apparent_power_mva, destination.apparent_power_mva); \
	DATA_copy_accumulated(source.power_factor, destination.power_factor); \
}

/*******************************************************************/
#define DATA_add_run_sample(result, new_sample) { \
	/* Compute rolling mean */ \
	temp_s64 = ((int64_t) result.value * (int64_t) result.number_of_samples) + (int64_t) new_sample; \
	result.value = (int32_t) ((temp_s64) / ((int64_t) (result.number_of_samples + 1))); \
	result.number_of_samples++; \
}

/*******************************************************************/
#define DATA_add_run_channel_sample(channel, result, new_sample) { \
	/* Compute rolling mean */ \
	temp_s64 = ((int64_t) channel.result.value * (int64_t) channel.result.number_of_samples) + (int64_t) new_sample; \
	channel.result.value = (int32_t) ((temp_s64) / ((int64_t) (channel.result.number_of_samples + 1))); \
	channel.result.number_of_samples++; \
}

/*******************************************************************/
#define DATA_add_accumulated_sample(result, new_sample) { \
	/* Compute absolute value of new sample */ \
	math_status = MATH_abs(new_sample, &new_sample_abs); \
	MATH_exit_error(MEASURE_ERROR_BASE_MATH); \
	/* Min */ \
	math_status = MATH_abs(result.min, &ref_abs); \
	MATH_exit_error(MEASURE_ERROR_BASE_MATH); \
	if (new_sample_abs < ref_abs) { \
		result.min = new_sample; \
	} \
	/* Max */ \
	math_status = MATH_abs(result.max, &ref_abs); \
	MATH_exit_error(MEASURE_ERROR_BASE_MATH); \
	if (new_sample_abs > ref_abs) { \
		result.max = new_sample; \
	} \
	/* Compute rolling mean */ \
	temp_s64 = ((int64_t) result.rolling_mean * (int64_t) result.number_of_samples) + (int64_t) new_sample; \
	result.rolling_mean = (int32_t) ((temp_s64) / ((int64_t) (result.number_of_samples + 1))); \
	result.number_of_samples++; \
}

/*******************************************************************/
#define DATA_add_accumulated_channel_sample(channel, result, new_sample) { \
	/* Compute absolute value of new sample */ \
	math_status = MATH_abs(new_sample, &new_sample_abs); \
	MATH_exit_error(MEASURE_ERROR_BASE_MATH); \
	/* Min */ \
	math_status = MATH_abs(channel.result.min, &ref_abs); \
	MATH_exit_error(MEASURE_ERROR_BASE_MATH); \
	if (new_sample_abs < ref_abs) { \
		channel.result.min = new_sample; \
	} \
	/* Max */ \
	math_status = MATH_abs(channel.result.max, &ref_abs); \
	MATH_exit_error(MEASURE_ERROR_BASE_MATH); \
	if (new_sample_abs > ref_abs) { \
		channel.result.max = new_sample; \
	} \
	/* Compute rolling mean */ \
	temp_s64 = ((int64_t) channel.result.rolling_mean * (int64_t) channel.result.number_of_samples) + (int64_t) new_sample; \
	channel.result.rolling_mean = (int32_t) ((temp_s64) / ((int64_t) (channel.result.number_of_samples + 1))); \
	channel.result.number_of_samples++; \
}

#endif /* __DATA_H__ */
