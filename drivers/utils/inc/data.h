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
 * \struct DATA_run_t
 * \brief Single run data structure.
 *******************************************************************/
typedef struct {
	float64_t value;
	uint32_t number_of_samples;
} DATA_run_t;

/*!******************************************************************
 * \struct DATA_accumulated_t
 * \brief Single accumulated data structure.
 *******************************************************************/
typedef struct {
	float64_t min;
	float64_t max;
	float64_t rolling_mean;
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
	DATA_run_t active_energy_mwh;
	DATA_run_t apparent_energy_mvah;
} DATA_accumulated_channel_t;

/*** DATA functions ***/

/*******************************************************************/
#define DATA_reset_run(source) { \
	source.value = 0.0; \
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
#define DATA_reset_accumulated(data) { \
	data.min = 1.7976931348623157e308L; \
	data.max = 0.0; \
	data.rolling_mean = 0.0; \
	data.number_of_samples = 0; \
}

/*******************************************************************/
#define DATA_reset_accumulated_channel(channel) { \
	DATA_reset_accumulated(channel.active_power_mw); \
	DATA_reset_accumulated(channel.rms_voltage_mv); \
	DATA_reset_accumulated(channel.rms_current_ma); \
	DATA_reset_accumulated(channel.apparent_power_mva); \
	DATA_reset_accumulated(channel.power_factor); \
	DATA_reset_run(channel.active_energy_mwh); \
	DATA_reset_run(channel.apparent_energy_mvah); \
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
	DATA_copy_run(source.active_energy_mwh, destination.active_energy_mwh); \
	DATA_copy_run(source.apparent_energy_mvah, destination.apparent_energy_mvah); \
}

/*******************************************************************/
#define DATA_add_run_sample(data, sample) { \
	/* Compute rolling mean */ \
	MATH_rolling_mean(data.value, data.number_of_samples, sample, float64_t); \
}

/*******************************************************************/
#define DATA_add_run_channel_sample(channel, data, sample) { \
	/* Compute rolling mean */ \
	MATH_rolling_mean(channel.data.value, channel.data.number_of_samples, sample, float64_t); \
}

/*******************************************************************/
#define DATA_add_accumulated_sample(data, source) { \
	/* Check source data */ \
	if (source.number_of_samples > 0) { \
		/* Compute absolute value of new sample */ \
		MATH_abs(source.value, sample_abs, float64_t); \
		/* Min */ \
		MATH_abs(data.min, ref_abs, float64_t); \
		if (sample_abs < ref_abs) { \
			data.min = source.value; \
		} \
		/* Max */ \
		MATH_abs(data.max, ref_abs, float64_t); \
		if (sample_abs > ref_abs) { \
			data.max = source.value; \
		} \
		/* Compute rolling mean */ \
		MATH_rolling_mean(data.rolling_mean, data.number_of_samples, source.value, float64_t); \
	} \
}

/*******************************************************************/
#define DATA_add_accumulated_channel_sample(channel, data, source) { \
	/* Check source data */ \
	if (source.number_of_samples > 0) { \
		/* Compute absolute value of new sample */ \
		MATH_abs(source.value, sample_abs, float64_t); \
		/* Min */ \
		MATH_abs(channel.data.min, ref_abs, float64_t); \
		if (sample_abs < ref_abs) { \
			channel.data.min = source.value; \
		} \
		/* Max */ \
		MATH_abs(channel.data.max, ref_abs, float64_t); \
		if (sample_abs > ref_abs) { \
			channel.data.max = source.value; \
		} \
		/* Compute rolling mean */ \
		MATH_rolling_mean(channel.data.rolling_mean, channel.data.number_of_samples, source.value, float64_t); \
	} \
}

#endif /* __DATA_H__ */
