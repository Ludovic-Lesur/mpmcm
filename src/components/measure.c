/*
 * measure.c
 *
 *  Created on: 18 jun. 2023
 *      Author: Ludo
 */

#include "measure.h"

#include "adc.h"
#include "dma.h"
#include "dsp/basic_math_functions.h"
#include "dsp/statistics_functions.h"
#include "error.h"
#include "exti.h"
#include "gpio.h"
#include "led.h"
#include "mapping.h"
#include "math_custom.h"
#include "mode.h"
#include "nvic.h"
#include "power.h"
#include "tim.h"
#include "types.h"

/*** MEASURE local macros ***/

#define MEASURE_MAINS_PERIOD_US							20000
#define MEASURE_ZERO_CROSS_PER_PERIOD					2
#define MEASURE_ZERO_CROSS_START_THRESHOLD				((1000000 * MEASURE_ZERO_CROSS_PER_PERIOD) / (MEASURE_MAINS_PERIOD_US)) // Wait for 1 second of mains voltage presence.

#define MEASURE_TRANSFORMER_GAIN_FACTOR					10	// For (10 * mV/mV) input unit.

// Note: this factor is used to add a margin to the buffer length (more than 1 mains periods long).
// Buffer switch then is triggered by zero cross detection instead of a fixed number of samples.
#define MEASURE_PERIOD_PER_BUFFER						2

#define MEASURE_PERIOD_BUFFER_SIZE						(MEASURE_MAINS_PERIOD_US / ADC_SAMPLING_PERIOD_US)
#define MEASURE_PERIOD_ADCX_BUFFER_SIZE					(MEASURE_PERIOD_PER_BUFFER * MEASURE_PERIOD_BUFFER_SIZE)
#define MEASURE_PERIOD_ADCX_DMA_BUFFER_SIZE				(ADC_NUMBER_OF_ACI_CHANNELS * MEASURE_PERIOD_ADCX_BUFFER_SIZE)
#define MEASURE_PERIOD_ADCX_DMA_BUFFER_DEPTH			2

// ADC buffers with size out of this range are not computed.
// Warning: it limits the acceptable input frequency range around 50Hz.
#define MEASURE_PERIOD_ADCX_BUFFER_SIZE_ERROR_PERCENT	20

#define MEASURE_POWER_FACTOR_MULTIPLIER					100

#define MEASURE_Q31_SHIFT_ADC							16
#define MEASURE_Q31_SHIFT_MULT							1

#define MEASURE_ACV_FREQUENCY_SAMPLING_HZ				1000000
#define MEASURE_PERIOD_TIM2_DMA_BUFFER_SIZE				3

#define MEASURE_LED_PULSE_DURATION_MS					50
#define MEASURE_LED_PULSE_PERIOD_SECONDS				5

#define MEASURE_SECONDS_PER_HOUR						3600

#define MEASURE_TIMEOUT_COUNT							10000000

/*** MEASURE local structures ***/

/*******************************************************************/
typedef enum {
	MEASURE_STATE_STOPPED = 0,
	MEASURE_STATE_ACTIVE,
	MEASURE_STATE_LAST
} MEASURE_state_t;

/*******************************************************************/
typedef struct {
	int16_t data[MEASURE_PERIOD_ADCX_DMA_BUFFER_SIZE];
	uint16_t size;
} MEASURE_buffer_t;

/*******************************************************************/
typedef struct {
	// Raw buffers filled by ADC and DMA for 1 period.
	MEASURE_buffer_t acv[MEASURE_PERIOD_ADCX_DMA_BUFFER_DEPTH];
	uint8_t acv_read_idx;
	uint8_t acv_write_idx;
	MEASURE_buffer_t aci[MEASURE_PERIOD_ADCX_DMA_BUFFER_DEPTH];
	uint8_t aci_read_idx;
	uint8_t aci_write_idx;
	// Raw buffer filled by TIM2 and DMA.
	uint32_t tim2_ccr1[MEASURE_PERIOD_TIM2_DMA_BUFFER_SIZE];
} MEASURE_sampling_t;

/*******************************************************************/
typedef struct {
	// Factors.
	int64_t acv_factor_num;
	int64_t acv_factor_den;
	int64_t aci_factor_num[ADC_NUMBER_OF_ACI_CHANNELS];
	int64_t aci_factor_den;
	int64_t acp_factor_num[ADC_NUMBER_OF_ACI_CHANNELS];
	int64_t acp_factor_den;
	// Temporary variables for individual channel processing on 1 period.
	q31_t period_acvx_buffer_q31[MEASURE_PERIOD_ADCX_BUFFER_SIZE];
	q31_t period_acix_buffer_q31[MEASURE_PERIOD_ADCX_BUFFER_SIZE];
	q31_t period_acpx_buffer_q31[MEASURE_PERIOD_ADCX_BUFFER_SIZE];
	uint32_t period_acxx_buffer_size;
	uint32_t period_acxx_buffer_size_low_limit;
	uint32_t period_acxx_buffer_size_high_limit;
	q31_t period_active_power_q31;
	q31_t period_rms_voltage_q31;
	q31_t period_rms_current_q31;
	q31_t period_apparent_power_q31;
	q31_t period_power_factor_q31;
	// AC channels results.
	MEASURE_channel_data_t chx_rolling_mean[ADC_NUMBER_OF_ACI_CHANNELS];
	MEASURE_channel_data_t chx_run_data[ADC_NUMBER_OF_ACI_CHANNELS];
	MEASURE_channel_accumulated_data_t chx_accumulated_data[ADC_NUMBER_OF_ACI_CHANNELS];
	int64_t active_energy_mws_sum[ADC_NUMBER_OF_ACI_CHANNELS];
	int64_t apparent_energy_mvas_sum[ADC_NUMBER_OF_ACI_CHANNELS];
	// Mains frequency.
	MEASURE_data_t acv_frequency_rolling_mean;
	MEASURE_data_t acv_frequency_run_data;
	MEASURE_accumulated_data_t acv_frequency_accumulated_data;
} MEASURE_internal_data_t;

/*******************************************************************/
typedef struct {
	MEASURE_state_t state;
	uint8_t processing_enable;
	uint8_t zero_cross_count;
	uint8_t dma_transfer_end_flag;
	uint32_t tick_led_seconds_count;
} MEASURE_context_t;

/*** MEASURE local global variables ***/

static const GPIO_pin_t* MEASURE_GPIO_ACI_DETECT[ADC_NUMBER_OF_ACI_CHANNELS] = {&GPIO_ACI1_DETECT, &GPIO_ACI2_DETECT, &GPIO_ACI3_DETECT, &GPIO_ACI4_DETECT};

static volatile MEASURE_sampling_t measure_sampling;
static volatile MEASURE_internal_data_t measure_data __attribute__((section(".bss_ccmsram")));

static volatile MEASURE_context_t measure_ctx;

/*** MEASURE local functions ***/

/*******************************************************************/
#define _MEASURE_reset_data(source) { \
	source.value = 0; \
	source.number_of_samples = 0; \
}

/*******************************************************************/
#define _MEASURE_reset_accumulated_data(result) { \
	result.min = 2147483647; \
	result.max = 0; \
	result.rolling_mean = 0; \
	result.number_of_samples = 0; \
}

/*******************************************************************/
#define _MEASURE_copy_data(source, destination) { \
	destination.value = source.value; \
	destination.number_of_samples = source.number_of_samples; \
}

/*******************************************************************/
#define _MEASURE_copy_accumulated_data(source, destination) { \
	destination.min = source.min; \
	destination.max = source.max; \
	destination.rolling_mean = source.rolling_mean; \
	destination.number_of_samples = source.number_of_samples; \
}

/*******************************************************************/
#define _MEASURE_reset_chx_data(source, channel_index) { \
	_MEASURE_reset_data(source[channel_index].active_power_mw); \
	_MEASURE_reset_data(source[channel_index].rms_voltage_mv); \
	_MEASURE_reset_data(source[channel_index].rms_current_ma); \
	_MEASURE_reset_data(source[channel_index].apparent_power_mva); \
	_MEASURE_reset_data(source[channel_index].power_factor); \
}

/*******************************************************************/
#define _MEASURE_reset_chx_accumulated_data(source, channel_index) { \
	_MEASURE_reset_accumulated_data(source[channel_index].active_power_mw); \
	_MEASURE_reset_accumulated_data(source[channel_index].rms_voltage_mv); \
	_MEASURE_reset_accumulated_data(source[channel_index].rms_current_ma); \
	_MEASURE_reset_accumulated_data(source[channel_index].apparent_power_mva); \
	_MEASURE_reset_accumulated_data(source[channel_index].power_factor); \
	source[channel_index].active_energy_mwh = 0; \
	source[channel_index].apparent_energy_mvah = 0; \
}

/*******************************************************************/
#define _MEASURE_add_sample(source, new_sample) { \
	/* Compute rolling mean */ \
	temp_s64 = ((int64_t) source.value * (int64_t) source.number_of_samples) + (int64_t) new_sample; \
	source.value = (int32_t) ((temp_s64) / ((int64_t) (source.number_of_samples + 1))); \
	source.number_of_samples++; \
}

/*******************************************************************/
#define _MEASURE_add_chx_sample(source, result, new_sample) { \
	/* Compute rolling mean */ \
	temp_s64 = ((int64_t) source.result.value * (int64_t) source.result.number_of_samples) + (int64_t) new_sample; \
	source.result.value = (int32_t) ((temp_s64) / ((int64_t) (source.result.number_of_samples + 1))); \
	source.result.number_of_samples++; \
}

/*******************************************************************/
#define _MEASURE_add_accumulated_sample(source, new_sample) { \
	/* Compute absolute value of new sample */ \
	math_status = MATH_abs(new_sample, &new_sample_abs); \
	MATH_exit_error(MEASURE_ERROR_BASE_MATH); \
	/* Min */ \
	math_status = MATH_abs(source.min, &ref_abs); \
	MATH_exit_error(MEASURE_ERROR_BASE_MATH); \
	if (new_sample_abs < ref_abs) { \
		source.min = new_sample; \
	} \
	/* Max */ \
	math_status = MATH_abs(source.max, &ref_abs); \
	MATH_exit_error(MEASURE_ERROR_BASE_MATH); \
	if (new_sample_abs > ref_abs) { \
		source.max = new_sample; \
	} \
	/* Compute rolling mean */ \
	temp_s64 = ((int64_t) source.rolling_mean * (int64_t) source.number_of_samples) + (int64_t) new_sample; \
	source.rolling_mean = (int32_t) ((temp_s64) / ((int64_t) (source.number_of_samples + 1))); \
	source.number_of_samples++; \
}

/*******************************************************************/
#define _MEASURE_add_chx_accumulated_sample(source, result, new_sample) { \
	/* Compute absolute value of new sample */ \
	math_status = MATH_abs(new_sample, &new_sample_abs); \
	MATH_exit_error(MEASURE_ERROR_BASE_MATH); \
	/* Min */ \
	math_status = MATH_abs(source.result.min, &ref_abs); \
	MATH_exit_error(MEASURE_ERROR_BASE_MATH); \
	if (new_sample_abs < ref_abs) { \
		source.result.min = new_sample; \
	} \
	/* Max */ \
	math_status = MATH_abs(source.result.max, &ref_abs); \
	MATH_exit_error(MEASURE_ERROR_BASE_MATH); \
	if (new_sample_abs > ref_abs) { \
		source.result.max = new_sample; \
	} \
	/* Compute rolling mean */ \
	temp_s64 = ((int64_t) source.result.rolling_mean * (int64_t) source.result.number_of_samples) + (int64_t) new_sample; \
	source.result.rolling_mean = (int32_t) ((temp_s64) / ((int64_t) (source.result.number_of_samples + 1))); \
	source.result.number_of_samples++; \
}

/*******************************************************************/
static void _MEASURE_compute_factors(void) {
	// Local variables.
	uint8_t chx_idx = 0;
	// ACV.
	measure_data.acv_factor_num = ((int64_t) MPMCM_TRANSFORMER_GAIN * (int64_t) MPMCM_TRANSFORMER_ATTEN * (int64_t) ADC_VREF_MV);
	measure_data.acv_factor_den = ((int64_t) MEASURE_TRANSFORMER_GAIN_FACTOR * (int64_t) ADC_FULL_SCALE);
	// ACI.
	for (chx_idx=0 ; chx_idx<ADC_NUMBER_OF_ACI_CHANNELS ; chx_idx++) {
		measure_data.aci_factor_num[chx_idx] = ((int64_t) MPMCM_SCT013_GAIN[chx_idx] * (int64_t) MPMCM_SCT013_ATTEN[chx_idx] * (int64_t) ADC_VREF_MV);
	}
	measure_data.aci_factor_den = (int64_t) ADC_FULL_SCALE;
	// ACP.
	for (chx_idx=0 ; chx_idx<ADC_NUMBER_OF_ACI_CHANNELS ; chx_idx++) {
		// Note: 1000 factor is used to get mW from mV and mA.
		// Conversion is done here to limit numerator value and avoid overflow during power computation.
		// There is no precision loss since ACV and ACI factors multiplication is necesarily a multiple of 1000 thanks to ADC_VREF_MV.
		measure_data.acp_factor_num[chx_idx] = (measure_data.acv_factor_num * measure_data.aci_factor_num[chx_idx]) / ((int64_t) 1000);
	}
	measure_data.acp_factor_den = (measure_data.acv_factor_den * measure_data.aci_factor_den);
	// Buffer size limits.
	measure_data.period_acxx_buffer_size_low_limit =  ((100 - MEASURE_PERIOD_ADCX_BUFFER_SIZE_ERROR_PERCENT) * MEASURE_PERIOD_BUFFER_SIZE) / (100);
	measure_data.period_acxx_buffer_size_high_limit = ((100 + MEASURE_PERIOD_ADCX_BUFFER_SIZE_ERROR_PERCENT) * MEASURE_PERIOD_BUFFER_SIZE) / (100);
}

/*******************************************************************/
static void _MEASURE_reset(void) {
	// Local variables.
	uint8_t chx_idx = 0;
	uint32_t idx0 = 0;
	uint32_t idx1 = 0;
	// Reset indexes.
	measure_sampling.acv_write_idx = 0;
	measure_sampling.acv_read_idx = 0;
	measure_sampling.aci_write_idx = 0;
	measure_sampling.aci_read_idx = 0;
	// Reset flags.
	measure_ctx.zero_cross_count = 0;
	measure_ctx.dma_transfer_end_flag = 0;
	// Reset sampling buffers.
	for (idx0=0 ; idx0<MEASURE_PERIOD_ADCX_DMA_BUFFER_DEPTH ; idx0++) {
		for (idx1=0 ; idx1<MEASURE_PERIOD_ADCX_DMA_BUFFER_SIZE ; idx1++) {
			measure_sampling.acv[idx0].data[idx1] = 0;
			measure_sampling.aci[idx0].data[idx1] = 0;
		}
	}
	// Reset channels data.
	for (chx_idx=0 ; chx_idx<ADC_NUMBER_OF_ACI_CHANNELS ; chx_idx++) {
		// Clear all data.
		_MEASURE_reset_chx_data(measure_data.chx_rolling_mean, chx_idx);
		_MEASURE_reset_chx_data(measure_data.chx_run_data, chx_idx);
		_MEASURE_reset_chx_accumulated_data(measure_data.chx_accumulated_data, chx_idx);
	}
	// Reset frequency data.
	_MEASURE_reset_data(measure_data.acv_frequency_rolling_mean);
	_MEASURE_reset_data(measure_data.acv_frequency_run_data);
	_MEASURE_reset_accumulated_data(measure_data.acv_frequency_accumulated_data);
	// Reset sampling buffers.
	for (idx1=0 ; idx1<MEASURE_PERIOD_TIM2_DMA_BUFFER_SIZE ; idx1++) {
		measure_sampling.tim2_ccr1[idx1] = 0;
	}
	// Set DMA address.
	DMA1_adcx_set_destination_address((uint32_t) &(measure_sampling.acv[measure_sampling.acv_write_idx].data), (uint32_t) &(measure_sampling.aci[measure_sampling.aci_write_idx].data), MEASURE_PERIOD_ADCX_DMA_BUFFER_SIZE);
}

/*******************************************************************/
MEASURE_status_t _MEASURE_start_analog_transfer(void) {
	// Local variables.
	MEASURE_status_t status = MEASURE_SUCCESS;
	ADC_status_t adc_status = ADC_SUCCESS;
	// Start ADC.
	adc_status = ADC_start();
	ADC_exit_error(MEASURE_ERROR_BASE_ADC);
	// Start DMA.
	DMA1_adcx_start();
	// Start trigger.
	TIM6_start();
errors:
	return status;
}

/*******************************************************************/
MEASURE_status_t _MEASURE_stop_analog_transfer(void) {
	// Local variables.
	MEASURE_status_t status = MEASURE_SUCCESS;
	ADC_status_t adc_status = ADC_SUCCESS;
	// Stop trigger.
	TIM6_stop();
	// Stop DMA.
	DMA1_adcx_stop();
	// Stop ADC.
	adc_status = ADC_stop();
	ADC_exit_error(MEASURE_ERROR_BASE_ADC);
errors:
	return status;
}

/*******************************************************************/
MEASURE_status_t _MEASURE_start(void) {
	// Local variables.
	MEASURE_status_t status = MEASURE_SUCCESS;
	// Start frequency measurement timer.
	TIM2_start();
	DMA1_tim2_start();
	// Start analog measurements.
	status = _MEASURE_start_analog_transfer();
	// Update flag.
	measure_ctx.processing_enable = 1;
	return status;
}

/*******************************************************************/
MEASURE_status_t _MEASURE_stop(void) {
	// Local variables.
	MEASURE_status_t status = MEASURE_SUCCESS;
	// Update flag.
	measure_ctx.processing_enable = 0;
	// Stop frequency measurement timer.
	DMA1_tim2_stop();
	TIM2_stop();
	// Start analog measurements.
	status = _MEASURE_stop_analog_transfer();
	return status;
}

/*******************************************************************/
static MEASURE_status_t _MEASURE_switch_dma_buffer(void) {
	// Local variables.
	MEASURE_status_t status = MEASURE_SUCCESS;
	// Stop ADC and DMA.
	status = _MEASURE_stop_analog_transfer();
	if (status != MEASURE_SUCCESS) goto errors;
	// Retrieve number of transfered data.
	DMA1_adcx_get_number_of_transfered_data((uint16_t*) &(measure_sampling.acv[measure_sampling.acv_write_idx].size), (uint16_t*) &(measure_sampling.aci[measure_sampling.acv_write_idx].size));
	// Update write indexes.
	measure_sampling.acv_write_idx = ((measure_sampling.acv_write_idx + 1) % MEASURE_PERIOD_ADCX_DMA_BUFFER_DEPTH);
	measure_sampling.aci_write_idx = ((measure_sampling.aci_write_idx + 1) % MEASURE_PERIOD_ADCX_DMA_BUFFER_DEPTH);
	// Set new address.
	DMA1_adcx_set_destination_address((uint32_t) &(measure_sampling.acv[measure_sampling.acv_write_idx].data), (uint32_t) &(measure_sampling.aci[measure_sampling.aci_write_idx].data), MEASURE_PERIOD_ADCX_DMA_BUFFER_SIZE);
	// Restart DMA.
	status = _MEASURE_start_analog_transfer();
errors:
	return status;
}

/*******************************************************************/
static void _MEASURE_compute_period_data(void) {
	// Local variables.
	uint32_t acv_buffer_size = 0;
	uint32_t aci_buffer_size = 0;
	uint8_t chx_idx = 0;
	int32_t active_power_mw = 0;
	q31_t mean_voltage_q31 = 0;
	int32_t rms_voltage_mv = 0;
	q31_t mean_current_q31 = 0;
	int32_t rms_current_ma = 0;
	int32_t apparent_power_mva = 0;
	int32_t power_factor = 0;
	uint32_t tim2_ccr1_delta = 0;
	int32_t frequency_mhz = 0;
	int64_t temp_s64 = 0;
	uint32_t idx = 0;
	// Check enable flag.
	if (measure_ctx.processing_enable == 0) goto errors;
	// Get size.
	acv_buffer_size = (uint32_t) ((measure_sampling.acv[measure_sampling.acv_read_idx].size) / (ADC_NUMBER_OF_ACI_CHANNELS));
	aci_buffer_size = (uint32_t) ((measure_sampling.aci[measure_sampling.acv_read_idx].size) / (ADC_NUMBER_OF_ACI_CHANNELS));
	// Take the minimum size between voltage and current.
	measure_data.period_acxx_buffer_size = (acv_buffer_size < aci_buffer_size) ? acv_buffer_size : aci_buffer_size;
	// Check size.
	if ((measure_data.period_acxx_buffer_size < measure_data.period_acxx_buffer_size_low_limit) ||
		(measure_data.period_acxx_buffer_size > measure_data.period_acxx_buffer_size_high_limit)) {
		goto errors;
	}
	// Processing each channel.
	for (chx_idx=0 ; chx_idx<ADC_NUMBER_OF_ACI_CHANNELS ; chx_idx++) {
		// Compute channel buffer.
		for (idx=0 ; idx<(measure_data.period_acxx_buffer_size) ; idx++) {
			// Copy samples by channel and convert to Q31 type.
			measure_data.period_acvx_buffer_q31[idx] = (measure_sampling.acv[measure_sampling.acv_read_idx].data[(ADC_NUMBER_OF_ACI_CHANNELS * idx) + chx_idx]) << MEASURE_Q31_SHIFT_ADC;
			measure_data.period_acix_buffer_q31[idx] = (measure_sampling.aci[measure_sampling.aci_read_idx].data[(ADC_NUMBER_OF_ACI_CHANNELS * idx) + chx_idx]) << MEASURE_Q31_SHIFT_ADC;
			// Force current to 0 if sensor is not connected.
			if (GPIO_read(MEASURE_GPIO_ACI_DETECT[chx_idx]) == 0) {
				measure_data.period_acix_buffer_q31[idx] = 0;
			}
		}
		// Mean voltage and current.
		arm_mean_q31((q31_t*) measure_data.period_acvx_buffer_q31, measure_data.period_acxx_buffer_size, &mean_voltage_q31);
		arm_mean_q31((q31_t*) measure_data.period_acix_buffer_q31, measure_data.period_acxx_buffer_size, &mean_current_q31);
		// DC removal.
		for (idx=0 ; idx<(measure_data.period_acxx_buffer_size) ; idx++) {
			measure_data.period_acvx_buffer_q31[idx] -= mean_voltage_q31;
			measure_data.period_acix_buffer_q31[idx] -= mean_current_q31;
		}
		// Instantaneous power.
		arm_mult_q31((q31_t*) measure_data.period_acvx_buffer_q31, (q31_t*) measure_data.period_acix_buffer_q31, (q31_t*) measure_data.period_acpx_buffer_q31, measure_data.period_acxx_buffer_size);
		// Active power.
		arm_mean_q31((q31_t*) measure_data.period_acpx_buffer_q31, measure_data.period_acxx_buffer_size, (q31_t*) &(measure_data.period_active_power_q31));
		temp_s64 = (int64_t) (measure_data.period_active_power_q31 >> MEASURE_Q31_SHIFT_MULT);
		temp_s64 *= measure_data.acp_factor_num[chx_idx];
		active_power_mw = (int32_t) (temp_s64 / measure_data.acp_factor_den);
		// RMS voltage.
		arm_rms_q31((q31_t*) measure_data.period_acvx_buffer_q31, measure_data.period_acxx_buffer_size, (q31_t*) &(measure_data.period_rms_voltage_q31));
		temp_s64 = measure_data.acv_factor_num * (int64_t) (measure_data.period_rms_voltage_q31 >> MEASURE_Q31_SHIFT_ADC);
		rms_voltage_mv = (int32_t) (temp_s64 / measure_data.acv_factor_den);
		// RMS current.
		arm_rms_q31((q31_t*) measure_data.period_acix_buffer_q31, measure_data.period_acxx_buffer_size, (q31_t*) &(measure_data.period_rms_current_q31));
		temp_s64 = measure_data.aci_factor_num[chx_idx] * (int64_t) (measure_data.period_rms_current_q31 >> MEASURE_Q31_SHIFT_ADC);
		rms_current_ma = (int32_t) (temp_s64 / measure_data.aci_factor_den);
		// Apparent power.
		temp_s64 = ((int64_t) rms_voltage_mv) * ((int64_t) rms_current_ma);
		apparent_power_mva = (int32_t) ((temp_s64) / ((int64_t) 1000));
		// Power factor.
		temp_s64 = (int64_t) MEASURE_POWER_FACTOR_MULTIPLIER * ((int64_t) active_power_mw);
		power_factor = (apparent_power_mva != 0) ? (int32_t) ((temp_s64) / ((int64_t) apparent_power_mva)) : 0;
		// Update accumulated data.
		_MEASURE_add_chx_sample(measure_data.chx_rolling_mean[chx_idx], active_power_mw, active_power_mw);
		_MEASURE_add_chx_sample(measure_data.chx_rolling_mean[chx_idx], rms_voltage_mv, rms_voltage_mv);
		_MEASURE_add_chx_sample(measure_data.chx_rolling_mean[chx_idx], rms_current_ma, rms_current_ma);
		_MEASURE_add_chx_sample(measure_data.chx_rolling_mean[chx_idx], apparent_power_mva, apparent_power_mva);
		_MEASURE_add_chx_sample(measure_data.chx_rolling_mean[chx_idx], power_factor, power_factor);
	}
	// Compute mains frequency.
	idx = 0;
	while (idx < MEASURE_PERIOD_TIM2_DMA_BUFFER_SIZE) {
		// Search two valid consecutive samples.
		if (measure_sampling.tim2_ccr1[idx] < measure_sampling.tim2_ccr1[(idx + 1) % MEASURE_PERIOD_TIM2_DMA_BUFFER_SIZE]) {
			// Compute delta.
			tim2_ccr1_delta	= measure_sampling.tim2_ccr1[(idx + 1) % MEASURE_PERIOD_TIM2_DMA_BUFFER_SIZE] - measure_sampling.tim2_ccr1[idx];
			// Avoid rollover case (clamp to 1Hz).
			if (tim2_ccr1_delta < MEASURE_ACV_FREQUENCY_SAMPLING_HZ) break;
		}
		idx++;
	}
	// Check index.
	if (idx < MEASURE_PERIOD_TIM2_DMA_BUFFER_SIZE) {
		// Compute mains frequency.
		temp_s64 = (((int64_t) MEASURE_ACV_FREQUENCY_SAMPLING_HZ * 1000) / ((int64_t) tim2_ccr1_delta));
		frequency_mhz = (int32_t) temp_s64;
		// Update accumulated data.
		_MEASURE_add_sample(measure_data.acv_frequency_rolling_mean, frequency_mhz);
	}
errors:
	// Update read indexes.
	measure_sampling.acv_read_idx = ((measure_sampling.acv_read_idx + 1) % MEASURE_PERIOD_ADCX_DMA_BUFFER_DEPTH);
	measure_sampling.aci_read_idx = ((measure_sampling.aci_read_idx + 1) % MEASURE_PERIOD_ADCX_DMA_BUFFER_DEPTH);
}

/*******************************************************************/
static void _MEASURE_compute_run_data(void) {
	// Local variables.
	uint8_t chx_idx = 0;
	// Compute AC channels run data.
	for (chx_idx=0 ; chx_idx<ADC_NUMBER_OF_ACI_CHANNELS ; chx_idx++) {
		// Copy all rolling means.
		_MEASURE_copy_data(measure_data.chx_rolling_mean[chx_idx].active_power_mw, measure_data.chx_run_data[chx_idx].active_power_mw);
		_MEASURE_copy_data(measure_data.chx_rolling_mean[chx_idx].rms_voltage_mv, measure_data.chx_run_data[chx_idx].rms_voltage_mv);
		_MEASURE_copy_data(measure_data.chx_rolling_mean[chx_idx].rms_current_ma, measure_data.chx_run_data[chx_idx].rms_current_ma);
		_MEASURE_copy_data(measure_data.chx_rolling_mean[chx_idx].apparent_power_mva, measure_data.chx_run_data[chx_idx].apparent_power_mva);
		_MEASURE_copy_data(measure_data.chx_rolling_mean[chx_idx].power_factor, measure_data.chx_run_data[chx_idx].power_factor);
		// Reset results.
		_MEASURE_reset_chx_data(measure_data.chx_rolling_mean, chx_idx);
	}
	// Compute frequency run data.
	measure_data.acv_frequency_run_data.value = measure_data.acv_frequency_rolling_mean.value;
	measure_data.acv_frequency_run_data.number_of_samples = measure_data.acv_frequency_rolling_mean.number_of_samples;
	// Reset results.
	_MEASURE_reset_data(measure_data.acv_frequency_rolling_mean);
}

/*******************************************************************/
static MEASURE_status_t _MEASURE_compute_accumulated_data(void) {
	// Local variables.
	MEASURE_status_t status = MEASURE_SUCCESS;
	MATH_status_t math_status = MATH_SUCCESS;
	uint32_t new_sample_abs = 0;
	uint32_t ref_abs = 0;
	uint8_t chx_idx = 0;
	int64_t temp_s64 = 0;
	// Compute AC channels accumulated data.
	for (chx_idx=0 ; chx_idx<ADC_NUMBER_OF_ACI_CHANNELS ; chx_idx++) {
		// Copy all rolling means.
		_MEASURE_add_chx_accumulated_sample(measure_data.chx_accumulated_data[chx_idx], active_power_mw, measure_data.chx_run_data[chx_idx].active_power_mw.value);
		_MEASURE_add_chx_accumulated_sample(measure_data.chx_accumulated_data[chx_idx], rms_voltage_mv, measure_data.chx_run_data[chx_idx].rms_voltage_mv.value);
		_MEASURE_add_chx_accumulated_sample(measure_data.chx_accumulated_data[chx_idx], rms_current_ma, measure_data.chx_run_data[chx_idx].rms_current_ma.value);
		_MEASURE_add_chx_accumulated_sample(measure_data.chx_accumulated_data[chx_idx], apparent_power_mva, measure_data.chx_run_data[chx_idx].apparent_power_mva.value);
		_MEASURE_add_chx_accumulated_sample(measure_data.chx_accumulated_data[chx_idx], power_factor, measure_data.chx_run_data[chx_idx].power_factor.value);
		// Increment energy.
		measure_data.active_energy_mws_sum[chx_idx] += measure_data.chx_run_data[chx_idx].active_power_mw.value;
		measure_data.apparent_energy_mvas_sum[chx_idx] += measure_data.chx_run_data[chx_idx].apparent_power_mva.value;
		// Reset results.
		_MEASURE_reset_chx_data(measure_data.chx_rolling_mean, chx_idx);
	}
	// Compute frequency accumulated data.
	_MEASURE_add_accumulated_sample(measure_data.acv_frequency_accumulated_data, measure_data.acv_frequency_run_data.value);
errors:
	return status;
}

/*******************************************************************/
static MEASURE_status_t _MEASURE_led_single_pulse(void) {
	// Local variables.
	MEASURE_status_t status = MEASURE_SUCCESS;
	LED_status_t led_status = LED_SUCCESS;
	LED_color_t led_color = LED_COLOR_OFF;
	// Check LED period.
	if (measure_ctx.tick_led_seconds_count >= MEASURE_LED_PULSE_PERIOD_SECONDS) {
		// Reset count.
		measure_ctx.tick_led_seconds_count = 0;
		// Compute LED color according to state.
		if (measure_ctx.state == MEASURE_STATE_STOPPED) {
			// Check current number of samples (CH1 RMS voltage as reference).
			led_color = (measure_data.chx_accumulated_data[0].rms_voltage_mv.number_of_samples == 0) ? LED_COLOR_RED : LED_COLOR_YELLOW;
		}
		else {
			led_color = LED_COLOR_GREEN;
		}
		// Perform LED pulse.
		led_status = LED_single_pulse(MEASURE_LED_PULSE_DURATION_MS, led_color);
		LED_exit_error(MEASURE_ERROR_BASE_LED);
	}
errors:
	return status;
}

/*******************************************************************/
static MEASURE_status_t _MEASURE_internal_process(void) {
	// Local variables.
	MEASURE_status_t status = MEASURE_SUCCESS;
	// Perform state machine.
	switch (measure_ctx.state) {
	case MEASURE_STATE_STOPPED:
		// Synchronize on zero cross.
		if (measure_ctx.zero_cross_count >= MEASURE_ZERO_CROSS_START_THRESHOLD) {
			// Reset context.
			_MEASURE_reset();
			// Start measure.
			status = _MEASURE_start();
			if (status != MEASURE_SUCCESS) goto errors;
			// Update state.
			measure_ctx.state = MEASURE_STATE_ACTIVE;
		}
		break;
	case MEASURE_STATE_ACTIVE:
		// Check zero cross count.
		if (measure_ctx.zero_cross_count >= MEASURE_ZERO_CROSS_PER_PERIOD) {
			// Clear counters.
			measure_ctx.zero_cross_count = 0;
			measure_ctx.dma_transfer_end_flag = 0;
			// Switch to next buffer.
			status = _MEASURE_switch_dma_buffer();
			if (status != MEASURE_SUCCESS) goto errors;
			// Compute data.
			_MEASURE_compute_period_data();
		}
		// Check DMA transfer end flag.
		if (measure_ctx.dma_transfer_end_flag != 0) {
			// Clear counters.
			measure_ctx.zero_cross_count = 0;
			measure_ctx.dma_transfer_end_flag = 0;
			// Stop measure.
			_MEASURE_stop();
			// Update state.
			measure_ctx.state = MEASURE_STATE_STOPPED;
		}
		break;
	default:
		status = MEASURE_ERROR_STATE;
		goto errors;
	}
errors:
	return status;
}

/*******************************************************************/
static void _MEASURE_increment_zero_cross_count(void) {
	// Local variables.
	MEASURE_status_t measure_status = MEASURE_SUCCESS;
	// Increment counts.
	measure_ctx.zero_cross_count++;
	// Process measure.
	measure_status = _MEASURE_internal_process();
	MEASURE_stack_error();
}

/*******************************************************************/
static void _MEASURE_set_dma_transfer_end_flag(void) {
	// Local variables.
	MEASURE_status_t measure_status = MEASURE_SUCCESS;
	// Set local flag.
	measure_ctx.dma_transfer_end_flag = 1;
	// Process measure.
	measure_status = _MEASURE_internal_process();
	MEASURE_stack_error();
}

/*** MEASURE functions ***/

/*******************************************************************/
MEASURE_status_t MEASURE_init(void) {
	// Local variables.
	MEASURE_status_t status = MEASURE_SUCCESS;
	POWER_status_t power_status = POWER_SUCCESS;
	uint8_t chx_idx = 0;
	// Init context.
	measure_ctx.state = MEASURE_STATE_STOPPED;
	measure_ctx.tick_led_seconds_count = 0;
	// Init energy data.
	for (chx_idx=0 ; chx_idx<ADC_NUMBER_OF_ACI_CHANNELS ; chx_idx++) {
		measure_data.active_energy_mws_sum[chx_idx] = 0;
		measure_data.apparent_energy_mvas_sum[chx_idx] = 0;
	}
	// Init detect pins.
	for (chx_idx=0 ; chx_idx<ADC_NUMBER_OF_ACI_CHANNELS ; chx_idx++) {
		GPIO_configure(MEASURE_GPIO_ACI_DETECT[chx_idx], GPIO_MODE_INPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	}
	// Turn analog front-end on to have VREF+ for ADC calibration.
	power_status = POWER_enable(POWER_DOMAIN_ANALOG, LPTIM_DELAY_MODE_ACTIVE);
	POWER_exit_error(MEASURE_ERROR_BASE_POWER);
	// Init RGB LED.
	LED_init();
	// Init zero cross pulse GPIO.
	GPIO_configure(&GPIO_ZERO_CROSS_PULSE, GPIO_MODE_INPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	EXTI_configure_gpio(&GPIO_ZERO_CROSS_PULSE, EXTI_TRIGGER_RISING_EDGE, &_MEASURE_increment_zero_cross_count);
	NVIC_enable_interrupt(NVIC_INTERRUPT_EXTI2, NVIC_PRIORITY_EXTI2);
	// Init frequency measurement timer and ADC timer.
	TIM2_init(MEASURE_ACV_FREQUENCY_SAMPLING_HZ);
	TIM6_init();
	// Init ADC DMA.
	DMA1_adcx_init(&_MEASURE_set_dma_transfer_end_flag);
	DMA1_adcx_set_destination_address((uint32_t) &(measure_sampling.acv[measure_sampling.acv_write_idx].data), (uint32_t) &(measure_sampling.aci[measure_sampling.aci_write_idx].data), MEASURE_PERIOD_ADCX_DMA_BUFFER_SIZE);
	// Init timer DMA.
	DMA1_tim2_init();
	DMA1_tim2_set_destination_address((uint32_t) &(measure_sampling.tim2_ccr1), MEASURE_PERIOD_TIM2_DMA_BUFFER_SIZE);
errors:
	_MEASURE_compute_factors();
	_MEASURE_reset();
	return status;
}

/*******************************************************************/
MEASURE_status_t MEASURE_tick_second(void) {
	// Local variables.
	MEASURE_status_t status = MEASURE_SUCCESS;
	MEASURE_status_t measure_status = MEASURE_SUCCESS;
	// Increment seconds count.
	measure_ctx.tick_led_seconds_count++;
	// Check state.
	if (measure_ctx.state != MEASURE_STATE_STOPPED) {
		// Compute run data from last second.
		measure_ctx.processing_enable = 0;
		_MEASURE_compute_run_data();
		measure_ctx.processing_enable = 1;
		// Compute accumulated data.
		measure_status = _MEASURE_compute_accumulated_data();
		MEASURE_stack_error();
	}
	// Manage LED.
	status = _MEASURE_led_single_pulse();
	return status;
}

/*******************************************************************/
MEASURE_status_t MEASURE_get_probe_detect_flag(uint8_t ac_channel_index, uint8_t* current_sensor_connected) {
	// Local variables.
	MEASURE_status_t status = MEASURE_SUCCESS;
	// Check parameters.
	if (ac_channel_index >= ADC_NUMBER_OF_ACI_CHANNELS) {
		status = MEASURE_ERROR_AC_CHANNEL;
		goto errors;
	}
	if (current_sensor_connected == NULL) {
		status = MEASURE_ERROR_NULL_PARAMETER;
		goto errors;
	}
	// Update flag.
	(*current_sensor_connected) = GPIO_read(MEASURE_GPIO_ACI_DETECT[ac_channel_index]);
errors:
	return status;
}

/*******************************************************************/
MEASURE_status_t MEASURE_get_mains_detect_flag(uint8_t* mains_voltage_detected) {
	// Local variables.
	MEASURE_status_t status = MEASURE_SUCCESS;
	// Check parameters.
	if (mains_voltage_detected == NULL) {
		status = MEASURE_ERROR_NULL_PARAMETER;
		goto errors;
	}
	// Update flag.
	(*mains_voltage_detected) = (measure_ctx.state == MEASURE_STATE_STOPPED) ? 0 : 1;
errors:
	return status;
}

/*******************************************************************/
MEASURE_status_t MEASURE_get_run_data(MEASURE_data_type_t data_type, MEASURE_data_t* run_data) {
	// Local variables.
	MEASURE_status_t status = MEASURE_SUCCESS;
	// Check parameters.
	if (run_data == NULL) {
		status = MEASURE_ERROR_NULL_PARAMETER;
		goto errors;
	}
	// Check data type.
	switch (data_type) {
	case MEASURE_DATA_TYPE_MAINS_FREQUENCY:
		// Copy data.
		(run_data -> value) = measure_data.acv_frequency_run_data.value;
		(run_data -> number_of_samples) = measure_data.acv_frequency_run_data.number_of_samples;
		break;
	default:
		status = MEASURE_ERROR_DATA_TYPE;
		goto errors;
	}
errors:
	return status;
}

/*******************************************************************/
MEASURE_status_t MEASURE_get_accumulated_data(MEASURE_data_type_t data_type, MEASURE_accumulated_data_t* accumulated_data) {
	// Local variables.
	MEASURE_status_t status = MEASURE_SUCCESS;
	// Check parameters.
	if (accumulated_data == NULL) {
		status = MEASURE_ERROR_NULL_PARAMETER;
		goto errors;
	}
	// Check data type.
	switch (data_type) {
	case MEASURE_DATA_TYPE_MAINS_FREQUENCY:
		// Copy data.
		_MEASURE_copy_accumulated_data(measure_data.acv_frequency_accumulated_data, (*accumulated_data));
		// Reset data.
		_MEASURE_reset_accumulated_data(measure_data.acv_frequency_accumulated_data);
		break;
	default:
		status = MEASURE_ERROR_DATA_TYPE;
		goto errors;
	}
errors:
	return status;
}

/*******************************************************************/
MEASURE_status_t MEASURE_get_channel_run_data(uint8_t ac_channel, MEASURE_channel_data_t* ac_channel_run_data) {
	// Local variables.
	MEASURE_status_t status = MEASURE_SUCCESS;
	// Check parameters.
	if (ac_channel_run_data == NULL) {
		status = MEASURE_ERROR_NULL_PARAMETER;
		goto errors;
	}
	if (ac_channel >= ADC_NUMBER_OF_ACI_CHANNELS) {
		status = MEASURE_ERROR_AC_CHANNEL;
		goto errors;
	}
	// Copy data.
	_MEASURE_copy_data(measure_data.chx_run_data[ac_channel].active_power_mw, (ac_channel_run_data -> active_power_mw));
	_MEASURE_copy_data(measure_data.chx_run_data[ac_channel].rms_voltage_mv, (ac_channel_run_data -> rms_voltage_mv));
	_MEASURE_copy_data(measure_data.chx_run_data[ac_channel].rms_current_ma, (ac_channel_run_data -> rms_current_ma));
	_MEASURE_copy_data(measure_data.chx_run_data[ac_channel].apparent_power_mva, (ac_channel_run_data -> apparent_power_mva));
	_MEASURE_copy_data(measure_data.chx_run_data[ac_channel].power_factor, (ac_channel_run_data -> power_factor));
errors:
	return status;
}

/*******************************************************************/
MEASURE_status_t MEASURE_get_channel_accumulated_data(uint8_t ac_channel, MEASURE_channel_accumulated_data_t* ac_channel_accumulated_data) {
	// Local variables.
	MEASURE_status_t status = MEASURE_SUCCESS;
	// Check parameters.
	if (ac_channel_accumulated_data == NULL) {
		status = MEASURE_ERROR_NULL_PARAMETER;
		goto errors;
	}
	if (ac_channel >= ADC_NUMBER_OF_ACI_CHANNELS) {
		status = MEASURE_ERROR_AC_CHANNEL;
		goto errors;
	}
	// Copy data.
	_MEASURE_copy_accumulated_data(measure_data.chx_accumulated_data[ac_channel].active_power_mw, (ac_channel_accumulated_data -> active_power_mw));
	_MEASURE_copy_accumulated_data(measure_data.chx_accumulated_data[ac_channel].rms_voltage_mv, (ac_channel_accumulated_data -> rms_voltage_mv));
	_MEASURE_copy_accumulated_data(measure_data.chx_accumulated_data[ac_channel].rms_current_ma, (ac_channel_accumulated_data -> rms_current_ma));
	_MEASURE_copy_accumulated_data(measure_data.chx_accumulated_data[ac_channel].apparent_power_mva, (ac_channel_accumulated_data -> apparent_power_mva));
	_MEASURE_copy_accumulated_data(measure_data.chx_accumulated_data[ac_channel].power_factor, (ac_channel_accumulated_data -> power_factor));
	// Compute energy.
	(ac_channel_accumulated_data -> active_energy_mwh) = (int32_t) ((measure_data.active_energy_mws_sum[ac_channel]) / ((uint64_t) MEASURE_SECONDS_PER_HOUR));
	(ac_channel_accumulated_data -> apparent_energy_mvah) = (int32_t) ((measure_data.apparent_energy_mvas_sum[ac_channel]) / ((uint64_t) MEASURE_SECONDS_PER_HOUR));
	// Reset data.
	_MEASURE_reset_chx_accumulated_data(measure_data.chx_accumulated_data, ac_channel);
	measure_data.active_energy_mws_sum[ac_channel] = 0;
	measure_data.apparent_energy_mvas_sum[ac_channel] = 0;
errors:
	return status;
}
