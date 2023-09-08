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
#include "exti.h"
#include "gpio.h"
#include "mapping.h"
#include "mode.h"
#include "nvic.h"
#include "power.h"
#include "tim.h"
#include "types.h"

/*** MEASURE local macros ***/

#define MEASURE_MAINS_PERIOD_US				20000
#define MEASURE_ZERO_CROSS_PER_PERIOD		2

#define MEASURE_TRANSFORMER_GAIN			30 // Unit mV/mV.
#define MEASURE_TRANSFORMER_ATTEN			11 // Unit mV/mV.
#define MEASURE_ACV_FACTOR_NUM				((int64_t) MEASURE_TRANSFORMER_GAIN * (int64_t) MEASURE_TRANSFORMER_ATTEN * (int64_t) ADC_VREF_MV)
#define MEASURE_ACV_FACTOR_DEN				((int64_t) ADC_FULL_SCALE)

#define MEASURE_SCT013_ATTEN				1 // Unit mV/mV
#define MEASURE_ACI_FACTOR_NUM				((int64_t) SCT013_GAIN[ac_channel_idx] * (int64_t) MEASURE_SCT013_ATTEN * (int64_t) ADC_VREF_MV)
#define MEASURE_ACI_FACTOR_DEN				((int64_t) ADC_FULL_SCALE)

#define MEASURE_ACP_FACTOR_NUM				(MEASURE_ACV_FACTOR_NUM * MEASURE_ACI_FACTOR_NUM)
#define MEASURE_ACP_FACTOR_DEN				(MEASURE_ACV_FACTOR_DEN * MEASURE_ACI_FACTOR_DEN * (int64_t) 1000) // To get mW from mV and mA.

// Note: factor 2 is used to add a margin to the buffer length (2 mains periods long instead of 1).
// Buffer switch is triggered by the zero cross detection instead of a fixed number of samples.
#define MEASURE_PERIOD_BUFFER_SIZE			(2 * (MEASURE_MAINS_PERIOD_US / ADC_SAMPLING_PERIOD_US))

#define MEASURE_PERIOD_DMA_BUFFER_SIZE		(ADC_NUMBER_OF_ACI_CHANNELS * MEASURE_PERIOD_BUFFER_SIZE)
#define MEASURE_PERIOD_DMA_BUFFER_DEPTH		2

#define MEASURE_POWER_FACTOR_MULTIPLIER		1000

#define MEASURE_Q31_SHIFT_ADC				16
#define MEASURE_Q31_SHIFT_MULT				1

#define MEASURE_TIMEOUT_COUNT				10000000

//#define MEASURE_CONTINUOUS

/*** MEASURE local structures ***/

/*******************************************************************/
typedef enum {
	MEASURE_STATE_STOPPED = 0,
	MEASURE_STATE_IDLE,
	MEASURE_STATE_PERIOD_PROCESS,
	MEASURE_STATE_LAST
} MEASURE_state_t;

/*******************************************************************/
typedef struct {
	int16_t data[MEASURE_PERIOD_DMA_BUFFER_SIZE];
	uint16_t size;
} MEASURE_buffer_t;

/*******************************************************************/
typedef struct {
	// Raw buffers filled by ADC and DMA for 1 period.
	MEASURE_buffer_t acv[MEASURE_PERIOD_DMA_BUFFER_DEPTH];
	uint8_t acv_read_idx;
	uint8_t acv_write_idx;
	MEASURE_buffer_t aci[MEASURE_PERIOD_DMA_BUFFER_DEPTH];
	uint8_t aci_read_idx;
	uint8_t aci_write_idx;
} MEASURE_sampling_t;

/*******************************************************************/
typedef struct {
	// Temporary variables for individual channel processing on 1 period.
	q31_t period_acvx_buffer_q31[MEASURE_PERIOD_BUFFER_SIZE];
	q31_t period_acix_buffer_q31[MEASURE_PERIOD_BUFFER_SIZE];
	q31_t period_acpx_buffer_q31[MEASURE_PERIOD_BUFFER_SIZE];
	uint32_t period_acxx_buffer_size;
	q31_t period_active_power_q31;
	q31_t period_rms_voltage_q31;
	q31_t period_rms_current_q31;
	q31_t period_apparent_power_q31;
	q31_t period_power_factor_q31;
	// Results.
	MEASURE_channel_result_t result[ADC_NUMBER_OF_ACI_CHANNELS];
} MEASURE_data_t;

/*******************************************************************/
typedef struct {
	MEASURE_state_t state;
	uint8_t zero_cross_count;
	uint8_t dma_transfer_end_flag;
} MEASURE_context_t;

/*** MEASURE local global variables ***/

static const GPIO_pin_t* MEASURE_GPIO_ACI_DETECT[ADC_NUMBER_OF_ACI_CHANNELS] = {&GPIO_ACI1_DETECT, &GPIO_ACI2_DETECT, &GPIO_ACI3_DETECT, &GPIO_ACI4_DETECT};

static volatile MEASURE_sampling_t measure_sampling;
static MEASURE_data_t measure_data __attribute__((section(".bss_ccmsram")));

static volatile MEASURE_context_t measure_ctx;

/*** MEASURE local functions ***/

/*******************************************************************/
#define _MEASURE_reset_result(result) { \
	result.min = 0x7FFFFFFF; \
	result.max = 0; \
	result.rolling_mean = 0; \
	result.number_of_samples = 0; \
}

/*******************************************************************/
#define _MEASURE_copy_result(source, destination) { \
	destination.min = source.min; \
	destination.max = source.max; \
	destination.rolling_mean = source.rolling_mean; \
	destination.number_of_samples = source.number_of_samples; \
}

/*******************************************************************/
#define _MEASURE_result_add_sample(result, new_sample) { \
	/* Min */ \
	if (new_sample < result.min) { \
		result.min = new_sample; \
	} \
	/* Max */ \
	if (new_sample > result.max) { \
		result.max = new_sample; \
	} \
	/* Rolling mean */ \
	temp_s64 = ((int64_t) result.rolling_mean * (int64_t) result.number_of_samples) + (int64_t) new_sample; \
	result.rolling_mean = (int32_t) ((temp_s64) / ((int64_t) (result.number_of_samples + 1))); \
	result.number_of_samples++; \
}

/*******************************************************************/
static void _MEASURE_increment_zero_cross_count(void) {
	// Set local flag.
	measure_ctx.zero_cross_count++;
}

/*******************************************************************/
static void _MEASURE_set_dma_transfer_end_flag(void) {
	// Set local flag.
	measure_ctx.dma_transfer_end_flag = 1;
}

/*******************************************************************/
static void _MEASURE_reset(void) {
	// Reset indexes.
	measure_sampling.acv_write_idx = 0;
	measure_sampling.acv_read_idx = 0;
	measure_sampling.aci_write_idx = 0;
	measure_sampling.aci_read_idx = 0;
	// Reset flags.
	measure_ctx.zero_cross_count = 0;
	measure_ctx.dma_transfer_end_flag = 0;
	// Set DMA address.
	DMA1_set_destination_address((uint32_t) &(measure_sampling.acv[measure_sampling.acv_write_idx].data), (uint32_t) &(measure_sampling.aci[measure_sampling.aci_write_idx].data), MEASURE_PERIOD_DMA_BUFFER_SIZE);
}

/*******************************************************************/
static MEASURE_status_t _MEASURE_switch_dma_buffer(void) {
	// Local variables.
	MEASURE_status_t status = MEASURE_SUCCESS;
	// Stop ADC and DMA.
	status = MEASURE_stop();
	if (status != MEASURE_SUCCESS) goto errors;
	// Retrieve number of transfered data.
	DMA1_get_number_of_transfered_data((uint16_t*) &(measure_sampling.acv[measure_sampling.acv_write_idx].size), (uint16_t*) &(measure_sampling.aci[measure_sampling.acv_write_idx].size));
	// Update write indexes.
	measure_sampling.acv_write_idx = ((measure_sampling.acv_write_idx + 1) % MEASURE_PERIOD_DMA_BUFFER_DEPTH);
	measure_sampling.aci_write_idx = ((measure_sampling.aci_write_idx + 1) % MEASURE_PERIOD_DMA_BUFFER_DEPTH);
	// Set new address.
	DMA1_set_destination_address((uint32_t) &(measure_sampling.acv[measure_sampling.acv_write_idx].data), (uint32_t) &(measure_sampling.aci[measure_sampling.aci_write_idx].data), MEASURE_PERIOD_DMA_BUFFER_SIZE);
	// Restart DMA.
	status = MEASURE_start();
errors:
	return status;
}

/*******************************************************************/
static inline void _MEASURE_reset_channel_results(uint8_t ac_channel_index) {
	// Local variables.
	uint8_t ac_channel_idx = 0;
	// Clear all data.
	_MEASURE_reset_result(measure_data.result[ac_channel_idx].active_power_mw);
	_MEASURE_reset_result(measure_data.result[ac_channel_idx].rms_voltage_mv);
	_MEASURE_reset_result(measure_data.result[ac_channel_idx].rms_current_ma);
	_MEASURE_reset_result(measure_data.result[ac_channel_idx].apparent_power_mva);
	_MEASURE_reset_result(measure_data.result[ac_channel_idx].power_factor);
}

/*******************************************************************/
static inline void _MEASURE_reset_all_channels_results(void) {
	// Local variables.
	uint8_t ac_channel_idx = 0;
	// Channels loop.
	for (ac_channel_idx=0 ; ac_channel_idx<ADC_NUMBER_OF_ACI_CHANNELS ; ac_channel_idx++) {
		// Clear all results.
		_MEASURE_reset_channel_results(ac_channel_idx);
	}
}

/*******************************************************************/
static MEASURE_status_t _MEASURE_internal_process(void) {
	// Local variables.
	MEASURE_status_t status = MEASURE_SUCCESS;
	uint32_t acv_buffer_size = 0;
	uint32_t aci_buffer_size = 0;
	uint8_t ac_channel_idx = 0;
	int32_t active_power_mw = 0;
	q31_t mean_voltage_q31 = 0;
	int32_t rms_voltage_mv = 0;
	q31_t mean_current_q31 = 0;
	int32_t rms_current_ma = 0;
	int32_t apparent_power_mva = 0;
	int32_t power_factor = 0;
	int64_t temp_s64 = 0;
	uint32_t idx = 0;
	// Perform state machine.
	switch (measure_ctx.state) {
	case MEASURE_STATE_STOPPED:
#ifdef MEASURE_CONTINUOUS
		// Start measure.
		status = _MEASURE_start();
		if (status != MEASURE_SUCCESS) goto errors;
		// Update state.
		measure_ctx.state = MEASURE_STATE_IDLE;
#else
		// Synchronize on zero cross.
		if (measure_ctx.zero_cross_count > MEASURE_ZERO_CROSS_PER_PERIOD) {
			// Clear flag.
			measure_ctx.zero_cross_count = 0;
			// Reset context.
			_MEASURE_reset();
			// Start measure.
			status = MEASURE_start();
			if (status != MEASURE_SUCCESS) goto errors;
			// Update state.
			measure_ctx.state = MEASURE_STATE_IDLE;
		}
#endif
		break;
	case MEASURE_STATE_IDLE:
#ifdef MEASURE_CONTINUOUS
		// Check DMA transfer end flag.
		if (measure_ctx.dma_transfer_end_flag != 0) {
			// Clear flag.
			measure_ctx.dma_transfer_end_flag = 0;
#else
		// Check zero cross flag.
		if (measure_ctx.zero_cross_count >= MEASURE_ZERO_CROSS_PER_PERIOD) {
			// Clear flag.
			measure_ctx.zero_cross_count = 0;
#endif
			// Switch to next buffer.
			status = _MEASURE_switch_dma_buffer();
			if (status != MEASURE_SUCCESS) goto errors;
			// Update state.
			measure_ctx.state = MEASURE_STATE_PERIOD_PROCESS;
		}
		break;
	case MEASURE_STATE_PERIOD_PROCESS:
		// Get size.
		acv_buffer_size = (uint32_t) ((measure_sampling.acv[measure_sampling.acv_read_idx].size) / (ADC_NUMBER_OF_ACI_CHANNELS));
		aci_buffer_size = (uint32_t) ((measure_sampling.aci[measure_sampling.acv_read_idx].size) / (ADC_NUMBER_OF_ACI_CHANNELS));
		// Take the minimum size between voltage and current.
		measure_data.period_acxx_buffer_size = (acv_buffer_size < aci_buffer_size) ? acv_buffer_size : aci_buffer_size;
		// Processing each channel.
		for (ac_channel_idx=0 ; ac_channel_idx<ADC_NUMBER_OF_ACI_CHANNELS ; ac_channel_idx++) {
			// Compute channel buffer.
			for (idx=0 ; idx<(measure_data.period_acxx_buffer_size) ; idx++) {
				// Copy samples by channel and convert to Q31 type.
				measure_data.period_acvx_buffer_q31[idx] = (measure_sampling.acv[measure_sampling.acv_read_idx].data[(ADC_NUMBER_OF_ACI_CHANNELS * idx) + ac_channel_idx]) << MEASURE_Q31_SHIFT_ADC;
				measure_data.period_acix_buffer_q31[idx] = (measure_sampling.aci[measure_sampling.aci_read_idx].data[(ADC_NUMBER_OF_ACI_CHANNELS * idx) + ac_channel_idx]) << MEASURE_Q31_SHIFT_ADC;
				// Force current to 0 if sensor is not connected.
				if (GPIO_read(MEASURE_GPIO_ACI_DETECT[ac_channel_idx]) == 0) {
					measure_data.period_acix_buffer_q31[idx] = 0;
				}
			}
			// Mean voltage and current.
			arm_mean_q31(measure_data.period_acvx_buffer_q31, measure_data.period_acxx_buffer_size, &mean_voltage_q31);
			arm_mean_q31(measure_data.period_acix_buffer_q31, measure_data.period_acxx_buffer_size, &mean_current_q31);
			// DC removal.
			for (idx=0 ; idx<(measure_data.period_acxx_buffer_size) ; idx++) {
				measure_data.period_acvx_buffer_q31[idx] -= mean_voltage_q31;
				measure_data.period_acix_buffer_q31[idx] -= mean_current_q31;
			}
			// Instantaneous power.
			arm_mult_q31(measure_data.period_acvx_buffer_q31, measure_data.period_acix_buffer_q31, measure_data.period_acpx_buffer_q31, measure_data.period_acxx_buffer_size);
			// Active power.
			arm_mean_q31(measure_data.period_acpx_buffer_q31, measure_data.period_acxx_buffer_size, &(measure_data.period_active_power_q31));
			temp_s64 = (int64_t) (measure_data.period_active_power_q31 >> MEASURE_Q31_SHIFT_MULT);
			temp_s64 *= MEASURE_ACP_FACTOR_NUM;
			active_power_mw = (int32_t) (temp_s64 / MEASURE_ACP_FACTOR_DEN);
			// RMS voltage.
			arm_rms_q31(measure_data.period_acvx_buffer_q31, measure_data.period_acxx_buffer_size, &(measure_data.period_rms_voltage_q31));
			temp_s64 = MEASURE_ACV_FACTOR_NUM * (int64_t) (measure_data.period_rms_voltage_q31 >> MEASURE_Q31_SHIFT_ADC);
			rms_voltage_mv = (int32_t) (temp_s64 / MEASURE_ACV_FACTOR_DEN);
			// RMS current.
			arm_rms_q31(measure_data.period_acix_buffer_q31, measure_data.period_acxx_buffer_size, &(measure_data.period_rms_current_q31));
			temp_s64 = MEASURE_ACI_FACTOR_NUM * (int64_t) (measure_data.period_rms_current_q31 >> MEASURE_Q31_SHIFT_ADC);
			rms_current_ma = (int32_t) (temp_s64 / MEASURE_ACI_FACTOR_DEN);
			// Apparent power.
			temp_s64 = (int64_t) rms_voltage_mv * (int64_t) rms_current_ma;
			apparent_power_mva = (int32_t) ((temp_s64) / ((int64_t) 1000));
			// Power factor.
			temp_s64 = (int64_t) MEASURE_POWER_FACTOR_MULTIPLIER * (int64_t) active_power_mw;
			power_factor = (apparent_power_mva != 0) ? (int32_t) ((temp_s64) / ((int64_t) apparent_power_mva)) : 0;
			// Update results.
			_MEASURE_result_add_sample(measure_data.result[ac_channel_idx].active_power_mw, active_power_mw);
			_MEASURE_result_add_sample(measure_data.result[ac_channel_idx].rms_voltage_mv, rms_voltage_mv);
			_MEASURE_result_add_sample(measure_data.result[ac_channel_idx].rms_current_ma, rms_current_ma);
			_MEASURE_result_add_sample(measure_data.result[ac_channel_idx].apparent_power_mva, apparent_power_mva);
			_MEASURE_result_add_sample(measure_data.result[ac_channel_idx].power_factor, power_factor);
		}
		// Update read indexes.
		measure_sampling.acv_read_idx = ((measure_sampling.acv_read_idx + 1) % MEASURE_PERIOD_DMA_BUFFER_DEPTH);
		measure_sampling.aci_read_idx = ((measure_sampling.aci_read_idx + 1) % MEASURE_PERIOD_DMA_BUFFER_DEPTH);
		// Go back to idle.
		measure_ctx.state = MEASURE_STATE_IDLE;
		break;
	default:
		status = MEASURE_ERROR_STATE;
		goto errors;
	}
errors:
	return status;
}

/*** MEASURE functions ***/

/*******************************************************************/
MEASURE_status_t MEASURE_init(void) {
	// Local variables.
	MEASURE_status_t status = MEASURE_SUCCESS;
	POWER_status_t power_status = POWER_SUCCESS;
	ADC_status_t adc_status = ADC_SUCCESS;
	uint8_t idx = 0;
	// Init context.
	measure_ctx.state = MEASURE_STATE_STOPPED;
	_MEASURE_reset_all_channels_results();
	// Init current sensors detector.
	for (idx=0 ; idx<ADC_NUMBER_OF_ACI_CHANNELS ; idx++) {
		GPIO_configure(MEASURE_GPIO_ACI_DETECT[idx], GPIO_MODE_INPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	}
	// Init zero cross pulse GPIO.
	GPIO_configure(&GPIO_ZERO_CROSS_PULSE, GPIO_MODE_INPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	EXTI_configure_gpio(&GPIO_ZERO_CROSS_PULSE, EXTI_TRIGGER_RISING_EDGE, &_MEASURE_increment_zero_cross_count);
	NVIC_enable_interrupt(NVIC_INTERRUPT_EXTI2, NVIC_PRIORITY_EXTI2);
	// Init DMA.
	DMA1_init(&_MEASURE_set_dma_transfer_end_flag);
	// Turn analog front-end on to have VREF+ for ADC calibration.
	power_status = POWER_enable(POWER_DOMAIN_ANALOG, LPTIM_DELAY_MODE_ACTIVE);
	POWER_exit_error(MEASURE_ERROR_BASE_POWER);
	// Init ADC.
	adc_status = ADC_init();
	ADC_exit_error(MEASURE_ERROR_BASE_ADC);
	// Init timer.
	TIM6_init();
errors:
	return status;
}

/*******************************************************************/
MEASURE_status_t MEASURE_start(void) {
	// Local variables.
	MEASURE_status_t status = MEASURE_SUCCESS;
	ADC_status_t adc_status = ADC_SUCCESS;
	// Start DMA.
	DMA1_start();
	// Start ADC.
	adc_status = ADC_start();
	ADC_exit_error(MEASURE_ERROR_BASE_ADC);
	// Start trigger.
	TIM6_start();
errors:
	return status;
}

/*******************************************************************/
MEASURE_status_t MEASURE_stop(void) {
	// Local variables.
	MEASURE_status_t status = MEASURE_SUCCESS;
	ADC_status_t adc_status = ADC_SUCCESS;
	// Stop trigger.
	TIM6_stop();
	// Stop ADC.
	adc_status = ADC_stop();
	ADC_exit_error(MEASURE_ERROR_BASE_ADC);
	// Stop DMA.
	DMA1_stop();
errors:
	// Update state.
	measure_ctx.state = MEASURE_STATE_STOPPED;
	return status;
}

/*******************************************************************/
MEASURE_status_t MEASURE_process(void) {
	// Local variables.
	MEASURE_status_t status = MEASURE_SUCCESS;
	// Call state machine until state is idle again.
	do {
		status = _MEASURE_internal_process();
		if (status != MEASURE_SUCCESS) goto errors;
	}
	while (measure_ctx.state == MEASURE_STATE_PERIOD_PROCESS);
errors:
	return status;
}

/*******************************************************************/
MEASURE_status_t MEASURE_get_ac_channel_detect_flag(uint8_t ac_channel_index, uint8_t* current_sensor_connected) {
	// Local variables.
	MEASURE_status_t status = MEASURE_SUCCESS;
	// Check parameters.
	if (ac_channel_index >= ADC_NUMBER_OF_ACI_CHANNELS) {
		status = MEASURE_ERROR_AC_LINE_INDEX;
		goto errors;
	}
	(*current_sensor_connected) = GPIO_read(MEASURE_GPIO_ACI_DETECT[ac_channel_index]);
errors:
	return status;
}

/*******************************************************************/
MEASURE_status_t MEASURE_get_ac_channel_data(uint8_t ac_channel_index, MEASURE_channel_result_t* ac_channel_data) {
	// Local variables.
	MEASURE_status_t status = MEASURE_SUCCESS;
	// Check parameters.
	if (ac_channel_data == NULL) {
		status = MEASURE_ERROR_NULL_PARAMETER;
		goto errors;
	}
	if (ac_channel_index >= ADC_NUMBER_OF_ACI_CHANNELS) {
		status = MEASURE_ERROR_AC_LINE_INDEX;
		goto errors;
	}
	// Copy data.
	_MEASURE_copy_result(measure_data.result[ac_channel_index].active_power_mw, (ac_channel_data -> active_power_mw));
	_MEASURE_copy_result(measure_data.result[ac_channel_index].rms_voltage_mv, (ac_channel_data -> rms_voltage_mv));
	_MEASURE_copy_result(measure_data.result[ac_channel_index].rms_current_ma, (ac_channel_data -> rms_current_ma));
	_MEASURE_copy_result(measure_data.result[ac_channel_index].apparent_power_mva, (ac_channel_data -> apparent_power_mva));
	_MEASURE_copy_result(measure_data.result[ac_channel_index].power_factor, (ac_channel_data -> power_factor));
	// Reset data.
	_MEASURE_reset_channel_results(ac_channel_index);
errors:
	return status;
}
