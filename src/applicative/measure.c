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
#include "nvic.h"
#include "tim.h"
#include "types.h"

/*** MEASURE local macros ***/

#define MEASURE_MAINS_PERIOD_US				20000
#define MEASURE_ZERO_CROSS_PER_PERIOD		2

// Expressed in number of periods.
#define MEASURE_INTEGRATION_BUFFER_SIZE		((1000 * MEASURE_INTEGRATION_PERIOD_MS) / (MEASURE_MAINS_PERIOD_US))

// Note: factor 2 is used to add a margin to the buffer length (2 mains periods long instead of 1).
// Buffer switch is triggered by the zero cross detection instead of a fixed number of samples.
#define MEASURE_PERIOD_BUFFER_SIZE			(2 * (MEASURE_MAINS_PERIOD_US / ADC_SAMPLING_PERIOD_US))

#define MEASURE_PERIOD_DMA_BUFFER_SIZE		(ADC_NUMBER_OF_ACI_CHANNELS * MEASURE_PERIOD_BUFFER_SIZE)
#define MEASURE_PERIOD_DMA_BUFFER_DEPTH		2

#define MEASURE_TRANSFORMER_GAIN			30
#define MEASURE_TRANSFORMER_ATTEN			10
#define MEASURE_ACV_FACTOR					((MEASURE_TRANSFORMER_GAIN * MEASURE_TRANSFORMER_ATTEN * ADC_VREF_MV) / (ADC_FULL_SCALE))

#define MEASURE_SCT013_GAIN					20
#define MEASURE_SCT013_ATTEN				1
#define MEASURE_ACI_FACTOR					((MEASURE_SCT013_GAIN * MEASURE_SCT013_ATTEN * ADC_VREF_MV) / (ADC_FULL_SCALE))

#define MEASURE_POWER_FACTOR_MULTIPLIER		1000

#define MEASURE_TIMEOUT_COUNT				10000000

//#define MEASURE_CONTINUOUS

/*** MEASURE local structures ***/

typedef enum {
	MEASURE_STATE_STOPPED = 0,
	MEASURE_STATE_IDLE,
	MEASURE_STATE_PERIOD_PROCESS,
	MEASURE_STATE_LAST
} MEASURE_state_t;

typedef struct {
	volatile int16_t data[MEASURE_PERIOD_DMA_BUFFER_SIZE];
	uint16_t size;
} MEASURE_buffer_t;

typedef struct {
	// Raw buffer filled by ADC and DMA for 1 period.
	MEASURE_buffer_t acv_sampling[MEASURE_PERIOD_DMA_BUFFER_DEPTH];
	uint8_t acv_sampling_read_idx;
	uint8_t acv_sampling_write_idx;
	MEASURE_buffer_t aci_sampling[MEASURE_PERIOD_DMA_BUFFER_DEPTH];
	uint8_t aci_sampling_read_idx;
	uint8_t aci_sampling_write_idx;
	// Temporary variables for individual channel processing on 1 period.
	int32_t period_acvx_buffer[MEASURE_PERIOD_BUFFER_SIZE];
	int32_t period_acix_buffer[MEASURE_PERIOD_BUFFER_SIZE];
	int32_t period_acpx_buffer[MEASURE_PERIOD_BUFFER_SIZE];
	uint16_t period_acxx_buffer_size;
	int32_t period_active_power;
	int32_t period_rms_voltage;
	int32_t period_rms_current;
	int32_t period_apparent_power;
	int32_t period_power_factor;
	// Results.
	MEASURE_channel_result_t result[ADC_NUMBER_OF_ACI_CHANNELS];
} MEASURE_data_t;

typedef struct {
	MEASURE_state_t state;
	volatile uint8_t zero_cross_count;
	volatile uint8_t dma_transfer_end_flag;
} MEASURE_context_t;

/*** MEASURE local global variables ***/

static MEASURE_data_t measure_data __attribute__((section(".bssCCMSRAM")));
static MEASURE_context_t measure_ctx;

/*** MEASURE local functions ***/

/* RESET RESULT.
 * @param result:	Result structure to reset.
 * @return:			None.
 */
#define _MEASURE_reset_result(result) { \
	result.min = 0x7FFFFFFF; \
	result.max = 0; \
	result.rolling_mean = 0; \
	result.number_of_samples = 0; \
}

/* COPY RESULT.
 * @param source:		Source result structure.
 * @param destination:	Destination result structure.
 * @return:				None.
 */
#define _MEASURE_copy_result(source, destination) { \
	destination.min = source.min; \
	destination.max = source.max; \
	destination.rolling_mean = source.rolling_mean; \
	destination.number_of_samples = source.number_of_samples; \
}

/* UPDATE RESULT WITH NEW SAMPLE.
 * @param result:		Result structure to update.
 * @param new_sample:	New value to take into account.
 * @return:				None.
 */
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
	result.rolling_mean = ((result.rolling_mean * result.number_of_samples) + new_sample) / (result.number_of_samples + 1); \
	result.number_of_samples++; \
}

/* SWITCH DMA BUFFER.
 * @param:	None.
 * @return:	None.
 */
static void _MEASURE_switch_dma_buffer(void) {
	// Stop DMA.
	DMA1_stop();
	// Retrieve number of transfered data.
	DMA1_get_number_of_transfered_data(&(measure_data.acv_sampling[measure_data.acv_sampling_write_idx].size), &(measure_data.aci_sampling[measure_data.acv_sampling_write_idx].size));
	// Update write indexes.
	measure_data.acv_sampling_write_idx = ((measure_data.acv_sampling_write_idx + 1) % MEASURE_PERIOD_DMA_BUFFER_DEPTH);
	measure_data.aci_sampling_write_idx = ((measure_data.aci_sampling_write_idx + 1) % MEASURE_PERIOD_DMA_BUFFER_DEPTH);
	// Set new address.
	DMA1_set_destination_address((uint32_t) &(measure_data.acv_sampling[measure_data.acv_sampling_write_idx].data), (uint32_t) &(measure_data.aci_sampling[measure_data.aci_sampling_write_idx].data), MEASURE_PERIOD_DMA_BUFFER_SIZE);
	// Restart DMA.
	DMA1_start();
}

/* START MAINS MEASUREMENTS.
 * @param:			None.
 * @return status:	Function execution status.
 */
static MEASURE_status_t _MEASURE_start(void) {
	// Local variables.
	MEASURE_status_t status = MEASURE_SUCCESS;
	ADC_status_t adc_status = ADC_SUCCESS;
	// Reset indexed.
	measure_data.acv_sampling_write_idx = 0;
	measure_data.acv_sampling_read_idx = 0;
	measure_data.aci_sampling_write_idx = 0;
	measure_data.aci_sampling_read_idx = 0;
	// Reset flags.
	measure_ctx.zero_cross_count = 0;
	measure_ctx.dma_transfer_end_flag = 0;
	// Start DMA.
	DMA1_set_destination_address((uint32_t) &(measure_data.acv_sampling[measure_data.acv_sampling_write_idx].data), (uint32_t) &(measure_data.aci_sampling[measure_data.aci_sampling_write_idx].data), MEASURE_PERIOD_DMA_BUFFER_SIZE);
	DMA1_start();
	// Start ADC.
	adc_status = ADC_start();
	ADC_status_check(MEASURE_ERROR_BASE_ADC);
	// Start trigger.
	TIM6_start();
errors:
	return status;
}

/* STOP MAINS MEASUREMENTS.
 * @param:			None.
 * @return status:	Function execution status.
 */
static MEASURE_status_t _MEASURE_stop(void) {
	// Local variables.
	MEASURE_status_t status = MEASURE_SUCCESS;
	ADC_status_t adc_status = ADC_SUCCESS;
	// Stop trigger.
	TIM6_stop();
	// Stop ADC.
	adc_status = ADC_stop();
	ADC_status_check(MEASURE_ERROR_BASE_ADC);
	// Stop DMA.
	DMA1_stop();
errors:
	return status;
}

/* CREATE AC BUFFER FROM RAW BUFFER.
 * @param ac_channel_index:	AC line index.
 * @return status:			Function execution status.
 */
static MEASURE_status_t _MEASURE_compute_channel_buffer(uint8_t ac_channel_index) {
	// Local variables.
	MEASURE_status_t status = MEASURE_SUCCESS;
	uint16_t idx = 0;
	// Check index.
	if (ac_channel_index >= ADC_NUMBER_OF_ACI_CHANNELS) {
		status = MEASURE_ERROR_AC_LINE_INDEX;
		goto errors;
	}
	// Data loop.
	for (idx=0 ; idx<(measure_data.period_acxx_buffer_size) ; idx++) {
		// Copy data.
		measure_data.period_acvx_buffer[idx] = (int32_t) measure_data.acv_sampling[measure_data.acv_sampling_read_idx].data[(ADC_NUMBER_OF_ACI_CHANNELS * idx) + ac_channel_index];
		measure_data.period_acix_buffer[idx] = (int32_t) measure_data.aci_sampling[measure_data.aci_sampling_read_idx].data[(ADC_NUMBER_OF_ACI_CHANNELS * idx) + ac_channel_index];
	}
errors:
	return status;
}

/* RESET ALL CHANNELS RESULTS.
 * @param:	None.
 * @return:	None.
 */
static void _MEASURE_reset_results(void) {
	// Local variables.
	uint8_t ac_channel_idx = 0;
	// Channels loop.
	for (ac_channel_idx=0 ; ac_channel_idx<ADC_NUMBER_OF_ACI_CHANNELS ; ac_channel_idx++) {
		// Clear all results.
		_MEASURE_reset_result(measure_data.result[ac_channel_idx].active_power);
		_MEASURE_reset_result(measure_data.result[ac_channel_idx].rms_voltage);
		_MEASURE_reset_result(measure_data.result[ac_channel_idx].rms_current);
		_MEASURE_reset_result(measure_data.result[ac_channel_idx].apparent_power);
		_MEASURE_reset_result(measure_data.result[ac_channel_idx].power_factor);
	}
}

/*** MEASURE functions ***/

/* INIT MAINS MEASUREMENT PERIPHERALS
 * @param:			None.
 * @return status:	Function execution status.
 */
MEASURE_status_t MEASURE_init(void) {
	// Local variables.
	MEASURE_status_t status = MEASURE_SUCCESS;
	ADC_status_t adc_status = ADC_SUCCESS;
	// Init context.
	measure_ctx.state = MEASURE_STATE_STOPPED;
	_MEASURE_reset_results();
	// Init zero cross pulse GPIO.
	GPIO_configure(&GPIO_ZERO_CROSS_PULSE, GPIO_MODE_INPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	EXTI_configure_gpio(&GPIO_ZERO_CROSS_PULSE, EXTI_TRIGGER_RISING_EDGE);
	NVIC_enable_interrupt(NVIC_INTERRUPT_EXTI2);
	// Init DMA.
	DMA1_init();
	// Init ADC.
	adc_status = ADC_init();
	ADC_status_check(MEASURE_ERROR_BASE_ADC);
	// Init timer.
	TIM6_init();
errors:
	return status;
}

/* INCREMENT ZERO CROSS COUNTER (CALLED BY EXTI INTERRUPT).
 * @param:	None.
 * @return:	None.
 */
void MEASURE_increment_zero_cross_count(void) {
	// Set local flag.
	measure_ctx.zero_cross_count++;
}

/* SET DMA TIMEOUT FLAG (CALLED BY DMA INTERRUPT).
 * @param:	None.
 * @return:	None.
 */
void MEASURE_set_dma_transfer_end_flag(void) {
	// Set local flag.
	measure_ctx.dma_transfer_end_flag = 1;
}

/* MAIN TASK OF MAINS MEASUREMENTS.
 * @param:			None.
 * @return status:	Function execution status.
 */
MEASURE_status_t MEASURE_task(void) {
	// Local variables.
	MEASURE_status_t status = MEASURE_SUCCESS;
	uint16_t acv_buffer_size = 0;
	uint16_t aci_buffer_size = 0;
	uint8_t ac_channel_idx = 0;
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
		if (measure_ctx.zero_cross_count > 0) {
			// Clear flag.
			measure_ctx.zero_cross_count = 0;
			// Start measure.
			status = _MEASURE_start();
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
			_MEASURE_switch_dma_buffer();
			// Update state.
			measure_ctx.state = MEASURE_STATE_PERIOD_PROCESS;
		}
#ifndef MEASURE_CONTINUOUS
		// Check DMA transfer end flag.
		if (measure_ctx.dma_transfer_end_flag != 0) {
			// Clear flag.
			measure_ctx.dma_transfer_end_flag = 0;
			// Stop measure.
			status = _MEASURE_stop();
			if (status != MEASURE_SUCCESS) goto errors;
			// Update state.
			measure_ctx.state = MEASURE_STATE_STOPPED;
		}
#endif
		break;
	case MEASURE_STATE_PERIOD_PROCESS:
		// Get size.
		acv_buffer_size = ((measure_data.acv_sampling[measure_data.acv_sampling_read_idx].size) / (ADC_NUMBER_OF_ACI_CHANNELS));
		aci_buffer_size = ((measure_data.aci_sampling[measure_data.acv_sampling_read_idx].size) / (ADC_NUMBER_OF_ACI_CHANNELS));
		// Get the minimum size between voltage and current.
		measure_data.period_acxx_buffer_size = (acv_buffer_size < aci_buffer_size) ? acv_buffer_size : aci_buffer_size;
		// Processing each channel.
		for (ac_channel_idx=0 ; ac_channel_idx<ADC_NUMBER_OF_ACI_CHANNELS ; ac_channel_idx++) {
			// Compute channel buffer.
			status = _MEASURE_compute_channel_buffer(ac_channel_idx);
			if (status != MEASURE_SUCCESS) goto errors;
			// Instantaneous power.
			arm_mult_q31(measure_data.period_acvx_buffer, measure_data.period_acix_buffer, measure_data.period_acpx_buffer, measure_data.period_acxx_buffer_size);
			// Active power.
			arm_mean_q31(measure_data.period_acpx_buffer, measure_data.period_acxx_buffer_size, &(measure_data.period_active_power));
			// RMS voltage and current.
			arm_rms_q31(measure_data.period_acvx_buffer, measure_data.period_acxx_buffer_size, &(measure_data.period_rms_voltage));
			arm_rms_q31(measure_data.period_acix_buffer, measure_data.period_acxx_buffer_size, &(measure_data.period_rms_current));
			// Apparent power.
			measure_data.period_apparent_power = (measure_data.period_rms_voltage * measure_data.period_rms_current);
			// Power factor.
			measure_data.period_power_factor = (measure_data.period_active_power * MEASURE_POWER_FACTOR_MULTIPLIER) / (measure_data.period_apparent_power);
			// Update results.
			_MEASURE_result_add_sample(measure_data.result[ac_channel_idx].active_power, measure_data.period_active_power);
			_MEASURE_result_add_sample(measure_data.result[ac_channel_idx].rms_voltage, measure_data.period_rms_voltage);
			_MEASURE_result_add_sample(measure_data.result[ac_channel_idx].rms_current, measure_data.period_rms_current);
			_MEASURE_result_add_sample(measure_data.result[ac_channel_idx].apparent_power, measure_data.period_apparent_power);
			_MEASURE_result_add_sample(measure_data.result[ac_channel_idx].power_factor, measure_data.period_power_factor);
		}
		// Update read indexes.
		measure_data.acv_sampling_read_idx = ((measure_data.acv_sampling_read_idx + 1) % MEASURE_PERIOD_DMA_BUFFER_DEPTH);
		measure_data.aci_sampling_read_idx = ((measure_data.aci_sampling_read_idx + 1) % MEASURE_PERIOD_DMA_BUFFER_DEPTH);
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

/* READ MAINS MEASUREMENTS.
 * @param ac_channel_index:	AC channel index to read.
 * @param data:				Pointer to the result structure.
 * @return status:			Function execution status.
 */
MEASURE_status_t MEASURE_get_ac_channel_data(uint8_t ac_channel_index, MEASURE_channel_result_t* ac_channel_data) {
	// Local variables.
	MEASURE_status_t status = MEASURE_SUCCESS;
	// Check parameters.
	if (ac_channel_data) {
		status = MEASURE_ERROR_NULL_PARAMETER;
		goto errors;
	}
	if (ac_channel_index >= ADC_NUMBER_OF_ACI_CHANNELS) {
		status = MEASURE_ERROR_AC_LINE_INDEX;
		goto errors;
	}
	// Copy data.
	_MEASURE_copy_result(measure_data.result[ac_channel_index].active_power, (ac_channel_data -> active_power));
	_MEASURE_copy_result(measure_data.result[ac_channel_index].rms_voltage, (ac_channel_data -> rms_voltage));
	_MEASURE_copy_result(measure_data.result[ac_channel_index].rms_current, (ac_channel_data -> rms_current));
	_MEASURE_copy_result(measure_data.result[ac_channel_index].apparent_power, (ac_channel_data -> apparent_power));
	_MEASURE_copy_result(measure_data.result[ac_channel_index].power_factor, (ac_channel_data -> power_factor));
	// Reset data.
	_MEASURE_reset_results();
errors:
	return status;
}
