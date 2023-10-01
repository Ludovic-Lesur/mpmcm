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
#include "led.h"
#include "mapping.h"
#include "mode.h"
#include "nvic.h"
#include "power.h"
#include "tim.h"
#include "types.h"

/*** MEASURE local macros ***/

#define MEASURE_MAINS_PERIOD_US					20000
#define MEASURE_ZERO_CROSS_PER_PERIOD			2

#define MEASURE_TRANSFORMER_GAIN				30 // Unit mV/mV.
#define MEASURE_TRANSFORMER_ATTEN				11 // Unit mV/mV.
#define MEASURE_ACV_FACTOR_NUM					((int64_t) MEASURE_TRANSFORMER_GAIN * (int64_t) MEASURE_TRANSFORMER_ATTEN * (int64_t) ADC_VREF_MV)
#define MEASURE_ACV_FACTOR_DEN					((int64_t) ADC_FULL_SCALE)

#define MEASURE_SCT013_ATTEN					1 // Unit mV/mV
#define MEASURE_ACI_FACTOR_NUM					((int64_t) MPMCM_SCT013_GAIN[chx_idx] * (int64_t) MEASURE_SCT013_ATTEN * (int64_t) ADC_VREF_MV)
#define MEASURE_ACI_FACTOR_DEN					((int64_t) ADC_FULL_SCALE)

#define MEASURE_ACP_FACTOR_NUM					(MEASURE_ACV_FACTOR_NUM * MEASURE_ACI_FACTOR_NUM)
#define MEASURE_ACP_FACTOR_DEN					(MEASURE_ACV_FACTOR_DEN * MEASURE_ACI_FACTOR_DEN * (int64_t) 1000) // To get mW from mV and mA.

// Note: factor 2 is used to add a margin to the buffer length (2 mains periods long instead of 1).
// Buffer switch is triggered by the zero cross detection instead of a fixed number of samples.
#define MEASURE_PERIOD_ADCX_BUFFER_SIZE			(2 * (MEASURE_MAINS_PERIOD_US / ADC_SAMPLING_PERIOD_US))
#define MEASURE_PERIOD_ADCX_DMA_BUFFER_SIZE		(ADC_NUMBER_OF_ACI_CHANNELS * MEASURE_PERIOD_ADCX_BUFFER_SIZE)
#define MEASURE_PERIOD_ADCX_DMA_BUFFER_DEPTH	2

#define MEASURE_POWER_FACTOR_MULTIPLIER			1000

#define MEASURE_Q31_SHIFT_ADC					16
#define MEASURE_Q31_SHIFT_MULT					1

#define MEASURE_ACV_FREQUENCY_SAMPLING_HZ		1000000
#define MEASURE_PERIOD_TIM2_DMA_BUFFER_SIZE		3

#define MEASURE_LED_PULSE_DURATION_MS			50
#define MEASURE_LED_PULSE_PERIOD_SECONDS		5

#define MEASURE_TIMEOUT_COUNT					10000000

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
	// Temporary variables for individual channel processing on 1 period.
	q31_t period_acvx_buffer_q31[MEASURE_PERIOD_ADCX_BUFFER_SIZE];
	q31_t period_acix_buffer_q31[MEASURE_PERIOD_ADCX_BUFFER_SIZE];
	q31_t period_acpx_buffer_q31[MEASURE_PERIOD_ADCX_BUFFER_SIZE];
	uint32_t period_acxx_buffer_size;
	q31_t period_active_power_q31;
	q31_t period_rms_voltage_q31;
	q31_t period_rms_current_q31;
	q31_t period_apparent_power_q31;
	q31_t period_power_factor_q31;
	// AC channels results.
	MEASURE_channel_run_data_t chx_run_data[ADC_NUMBER_OF_ACI_CHANNELS];
	MEASURE_channel_accumulated_data_t chx_accumulated_data_run[ADC_NUMBER_OF_ACI_CHANNELS];
	MEASURE_channel_accumulated_data_t chx_accumulated_data[ADC_NUMBER_OF_ACI_CHANNELS];
	// Mains frequency.
	MEASURE_run_data_t acv_frequency_run_data;
	MEASURE_accumulated_data_t acv_frequency_accumulated_data_run;
	MEASURE_accumulated_data_t acv_frequency_accumulated_data;
} MEASURE_data_t;

/*******************************************************************/
typedef struct {
	MEASURE_state_t state;
	uint8_t zero_cross_count;
	uint8_t dma_transfer_end_flag;
	uint32_t period_process_count;
	uint32_t tick_led_count;
} MEASURE_context_t;

/*** MEASURE local global variables ***/

static const GPIO_pin_t* MEASURE_GPIO_ACI_DETECT[ADC_NUMBER_OF_ACI_CHANNELS] = {&GPIO_ACI1_DETECT, &GPIO_ACI2_DETECT, &GPIO_ACI3_DETECT, &GPIO_ACI4_DETECT};

static volatile MEASURE_sampling_t measure_sampling;
static MEASURE_data_t measure_data __attribute__((section(".bss_ccmsram")));

static volatile MEASURE_context_t measure_ctx;

/*** MEASURE local functions ***/

/*******************************************************************/
#define _MEASURE_reset_accumulated_data(result) { \
	result.min = 2147483647; \
	result.max = -2147483648; \
	result.rolling_mean = 0; \
	result.number_of_samples = 0; \
}

/*******************************************************************/
#define _MEASURE_reset_chx_accumulated_data(source, channel_index) { \
	_MEASURE_reset_accumulated_data(source[channel_index].active_power_mw); \
	_MEASURE_reset_accumulated_data(source[channel_index].rms_voltage_mv); \
	_MEASURE_reset_accumulated_data(source[channel_index].rms_current_ma); \
	_MEASURE_reset_accumulated_data(source[channel_index].apparent_power_mva); \
	_MEASURE_reset_accumulated_data(source[channel_index].power_factor); \
}

/*******************************************************************/
#define _MEASURE_copy_accumulated_data(source, destination) { \
	destination.min = source.min; \
	destination.max = source.max; \
	destination.rolling_mean = source.rolling_mean; \
	destination.number_of_samples = source.number_of_samples; \
}

/*******************************************************************/
#define _MEASURE_add_sample(result, new_sample) { \
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
	// Increment counts.
	measure_ctx.zero_cross_count++;
}

/*******************************************************************/
static void _MEASURE_set_dma_adcx_transfer_end_flag(void) {
	// Set local flag.
	measure_ctx.dma_transfer_end_flag = 1;
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
	// Reset channels data.
	for (chx_idx=0 ; chx_idx<ADC_NUMBER_OF_ACI_CHANNELS ; chx_idx++) {
		// Clear all data.
		_MEASURE_reset_chx_accumulated_data(measure_data.chx_accumulated_data, chx_idx);
		_MEASURE_reset_chx_accumulated_data(measure_data.chx_accumulated_data_run, chx_idx);
		// Reset sampling buffers.
		for (idx0=0 ; idx0<MEASURE_PERIOD_ADCX_DMA_BUFFER_DEPTH ; idx0++) {
			for (idx1=0 ; idx1<MEASURE_PERIOD_ADCX_DMA_BUFFER_SIZE ; idx1++) {
				measure_sampling.acv[idx0].data[idx1] = 0;
				measure_sampling.aci[idx0].data[idx1] = 0;
			}
		}
	}
	// Reset frequency data.
	_MEASURE_reset_accumulated_data(measure_data.acv_frequency_accumulated_data);
	_MEASURE_reset_accumulated_data(measure_data.acv_frequency_accumulated_data_run);
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
	// Start DMA.
	DMA1_adcx_start();
	// Start ADC.
	adc_status = ADC_start();
	ADC_exit_error(MEASURE_ERROR_BASE_ADC);
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
	// Clear count.
	measure_ctx.zero_cross_count = 0;
	// Stop trigger.
	TIM6_stop();
	// Stop ADC.
	adc_status = ADC_stop();
	ADC_exit_error(MEASURE_ERROR_BASE_ADC);
	// Stop DMA.
	DMA1_adcx_stop();
errors:
	// Update state.
	measure_ctx.state = MEASURE_STATE_STOPPED;
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
	return status;
}

/*******************************************************************/
MEASURE_status_t _MEASURE_stop(void) {
	// Local variables.
	MEASURE_status_t status = MEASURE_SUCCESS;
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
static MEASURE_status_t _MEASURE_internal_process(void) {
	// Local variables.
	MEASURE_status_t status = MEASURE_SUCCESS;
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
	int32_t frequency_mhz = 0;
	int64_t temp_s64 = 0;
	uint32_t idx = 0;
	// Perform state machine.
	switch (measure_ctx.state) {
	case MEASURE_STATE_STOPPED:
#ifdef MEASURE_CONTINUOUS
		// Start measure.
		status = __MEASURE_start();
		if (status != MEASURE_SUCCESS) goto errors;
		// Update state.
		measure_ctx.state = MEASURE_STATE_IDLE;
#else
		// Synchronize on zero cross.
		if (measure_ctx.zero_cross_count > MEASURE_ZERO_CROSS_PER_PERIOD) {
			// Reset context.
			_MEASURE_reset();
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
			temp_s64 = ((int64_t) rms_voltage_mv) * ((int64_t) rms_current_ma);
			apparent_power_mva = (int32_t) ((temp_s64) / ((int64_t) 1000));
			// Power factor.
			temp_s64 = (int64_t) MEASURE_POWER_FACTOR_MULTIPLIER * ((int64_t) active_power_mw);
			power_factor = (apparent_power_mva != 0) ? (int32_t) ((temp_s64) / ((int64_t) apparent_power_mva)) : 0;
			// Update accumulated data for run.
			_MEASURE_add_sample(measure_data.chx_accumulated_data_run[chx_idx].active_power_mw, active_power_mw);
			_MEASURE_add_sample(measure_data.chx_accumulated_data_run[chx_idx].rms_voltage_mv, rms_voltage_mv);
			_MEASURE_add_sample(measure_data.chx_accumulated_data_run[chx_idx].rms_current_ma, rms_current_ma);
			_MEASURE_add_sample(measure_data.chx_accumulated_data_run[chx_idx].apparent_power_mva, apparent_power_mva);
			_MEASURE_add_sample(measure_data.chx_accumulated_data_run[chx_idx].power_factor, power_factor);
			// Update accumulated data.
			_MEASURE_add_sample(measure_data.chx_accumulated_data[chx_idx].active_power_mw, active_power_mw);
			_MEASURE_add_sample(measure_data.chx_accumulated_data[chx_idx].rms_voltage_mv, rms_voltage_mv);
			_MEASURE_add_sample(measure_data.chx_accumulated_data[chx_idx].rms_current_ma, rms_current_ma);
			_MEASURE_add_sample(measure_data.chx_accumulated_data[chx_idx].apparent_power_mva, apparent_power_mva);
			_MEASURE_add_sample(measure_data.chx_accumulated_data[chx_idx].power_factor, power_factor);
		}
		// Update read indexes.
		measure_sampling.acv_read_idx = ((measure_sampling.acv_read_idx + 1) % MEASURE_PERIOD_ADCX_DMA_BUFFER_DEPTH);
		measure_sampling.aci_read_idx = ((measure_sampling.aci_read_idx + 1) % MEASURE_PERIOD_ADCX_DMA_BUFFER_DEPTH);
		// Compute mains frequency.
		idx = 0;
		while (idx < MEASURE_PERIOD_TIM2_DMA_BUFFER_SIZE) {
			// Manage init state and rollover case.
			if (measure_sampling.tim2_ccr1[idx] < measure_sampling.tim2_ccr1[(idx + 1) % MEASURE_PERIOD_TIM2_DMA_BUFFER_SIZE]) break;
			idx++;
		}
		// Check index.
		if (idx < MEASURE_PERIOD_TIM2_DMA_BUFFER_SIZE) {
			// Compute mains frequency.
			temp_s64 = (((int64_t) MEASURE_ACV_FREQUENCY_SAMPLING_HZ * 1000) / ((int64_t) measure_sampling.tim2_ccr1[(idx + 1) % MEASURE_PERIOD_TIM2_DMA_BUFFER_SIZE] - (int64_t) measure_sampling.tim2_ccr1[idx]));
			frequency_mhz = (int32_t) temp_s64;
			// Update accumulated data.
			_MEASURE_add_sample(measure_data.acv_frequency_accumulated_data_run, frequency_mhz);
			_MEASURE_add_sample(measure_data.acv_frequency_accumulated_data, frequency_mhz);
		}
		// Increment period process count.
		measure_ctx.period_process_count++;
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
	uint8_t chx_idx = 0;
	// Init context.
	measure_ctx.state = MEASURE_STATE_STOPPED;
	measure_ctx.period_process_count = 0;
	measure_ctx.tick_led_count = 0;
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
	DMA1_adcx_init(&_MEASURE_set_dma_adcx_transfer_end_flag);
	DMA1_adcx_set_destination_address((uint32_t) &(measure_sampling.acv[measure_sampling.acv_write_idx].data), (uint32_t) &(measure_sampling.aci[measure_sampling.aci_write_idx].data), MEASURE_PERIOD_ADCX_DMA_BUFFER_SIZE);
	// Init timer DMA.
	DMA1_tim2_init();
	DMA1_tim2_set_destination_address((uint32_t) &(measure_sampling.tim2_ccr1), MEASURE_PERIOD_TIM2_DMA_BUFFER_SIZE);
	// Init ADC.
	adc_status = ADC_init();
	ADC_exit_error(MEASURE_ERROR_BASE_ADC);
errors:
	_MEASURE_reset();
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
MEASURE_status_t MEASURE_tick(void) {
	// Local variables.
	MEASURE_status_t status = MEASURE_SUCCESS;
	LED_color_t led_color = LED_COLOR_OFF;
	uint8_t chx_idx = 0;
	// Compute AC channels run data.
	for (chx_idx=0 ; chx_idx<ADC_NUMBER_OF_ACI_CHANNELS ; chx_idx++) {
		// Copy all rolling means.
		measure_data.chx_run_data[chx_idx].active_power_mw = measure_data.chx_accumulated_data_run[chx_idx].active_power_mw.rolling_mean;
		measure_data.chx_run_data[chx_idx].rms_voltage_mv = measure_data.chx_accumulated_data_run[chx_idx].rms_voltage_mv.rolling_mean;
		measure_data.chx_run_data[chx_idx].rms_current_ma = measure_data.chx_accumulated_data_run[chx_idx].rms_current_ma.rolling_mean;
		measure_data.chx_run_data[chx_idx].apparent_power_mva = measure_data.chx_accumulated_data_run[chx_idx].apparent_power_mva.rolling_mean;
		measure_data.chx_run_data[chx_idx].power_factor = measure_data.chx_accumulated_data_run[chx_idx].power_factor.rolling_mean;
		// Use RMS voltage as reference for number of samples.
		measure_data.chx_run_data[chx_idx].number_of_samples = measure_data.chx_accumulated_data_run[chx_idx].rms_voltage_mv.number_of_samples;
		// Reset results.
		_MEASURE_reset_chx_accumulated_data(measure_data.chx_accumulated_data_run, chx_idx);
	}
	// Compute frequency run data.
	measure_data.acv_frequency_run_data.value = measure_data.acv_frequency_accumulated_data_run.rolling_mean;
	measure_data.acv_frequency_run_data.number_of_samples = measure_data.acv_frequency_accumulated_data_run.number_of_samples;
	_MEASURE_reset_accumulated_data(measure_data.acv_frequency_accumulated_data_run);
	// Check number processed periods during last second.
	if ((measure_ctx.state != MEASURE_STATE_STOPPED) && (measure_ctx.period_process_count == 0)) {
		// Stop measure.
		_MEASURE_stop();
	}
	// Increment tick count.
	measure_ctx.tick_led_count++;
	// Check LED period.
	if (measure_ctx.tick_led_count >= MEASURE_LED_PULSE_PERIOD_SECONDS) {
		// Compute LED color according to state.
		if (measure_ctx.state == MEASURE_STATE_STOPPED) {
			// Check current number of samples (CH1 RMS voltage as reference).
			led_color = (measure_data.chx_accumulated_data[0].rms_voltage_mv.number_of_samples == 0) ? LED_COLOR_RED : LED_COLOR_YELLOW;
		}
		else {
			led_color = LED_COLOR_GREEN;
		}
		// Perform LED pulse.
		LED_single_pulse(MEASURE_LED_PULSE_DURATION_MS, led_color);
		// Reset count.
		measure_ctx.tick_led_count = 0;
	}
	// Reset count.
	measure_ctx.period_process_count = 0;
	return status;
}

/*******************************************************************/
MEASURE_status_t MEASURE_get_detect_flag(uint8_t ac_channel_index, uint8_t* current_sensor_connected) {
	// Local variables.
	MEASURE_status_t status = MEASURE_SUCCESS;
	// Check parameters.
	if (ac_channel_index >= ADC_NUMBER_OF_ACI_CHANNELS) {
		status = MEASURE_ERROR_AC_CHANNEL;
		goto errors;
	}
	(*current_sensor_connected) = GPIO_read(MEASURE_GPIO_ACI_DETECT[ac_channel_index]);
errors:
	return status;
}

/*******************************************************************/
MEASURE_status_t MEASURE_get_run_data(MEASURE_data_type_t data_type, MEASURE_run_data_t* run_data) {
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
MEASURE_status_t MEASURE_get_channel_run_data(uint8_t ac_channel, MEASURE_channel_run_data_t* ac_channel_run_data) {
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
	(ac_channel_run_data -> active_power_mw) = measure_data.chx_run_data[ac_channel].active_power_mw;
	(ac_channel_run_data -> rms_voltage_mv) = measure_data.chx_run_data[ac_channel].rms_voltage_mv;
	(ac_channel_run_data -> rms_current_ma) = measure_data.chx_run_data[ac_channel].rms_current_ma;
	(ac_channel_run_data -> apparent_power_mva) = measure_data.chx_run_data[ac_channel].apparent_power_mva;
	(ac_channel_run_data -> power_factor) = measure_data.chx_run_data[ac_channel].power_factor;
	(ac_channel_run_data -> number_of_samples) = measure_data.chx_run_data[ac_channel].number_of_samples;
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
	// Reset data.
	_MEASURE_reset_chx_accumulated_data(measure_data.chx_accumulated_data, ac_channel);
errors:
	return status;
}
