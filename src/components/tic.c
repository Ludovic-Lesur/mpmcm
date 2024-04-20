/*
 * tic.c
 *
 *  Created on: 09 apr. 2024
 *      Author: Ludo
 */

#include "tic.h"

#include "error.h"
#include "data.h"
#include "dma.h"
#include "led.h"
#include "math_custom.h"
#include "mode.h"
#include "parser.h"
#include "power.h"
#include "usart.h"
#include "string_custom.h"
#include "types.h"

/*** TIC local macros ***/

#define TIC_RX_BUFFER_SIZE					64

#define TIC_SAMPLING_TIMEOUT_SECONDS		5

#define TIC_INACTIVITY_TIMER_SECONDS		60

#define TIC_SAMPLING_PERIOD_MIN_SECONDS		(TIC_SAMPLING_TIMEOUT_SECONDS + 1)
#define TIC_SAMPLING_PERIOD_MAX_SECONDS		3600

#define TIC_LED_PULSE_DURATION_MS			50

#ifdef LINKY_TIC_MODE_HISTORIC
#define TIC_BAUD_RATE						1200
#define TIC_NUMBER_OF_DATA					11
#define TIC_FRAME_SEPARATOR_CHAR			STRING_CHAR_SPACE
#define TIC_FRAME_END_CHAR					STRING_CHAR_LF
#endif

/*** TIC local structures ***/

/*******************************************************************/
typedef enum {
	TIC_SAMPLE_INDEX_APPARENT_POWER_VA = 0,
	TIC_SAMPLE_INDEX_LAST
} TIC_sample_index_t;

/*******************************************************************/
typedef struct {
	char_t* name;
	STRING_format_t format;
} TIC_sample_t;

/*******************************************************************/
typedef union {
	struct {
		unsigned fill_buffer0 : 1;
		unsigned irq_received : 1;
		unsigned frame_received : 1;
		unsigned decode_success : 1;
	};
	uint8_t all;
} TIC_flags_t;

/*******************************************************************/
typedef struct {
	DATA_run_channel_t run;
	DATA_accumulated_channel_t accumulated;
	DATA_run_64_t active_energy_mws_sum;
	DATA_run_64_t apparent_energy_mvas_sum;
} TIC_data_t;

/*******************************************************************/
typedef struct {
	// State machine.
	TIC_state_t state;
	uint32_t sampling_period_seconds;
	uint32_t second_count_period;
	uint32_t second_count_sampling;
	volatile uint32_t second_count_inactivity;
	volatile TIC_flags_t flags;
	// DMA Buffers.
	volatile char_t dma_buffer0[TIC_RX_BUFFER_SIZE]; // TIC input messages buffer 1.
	volatile char_t dma_buffer1[TIC_RX_BUFFER_SIZE]; // TIC input messages buffer 2.
	// Parsing buffer.
	char_t frame[TIC_RX_BUFFER_SIZE];
	uint8_t frame_size;
	PARSER_context_t parser;
	uint8_t decoding_count;
} TIC_context_t;

/*** TIC local global variables ***/

#ifdef LINKY_TIC_ENABLE
#ifdef LINKY_TIC_MODE_HISTORIC
static const TIC_sample_t TIC_SAMPLE[TIC_SAMPLE_INDEX_LAST] = {
	{"PAPP ", STRING_FORMAT_DECIMAL}
};
#endif
#endif
static TIC_data_t tic_data;
static TIC_context_t tic_ctx;

/*** TIC local functions ***/

#ifdef LINKY_TIC_ENABLE
/*******************************************************************/
static void _TIC_switch_dma_buffer(uint8_t line_end_flag) {
	// Stop and start DMA transfer to switch buffer.
	DMA1_usart2_stop();
	// Switch buffer.
	if (tic_ctx.flags.fill_buffer0 == 0) {
		DMA1_usart2_set_destination_address((uint32_t) &(tic_ctx.dma_buffer0), TIC_RX_BUFFER_SIZE); // Switch to buffer 0.
		tic_ctx.flags.fill_buffer0 = 1;
	}
	else {
		DMA1_usart2_set_destination_address((uint32_t) &(tic_ctx.dma_buffer1), TIC_RX_BUFFER_SIZE); // Switch to buffer 1.
		tic_ctx.flags.fill_buffer0 = 0;
	}
	// Update flags.
	tic_ctx.flags.irq_received = 1;
	tic_ctx.flags.frame_received = line_end_flag;
	tic_ctx.second_count_inactivity = 0;
	// Restart DMA transfer.
	DMA1_usart2_start();
}
#endif

#ifdef LINKY_TIC_ENABLE
/*******************************************************************/
static void _TIC_usart_cm_irq_callback(void) {
	// Switch buffer.
	_TIC_switch_dma_buffer(1);
}
#endif

#ifdef LINKY_TIC_ENABLE
/*******************************************************************/
static void _TIC_dma_tc_irq_callback(void) {
	// Switch buffer.
	_TIC_switch_dma_buffer(0);
}
#endif

#ifdef LINKY_TIC_ENABLE
/*******************************************************************/
static void _TIC_build_frame(void) {
	// Local variables.
	uint32_t idx = 0;
	uint8_t frame_end_char_found = 0;
	// Reset frame size.
	tic_ctx.frame_size = 0;
	// Copy from buffer 0.
	for (idx=0 ; idx<TIC_RX_BUFFER_SIZE ; idx++) {
		// Copy from free buffer.
		if (frame_end_char_found != 0) {
			tic_ctx.frame[idx] = 0;
		}
		else {
			// Copy from free buffer.
			tic_ctx.frame[idx] = (tic_ctx.flags.fill_buffer0 == 0) ? tic_ctx.dma_buffer0[idx] : tic_ctx.dma_buffer1[idx];
			// Remove parity bit.
			tic_ctx.frame[idx] &= 0x7F;
			// Increment size.
			tic_ctx.frame_size++;
		}
		// Check line end character.
		if (tic_ctx.frame[idx] == TIC_FRAME_END_CHAR) {
			frame_end_char_found = 1;
		}
	}
}
#endif

/*******************************************************************/
static void _TIC_reset_parser(void) {
	// Reset parser.
	tic_ctx.parser.buffer = (char_t*) tic_ctx.frame;
	tic_ctx.parser.buffer_size = tic_ctx.frame_size;
	tic_ctx.parser.separator_idx = 0;
	tic_ctx.parser.start_idx = 0;
}

#ifdef LINKY_TIC_ENABLE
/*******************************************************************/
static TIC_status_t _TIC_decode_sample(TIC_sample_index_t sample_index) {
	// Local variables.
	TIC_status_t status = TIC_SUCCESS;
	PARSER_status_t parser_status = PARSER_SUCCESS;
	MATH_status_t math_status = MATH_SUCCESS;
	int32_t sample = 0;
	uint32_t new_sample_abs = 0;
	uint32_t ref_abs = 0;
	int64_t temp_s64 = 0;
	// Check index.
	if (sample_index >= TIC_SAMPLE_INDEX_LAST) {
		status = TIC_ERROR_DATA_INDEX;
		goto errors;
	}
	// Decode apparent power.
	if (PARSER_compare(&tic_ctx.parser, PARSER_MODE_HEADER, TIC_SAMPLE[sample_index].name) == PARSER_SUCCESS) {
		// Get value.
		parser_status = PARSER_get_parameter(&tic_ctx.parser, STRING_FORMAT_DECIMAL, TIC_FRAME_SEPARATOR_CHAR, &sample);
		if (parser_status == PARSER_SUCCESS) {
			// Update run data.
			tic_data.run.apparent_power_mva.value = (sample * 1000);
			tic_data.run.apparent_power_mva.number_of_samples = 1;
			// Update accumulated.
			DATA_add_accumulated_channel_sample(tic_data.accumulated, apparent_power_mva, tic_data.run.apparent_power_mva);
			// Increase apparent energy.
			tic_data.apparent_energy_mvas_sum.value += (int64_t) (tic_data.run.apparent_power_mva.value);
			tic_data.apparent_energy_mvas_sum.number_of_samples++;
			// Set flag.
			tic_ctx.flags.decode_success = 1;
		}
	}
errors:
	return status;
}
#endif

#ifdef LINKY_TIC_ENABLE
/*******************************************************************/
static TIC_status_t _TIC_start(void) {
	// Local variables.
	TIC_status_t status = TIC_SUCCESS;
	POWER_status_t power_status = POWER_SUCCESS;
	// Turn TIC interface on.
	power_status = POWER_enable(POWER_DOMAIN_TIC, LPTIM_DELAY_MODE_ACTIVE);
	POWER_exit_error(TIC_ERROR_BASE_POWER);
	// Clear flags.
	tic_ctx.flags.all = 0;
	tic_ctx.flags.fill_buffer0 = 1;
	tic_ctx.decoding_count = 0;
	// Start with buffer 1.
	DMA1_usart2_set_destination_address((uint32_t) &(tic_ctx.dma_buffer0), TIC_RX_BUFFER_SIZE);
	// Start DMA transfer.
	DMA1_usart2_start();
	USART2_enable_rx();
errors:
	return status;
}
#endif

#ifdef LINKY_TIC_ENABLE
/*******************************************************************/
static TIC_status_t _TIC_stop(void) {
	// Local variables.
	TIC_status_t status = TIC_SUCCESS;
	POWER_status_t power_status = POWER_SUCCESS;
	// Stop DMA transfer.
	USART2_disable_rx();
	DMA1_usart2_stop();
	// Turn TIC interface off.
	power_status = POWER_disable(POWER_DOMAIN_TIC);
	POWER_exit_error(TIC_ERROR_BASE_POWER);
errors:
	return status;
}
#endif

/*** TIC functions ***/

/*******************************************************************/
TIC_status_t TIC_init(void) {
	// Local variables.
	TIC_status_t status = TIC_SUCCESS;
#ifdef LINKY_TIC_ENABLE
	USART_status_t usart2_status = USART_SUCCESS;
#endif
	uint32_t idx = 0;
	// Init context.
	tic_ctx.state = TIC_STATE_OFF;
	tic_ctx.sampling_period_seconds = TIC_SAMPLING_PERIOD_DEFAULT_SECONDS;
	tic_ctx.second_count_sampling = 0;
	tic_ctx.second_count_period = 0;
	tic_ctx.second_count_inactivity = (TIC_INACTIVITY_TIMER_SECONDS << 1);
	tic_ctx.flags.all = 0;
	for (idx=0 ; idx<TIC_RX_BUFFER_SIZE ; idx++) tic_ctx.dma_buffer0[idx] = 0;
	for (idx=0 ; idx<TIC_RX_BUFFER_SIZE ; idx++) tic_ctx.dma_buffer1[idx] = 0;
	_TIC_reset_parser();
	// Reset data.
	DATA_reset_run_channel(tic_data.run);
	DATA_reset_accumulated_channel(tic_data.accumulated);
	DATA_reset_run(tic_data.active_energy_mws_sum);
	DATA_reset_run(tic_data.apparent_energy_mvas_sum);
#ifdef LINKY_TIC_ENABLE
	// Init USART interface.
	usart2_status = USART2_init(TIC_BAUD_RATE, TIC_FRAME_END_CHAR, &_TIC_usart_cm_irq_callback);
	USART2_exit_error(TIC_ERROR_BASE_USART2);
	// Init DMA.
	DMA1_usart2_init(&_TIC_dma_tc_irq_callback);
errors:
#endif
	return status;
}

#ifdef LINKY_TIC_ENABLE
/*******************************************************************/
TIC_status_t TIC_process(void) {
	// Local variables.
	TIC_status_t status = TIC_SUCCESS;
#ifndef ANALOG_MEASURE_ENABLE
	LED_status_t led_status = LED_SUCCESS;
	LED_color_t led_color = LED_COLOR_OFF;
#endif
	// Perform state machine.
	switch (tic_ctx.state) {
	case TIC_STATE_OFF:
		// Check period.
		if (tic_ctx.second_count_period >= tic_ctx.sampling_period_seconds) {
			// Reset seconds count.
			tic_ctx.second_count_period = 0;
			tic_ctx.second_count_sampling = 0;
			// Start acquisition.
			status = _TIC_start();
			if (status != TIC_SUCCESS) goto errors;
			// Update state.
			tic_ctx.state = TIC_STATE_ACTIVE;
		}
		break;
	case TIC_STATE_ACTIVE:
		if (tic_ctx.flags.frame_received != 0) {
			// Build frame.
			_TIC_build_frame();
			_TIC_reset_parser();
			// Decode data.
			status = _TIC_decode_sample(TIC_SAMPLE_INDEX_APPARENT_POWER_VA);
			if (status != TIC_SUCCESS) goto errors;
			// Increment decoding count.
			tic_ctx.decoding_count++;
		}
		// Check exit conditions.
		if ((tic_ctx.flags.decode_success != 0) || (tic_ctx.decoding_count > (TIC_NUMBER_OF_DATA << 1)) || (tic_ctx.second_count_sampling >= TIC_SAMPLING_TIMEOUT_SECONDS)) {
			// Update state.
			tic_ctx.state = TIC_STATE_OFF;
			// Stop acquisition.
			status = _TIC_stop();
			if (status != TIC_SUCCESS) goto errors;
#ifndef ANALOG_MEASURE_ENABLE
			// Perform LED pulse.
			if (tic_ctx.flags.irq_received == 0) {
				led_color = LED_COLOR_RED;
			}
			else {
				led_color = (tic_ctx.flags.decode_success) ? LED_COLOR_GREEN : LED_COLOR_YELLOW;
			}
			led_status = LED_single_pulse(TIC_LED_PULSE_DURATION_MS, led_color);
			LED_exit_error(TIC_ERROR_BASE_LED);
#endif
		}
		break;
	default:
		status = TIC_ERROR_STATE;
		goto errors;
	}
	return status;
errors:
	// Force state to off.
	tic_ctx.state = TIC_STATE_OFF;
	_TIC_stop();
	return status;
}
#endif

/*******************************************************************/
TIC_state_t TIC_get_state(void) {
	return (tic_ctx.state);
}

/*******************************************************************/
TIC_status_t TIC_set_sampling_period(uint32_t period_seconds) {
	// Local variables.
	TIC_status_t status = TIC_SUCCESS;
	// Check period.
	if (period_seconds < TIC_SAMPLING_PERIOD_MIN_SECONDS) {
		status = TIC_ERROR_SAMPLING_PERIOD_UNDERFLOW;
		goto errors;
	}
	if (period_seconds > TIC_SAMPLING_PERIOD_MAX_SECONDS) {
		status = TIC_ERROR_SAMPLING_PERIOD_OVERFLOW;
		goto errors;
	}
	// Update local context.
	tic_ctx.sampling_period_seconds = period_seconds;
errors:
	return status;
}

#ifdef LINKY_TIC_ENABLE
/*******************************************************************/
void TIC_tick_second(void) {
	// Increment seconds.
	tic_ctx.second_count_period++;
	tic_ctx.second_count_sampling++;
	tic_ctx.second_count_inactivity++;
}
#endif

/*******************************************************************/
TIC_status_t TIC_get_detect_flag(uint8_t* linky_tic_connected) {
	// Local variables.
	TIC_status_t status = TIC_SUCCESS;
	// Check parameter.
	if (linky_tic_connected == NULL) {
		status = TIC_ERROR_NULL_PARAMETER;
		goto errors;
	}
#ifdef LINKY_TIC_ENABLE
	(*linky_tic_connected) = (tic_ctx.second_count_inactivity > TIC_INACTIVITY_TIMER_SECONDS) ? 0 : 1;
#else
	(*linky_tic_connected) = 0;
#endif
errors:
	return status;
}

/*******************************************************************/
TIC_status_t TIC_get_channel_run_data(DATA_run_channel_t* channel_run_data) {
	// Local variables.
	TIC_status_t status = TIC_SUCCESS;
	// Check parameter.
	if (channel_run_data == NULL) {
		status = TIC_ERROR_NULL_PARAMETER;
		goto errors;
	}
	DATA_copy_run_channel(tic_data.run, (*channel_run_data));
errors:
	return status;
}

/*******************************************************************/
TIC_status_t TIC_get_channel_accumulated_data(DATA_accumulated_channel_t* channel_accumulated_data) {
	// Local variables.
	TIC_status_t status = TIC_SUCCESS;
	// Check parameter.
	if (channel_accumulated_data == NULL) {
		status = TIC_ERROR_NULL_PARAMETER;
		goto errors;
	}
	// Compute active energy.
	tic_data.accumulated.active_energy_mwh.value = (int32_t) ((tic_data.active_energy_mws_sum.value) / ((int64_t) DATA_SECONDS_PER_HOUR));
	tic_data.accumulated.active_energy_mwh.number_of_samples = tic_data.active_energy_mws_sum.number_of_samples;
	// Compute apparent energy.
	tic_data.accumulated.apparent_energy_mvah.value = (int32_t) ((tic_data.apparent_energy_mvas_sum.value) / ((int64_t) DATA_SECONDS_PER_HOUR));
	tic_data.accumulated.apparent_energy_mvah.number_of_samples = tic_data.apparent_energy_mvas_sum.number_of_samples;
	// Copy data.
	DATA_copy_accumulated_channel(tic_data.accumulated, (*channel_accumulated_data));
	// Reset data.
	DATA_reset_accumulated_channel(tic_data.accumulated);
	DATA_reset_run(tic_data.active_energy_mws_sum);
	DATA_reset_run(tic_data.apparent_energy_mvas_sum);
errors:
	return status;
}
