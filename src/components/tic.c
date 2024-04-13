/*
 * tic.c
 *
 *  Created on: 09 apr. 2024
 *      Author: Ludo
 */

#include "tic.h"

#include "dma.h"
#include "math_custom.h"
#include "mode.h"
#include "parser.h"
#include "power.h"
#include "usart.h"
#include "string_custom.h"
#include "types.h"

/*** TIC local macros ***/

#define TIC_RX_BUFFER_SIZE					64

#define TIC_SAMPLING_PERIOD_MIN_SECONDS		5
#define TIC_SAMPLING_PERIOD_MAX_SECONDS		3600

#ifdef LINKY_TIC_MODE_HISTORIC
#define TIC_BAUD_RATE						1200
#define TIC_NUMBER_OF_DATA					11
#define TIC_FRAME_SEPARATOR_CHAR			STRING_CHAR_SPACE
#define TIC_FRAME_END_CHAR					STRING_CHAR_LF
#endif

/*** TIC local structures ***/

/*******************************************************************/
typedef enum {
	TIC_STATE_SLEEP = 0,
	TIC_STATE_DECODE,
	TIC_STATE_LAST
} TIC_state_t;

/*******************************************************************/
typedef struct {
	char_t* name;
	STRING_format_t format;
} TIC_sample_t;

/*******************************************************************/
typedef union {
	struct {
		unsigned detect : 1;
		unsigned fill_buffer0 : 1;
		unsigned frame_received : 1;
		unsigned decode_success : 1;
	};
	uint8_t all;
} TIC_flags_t;

/*******************************************************************/
typedef struct {
	// State machine.
	TIC_state_t state;
	uint32_t sampling_period_seconds;
	uint32_t second_count;
	volatile TIC_flags_t flags;
	// DMA Buffers.
	volatile char_t dma_buffer0[TIC_RX_BUFFER_SIZE]; // TIC input messages buffer 1.
	volatile char_t dma_buffer1[TIC_RX_BUFFER_SIZE]; // TIC input messages buffer 2.
	// Parsing buffer.
	char_t frame[TIC_RX_BUFFER_SIZE];
	uint8_t frame_size;
	PARSER_context_t parser;
	uint8_t decoding_count;
	// Data.
	TIC_data_t run_data[TIC_DATA_INDEX_LAST];
	TIC_accumulated_data_t accumulated_data[TIC_DATA_INDEX_LAST];
} TIC_context_t;

/*** TIC local global variables ***/

#ifdef LINKY_TIC_MODE_HISTORIC
static const TIC_sample_t TIC_SAMPLE[TIC_DATA_INDEX_LAST] = {
	{"PAPP ", STRING_FORMAT_DECIMAL}
};
#endif
static TIC_context_t tic_ctx;

/*** TIC local functions ***/

/*******************************************************************/
#define _TIC_reset_accumulated_data(result) { \
	result.min = 2147483647; \
	result.max = 0; \
	result.rolling_mean = 0; \
	result.number_of_samples = 0; \
}

/*******************************************************************/
#define _TIC_add_accumulated_sample(source, new_sample) { \
	/* Compute absolute value of new sample */ \
	math_status = MATH_abs(new_sample, &new_sample_abs); \
	MATH_exit_error(TIC_ERROR_BASE_MATH); \
	/* Min */ \
	math_status = MATH_abs(source.min, &ref_abs); \
	MATH_exit_error(TIC_ERROR_BASE_MATH); \
	if (new_sample_abs < ref_abs) { \
		source.min = new_sample; \
	} \
	/* Max */ \
	math_status = MATH_abs(source.max, &ref_abs); \
	MATH_exit_error(TIC_ERROR_BASE_MATH); \
	if (new_sample_abs > ref_abs) { \
		source.max = new_sample; \
	} \
	/* Compute rolling mean */ \
	temp_s64 = ((int64_t) source.rolling_mean * (int64_t) source.number_of_samples) + (int64_t) new_sample; \
	source.rolling_mean = (int32_t) ((temp_s64) / ((int64_t) (source.number_of_samples + 1))); \
	source.number_of_samples++; \
}

/*******************************************************************/
#define _TIC_copy_accumulated_data(source, destination) { \
	destination.min = source.min; \
	destination.max = source.max; \
	destination.rolling_mean = source.rolling_mean; \
	destination.number_of_samples = source.number_of_samples; \
}

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
	// Update flag.
	tic_ctx.flags.frame_received = line_end_flag;
	// Restart DMA transfer.
	DMA1_usart2_start();
}

/*******************************************************************/
static void _TIC_usart_cm_irq_callback(void) {
	// Switch buffer.
	_TIC_switch_dma_buffer(1);
}

/*******************************************************************/
static void _TIC_dma_tc_irq_callback(void) {
	// Switch buffer.
	_TIC_switch_dma_buffer(0);
}

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

/*******************************************************************/
static void _TIC_reset_parser(void) {
	// Reset parser.
	tic_ctx.parser.buffer = (char_t*) tic_ctx.frame;
	tic_ctx.parser.buffer_size = tic_ctx.frame_size;
	tic_ctx.parser.separator_idx = 0;
	tic_ctx.parser.start_idx = 0;
}

/*******************************************************************/
static TIC_status_t _TIC_decode_sample(uint8_t sample_index) {
	// Local variables.
	TIC_status_t status = TIC_SUCCESS;
	PARSER_status_t parser_status = PARSER_SUCCESS;
	MATH_status_t math_status = MATH_SUCCESS;
	int32_t sample = 0;
	uint32_t new_sample_abs = 0;
	uint32_t ref_abs = 0;
	int64_t temp_s64 = 0;
	// Check index.
	if (sample_index >= TIC_DATA_INDEX_LAST) {
		status = TIC_ERROR_DATA_INDEX;
		goto errors;
	}
	// Decode apparent power.
	if (PARSER_compare(&tic_ctx.parser, PARSER_MODE_HEADER, TIC_SAMPLE[sample_index].name) == PARSER_SUCCESS) {
		// Get value.
		parser_status = PARSER_get_parameter(&tic_ctx.parser, STRING_FORMAT_DECIMAL, TIC_FRAME_SEPARATOR_CHAR, &sample);
		if (parser_status == PARSER_SUCCESS) {
			// Update data.
			tic_ctx.run_data[sample_index] = (sample / 1000);
			_TIC_add_accumulated_sample(tic_ctx.accumulated_data[sample_index], (sample * 1000));
			// Set flag.
			tic_ctx.flags.decode_success = 1;
		}
	}
errors:
	return status;
}

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
	// Start with buffer 1.
	DMA1_usart2_set_destination_address((uint32_t) &(tic_ctx.dma_buffer0), TIC_RX_BUFFER_SIZE);
	// Start DMA transfer.
	DMA1_usart2_start();
	USART2_enable_rx();
errors:
	return status;
}

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

/*** TIC functions ***/

/*******************************************************************/
TIC_status_t TIC_init(void) {
	// Local variables.
	TIC_status_t status = TIC_SUCCESS;
	USART_status_t usart2_status = USART_SUCCESS;
	uint32_t idx = 0;
	// Init context.
	tic_ctx.state = TIC_STATE_SLEEP;
	tic_ctx.sampling_period_seconds = TIC_SAMPLING_PERIOD_DEFAULT_SECONDS;
	tic_ctx.second_count = 0;
	tic_ctx.flags.all = 0;
	for (idx=0 ; idx<TIC_RX_BUFFER_SIZE ; idx++) tic_ctx.dma_buffer0[idx] = 0;
	for (idx=0 ; idx<TIC_RX_BUFFER_SIZE ; idx++) tic_ctx.dma_buffer1[idx] = 0;
	_TIC_reset_parser();
	// Reset data.
	for (idx=0 ; idx<TIC_DATA_INDEX_LAST ; idx++) {
		tic_ctx.run_data[idx] = 0;
		_TIC_reset_accumulated_data(tic_ctx.accumulated_data[idx])
	}
	// Init USART interface.
	usart2_status = USART2_init(TIC_BAUD_RATE, TIC_FRAME_END_CHAR, &_TIC_usart_cm_irq_callback);
	USART2_exit_error(TIC_ERROR_BASE_USART2);
	// Init DMA.
	DMA1_usart2_init(&_TIC_dma_tc_irq_callback);
errors:
	return status;
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

/*******************************************************************/
TIC_status_t TIC_tick_second(void) {
	// Local variables.
	TIC_status_t status = TIC_SUCCESS;
	// Increment seconds.
	tic_ctx.second_count++;
	// Perform state machine.
	switch (tic_ctx.state) {
	case TIC_STATE_SLEEP:
		// Check seconds count.
		if (tic_ctx.second_count >= tic_ctx.sampling_period_seconds) {
			// Reset seconds count.
			tic_ctx.second_count = 0;
			// Start acquisition.
			status = _TIC_start();
			if (status != TIC_SUCCESS) goto errors;
			// Update state.
			tic_ctx.state = TIC_STATE_DECODE;
		}
		break;
	case TIC_STATE_DECODE:
		// Reset decoding count.
		tic_ctx.decoding_count = 0;
		// Loop until data is retrieved.
		while ((tic_ctx.flags.decode_success == 0) && (tic_ctx.decoding_count <= TIC_NUMBER_OF_DATA)) {
			// Check RX IRQ flag.
			if (tic_ctx.flags.frame_received != 0) {
				// Update flags.
				tic_ctx.flags.frame_received = 0;
				tic_ctx.flags.detect = 1;
				tic_ctx.decoding_count++;
				// Build frame.
				_TIC_build_frame();
				_TIC_reset_parser();
				// Decode data.
				status = _TIC_decode_sample(TIC_DATA_INDEX_APPARENT_POWER_MVA);
				if (status != TIC_SUCCESS) goto errors;
			}
		}
		// Stop acquisition.
		status = _TIC_stop();
		if (status != TIC_SUCCESS) goto errors;
		// Update state.
		tic_ctx.state = TIC_STATE_SLEEP;
		break;
	default:
		status = TIC_ERROR_STATE;
		goto errors;
	}
	return status;
errors:
	_TIC_stop();
	return status;
}

/*******************************************************************/
TIC_status_t TIC_get_detect_flag(uint8_t* linky_tic_connected) {
	// Local variables.
	TIC_status_t status = TIC_SUCCESS;
	// Check parameter.
	if (linky_tic_connected == NULL) {
		status = TIC_ERROR_NULL_PARAMETER;
		goto errors;
	}
	(*linky_tic_connected) = tic_ctx.flags.detect;
errors:
	return status;
}

/*******************************************************************/
TIC_status_t TIC_get_run_data(TIC_data_index_t data_index, TIC_data_t* run_data) {
	// Local variables.
	TIC_status_t status = TIC_SUCCESS;
	// Check parameters.
	if (data_index >= TIC_DATA_INDEX_LAST) {
		status = TIC_ERROR_DATA_INDEX;
		goto errors;
	}
	if (run_data == NULL) {
		status = TIC_ERROR_NULL_PARAMETER;
		goto errors;
	}
	(*run_data) = tic_ctx.run_data[data_index];
errors:
	return status;
}

/*******************************************************************/
TIC_status_t TIC_get_accumulated_data(TIC_data_index_t data_index, TIC_accumulated_data_t* accumulated_data) {
	// Local variables.
	TIC_status_t status = TIC_SUCCESS;
	// Check parameters.
	if (data_index >= TIC_DATA_INDEX_LAST) {
		status = TIC_ERROR_DATA_INDEX;
		goto errors;
	}
	if (accumulated_data == NULL) {
		status = TIC_ERROR_NULL_PARAMETER;
		goto errors;
	}
	_TIC_copy_accumulated_data(tic_ctx.accumulated_data[data_index], (*accumulated_data));
errors:
	return status;
}
