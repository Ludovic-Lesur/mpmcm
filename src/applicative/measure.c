/*
 * measure.c
 *
 *  Created on: 18 jun. 2023
 *      Author: Ludo
 */

#include "measure.h"

#include "adc.h"
#include "dma.h"
#include "exti.h"
#include "gpio.h"
#include "mapping.h"
#include "nvic.h"
#include "tim.h"

/*** MEASURE local macros ***/

#define MEASURE_TIMEOUT_COUNT			10000000
#define MEASURE_MAINS_PERIOD_US			20000
// Note: factor 2 is used to add a margin to the buffer length (2 mains periods long instead of 1).
// Buffer switch is triggered by the zero cross detection instead of a fixed number of samples.
#define MEASURE_AC_BUFFER_SIZE			(2 * (ADC_NUMBER_OF_ACI_CHANNELS * (MEASURE_MAINS_PERIOD_US / ADC_SAMPLING_PERIOD_US)))
#define MEASURE_AC_BUFFER_DEPTH			2

#define MEASURE_CONTINUOUS

/*** MEASURE local structures ***/

typedef enum {
	MEASURE_STATE_STOPPED = 0,
	MEASURE_STATE_IDLE,
	MEASURE_STATE_PROCESS,
	MEASURE_STATE_LAST
} MEASURE_state_t;

typedef union {
	struct {
		unsigned zero_cross : 1;
		unsigned dma_transfer_end : 1;
		unsigned dma_buffer_index : 1;
	};
	uint8_t all;
} MEASURE_flags_t;

typedef struct {
	volatile int16_t data[MEASURE_AC_BUFFER_SIZE];
	uint16_t size;
} MEASURE_buffer_t;

typedef struct {
	MEASURE_state_t state;
	volatile MEASURE_flags_t flags;
	MEASURE_buffer_t acv_sampling[MEASURE_AC_BUFFER_DEPTH];
	uint8_t acv_buffer_read_idx;
	uint8_t acv_buffer_write_idx;
	MEASURE_buffer_t acix_sampling[MEASURE_AC_BUFFER_DEPTH];
	uint8_t acix_buffer_read_idx;
	uint8_t acix_buffer_write_idx;
} MEASURE_context_t;

/*** MEASURE local global variables ***/

static MEASURE_context_t measure_ctx;

/*** MEASURE local functions ***/

void _MEASURE_switch_dma_buffer(void) {
	// Stop DMA.
	DMA1_stop();
	// Retrieve number of transfered data.
	DMA1_get_number_of_transfered_data(&(measure_ctx.acv_sampling[measure_ctx.acv_buffer_write_idx].size), &(measure_ctx.acix_sampling[measure_ctx.acv_buffer_write_idx].size));
	// Update write indexes.
	measure_ctx.acv_buffer_write_idx = ((measure_ctx.acv_buffer_write_idx + 1) % MEASURE_AC_BUFFER_DEPTH);
	measure_ctx.acix_buffer_write_idx = ((measure_ctx.acix_buffer_write_idx + 1) % MEASURE_AC_BUFFER_DEPTH);
	// Set new address.
	DMA1_set_destination_address((uint32_t) &(measure_ctx.acv_sampling[measure_ctx.acv_buffer_write_idx].data), (uint32_t) &(measure_ctx.acix_sampling[measure_ctx.acix_buffer_write_idx].data), MEASURE_AC_BUFFER_SIZE);
	// Restart DMA.
	DMA1_start();
}

/* START MAINS MEASUREMENTS.
 * @param:			None.
 * @return status:	Function execution status.
 */
MEASURE_status_t _MEASURE_start(void) {
	// Local variables.
	MEASURE_status_t status = MEASURE_SUCCESS;
	ADC_status_t adc_status = ADC_SUCCESS;
	// Reset indexed.
	measure_ctx.acv_buffer_write_idx = 0;
	measure_ctx.acv_buffer_read_idx = 0;
	measure_ctx.acix_buffer_write_idx = 0;
	measure_ctx.acix_buffer_read_idx = 0;
	// Reset flags.
	measure_ctx.flags.all = 0;
	// Start DMA.
	DMA1_set_destination_address((uint32_t) &(measure_ctx.acv_sampling[measure_ctx.acv_buffer_write_idx].data), (uint32_t) &(measure_ctx.acix_sampling[measure_ctx.acix_buffer_write_idx].data), MEASURE_AC_BUFFER_SIZE);
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
MEASURE_status_t _MEASURE_stop(void) {
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

/* SET ZERO CROSS FLAG (CALLED BY EXTI INTERRUPT).
 * @param:	None.
 * @return:	None.
 */
void MEASURE_set_zero_cross_flag(void) {
	// Set local flag.
	measure_ctx.flags.zero_cross = 1;
}

/* SET DMA TIMEOUT FLAG (CALLED BY DMA INTERRUPT).
 * @param:	None.
 * @return:	None.
 */
void MEASURE_set_dma_transfer_end_flag(void) {
	// Set local flag.
	measure_ctx.flags.dma_transfer_end = 1;
}

/* MAIN TASK OF MAINS MEASUREMENTS.
 * @param:			None.
 * @return status:	Function execution status.
 */
MEASURE_status_t MEASURE_task(void) {
	// Local variables.
	MEASURE_status_t status = MEASURE_SUCCESS;
	int16_t* acv_data_ptr = NULL;
	uint16_t acv_data_size = 0;
	int16_t* acix_data_ptr = NULL;
	uint16_t acix_data_size = 0;
	uint8_t aci_idx = 0;
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
		// Check start request.
		if (measure_ctx.flags.zero_cross != 0) {
			// Clear flag.
			measure_ctx.flags.zero_cross = 0;
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
		if (measure_ctx.flags.dma_transfer_end != 0) {
			// Clear flag.
			measure_ctx.flags.dma_transfer_end = 0;
#else
		// Check zero cross flag.
		if (measure_ctx.flags.zero_cross != 0) {
			// Clear flag.
			measure_ctx.flags.zero_cross = 0;
#endif
			// Switch to next buffer.
			_MEASURE_switch_dma_buffer();
			// Update state.
			measure_ctx.state = MEASURE_STATE_PROCESS;
		}
#ifndef MEASURE_CONTINUOUS
		// Check DMA transfer end flag.
		if (measure_ctx.flags.dma_transfer_end != 0) {
			// Clear flag.
			measure_ctx.flags.dma_transfer_end = 0;
			// Stop measure.
			status = _MEASURE_stop();
			if (status != MEASURE_SUCCESS) goto errors;
			// Update state.
			measure_ctx.state = MEASURE_STATE_STOPPED;
		}
#endif
		break;
	case MEASURE_STATE_PROCESS:
		// Select last buffer.
		acv_data_ptr = &(measure_ctx.acv_sampling[measure_ctx.acv_buffer_read_idx].data);
		acix_data_ptr = &(measure_ctx.acix_sampling[measure_ctx.acix_buffer_read_idx].data);
		// Get size.
		acv_data_size = ((measure_ctx.acv_sampling[measure_ctx.acv_buffer_read_idx].size) / (ADC_NUMBER_OF_ACI_CHANNELS));
		acix_data_size = ((measure_ctx.acix_sampling[measure_ctx.acv_buffer_read_idx].size) / (ADC_NUMBER_OF_ACI_CHANNELS));
		// TODO processing.
		// Update read indexes.
		measure_ctx.acv_buffer_read_idx = ((measure_ctx.acv_buffer_read_idx + 1) % MEASURE_AC_BUFFER_DEPTH);
		measure_ctx.acix_buffer_read_idx = ((measure_ctx.acix_buffer_read_idx + 1) % MEASURE_AC_BUFFER_DEPTH);
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
