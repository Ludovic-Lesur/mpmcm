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
#include "tim.h"

/*** MEASURE local macros ***/

#define MEASURE_TIMEOUT_COUNT			10000000
#define MEASURE_MAINS_PERIOD_US			20000
#define MEASURE_NUMBER_OF_ACI_INPUTS	4
// Note: factor 2 is used to add a margin to the buffer length (2 mains periods long instead of 1).
// Buffer switch is triggered by the zero cross detection instead of a fixed number of samples.
#define MEASURE_AC_BUFFER_SIZE			(2 * (MEASURE_NUMBER_OF_ACI_INPUTS * (MEASURE_MAINS_PERIOD_US / ADC_SAMPLING_PERIOD_US)))

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
		unsigned dma_timeout : 1;
		unsigned dma_buffer_index : 1;
	};
	uint8_t all;
} MEASURE_flags_t;

typedef struct {
	volatile uint16_t buffer_0[MEASURE_AC_BUFFER_SIZE];
	volatile uint16_t buffer_1[MEASURE_AC_BUFFER_SIZE];
} MEASURE_buffer_t;

typedef struct {
	MEASURE_state_t state;
	volatile MEASURE_flags_t flags;
	MEASURE_buffer_t acv_sampling;
	MEASURE_buffer_t acix_sampling;
} MEASURE_context_t;

/*** MEASURE local global variables ***/

static MEASURE_context_t measure_ctx;

/*** MEASURE local functions ***/

/* START MAINS MEASUREMENTS.
 * @param:			None.
 * @return status:	Function execution status.
 */
MEASURE_status_t _MEASURE_start(void) {
	// Local variables.
	MEASURE_status_t status = MEASURE_SUCCESS;
	ADC_status_t adc_status = ADC_SUCCESS;
	// Start DMA with buffer 0
	DMA1_set_destination_address((uint32_t) &(measure_ctx.acv_sampling.buffer_0), (uint32_t) &(measure_ctx.acix_sampling.buffer_0), MEASURE_AC_BUFFER_SIZE);
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
	measure_ctx.flags.all = 0;
	// Init zero cross pulse GPIO.
	GPIO_configure(&GPIO_ZERO_CROSS_PULSE, GPIO_MODE_INPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	EXTI_configure_gpio(&GPIO_ZERO_CROSS_PULSE, EXTI_TRIGGER_RISING_EDGE);
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
void MEASURE_set_dma_timeout_flag(void) {
	// Set local flag.
	measure_ctx.flags.dma_timeout = 1;
}

/* MAIN TASK OF MAINS MEASUREMENTS.
 * @param:			None.
 * @return status:	Function execution status.
 */
MEASURE_status_t MEASURE_task(void) {
	// Local variables.
	MEASURE_status_t status = MEASURE_SUCCESS;
	// Perform state machine.
	switch (measure_ctx.state) {
	case MEASURE_STATE_STOPPED:
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
		break;
	case MEASURE_STATE_IDLE:
		// Check zero cross flag.
		if (measure_ctx.flags.zero_cross != 0) {
			// Clear flag.
			measure_ctx.flags.zero_cross = 0;
			// Perform double buffer mode.
			if (measure_ctx.flags.dma_buffer_index == 0) {
				// Switch DMA to buffer 1.
				DMA1_set_destination_address((uint32_t) &(measure_ctx.acv_sampling.buffer_1), (uint32_t) &(measure_ctx.acix_sampling.buffer_1), MEASURE_AC_BUFFER_SIZE);
				measure_ctx.flags.dma_buffer_index = 1;
			}
			else {
				// Switch DMA to buffer 0.
				DMA1_set_destination_address((uint32_t) &(measure_ctx.acv_sampling.buffer_0), (uint32_t) &(measure_ctx.acix_sampling.buffer_0), MEASURE_AC_BUFFER_SIZE);
				measure_ctx.flags.dma_buffer_index = 0;
			}
			// Update state.
			measure_ctx.state = MEASURE_STATE_PROCESS;
		}
		// Check DMA timeout flag.
		if (measure_ctx.flags.dma_timeout != 0) {
			// Clear flag.
			measure_ctx.flags.dma_timeout = 0;
			// Stop measure.
			status = _MEASURE_stop();
			if (status != MEASURE_SUCCESS) goto errors;
			// Update state.
			measure_ctx.state = MEASURE_STATE_STOPPED;
		}
		break;
	case MEASURE_STATE_PROCESS:
		// Select free buffer.
		if (measure_ctx.flags.dma_buffer_index == 0) {
			// Process buffer 1.
		}
		else {
			// Process buffer 0.
		}
		// TODO processing.
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
