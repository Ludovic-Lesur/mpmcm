/*
 * adc.c
 *
 *  Created on: 17 jun. 2023
 *      Author: Ludo
 */

#include "adc.h"

#include "adc_reg.h"
#include "gpio.h"
#include "lptim.h"
#include "mapping.h"
#include "rcc_reg.h"
#include "types.h"

/*** ADC local macros ***/

#define ADC_TIMEOUT_COUNT						1000000

#define ADC_FULL_SCALE_12_BITS					4095
#define ADC_OFFSET_12_BITS						((ADC_FULL_SCALE_12_BITS / 2) + 1)
#define ADC_NUMBER_OF_OFFSETS					4

#define ADC_SMPR_CHANNEL_THRESHOLD				10
#define ADC_SAMPLING_TIME						0b110

#define ADC_VBIAS_COMPENSATION

/*** ADC local structures ***/

/*******************************************************************/
typedef enum {
	ADC_CHANNEL_ACV_SAMPLING = 1,
	ADC_CHANNEL_ACI1_SAMPLING = 2,
	ADC_CHANNEL_ACI2_SAMPLING = 17,
	ADC_CHANNEL_ACI3_SAMPLING = 3,
	ADC_CHANNEL_ACI4_SAMPLING = 4
} ADC_channels_t;

/*** ADC local global variables ***/

static const ADC_channels_t ADC1_REGULAR_CHANNELS[ADC_NUMBER_OF_ACI_CHANNELS] = {
	ADC_CHANNEL_ACV_SAMPLING,
	ADC_CHANNEL_ACV_SAMPLING,
	ADC_CHANNEL_ACV_SAMPLING,
	ADC_CHANNEL_ACV_SAMPLING
};

static const ADC_channels_t ADC2_REGULAR_CHANNELS[ADC_NUMBER_OF_ACI_CHANNELS] = {
	ADC_CHANNEL_ACI1_SAMPLING,
	ADC_CHANNEL_ACI2_SAMPLING,
	ADC_CHANNEL_ACI3_SAMPLING,
	ADC_CHANNEL_ACI4_SAMPLING
};

/*** ADC local functions ***/

/*******************************************************************/
static ADC_status_t _ADC_enable_regulator(ADC_registers_t* ADC) {
	// Local variables.
	ADC_status_t status = ADC_SUCCESS;
	LPTIM_status_t lptim1_status = LPTIM_SUCCESS;
	// Exit deep power down.
	ADC -> CR &= ~(0b1 << 29);
	// Enable voltage regulator.
	ADC -> CR |= (0b1 << 28);
	lptim1_status = LPTIM1_delay_milliseconds(10, LPTIM_DELAY_MODE_ACTIVE);
	LPTIM1_exit_error(ADC_ERROR_BASE_LPTIM1);
errors:
	return status;
}

/*******************************************************************/
static ADC_status_t _ADC_calibrate(ADC_registers_t* ADC) {
	// Local variables.
	ADC_status_t status = ADC_SUCCESS;
	LPTIM_status_t lptim1_status = LPTIM_SUCCESS;
	uint32_t loop_count = 0;
	// Start calibration in single-ended mode.
	ADC -> CR &= ~(0b1 << 30);
	ADC -> CR |= (0b1 << 31);
	// Wait for calibration to complete.
	loop_count = 0;
	while (((ADC -> CR) & (0b1 << 31)) != 0) {
		// Wait until calibration is done or timeout.
		loop_count++;
		if (loop_count > ADC_TIMEOUT_COUNT) {
			status = ADC_ERROR_CALIBRATION;
			goto errors;
		}
	}
	lptim1_status = LPTIM1_delay_milliseconds(10, LPTIM_DELAY_MODE_ACTIVE);
	LPTIM1_exit_error(ADC_ERROR_BASE_LPTIM1);
errors:
	return status;
}

/*******************************************************************/
static ADC_status_t _ADC_init(ADC_registers_t* ADC, ADC_channels_t* ADC_REGULAR_CHANNELS) {
	// Local variables.
	ADC_status_t status = ADC_SUCCESS;
	uint8_t idx = 0;
	// Check parameters.
	if ((ADC == NULL) || (ADC_REGULAR_CHANNELS == NULL)) {
		status = ADC_ERROR_NULL_PARAMETER;
		goto errors;
	}
	// Clear ready flag.
	if (((ADC -> ISR) & (0b1 << 0)) != 0) {
		ADC -> ISR |= (0b1 << 0); // ADRDY='1'.
	}
	// Ensure ADC is disabled.
	if (((ADC -> CR) & (0b1 << 0)) != 0) {
		ADC -> CR |= (0b1 << 1); // ADDIS='1'.
	}
	// Single conversion mode.
	ADC -> CFGR &= ~(0b1 << 13);
	ADC -> CFGR &= ~(0b1 << 16);
	// 12-bits resolution and right alignment.
	ADC -> CFGR &= ~(0b1 << 15);
	ADC -> CFGR &= ~(0b11 << 3);
	// Use rising edge hardware trigger
	// Event 0b01101 = TIM6_TRGO.
	ADC -> CFGR |= (0b01 << 10);
	ADC -> CFGR |= (0b01101 << 5);
	// Overwritte register in case of overrun.
	ADC -> CFGR |= (0b1 << 12);
	// Enable circular DMA.
	ADC -> CFGR |= (0b11 << 0);
	// Sampling time.
	// 247.5 ADC clock cycles on 12 bits resolution with Fadc=8MHz: Tconv=32.5µs per channel.
	// For 4 channels regular group (4 voltage / current pairs): Tconv=130µs < 200µs (5kHz sampling frequency).
	for (idx=0 ; idx<ADC_NUMBER_OF_ACI_CHANNELS ; idx++) {
		// Check channel.
		if (ADC_REGULAR_CHANNELS[idx] < ADC_SMPR_CHANNEL_THRESHOLD) {
			ADC -> SMPR1 |= (ADC_SAMPLING_TIME << (3 * ADC_REGULAR_CHANNELS[idx]));
		}
		else {
			ADC -> SMPR2 |= (ADC_SAMPLING_TIME << (3 * (ADC_REGULAR_CHANNELS[idx] - ADC_SMPR_CHANNEL_THRESHOLD)));
		}
	}
	// Regular sequence definition.
	// Length.
	ADC -> SQR1 = ((ADC_NUMBER_OF_ACI_CHANNELS - 1) & 0x0000000F);
	// Channels sequence.
	for (idx=0 ; idx<ADC_NUMBER_OF_ACI_CHANNELS ; idx++) {
		ADC -> SQR[idx >> 2] |= (ADC_REGULAR_CHANNELS[idx] << (6 * ((idx % 4) + 1)));
	}
#ifdef ADC_VBIAS_COMPENSATION
	// Configure VBIAS offset.
	for (idx=0 ; idx<ADC_NUMBER_OF_ACI_CHANNELS ; idx++) {
		// Check index.
		if (idx >= ADC_NUMBER_OF_OFFSETS) break;
		// Program offset.
		ADC -> OFR[idx] = ADC_OFFSET_12_BITS & 0x00000FFF;
		ADC -> OFR[idx] |= ADC_REGULAR_CHANNELS[idx] << 26;
		ADC -> OFR[idx] |= 0b1 << 31;
	}
#endif
errors:
	return status;
}

/*******************************************************************/
static ADC_status_t _ADC_start(ADC_registers_t* ADC) {
	// Local variables.
	ADC_status_t status = ADC_SUCCESS;
	uint32_t loop_count = 0;
	// Enable ADC.
	ADC -> CR |= (0b1 << 0);
	// Wait for ADC to be ready.
	while (((ADC -> ISR) & (0b1 << 0)) == 0) {
		// Wait for ADRDY='1' or timeout.
		loop_count++;
		if (loop_count > ADC_TIMEOUT_COUNT) {
			status = ADC_ERROR_READY_TIMEOUT;
			goto errors;
		}
	}
	// Start regular conversions.
	ADC -> CR |= (0b1 << 2);
errors:
	return status;
}

/*******************************************************************/
static ADC_status_t _ADC_stop(ADC_registers_t* ADC) {
	// Local variables.
	ADC_status_t status = ADC_SUCCESS;
	uint32_t loop_count = 0;
	// Stop ongoing conversions.
	ADC -> CR |= (0b1 << 4);
	// Wait for ongoing conversions to complete.
	while (((ADC -> CR) & (0b1 << 4)) != 0) {
		// Wait command completion or timeout.
		loop_count++;
		if (loop_count > ADC_TIMEOUT_COUNT) break;
	}
	// Disable ADC.
	if (((ADC -> CR) & (0b1 << 0)) != 0) {
		ADC -> CR |= (0b1 << 1); // ADDIS='1'.
	}
	return status;
}

/*******************************************************************/
static ADC_status_t _ADC_disable(ADC_registers_t* ADC) {
	// Local variables.
	ADC_status_t status = ADC_SUCCESS;
	uint32_t loop_count = 0;
	// Check ADC state.
	if (((ADC -> CR) & (0b1 << 0)) == 0) goto errors; // Not an error but to exit directly.
	// Disable ADC.
	ADC -> CR |= (0b1 << 1); // ADDIS='1'.
	// Wait for ADC to be disabled.
	while (((ADC -> CR) & (0b1 << 0)) != 0) {
		// Exit if timeout.
		loop_count++;
		if (loop_count > ADC_TIMEOUT_COUNT) {
			status = ADC_ERROR_DISABLE_TIMEOUT;
			break;
		}
	}
errors:
	// Disable voltage regulator.
	ADC -> CR &= ~(0b1 << 28);
	return status;
}

/*** ADC functions ***/

/*******************************************************************/
ADC_status_t ADC_init(void) {
	// Local variables.
	ADC_status_t status = ADC_SUCCESS;
	// Enable peripheral clock (PLLP).
	RCC -> CCIPR |= (0b01 << 28);
	RCC -> AHB2ENR |= (0b1 << 13);
	// Configure pins.
	GPIO_configure(&GPIO_ACV_SAMPLING, GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_configure(&GPIO_ACI1_SAMPLING, GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_configure(&GPIO_ACI2_SAMPLING, GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_configure(&GPIO_ACI3_SAMPLING, GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_configure(&GPIO_ACI4_SAMPLING, GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	// Use adc_ker_ck.
	ADCCR12 -> CCR &= ~(0b11 << 16);
	// ADC clocked on PLL with no prescaler -> Fadc = 8Mhz.
	ADCCR12 -> CCR &= ~(0b1111 << 18);
	// Use independant circular DMA channel for each ADC.
	ADCCR12 -> CCR &= ~(0b11 << 14);
	ADCCR12 -> CCR |= (0b1 << 13);
	// Dual mode (regular simultaneous only).
	ADCCR12 -> CCR |= (0b00110 << 0);
	// Common initialization of master and slave ADCs.
	status = _ADC_init(ADC1, (ADC_channels_t*) ADC1_REGULAR_CHANNELS);
	if (status != ADC_SUCCESS) goto errors;
	status = _ADC_init(ADC2, (ADC_channels_t*) ADC2_REGULAR_CHANNELS);
	if (status != ADC_SUCCESS) goto errors;
	// Enable regulator.
	status = _ADC_enable_regulator(ADC1);
	if (status != ADC_SUCCESS) goto errors;
	status = _ADC_enable_regulator(ADC2);
	if (status != ADC_SUCCESS) goto errors;
	// Calibration.
	status = _ADC_calibrate(ADC1);
	if (status != ADC_SUCCESS) goto errors;
	status = _ADC_calibrate(ADC2);
	if (status != ADC_SUCCESS) goto errors;
errors:
	return status;
}

/*******************************************************************/
ADC_status_t ADC_de_init(void) {
	// Local variables.
	ADC_status_t status = ADC_SUCCESS;
	// Disable all ADCs.
	status = _ADC_disable(ADC1);
	if (status != ADC_SUCCESS) goto errors;
	status = _ADC_disable(ADC2);
	if (status != ADC_SUCCESS) goto errors;
errors:
	// Disable peripheral clock.
	RCC -> AHB2ENR &= ~(0b1 << 13);
	return status;
}

/*******************************************************************/
ADC_status_t ADC_start(void) {
	// Local variables.
	ADC_status_t status = ADC_SUCCESS;
	// Enable ADCs.
	status = _ADC_start(ADC1);
	if (status != ADC_SUCCESS) goto errors;
	status = _ADC_start(ADC2);
	if (status != ADC_SUCCESS) goto errors;
errors:
	return status;
}

/*******************************************************************/
ADC_status_t ADC_stop(void) {
	// Local variables.
	ADC_status_t status = ADC_SUCCESS;
	// Stop ongoing conversion.
	status = _ADC_stop(ADC1);
	if (status != ADC_SUCCESS) goto errors;
	status = _ADC_stop(ADC2);
	if (status != ADC_SUCCESS) goto errors;
errors:
	return status;
}
