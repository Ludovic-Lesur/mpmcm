/*
 * power.c
 *
 *  Created on: 22 jul. 2023
 *      Author: Ludo
 */

#include "power.h"

#include "adc.h"
#include "gpio.h"
#include "lptim.h"
#include "mapping.h"
#include "types.h"

/*** POWER local macros ***/

#define POWER_ON_DELAY_MS_ANALOG	2000

/*** POWER local global variables ***/

static uint8_t power_domain_state[POWER_DOMAIN_LAST];

/*** POWER functions ***/

/*******************************************************************/
void POWER_init(void) {
	// Local variables.
	POWER_domain_t domain = 0;
	// Init power control pins.
	GPIO_configure(&GPIO_ANA_POWER_ENABLE, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	// Disable all domains by default.
	for (domain=0 ; domain<POWER_DOMAIN_LAST ; domain++) {
		POWER_disable(domain);
	}
}

/*******************************************************************/
POWER_status_t POWER_enable(POWER_domain_t domain, LPTIM_delay_mode_t delay_mode) {
	// Local variables.
	POWER_status_t status = POWER_SUCCESS;
	ADC_status_t adc_status = ADC_SUCCESS;
	LPTIM_status_t lptim1_status = LPTIM_SUCCESS;
	uint32_t delay_ms = 0;
	// Check domain.
	switch (domain) {
	case POWER_DOMAIN_ANALOG:
		// Turn analog front-end on.
		GPIO_write(&GPIO_ANA_POWER_ENABLE, 1);
		delay_ms = POWER_ON_DELAY_MS_ANALOG;
		// Init ADC.
		adc_status = ADC_init();
		ADC_exit_error(POWER_ERROR_BASE_ADC);
		break;
	default:
		status = POWER_ERROR_DOMAIN;
		goto errors;
	}
	// Update state.
	power_domain_state[domain] = 1;
	// Power on delay.
	if (delay_ms != 0) {
		lptim1_status = LPTIM1_delay_milliseconds(delay_ms, delay_mode);
		LPTIM1_exit_error(POWER_ERROR_BASE_LPTIM1);
	}
errors:
	return status;
}

/*******************************************************************/
POWER_status_t POWER_disable(POWER_domain_t domain) {
	// Local variables.
	POWER_status_t status = POWER_SUCCESS;
	ADC_status_t adc_status = ADC_SUCCESS;
	// Check domain.
	switch (domain) {
	case POWER_DOMAIN_ANALOG:
		// Turn analog front-end off and release ADC.
		GPIO_write(&GPIO_ANA_POWER_ENABLE, 0);
		// Release ADC.
		adc_status = ADC_de_init();
		ADC_exit_error(POWER_ERROR_BASE_ADC);
		break;
	default:
		status = POWER_ERROR_DOMAIN;
		goto errors;
	}
	// Update state.
	power_domain_state[domain] = 0;
errors:
	return status;
}

/*******************************************************************/
POWER_status_t POWER_get_state(POWER_domain_t domain, uint8_t* state) {
	// Local variables.
	POWER_status_t status = POWER_SUCCESS;
	// Check parameters.
	if (domain >= POWER_DOMAIN_LAST) {
		status = POWER_ERROR_DOMAIN;
		goto errors;
	}
	if (state == NULL) {
		status = POWER_ERROR_NULL_PARAMETER;
		goto errors;
	}
	(*state) = power_domain_state[domain];
errors:
	return status;
}
