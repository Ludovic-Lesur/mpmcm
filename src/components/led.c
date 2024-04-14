/*
 * led.c
 *
 *  Created on: 01 oct. 2023
 *      Author: Ludo
 */

#include "led.h"

#include "gpio.h"
#include "mapping.h"
#include "mode.h"
#include "tim.h"
#include "types.h"

/*** LED local functions ***/

/*******************************************************************/
static void _LED_off(void) {
	// Configure pins as output high.
	GPIO_write(&GPIO_LED_RED, 1);
	GPIO_write(&GPIO_LED_GREEN, 1);
	GPIO_write(&GPIO_LED_BLUE, 1);
	GPIO_configure(&GPIO_LED_RED, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_configure(&GPIO_LED_GREEN, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_configure(&GPIO_LED_BLUE, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
}

/*** LED functions ***/

/*******************************************************************/
LED_status_t LED_init(void) {
	// Local variables.
	LED_status_t status = LED_SUCCESS;
	TIM_status_t tim4_status = TIM_SUCCESS;
	// Init timer.
	tim4_status = TIM4_init();
	TIM4_exit_error(LED_ERROR_BASE_TIM4);
errors:
	// Turn LED off.
	_LED_off();
	return status;
}

/*******************************************************************/
LED_status_t LED_single_pulse(uint32_t pulse_duration_ms, LED_color_t color) {
	// Local variables.
	LED_status_t status = LED_SUCCESS;
	GPIO_mode_t gpio_mode = GPIO_MODE_OUTPUT;
	// Check parameters.
	if (pulse_duration_ms == 0) {
		status = LED_ERROR_NULL_DURATION;
		goto errors;
	}
	if (color >= LED_COLOR_LAST) {
		status = LED_ERROR_COLOR;
		goto errors;
	}
	// Link required GPIOs to timer.
	gpio_mode = (color & TIM4_CHANNEL_MASK_RED) ? GPIO_MODE_ALTERNATE_FUNCTION : GPIO_MODE_OUTPUT;
	GPIO_configure(&GPIO_LED_RED, gpio_mode, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	gpio_mode = (color & TIM4_CHANNEL_MASK_GREEN) ? GPIO_MODE_ALTERNATE_FUNCTION : GPIO_MODE_OUTPUT;
	GPIO_configure(&GPIO_LED_GREEN, gpio_mode, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	gpio_mode = (color & TIM4_CHANNEL_MASK_BLUE) ? GPIO_MODE_ALTERNATE_FUNCTION : GPIO_MODE_OUTPUT;
	GPIO_configure(&GPIO_LED_BLUE, gpio_mode, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	// Make pulse.
	TIM4_single_pulse(pulse_duration_ms, color);
errors:
	return status;
}

/*******************************************************************/
LED_state_t LED_get_state(void) {
	// Local variables.
	LED_state_t state = (TIM4_is_single_pulse_done() == 0) ? LED_STATE_ACTIVE : LED_STATE_OFF;
	return state;
}
