/*
 * led.c
 *
 *  Created on: 22 aug. 2020
 *      Author: Ludo
 */

#include "led.h"

#include "error.h"
#include "error_base.h"
#include "gpio.h"
#include "gpio_mapping.h"
#include "maths.h"
#include "nvic_priority.h"
#include "tim.h"
#include "types.h"

/*** LED local macros ***/

#define LED_TIM_INSTANCE    TIM_INSTANCE_TIM4

/*** LED functions ***/

/*******************************************************************/
LED_status_t LED_init(void) {
    // Local variables.
    LED_status_t status = LED_SUCCESS;
    TIM_status_t tim_status = TIM_SUCCESS;
    // Init timers.
    tim_status = TIM_OPM_init(LED_TIM_INSTANCE, (TIM_gpio_t*) &GPIO_LED_TIM);
    TIM_exit_error(LED_ERROR_BASE_TIM_OPM);
errors:
    return status;
}

/*******************************************************************/
LED_status_t LED_de_init(void) {
    // Local variables.
    LED_status_t status = LED_SUCCESS;
    TIM_status_t tim_status = TIM_SUCCESS;
    // Release timers.
    tim_status = TIM_OPM_de_init(LED_TIM_INSTANCE, (TIM_gpio_t*) &GPIO_LED_TIM);
    TIM_exit_error(LED_ERROR_BASE_TIM_OPM);
errors:
    return status;
}

/*******************************************************************/
LED_status_t LED_single_pulse(uint32_t pulse_duration_ms, LED_color_t color) {
    // Local variables.
    LED_status_t status = LED_SUCCESS;
    TIM_status_t tim_status = TIM_SUCCESS;
    uint8_t channels_mask = 0;
    uint8_t idx = 0;
    // Check parameters.
    if (pulse_duration_ms == 0) {
        status = LED_ERROR_NULL_DURATION;
        goto errors;
    }
    if (color >= LED_COLOR_LAST) {
        status = LED_ERROR_COLOR;
        goto errors;
    }
    // Make pulse on required channels.
    for (idx = 0; idx < GPIO_LED_TIM_CHANNEL_INDEX_LAST; idx++) {
        // Apply color mask.
        channels_mask |= (((color >> idx) & 0x01) << ((GPIO_LED_TIM.list[idx])->channel));
    }
    // Make pulse on channel.
    tim_status = TIM_OPM_make_pulse(LED_TIM_INSTANCE, channels_mask, 0, (pulse_duration_ms * MATH_POWER_10[6]));
    TIM_stack_error(ERROR_BASE_TIM_LED);
errors:
    return status;
}

/*******************************************************************/
LED_state_t LED_get_state(void) {
    // Local variables.
    LED_state_t state = LED_STATE_OFF;
    TIM_status_t tim_status = TIM_SUCCESS;
    uint8_t pulse_is_done = 0;
    // Get status.
    tim_status = TIM_OPM_get_pulse_status(LED_TIM_INSTANCE, &pulse_is_done);
    TIM_stack_error(ERROR_BASE_TIM_LED);
    // Update state.
    state = (pulse_is_done == 0) ? LED_STATE_ACTIVE : LED_STATE_OFF;
    return state;
}
