/*
 * led.h
 *
 *  Created on: 01 oct. 2023
 *      Author: Ludo
 */

#ifndef __LED_H__
#define __LED_H__

#include "tim.h"
#include "types.h"

/*** LED structures ***/

/*!******************************************************************
 * \enum LED_status_t
 * \brief LED driver error codes.
 *******************************************************************/
typedef enum {
	// Driver errors.
	LED_SUCCESS,
	LED_ERROR_NULL_DURATION,
	LED_ERROR_COLOR,
	// Last base value.
	LED_ERROR_BASE_LAST = 0x0100
} LED_status_t;

/*!******************************************************************
 * \enum LED_color_t
 * \brief LED colors list.
 *******************************************************************/
typedef enum {
	LED_COLOR_OFF = TIM4_CHANNEL_MASK_OFF,
	LED_COLOR_RED = TIM4_CHANNEL_MASK_RED,
	LED_COLOR_GREEN = TIM4_CHANNEL_MASK_GREEN,
	LED_COLOR_YELLOW = TIM4_CHANNEL_MASK_YELLOW,
	LED_COLOR_BLUE = TIM4_CHANNEL_MASK_BLUE,
	LED_COLOR_MAGENTA = TIM4_CHANNEL_MASK_MAGENTA,
	LED_COLOR_CYAN = TIM4_CHANNEL_MASK_CYAN,
	LED_COLOR_WHITE = TIM4_CHANNEL_MASK_WHITE,
	LED_COLOR_LAST
} LED_color_t;

/*** LED functions ***/

/*!******************************************************************
 * \fn void LED_init(void)
 * \brief Init LED driver.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void LED_init(void);

/*!******************************************************************
 * \fn LED_status_t LED_single_pulse(uint32_t pulse_duration_ms, LED_color_t color)
 * \brief Start single blink.
 * \param[in]  	pulse_duration_ms: Blink duration in ms.
 * \param[in]	color: LED color.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
LED_status_t LED_single_pulse(uint32_t pulse_duration_ms, LED_color_t color);

/*******************************************************************/
#define LED_exit_error(error_base) { if (led_status != LED_SUCCESS) { status = (error_base + led_status); goto errors; } }

/*******************************************************************/
#define LED_stack_error(void) { if (led_status != LED_SUCCESS) { ERROR_stack_add(ERROR_BASE_LED + led_status); } }

/*******************************************************************/
#define LED_stack_exit_error(error_code) { if (led_status != LED_SUCCESS) { ERROR_stack_add(ERROR_BASE_LED + led_status); status = error_code; goto errors; } }

#endif /* __LED_H__ */
