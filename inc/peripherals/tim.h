/*
 * tim.h
 *
 *  Created on: 17 jun. 2023
 *      Author: Ludo
 */

#ifndef __TIM_H__
#define __TIM_H__

#include "types.h"

/*** TIM structures ***/

/*!******************************************************************
 * \enum TIM4_channel_t
 * \brief TIM channels mapping defined as (channel number - 1).
 *******************************************************************/
typedef enum {
	TIM4_CHANNEL_LED_RED = 0,
	TIM4_CHANNEL_LED_GREEN = 1,
	TIM4_CHANNEL_LED_BLUE = 3
} TIM4_channel_t;

/*!******************************************************************
 * \enum TIM4_channel_mask_t
 * \brief LED color bit mask defined as 0b<CH4><CH3><CH2><CH1>.
 *******************************************************************/
typedef enum {
	TIM4_CHANNEL_MASK_OFF = 0b0000,
	TIM4_CHANNEL_MASK_RED = (0b1 << TIM4_CHANNEL_LED_RED),
	TIM4_CHANNEL_MASK_GREEN = (0b1 << TIM4_CHANNEL_LED_GREEN),
	TIM4_CHANNEL_MASK_YELLOW = (0b1 << TIM4_CHANNEL_LED_RED) | (0b1 << TIM4_CHANNEL_LED_GREEN),
	TIM4_CHANNEL_MASK_BLUE = (0b1 << TIM4_CHANNEL_LED_BLUE),
	TIM4_CHANNEL_MASK_MAGENTA = (0b1 << TIM4_CHANNEL_LED_RED) | (0b1 << TIM4_CHANNEL_LED_BLUE),
	TIM4_CHANNEL_MASK_CYAN = (0b1 << TIM4_CHANNEL_LED_GREEN) | (0b1 << TIM4_CHANNEL_LED_BLUE),
	TIM4_CHANNEL_MASK_WHITE	= (0b1 << TIM4_CHANNEL_LED_RED) | (0b1 << TIM4_CHANNEL_LED_GREEN) | (0b1 << TIM4_CHANNEL_LED_BLUE)
} TIM4_channel_mask_t;

/*** TIM functions ***/

/*!******************************************************************
 * \fn void TIM4_init(void)
 * \brief Init TIM4 peripheral for RGB LED blinking operation.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void TIM4_init(void);

/*!******************************************************************
 * \fn void TIM4_single_pulse(uint32_t pulse_duration_ms, TIM4_channel_mask_t led_color)
 * \brief Perform single blink for a given color.
 * \param[in]	pulse_duration_ms: Blink duration in ms.
 * \param[in]  	led_color: LED color.
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void TIM4_single_pulse(uint32_t pulse_duration_ms, TIM4_channel_mask_t led_color);

/*!******************************************************************
 * \fn void TIM6_init(void)
 * \brief Init TIM6 peripheral for ADC trigger operation.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void TIM6_init(void);

/*!******************************************************************
 * \fn void TIM6_start(void)
 * \brief Start TIM6.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void TIM6_start(void);

/*!******************************************************************
 * \fn void TIM6_stop(void)
 * \brief Stop TIM6.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void TIM6_stop(void);

#endif /* __TIM_H__ */
