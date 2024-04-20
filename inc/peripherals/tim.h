/*
 * tim.h
 *
 *  Created on: 17 jun. 2023
 *      Author: Ludo
 */

#ifndef __TIM_H__
#define __TIM_H__

#include "mode.h"
#include "rcc.h"
#include "types.h"

/*** TIM structures ***/

/*!******************************************************************
 * \enum TIM_status_t
 * \brief TIM driver error codes.
 *******************************************************************/
typedef enum {
	// Driver errors.
	TIM_SUCCESS = 0,
	TIM_ERROR_NULL_PARAMETER,
	TIM_ERROR_CAPTURE_TIMEOUT,
	// Low level drivers errors.
	TIM_ERROR_BASE_RCC = 0x0100,
	// Last base value.
	TIM_ERROR_BASE_LAST = (TIM_ERROR_BASE_RCC + RCC_ERROR_BASE_LAST)
} TIM_status_t;

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

#ifdef ANALOG_SIMULATION
/*!******************************************************************
 * \fn TIM_completion_irq_cb_t
 * \brief TIM completion callback.
 *******************************************************************/
typedef void (*TIM_completion_irq_cb_t)(void);
#endif

/*** TIM functions ***/

/*!******************************************************************
 * \fn TIM_status_t TIM2_init(uint32_t sampling_frequency_hz)
 * \brief Init TIM2 peripheral for mains voltage frequency measurement.
 * \param[in]  	sampling_frequency_hz: Input capture sampling frequency in Hz.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
TIM_status_t TIM2_init(uint32_t sampling_frequency_hz);

/*!******************************************************************
 * \fn void TIM2_de_init(void)
 * \brief Release TIM2 driver.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void TIM2_de_init(void);

/*!******************************************************************
 * \fn void TIM2_start(void)
 * \brief Start TIM2 operation.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void TIM2_start(void);

/*!******************************************************************
 * \fn void TIM2_stop(void)
 * \brief Stop TIM2 operation.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void TIM2_stop(void);

/*!******************************************************************
 * \fn TIM_status_t TIM4_init(void)
 * \brief Init TIM4 peripheral for RGB LED blinking operation.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
TIM_status_t TIM4_init(void);

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
 * \fn uint8_t TIM4_is_single_pulse_done(void)
 * \brief Get the pulse status.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		0 if the pulse is running, 1 if it is complete.
 *******************************************************************/
uint8_t TIM4_is_single_pulse_done(void);

/*!******************************************************************
 * \fn TIM_status_t TIM6_init(void)
 * \brief Init TIM6 peripheral for ADC trigger operation.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
TIM_status_t TIM6_init(void);

/*!******************************************************************
 * \fn void TIM6_de_init(void)
 * \brief Release TIM6 driver.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void TIM6_de_init(void);

/*!******************************************************************
 * \fn void TIM6_start(void)
 * \brief Start TIM6 operation.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void TIM6_start(void);

/*!******************************************************************
 * \fn void TIM6_stop(void)
 * \brief Stop TIM6 operation.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void TIM6_stop(void);

#ifdef ANALOG_SIMULATION
/*!******************************************************************
 * \fn TIM_status_t TIM15_init(uint32_t frequency_hz, TIM_completion_irq_cb_t irq_callback)
 * \brief Init TIM15 peripheral for zero cross IRQ simulation.
 * \param[in]	frequency_hz: Timer frequency in Hz.
 * \param[in]  	irq_callback: Function to call on timer completion.
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void TIM15_init(uint32_t frequency_hz, TIM_completion_irq_cb_t irq_callback);
#endif

#ifdef ANALOG_SIMULATION
/*!******************************************************************
 * \fn void TIM15_de_init(void)
 * \brief Release TIM15 driver.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void TIM15_de_init(void);
#endif

#ifdef ANALOG_SIMULATION
/*!******************************************************************
 * \fn void TIM15_start(void)
 * \brief Start TIM15 operation.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void TIM15_start(void);
#endif

#ifdef ANALOG_SIMULATION
/*!******************************************************************
 * \fn void TIM15_stop(void)
 * \brief Stop TIM15 operation.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void TIM15_stop(void);
#endif

/*!******************************************************************
 * \fn void TIM17_init(void)
 * \brief Init TIM17 peripheral for internal oscillators frequency measurement.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void TIM17_init(void);

/*!******************************************************************
 * \fn void TIM17_de_init(void)
 * \brief Release TIM17 peripheral.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void TIM17_de_init(void);

/*!******************************************************************
 * \fn TIM_status_t TIM17_mco_capture(uint16_t* ref_clock_pulse_count, uint16_t* mco_pulse_count)
 * \brief Perform MCO clock capture.
 * \param[in]  	none
 * \param[out] 	ref_clock_pulse_count: Pointer to the number of pulses of the timer reference clock during the capture.
 * \param[out]	mco_pulse_count: Pointer to the number of pulses of the MCO clock during the capture.
 * \retval		Function execution status.
 *******************************************************************/
TIM_status_t TIM17_mco_capture(uint16_t* ref_clock_pulse_count, uint16_t* mco_pulse_count);

/*******************************************************************/
#define TIM2_exit_error(error_base) { if (tim2_status != TIM_SUCCESS) { status = (error_base + tim2_status); goto errors; } }

/*******************************************************************/
#define TIM2_stack_error(void) { if (tim2_status != TIM_SUCCESS) { ERROR_stack_add(ERROR_BASE_TIM2 + tim2_status); } }

/*******************************************************************/
#define TIM2_stack_exit_error(error_code) { if (tim2_status != TIM_SUCCESS) { ERROR_stack_add(ERROR_BASE_TIM2 + tim2_status); status = error_code; goto errors; } }

/*******************************************************************/
#define TIM4_exit_error(error_base) { if (tim4_status != TIM_SUCCESS) { status = (error_base + tim4_status); goto errors; } }

/*******************************************************************/
#define TIM4_stack_error(void) { if (tim4_status != TIM_SUCCESS) { ERROR_stack_add(ERROR_BASE_TIM4 + tim4_status); } }

/*******************************************************************/
#define TIM4_stack_exit_error(error_code) { if (tim4_status != TIM_SUCCESS) { ERROR_stack_add(ERROR_BASE_TIM4 + tim4_status); status = error_code; goto errors; } }

/*******************************************************************/
#define TIM6_exit_error(error_base) { if (tim6_status != TIM_SUCCESS) { status = (error_base + tim6_status); goto errors; } }

/*******************************************************************/
#define TIM6_stack_error(void) { if (tim6_status != TIM_SUCCESS) { ERROR_stack_add(ERROR_BASE_TIM6 + tim6_status); } }

/*******************************************************************/
#define TIM6_stack_exit_error(error_code) { if (tim6_status != TIM_SUCCESS) { ERROR_stack_add(ERROR_BASE_TIM6 + tim6_status); status = error_code; goto errors; } }

/*******************************************************************/
#define TIM17_exit_error(error_base) { if (tim17_status != TIM_SUCCESS) { status = (error_base + tim17_status); goto errors; } }

/*******************************************************************/
#define TIM17_stack_error(void) { if (tim17_status != TIM_SUCCESS) { ERROR_stack_add(ERROR_BASE_TIM17 + tim17_status); } }

/*******************************************************************/
#define TIM17_stack_exit_error(error_code) { if (tim17_status != TIM_SUCCESS) { ERROR_stack_add(ERROR_BASE_TIM17 + tim17_status); status = error_code; goto errors; } }

#endif /* __TIM_H__ */
