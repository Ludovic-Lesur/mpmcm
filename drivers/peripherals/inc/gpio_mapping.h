/*
 * gpio_mapping.h
 *
 *  Created on: 21 mar. 2023
 *      Author: Ludo
 */

#ifndef __GPIO_MAPPING_H__
#define __GPIO_MAPPING_H__

#include "adc.h"
#include "lpuart.h"
#include "gpio.h"
#include "tim.h"
#include "usart.h"

/*** GPIO mapping macros ***/

#define GPIO_ADC_CHANNEL_ACV_SAMPLING   ADC_CHANNEL_IN1
#define GPIO_ADC_CHANNEL_ACI1_SAMPLING  ADC_CHANNEL_IN2
#define GPIO_ADC_CHANNEL_ACI2_SAMPLING  ADC_CHANNEL_IN17
#define GPIO_ADC_CHANNEL_ACI3_SAMPLING  ADC_CHANNEL_IN3
#define GPIO_ADC_CHANNEL_ACI4_SAMPLING  ADC_CHANNEL_IN4

#define GPIO_TIM_CHANNEL_ACV_FREQUENCY  TIM_CHANNEL_1

#define GPIO_TIM_CHANNEL_LED_RED        TIM_CHANNEL_1
#define GPIO_TIM_CHANNEL_LED_GREEN      TIM_CHANNEL_2
#define GPIO_TIM_CHANNEL_LED_BLUE       TIM_CHANNEL_4

/*** GPIO MAPPING structures ***/

/*******************************************************************/
typedef enum {
    GPIO_ADC_CHANNEL_INDEX_ACV = 0,
    GPIO_ADC_CHANNEL_INDEX_ACI1,
    GPIO_ADC_CHANNEL_INDEX_ACI2,
    GPIO_ADC_CHANNEL_INDEX_ACI3,
    GPIO_ADC_CHANNEL_INDEX_ACI4,
    GPIO_ADC_CHANNEL_INDEX_LAST
} GPIO_adc_channel_index_t;

/*******************************************************************/
typedef enum {
    GPIO_LED_TIM_CHANNEL_INDEX_RED = 0,
    GPIO_LED_TIM_CHANNEL_INDEX_GREEN,
    GPIO_LED_TIM_CHANNEL_INDEX_BLUE,
    GPIO_LED_TIM_CHANNEL_INDEX_LAST
} GPIO_led_tim_channel_t;

/*******************************************************************/
typedef enum {
    GPIO_ACV_FREQUENCY_TIM_CHANNEL_INDEX = 0,
    GPIO_ACV_FREQUENCY_TIM_CHANNEL_INDEX_LAST
} GPIO_acv_frequency_tim_channel_t;

/*** GPIO MAPPING global variables ***/

// TCXO.
extern const GPIO_pin_t GPIO_TCXO_POWER_ENABLE;
// Analog inputs.
extern const GPIO_pin_t GPIO_ANA_POWER_ENABLE;
extern const ADC_gpio_t GPIO_ADC;
// Current sensors detectors.
extern const GPIO_pin_t GPIO_ACI1_DETECT;
extern const GPIO_pin_t GPIO_ACI2_DETECT;
extern const GPIO_pin_t GPIO_ACI3_DETECT;
extern const GPIO_pin_t GPIO_ACI4_DETECT;
// Zero cross detector.
extern const GPIO_pin_t GPIO_ZERO_CROSS_RAW;
extern const GPIO_pin_t GPIO_ZERO_CROSS_PULSE;
// Frequency measure.
extern const TIM_gpio_t GPIO_ACV_FREQUENCY_TIM;
// RS485.
extern const LPUART_gpio_t GPIO_RS485_LPUART;
// TIC interface.
extern const GPIO_pin_t GPIO_TIC_POWER_ENABLE;
extern const USART_gpio_t GPIO_TIC_USART;
// RGB LED.
extern const TIM_gpio_t GPIO_LED_TIM;
// Test points.
extern const GPIO_pin_t GPIO_TP1;
extern const GPIO_pin_t GPIO_MCO;

#endif /* __GPIO_MAPPING_H__ */
