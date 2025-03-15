/*
 * mcu_mapping.h
 *
 *  Created on: 21 mar. 2023
 *      Author: Ludo
 */

#ifndef __MCU_MAPPING_H__
#define __MCU_MAPPING_H__

#include "adc.h"
#include "lpuart.h"
#include "gpio.h"
#include "tim.h"
#include "usart.h"

/*** MCU MAPPING macros ***/

#define ADC_INSTANCE_ACX_SAMPLING   ADC_INSTANCE_ADC1
#define ADC_CHANNEL_ACV_SAMPLING    ADC_CHANNEL_IN1
#define ADC_CHANNEL_ACI1_SAMPLING   ADC_CHANNEL_IN2
#define ADC_CHANNEL_ACI2_SAMPLING   ADC_CHANNEL_IN17
#define ADC_CHANNEL_ACI3_SAMPLING   ADC_CHANNEL_IN3
#define ADC_CHANNEL_ACI4_SAMPLING   ADC_CHANNEL_IN4

#define ADC_INSTANCE_ANALOG         ADC_INSTANCE_ADC1

#define DMA_INSTANCE_ACV_SAMPLING   DMA_INSTANCE_DMA1
#define DMA_CHANNEL_ACV_SAMPLING    DMA_CHANNEL_1

#define DMA_INSTANCE_ACI_SAMPLING   DMA_INSTANCE_DMA1
#define DMA_CHANNEL_ACI_SAMPLING    DMA_CHANNEL_2

#define DMA_INSTANCE_ACV_FREQUENCY  DMA_INSTANCE_DMA1
#define DMA_CHANNEL_ACV_FREQUENCY   DMA_CHANNEL_3

#define DMA_INSTANCE_TIC            DMA_INSTANCE_DMA1
#define DMA_CHANNEL_TIC             DMA_CHANNEL_4

#define TIM_INSTANCE_ADC_TRIGGER    TIM_INSTANCE_TIM6

#define TIM_INSTANCE_SIMULATION     TIM_INSTANCE_TIM15

#define TIM_INSTANCE_ACV_FREQUENCY  TIM_INSTANCE_TIM2
#define TIM_CHANNEL_ACV_FREQUENCY   TIM_CHANNEL_1

#define TIM_INSTANCE_LED            TIM_INSTANCE_TIM4
#define TIM_CHANNEL_LED_RED         TIM_CHANNEL_1
#define TIM_CHANNEL_LED_GREEN       TIM_CHANNEL_2
#define TIM_CHANNEL_LED_BLUE        TIM_CHANNEL_4

#define USART_INSTANCE_TIC          USART_INSTANCE_USART2

/*** MCU MAPPING structures ***/

/*!******************************************************************
 * \enum ADC_channel_index_t
 * \brief ADC channels index.
 *******************************************************************/
typedef enum {
    ADC_CHANNEL_INDEX_ACV_SAMPLING = 0,
    ADC_CHANNEL_INDEX_ACI1_SAMPLING,
    ADC_CHANNEL_INDEX_ACI2_SAMPLING,
    ADC_CHANNEL_INDEX_ACI3_SAMPLING,
    ADC_CHANNEL_INDEX_ACI4_SAMPLING,
    ADC_CHANNEL_INDEX_LAST
} ADC_channel_index_t;

/*!******************************************************************
 * \enum TIM_channel_index_led_t
 * \brief TIM RGB LED channels index.
 *******************************************************************/
typedef enum {
    TIM_CHANNEL_INDEX_LED_RED = 0,
    TIM_CHANNEL_INDEX_LED_GREEN,
    TIM_CHANNEL_INDEX_LED_BLUE,
    TIM_CHANNEL_INDEX_LED_LAST
} TIM_channel_index_led_t;

/*!******************************************************************
 * \enum TIM_channel_index_acv_frequency_t
 * \brief TIM ACV frequency channels index.
 *******************************************************************/
typedef enum {
    TIM_CHANNEL_INDEX_ACV_FREQUENCY = 0,
    TIM_CHANNEL_INDEX_ACV_FREQUENCY_LAST
} TIM_channel_index_acv_frequency_t;

/*** MCU MAPPING global variables ***/

// TCXO.
extern const GPIO_pin_t GPIO_TCXO_POWER_ENABLE;
// Analog inputs.
extern const GPIO_pin_t GPIO_ANA_POWER_ENABLE;
extern const ADC_gpio_t ADC_GPIO;
// Current sensors detectors.
extern const GPIO_pin_t GPIO_ACI1_DETECT;
extern const GPIO_pin_t GPIO_ACI2_DETECT;
extern const GPIO_pin_t GPIO_ACI3_DETECT;
extern const GPIO_pin_t GPIO_ACI4_DETECT;
// Zero cross detector.
extern const GPIO_pin_t GPIO_ZERO_CROSS_RAW;
extern const GPIO_pin_t GPIO_ZERO_CROSS_PULSE;
// Frequency measure.
extern const TIM_gpio_t TIM_GPIO_ACV_FREQUENCY;
// RS485.
extern const LPUART_gpio_t LPUART_GPIO_RS485;
// TIC interface.
extern const GPIO_pin_t GPIO_TIC_POWER_ENABLE;
extern const USART_gpio_t USART_GPIO_TIC;
// RGB LED.
extern const TIM_gpio_t TIM_GPIO_LED;
// Test points.
extern const GPIO_pin_t GPIO_TP1;
extern const GPIO_pin_t GPIO_MCO;

#endif /* __MCU_MAPPING_H__ */
