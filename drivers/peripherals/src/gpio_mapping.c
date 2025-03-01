/*
 * gpio_mapping.c
 *
 *  Created on: 18 apr. 2024
 *      Author: Ludo
 */

#include "gpio_mapping.h"

#include "adc.h"
#include "lpuart.h"
#include "gpio.h"
#include "gpio_registers.h"
#include "tim.h"
#include "usart.h"

/*** GPIO MAPPING local global variables ***/

// Analog inputs.
static const GPIO_pin_t GPIO_ACV_SAMPLING = (GPIO_pin_t) { GPIOA, 0, 0, 0 };
static const GPIO_pin_t GPIO_ACI1_SAMPLING = (GPIO_pin_t) { GPIOA, 0, 1, 0 };
static const GPIO_pin_t GPIO_ACI2_SAMPLING = (GPIO_pin_t) { GPIOA, 0, 4, 0 };
static const GPIO_pin_t GPIO_ACI3_SAMPLING = (GPIO_pin_t) { GPIOA, 0, 6, 0 };
static const GPIO_pin_t GPIO_ACI4_SAMPLING = (GPIO_pin_t) { GPIOA, 0, 7, 0 };
static const GPIO_pin_t* GPIO_ADC_PINS_LIST[GPIO_ADC_CHANNEL_INDEX_LAST] = { &GPIO_ACV_SAMPLING, &GPIO_ACI1_SAMPLING, &GPIO_ACI2_SAMPLING, &GPIO_ACI3_SAMPLING, &GPIO_ACI4_SAMPLING };
// RS485.
static const GPIO_pin_t GPIO_LPUART1_TX = (GPIO_pin_t) { GPIOB, 1, 11, 8 };
static const GPIO_pin_t GPIO_LPUART1_RX = (GPIO_pin_t) { GPIOB, 1, 10, 8 };
static const GPIO_pin_t GPIO_LPUART1_DE = (GPIO_pin_t) { GPIOB, 1, 12, 8 };
static const GPIO_pin_t GPIO_LPUART1_NRE = (GPIO_pin_t) { GPIOB, 1, 13, 0 };
// Frequency measure.
static const GPIO_pin_t GPIO_ACV_FREQUENCY = (GPIO_pin_t) { GPIOA, 0, 5, 1 };
static const TIM_channel_gpio_t GPIO_ACV_FREQUENCY_TIM_CHANNEL = { GPIO_TIM_CHANNEL_ACV_FREQUENCY, &GPIO_ACV_FREQUENCY, TIM_POLARITY_ACTIVE_HIGH };
static const TIM_channel_gpio_t* GPIO_ACV_FREQUENCY_TIM_PINS_LIST[GPIO_ACV_FREQUENCY_TIM_CHANNEL_INDEX_LAST] = { &GPIO_ACV_FREQUENCY_TIM_CHANNEL };
// TIC interface.
static const GPIO_pin_t GPIO_USART2_TX = (GPIO_pin_t) { GPIOB, 1, 3, 7 };
static const GPIO_pin_t GPIO_USART2_RX = (GPIO_pin_t) { GPIOA, 0, 15, 7 };
// RGB LED.
static const GPIO_pin_t GPIO_LED_RED = (GPIO_pin_t) { GPIOB, 1, 6, 2 };
static const GPIO_pin_t GPIO_LED_GREEN = (GPIO_pin_t) { GPIOB, 1, 7, 2 };
static const GPIO_pin_t GPIO_LED_BLUE = (GPIO_pin_t) { GPIOB, 1, 9, 2 };
static const TIM_channel_gpio_t GPIO_LED_TIM_CHANNEL_RED = { GPIO_TIM_CHANNEL_LED_RED, &GPIO_LED_RED, TIM_POLARITY_ACTIVE_LOW };
static const TIM_channel_gpio_t GPIO_LED_TIM_CHANNEL_GREEN = { GPIO_TIM_CHANNEL_LED_GREEN, &GPIO_LED_GREEN, TIM_POLARITY_ACTIVE_LOW };
static const TIM_channel_gpio_t GPIO_LED_TIM_CHANNEL_BLUE = { GPIO_TIM_CHANNEL_LED_BLUE, &GPIO_LED_BLUE, TIM_POLARITY_ACTIVE_LOW };
static const TIM_channel_gpio_t* GPIO_LED_TIM_PINS_LIST[GPIO_LED_TIM_CHANNEL_INDEX_LAST] = { &GPIO_LED_TIM_CHANNEL_RED, &GPIO_LED_TIM_CHANNEL_GREEN, &GPIO_LED_TIM_CHANNEL_BLUE };

/*** GPIO MAPPING global variables ***/

// TCXO.
const GPIO_pin_t GPIO_TCXO_POWER_ENABLE = (GPIO_pin_t) { GPIOC, 2, 13, 0 };
// Analog inputs.
const GPIO_pin_t GPIO_ANA_POWER_ENABLE = (GPIO_pin_t) { GPIOB, 1, 15, 0 };
const ADC_gpio_t GPIO_ADC = { (const GPIO_pin_t**) &GPIO_ADC_PINS_LIST, GPIO_ADC_CHANNEL_INDEX_LAST };
// Current sensors detectors.
const GPIO_pin_t GPIO_ACI1_DETECT = (GPIO_pin_t) { GPIOB, 1, 0, 0 };
const GPIO_pin_t GPIO_ACI2_DETECT = (GPIO_pin_t) { GPIOB, 1, 1, 0 };
const GPIO_pin_t GPIO_ACI3_DETECT = (GPIO_pin_t) { GPIOB, 1, 2, 0 };
const GPIO_pin_t GPIO_ACI4_DETECT = (GPIO_pin_t) { GPIOB, 1, 14, 0 };
// Zero cross detector.
const GPIO_pin_t GPIO_ZERO_CROSS_RAW = (GPIO_pin_t) { GPIOA, 0, 3, 0 };
const GPIO_pin_t GPIO_ZERO_CROSS_PULSE = (GPIO_pin_t) { GPIOA, 0, 2, 0 };
// Frequency measure.
const TIM_gpio_t GPIO_ACV_FREQUENCY_TIM = { (const TIM_channel_gpio_t**) &GPIO_ACV_FREQUENCY_TIM_PINS_LIST, GPIO_ACV_FREQUENCY_TIM_CHANNEL_INDEX_LAST };
// RS485.
const LPUART_gpio_t GPIO_RS485_LPUART = { &GPIO_LPUART1_TX, &GPIO_LPUART1_RX, &GPIO_LPUART1_DE, &GPIO_LPUART1_NRE };
// TIC interface.
const GPIO_pin_t GPIO_TIC_POWER_ENABLE = (GPIO_pin_t) { GPIOB, 1, 4, 0 };
const USART_gpio_t GPIO_TIC_USART = { &GPIO_USART2_TX, &GPIO_USART2_RX };
// LED.
const TIM_gpio_t GPIO_LED_TIM = { (const TIM_channel_gpio_t**) &GPIO_LED_TIM_PINS_LIST, GPIO_LED_TIM_CHANNEL_INDEX_LAST };
// Test points.
const GPIO_pin_t GPIO_TP1 = (GPIO_pin_t) { GPIOB, 1, 5, 0 };
const GPIO_pin_t GPIO_MCO = (GPIO_pin_t) { GPIOA, 0, 8, 0 }; // AF0 = MCO.
