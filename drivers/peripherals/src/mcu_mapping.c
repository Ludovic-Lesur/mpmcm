/*
 * mcu_mapping.c
 *
 *  Created on: 18 apr. 2024
 *      Author: Ludo
 */

#include "mcu_mapping.h"

#include "adc.h"
#include "lpuart.h"
#include "gpio.h"
#include "gpio_registers.h"
#include "tim.h"
#include "usart.h"

/*** MCU MAPPING local global variables ***/

// Analog inputs.
static const GPIO_pin_t GPIO_ACV_SAMPLING = { GPIOA, 0, 0, 0 };
static const GPIO_pin_t GPIO_ACI1_SAMPLING = { GPIOA, 0, 1, 0 };
static const GPIO_pin_t GPIO_ACI2_SAMPLING = { GPIOA, 0, 4, 0 };
static const GPIO_pin_t GPIO_ACI3_SAMPLING = { GPIOA, 0, 6, 0 };
static const GPIO_pin_t GPIO_ACI4_SAMPLING = { GPIOA, 0, 7, 0 };
static const GPIO_pin_t* ADC_GPIO_PINS_LIST[ADC_CHANNEL_INDEX_LAST] = { &GPIO_ACV_SAMPLING, &GPIO_ACI1_SAMPLING, &GPIO_ACI2_SAMPLING, &GPIO_ACI3_SAMPLING, &GPIO_ACI4_SAMPLING };
// RS485.
static const GPIO_pin_t GPIO_LPUART1_TX = { GPIOB, 1, 11, 8 };
static const GPIO_pin_t GPIO_LPUART1_RX = { GPIOB, 1, 10, 8 };
static const GPIO_pin_t GPIO_LPUART1_DE = { GPIOB, 1, 12, 8 };
static const GPIO_pin_t GPIO_LPUART1_NRE = { GPIOB, 1, 13, 0 };
// Frequency measure.
static const GPIO_pin_t GPIO_ACV_FREQUENCY = { GPIOA, 0, 5, 1 };
static const TIM_channel_gpio_t TIM_CHANNEL_GPIO_ACV_FREQUENCY = { TIM_CHANNEL_ACV_FREQUENCY, &GPIO_ACV_FREQUENCY, TIM_POLARITY_ACTIVE_HIGH };
static const TIM_channel_gpio_t* TIM_CHANNEL_GPIO_LIST_ACV_FREQUENCY[TIM_CHANNEL_INDEX_ACV_FREQUENCY_LAST] = { &TIM_CHANNEL_GPIO_ACV_FREQUENCY };
// TIC interface.
static const GPIO_pin_t GPIO_USART2_TX = { GPIOB, 1, 3, 7 };
static const GPIO_pin_t GPIO_USART2_RX = { GPIOA, 0, 15, 7 };
// RGB LED.
static const GPIO_pin_t GPIO_LED_RED = { GPIOB, 1, 6, 2 };
static const GPIO_pin_t GPIO_LED_GREEN = { GPIOB, 1, 7, 2 };
static const GPIO_pin_t GPIO_LED_BLUE = { GPIOB, 1, 9, 2 };
static const TIM_channel_gpio_t TIM_CHANNEL_GPIO_LED_RED = { TIM_CHANNEL_LED_RED, &GPIO_LED_RED, TIM_POLARITY_ACTIVE_LOW };
static const TIM_channel_gpio_t TIM_CHANNEL_GPIO_LED_GREEN = { TIM_CHANNEL_LED_GREEN, &GPIO_LED_GREEN, TIM_POLARITY_ACTIVE_LOW };
static const TIM_channel_gpio_t TIM_CHANNEL_GPIO_LED_BLUE = { TIM_CHANNEL_LED_BLUE, &GPIO_LED_BLUE, TIM_POLARITY_ACTIVE_LOW };
static const TIM_channel_gpio_t* TIM_CHANNEL_GPIO_LIST_LED[TIM_CHANNEL_INDEX_LED_LAST] = { &TIM_CHANNEL_GPIO_LED_RED, &TIM_CHANNEL_GPIO_LED_GREEN, &TIM_CHANNEL_GPIO_LED_BLUE };

/*** MCU MAPPING global variables ***/

// TCXO.
const GPIO_pin_t GPIO_TCXO_POWER_ENABLE = { GPIOC, 2, 13, 0 };
// Analog inputs.
const GPIO_pin_t GPIO_ANA_POWER_ENABLE = { GPIOB, 1, 15, 0 };
const ADC_gpio_t ADC_GPIO = { (const GPIO_pin_t**) &ADC_GPIO_PINS_LIST, ADC_CHANNEL_INDEX_LAST };
// Current sensors detectors.
const GPIO_pin_t GPIO_ACI1_DETECT = { GPIOB, 1, 0, 0 };
const GPIO_pin_t GPIO_ACI2_DETECT = { GPIOB, 1, 1, 0 };
const GPIO_pin_t GPIO_ACI3_DETECT = { GPIOB, 1, 2, 0 };
const GPIO_pin_t GPIO_ACI4_DETECT = { GPIOB, 1, 14, 0 };
// Zero cross detector.
const GPIO_pin_t GPIO_ZERO_CROSS_RAW = { GPIOA, 0, 3, 0 };
const GPIO_pin_t GPIO_ZERO_CROSS_PULSE = { GPIOA, 0, 2, 0 };
// Frequency measure.
const TIM_gpio_t TIM_GPIO_ACV_FREQUENCY = { (const TIM_channel_gpio_t**) &TIM_CHANNEL_GPIO_LIST_ACV_FREQUENCY, TIM_CHANNEL_INDEX_ACV_FREQUENCY_LAST };
// RS485.
const LPUART_gpio_t LPUART_GPIO_RS485 = { &GPIO_LPUART1_TX, &GPIO_LPUART1_RX, &GPIO_LPUART1_DE, &GPIO_LPUART1_NRE };
// TIC interface.
const GPIO_pin_t GPIO_TIC_POWER_ENABLE = { GPIOB, 1, 4, 0 };
const USART_gpio_t USART_GPIO_TIC = { &GPIO_USART2_TX, &GPIO_USART2_RX };
// RGB LED.
const TIM_gpio_t TIM_GPIO_LED = { (const TIM_channel_gpio_t**) &TIM_CHANNEL_GPIO_LIST_LED, TIM_CHANNEL_INDEX_LED_LAST };
// Test points.
const GPIO_pin_t GPIO_TP1 = { GPIOB, 1, 5, 0 };
const GPIO_pin_t GPIO_MCO = { GPIOA, 0, 8, 0 };
