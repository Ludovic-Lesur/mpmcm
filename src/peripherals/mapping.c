/*
 * mapping.c
 *
 *  Created on: 18 apr/ 2024
 *      Author: Ludo
 */

#include "mapping.h"

#include "gpio.h"
#include "gpio_reg.h"

// TCXO.
const GPIO_pin_t GPIO_TCXO_POWER_ENABLE =	(GPIO_pin_t) {GPIOC, 2, 13, 0};
// Analog inputs.
const GPIO_pin_t GPIO_ANA_POWER_ENABLE =	(GPIO_pin_t) {GPIOB, 1, 15, 0};
const GPIO_pin_t GPIO_ACV_SAMPLING =		(GPIO_pin_t) {GPIOA, 0, 0, 0};
const GPIO_pin_t GPIO_ACI1_SAMPLING =		(GPIO_pin_t) {GPIOA, 0, 1, 0};
const GPIO_pin_t GPIO_ACI2_SAMPLING =		(GPIO_pin_t) {GPIOA, 0, 4, 0};
const GPIO_pin_t GPIO_ACI3_SAMPLING =		(GPIO_pin_t) {GPIOA, 0, 6, 0};
const GPIO_pin_t GPIO_ACI4_SAMPLING =		(GPIO_pin_t) {GPIOA, 0, 7, 0};
// Current sensors detectors.
const GPIO_pin_t GPIO_ACI1_DETECT =			(GPIO_pin_t) {GPIOB, 1, 0, 0};
const GPIO_pin_t GPIO_ACI2_DETECT =			(GPIO_pin_t) {GPIOB, 1, 1, 0};
const GPIO_pin_t GPIO_ACI3_DETECT =			(GPIO_pin_t) {GPIOB, 1, 2, 0};
const GPIO_pin_t GPIO_ACI4_DETECT =			(GPIO_pin_t) {GPIOB, 1, 14, 0};
// Zero cross detector.
const GPIO_pin_t GPIO_ZERO_CROSS_RAW =		(GPIO_pin_t) {GPIOA, 0, 3, 0};
const GPIO_pin_t GPIO_ZERO_CROSS_PULSE =	(GPIO_pin_t) {GPIOA, 0, 2, 0};
// Frequency measure.
const GPIO_pin_t GPIO_ACV_FREQUENCY =		(GPIO_pin_t) {GPIOA, 0, 5, 1}; // AF1 = TIM2_CH1.
// Loads command.
const GPIO_pin_t GPIO_LOAD_TRIGGER =		(GPIO_pin_t) {GPIOA, 0, 12, 11}; // AF11 = TIM1_ETR.
const GPIO_pin_t GPIO_LOAD1_COMMAND =		(GPIO_pin_t) {GPIOA, 0, 8, 6}; // AF6 = TIM1_CH1.
const GPIO_pin_t GPIO_LOAD2_COMMAND =		(GPIO_pin_t) {GPIOA, 0, 9, 6}; // AF6 = TIM1_CH2.
const GPIO_pin_t GPIO_LOAD3_COMMAND =		(GPIO_pin_t) {GPIOA, 0, 10, 6}; // AF6 = TIM1_CH3.
const GPIO_pin_t GPIO_LOAD4_COMMAND =		(GPIO_pin_t) {GPIOA, 0, 11, 11}; // AF11 = TIM1_CH4.
// LPUART1 (RS485).
const GPIO_pin_t GPIO_LPUART1_TX =			(GPIO_pin_t) {GPIOB, 1, 11, 8}; // AF8 = LPUART1_TX.
const GPIO_pin_t GPIO_LPUART1_RX =			(GPIO_pin_t) {GPIOB, 1, 10, 8}; // AF8 = LPUART1_RX.
const GPIO_pin_t GPIO_LPUART1_DE =			(GPIO_pin_t) {GPIOB, 1, 12, 8}; // AF8 = LPUART1_DE.
const GPIO_pin_t GPIO_LPUART1_NRE =			(GPIO_pin_t) {GPIOB, 1, 13, 0};
// TIC interface.
const GPIO_pin_t GPIO_TIC_POWER_ENABLE =	(GPIO_pin_t) {GPIOB, 1, 4, 0};
const GPIO_pin_t GPIO_USART2_TX =			(GPIO_pin_t) {GPIOB, 1, 3, 7}; // AF7 = USART2_TX.
const GPIO_pin_t GPIO_USART2_RX =			(GPIO_pin_t) {GPIOA, 0, 15, 7}; // AF7 = USART2_RX.
// LED.
const GPIO_pin_t GPIO_LED_RED =				(GPIO_pin_t) {GPIOB, 1, 6, 2}; // AF2 = TIM4_CH1.
const GPIO_pin_t GPIO_LED_GREEN =			(GPIO_pin_t) {GPIOB, 1, 7, 2}; // AF2 = TIM4_CH2.
const GPIO_pin_t GPIO_LED_BLUE =			(GPIO_pin_t) {GPIOB, 1, 9, 2}; // AF2 = TIM4_CH4.
// Test points.
const GPIO_pin_t GPIO_TP1 =					(GPIO_pin_t) {GPIOB, 1, 5, 0};
const GPIO_pin_t GPIO_MCO = 				(GPIO_pin_t) {GPIOA, 0, 8, 0}; // AF0 = MCO.
// Programming.
const GPIO_pin_t GPIO_SWDIO =				(GPIO_pin_t) {GPIOA, 0, 13, 0};
const GPIO_pin_t GPIO_SWCLK =				(GPIO_pin_t) {GPIOA, 0, 14, 0};
