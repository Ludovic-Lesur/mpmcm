/*
 * mapping.h
 *
 *  Created on: 21 mar. 2023
 *      Author: Ludo
 */

#ifndef __MAPPING_H__
#define __MAPPING_H__

#include "gpio.h"
#include "gpio_reg.h"

// TCXO.
static const GPIO_pin_t GPIO_TCXO_POWER_ENABLE =	(GPIO_pin_t) {GPIOC, 2, 13, 0};
// Analog inputs.
static const GPIO_pin_t GPIO_ANA_POWER_ENABLE =		(GPIO_pin_t) {GPIOB, 1, 15, 0};
static const GPIO_pin_t GPIO_ACV_SAMPLING =			(GPIO_pin_t) {GPIOA, 0, 0, 0};
static const GPIO_pin_t GPIO_ACI1_SAMPLING =		(GPIO_pin_t) {GPIOA, 0, 1, 0};
static const GPIO_pin_t GPIO_ACI2_SAMPLING =		(GPIO_pin_t) {GPIOA, 0, 4, 0};
static const GPIO_pin_t GPIO_ACI3_SAMPLING =		(GPIO_pin_t) {GPIOA, 0, 6, 0};
static const GPIO_pin_t GPIO_ACI4_SAMPLING =		(GPIO_pin_t) {GPIOA, 0, 7, 0};
// Current sensors detectors.
static const GPIO_pin_t GPIO_ACI1_DETECT =			(GPIO_pin_t) {GPIOB, 1, 0, 0};
static const GPIO_pin_t GPIO_ACI2_DETECT =			(GPIO_pin_t) {GPIOB, 1, 1, 0};
static const GPIO_pin_t GPIO_ACI3_DETECT =			(GPIO_pin_t) {GPIOB, 1, 2, 0};
static const GPIO_pin_t GPIO_ACI4_DETECT =			(GPIO_pin_t) {GPIOB, 1, 14, 0};
// Zero cross detector.
static const GPIO_pin_t GPIO_ZERO_CROSS_RAW =		(GPIO_pin_t) {GPIOA, 0, 3, 0};
static const GPIO_pin_t GPIO_ZERO_CROSS_PULSE =		(GPIO_pin_t) {GPIOA, 0, 2, 0};
// Frequency measure.
static const GPIO_pin_t GPIO_ACV_FREQUENCY =		(GPIO_pin_t) {GPIOA, 0, 5, 2}; // AF2 = TIM2_ETR.
// Loads command.
static const GPIO_pin_t GPIO_LOAD_TRIGGER =			(GPIO_pin_t) {GPIOA, 0, 12, 11}; // AF11 = TIM1_ETR.
static const GPIO_pin_t GPIO_LOAD1_COMMAND =		(GPIO_pin_t) {GPIOA, 0, 8, 6}; // AF6 = TIM1_CH1.
static const GPIO_pin_t GPIO_LOAD2_COMMAND =		(GPIO_pin_t) {GPIOA, 0, 9, 6}; // AF6 = TIM1_CH2.
static const GPIO_pin_t GPIO_LOAD3_COMMAND =		(GPIO_pin_t) {GPIOA, 0, 10, 6}; // AF6 = TIM1_CH3.
static const GPIO_pin_t GPIO_LOAD4_COMMAND =		(GPIO_pin_t) {GPIOA, 0, 11, 11}; // AF11 = TIM1_CH4.
// LPUART1 (RS485).
static const GPIO_pin_t GPIO_LPUART1_TX =			(GPIO_pin_t) {GPIOB, 1, 11, 8}; // AF8 = LPUART1_TX.
static const GPIO_pin_t GPIO_LPUART1_RX =			(GPIO_pin_t) {GPIOB, 1, 10, 8}; // AF8 = LPUART1_RX.
static const GPIO_pin_t GPIO_LPUART1_DE =			(GPIO_pin_t) {GPIOB, 1, 12, 8}; // AF8 = LPUART1_DE.
static const GPIO_pin_t GPIO_LPUART1_NRE =			(GPIO_pin_t) {GPIOB, 1, 13, 0};
// TIC interface.
static const GPIO_pin_t GPIO_TIC_POWER_ENABLE =		(GPIO_pin_t) {GPIOB, 1, 4, 0};
static const GPIO_pin_t GPIO_USART2_TX =			(GPIO_pin_t) {GPIOB, 1, 3, 7}; // AF7 = USART2_TX.
static const GPIO_pin_t GPIO_USART2_RX =			(GPIO_pin_t) {GPIOA, 0, 15, 7}; // AF7 = USART2_RX.
// LED.
static const GPIO_pin_t GPIO_LED_RED =				(GPIO_pin_t) {GPIOB, 1, 6, 2}; // AF2 = TIM4_CH1.
static const GPIO_pin_t GPIO_LED_GREEN =			(GPIO_pin_t) {GPIOB, 1, 7, 2}; // AF2 = TIM4_CH2.
static const GPIO_pin_t GPIO_LED_BLUE =				(GPIO_pin_t) {GPIOB, 1, 9, 2}; // AF2 = TIM4_CH4.
// Test point.
static const GPIO_pin_t GPIO_TP1 =					(GPIO_pin_t) {GPIOB, 1, 5, 0};
// Programming.
static const GPIO_pin_t GPIO_SWDIO =				(GPIO_pin_t) {GPIOA, 0, 13, 0};
static const GPIO_pin_t GPIO_SWCLK =				(GPIO_pin_t) {GPIOA, 0, 14, 0};

#endif /* __MAPPING_H__ */
