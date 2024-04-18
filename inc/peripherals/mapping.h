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
extern const GPIO_pin_t GPIO_TCXO_POWER_ENABLE;
// Analog inputs.
extern const GPIO_pin_t GPIO_ANA_POWER_ENABLE;
extern const GPIO_pin_t GPIO_ACV_SAMPLING;
extern const GPIO_pin_t GPIO_ACI1_SAMPLING;
extern const GPIO_pin_t GPIO_ACI2_SAMPLING;
extern const GPIO_pin_t GPIO_ACI3_SAMPLING;
extern const GPIO_pin_t GPIO_ACI4_SAMPLING;
// Current sensors detectors.
extern const GPIO_pin_t GPIO_ACI1_DETECT;
extern const GPIO_pin_t GPIO_ACI2_DETECT;
extern const GPIO_pin_t GPIO_ACI3_DETECT;
extern const GPIO_pin_t GPIO_ACI4_DETECT;
// Zero cross detector.
extern const GPIO_pin_t GPIO_ZERO_CROSS_RAW;
extern const GPIO_pin_t GPIO_ZERO_CROSS_PULSE;
// Frequency measure.
extern const GPIO_pin_t GPIO_ACV_FREQUENCY;
// Loads command.
extern const GPIO_pin_t GPIO_LOAD_TRIGGER;
extern const GPIO_pin_t GPIO_LOAD1_COMMAND;
extern const GPIO_pin_t GPIO_LOAD2_COMMAND;
extern const GPIO_pin_t GPIO_LOAD3_COMMAND;
extern const GPIO_pin_t GPIO_LOAD4_COMMAND;
// LPUART1 (RS485).
extern const GPIO_pin_t GPIO_LPUART1_TX;
extern const GPIO_pin_t GPIO_LPUART1_RX;
extern const GPIO_pin_t GPIO_LPUART1_DE;
extern const GPIO_pin_t GPIO_LPUART1_NRE;
// TIC interface.
extern const GPIO_pin_t GPIO_TIC_POWER_ENABLE;
extern const GPIO_pin_t GPIO_USART2_TX;
extern const GPIO_pin_t GPIO_USART2_RX;
// LED.
extern const GPIO_pin_t GPIO_LED_RED;
extern const GPIO_pin_t GPIO_LED_GREEN;
extern const GPIO_pin_t GPIO_LED_BLUE;
// Test points.
extern const GPIO_pin_t GPIO_TP1;
extern const GPIO_pin_t GPIO_MCO;
// Programming.
extern const GPIO_pin_t GPIO_SWDIO;
extern const GPIO_pin_t GPIO_SWCLK;

#endif /* __MAPPING_H__ */
