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

// LED.
static const GPIO_pin_t GPIO_LED =		(GPIO_pin_t) {GPIOA, 0, 5, 0};
// Programming.
static const GPIO_pin_t GPIO_SWDIO =	(GPIO_pin_t) {GPIOA, 0, 13, 0};
static const GPIO_pin_t GPIO_SWCLK =	(GPIO_pin_t) {GPIOA, 0, 14, 0};

#endif /* __MAPPING_H__ */
