/*
 * exti.c
 *
 *  Created on: 18 jun. 2023
 *      Author: Ludo
 */

#include "exti.h"

#include "exti_reg.h"
#include "gpio.h"
#include "mapping.h"
#include "rcc_reg.h"
#include "syscfg_reg.h"
#include "types.h"

/*** EXTI local global variables ***/

static EXTI_gpio_irq_cb_t exti_gpio_irq_callbacks[GPIO_PINS_PER_PORT];

/*** EXTI local functions ***/

/*******************************************************************/
#define _EXTI_irq_handler(gpio) { \
	/* Check flag */ \
	if ((((EXTI -> EXTIx[0]).PR) & (0b1 << (gpio.pin))) != 0) { \
		/* Check mask and callback */ \
		if (((((EXTI -> EXTIx[0]).IMR) & (0b1 << (gpio.pin))) != 0) && (exti_gpio_irq_callbacks[gpio.pin] != NULL)) { \
			/* Execute callback */ \
			exti_gpio_irq_callbacks[gpio.pin](); \
		} \
		/* Clear flag */ \
		(EXTI -> EXTIx[0]).PR |= (0b1 << (gpio.pin)); \
	} \
}

/*******************************************************************/
void __attribute__((optimize("-O0"))) EXTI2_IRQHandler(void) {
	// Zero-cross detector.
	_EXTI_irq_handler(GPIO_ZERO_CROSS_PULSE);
}

/*******************************************************************/
static void _EXTI_set_trigger(EXTI_trigger_t trigger, uint8_t line) {
	// Select triggers.
	switch (trigger) {
	// Rising edge only.
	case EXTI_TRIGGER_RISING_EDGE:
		(EXTI -> EXTIx[line >> 5]).RTSR |= (0b1 << (line % 32)); // Rising edge enabled.
		(EXTI -> EXTIx[line >> 5]).FTSR &= ~(0b1 << (line % 32)); // Falling edge disabled.
		break;
	// Falling edge only.
	case EXTI_TRIGGER_FALLING_EDGE:
		(EXTI -> EXTIx[line >> 5]).RTSR &= ~(0b1 << (line % 32)); // Rising edge disabled.
		(EXTI -> EXTIx[line >> 5]).FTSR |= (0b1 << (line % 32)); // Falling edge enabled.
		break;
	// Both edges.
	case EXTI_TRIGGER_ANY_EDGE:
		(EXTI -> EXTIx[line >> 5]).RTSR |= (0b1 << (line % 32)); // Rising edge enabled.
		(EXTI -> EXTIx[line >> 5]).FTSR |= (0b1 << (line % 32)); // Falling edge enabled.
		break;
	// Unknown configuration.
	default:
		break;
	}
	// Clear flag.
	(EXTI -> EXTIx[line >> 5]).PR |= (0b1 << (line % 32));
}

/*** EXTI functions ***/

/*******************************************************************/
void EXTI_init(void) {
	// Local variables.
	uint8_t idx = 0;
	// Enable peripheral clock.
	RCC -> APB2ENR |= (0b1 << 0); // SYSCFEN='1'.
	// Mask all sources by default.
	(EXTI -> EXTIx[0]).IMR = 0;
	(EXTI -> EXTIx[1]).IMR = 0;
	// Clear all flags.
	(EXTI -> EXTIx[0]).PR |= 0xE07BFFFF; // PIFx='1'.
	(EXTI -> EXTIx[1]).PR |= 0x00000303; // PIFx='1'.
	// Reset callbacks.
	for (idx=0 ; idx<GPIO_PINS_PER_PORT ; idx++) {
		exti_gpio_irq_callbacks[idx] = NULL;
	}
}

/*******************************************************************/
void EXTI_configure_gpio(const GPIO_pin_t* gpio, EXTI_trigger_t trigger, EXTI_gpio_irq_cb_t irq_callback) {
	// Select GPIO port.
	SYSCFG -> EXTICR[((gpio -> pin) >> 2)] &= ~(0b1111 << (((gpio -> pin) % 4) << 2));
	SYSCFG -> EXTICR[((gpio -> pin) >> 2)] |= ((gpio -> port_index) << (((gpio -> pin) % 4) << 2));
	// Set mask.
	(EXTI -> EXTIx[0]).IMR |= (0b1 << (gpio -> pin));
	// Select triggers.
	_EXTI_set_trigger(trigger, (gpio -> pin));
	// Register callback.
	exti_gpio_irq_callbacks[gpio -> pin] = irq_callback;
}

/*******************************************************************/
void EXTI_release_gpio(const GPIO_pin_t* gpio) {
	// Set mask.
	(EXTI -> EXTIx[0]).IMR &= ~(0b1 << (gpio -> pin)); // IMx='0'.
}

/*******************************************************************/
void EXTI_configure_line(EXTI_line_t line, EXTI_trigger_t trigger) {
	// Set mask.
	(EXTI -> EXTIx[line >> 5]).IMR |= (0b1 << (line % 32));
	// Select triggers.
	_EXTI_set_trigger(trigger, line);
}

/*******************************************************************/
void EXTI_clear_flag(EXTI_line_t line) {
	// Clear flag.
	(EXTI -> EXTIx[line >> 5]).PR |= (0b1 << (line % 32));
}
