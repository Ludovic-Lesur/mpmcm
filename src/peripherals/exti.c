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
#include "nvic.h"
#include "rcc_reg.h"
#include "syscfg_reg.h"
#include "types.h"

/*** EXTI local macros ***/

#define EXTI_LINES_PER_REGISTER		32

/*** EXTI local global variables ***/

static volatile uint32_t* EXTI_IMR;
static volatile uint32_t* EXTI_PR;
static volatile uint32_t* EXTI_RTSR;
static volatile uint32_t* EXTI_FTSR;

/*** EXTI local functions ***/

/* SELECT PROPER IMR REGISTER.
 * @param:	None.
 * @return:	None.
 */
#define _EXTI_SELECT_IMR(void){ \
	EXTI_IMR = (line < EXTI_LINES_PER_REGISTER) ? &(EXTI -> IMR1) : &(EXTI -> IMR2); \
}

/* SELECT PROPER PR REGISTER.
 * @param:	None.
 * @return:	None.
 */
#define _EXTI_SELECT_PR(void) { \
	EXTI_PR = (line < EXTI_LINES_PER_REGISTER) ? &(EXTI -> PR1) : &(EXTI -> PR2); \
}

/* SELECT PROPER RTSR REGISTER.
 * @param:	None.
 * @return:	None.
 */
#define _EXTI_SELECT_RTSR(void) { \
	EXTI_RTSR = (line < EXTI_LINES_PER_REGISTER) ? &(EXTI -> RTSR1) : &(EXTI -> RTSR2); \
}

/* SELECT PROPER FTSR REGISTER.
 * @param:	None.
 * @return:	None.
 */
#define _EXTI_SELECT_FTSR(void) { \
	EXTI_FTSR = (line < EXTI_LINES_PER_REGISTER) ? &(EXTI -> FTSR1) : &(EXTI -> FTSR2); \
}



/* SET EXTI TRIGGER.
 * @param trigger:	Interrupt edge trigger (see EXTI_trigger_t enum).
 * @param line_idx:	Line index.
 * @return:			None.
 */
static void _EXTI_set_trigger(EXTI_trigger_t trigger, uint8_t line) {
	// Compute registers and bit index.
	uint8_t line_idx = (line % EXTI_LINES_PER_REGISTER);
	_EXTI_SELECT_RTSR();
	_EXTI_SELECT_FTSR();
	_EXTI_SELECT_PR();
	// Select triggers.
	switch (trigger) {
	// Rising edge only.
	case EXTI_TRIGGER_RISING_EDGE:
		(*EXTI_RTSR) |= (0b1 << line_idx); // Rising edge enabled.
		(*EXTI_FTSR) &= ~(0b1 << line_idx); // Falling edge disabled.
		break;
	// Falling edge only.
	case EXTI_TRIGGER_FALLING_EDGE:
		(*EXTI_RTSR) &= ~(0b1 << line_idx); // Rising edge disabled.
		(*EXTI_FTSR) |= (0b1 << line_idx); // Falling edge enabled.
		break;
	// Both edges.
	case EXTI_TRIGGER_ANY_EDGE:
		(*EXTI_RTSR) |= (0b1 << line_idx); // Rising edge enabled.
		(*EXTI_FTSR) |= (0b1 << line_idx); // Falling edge enabled.
		break;
	// Unknown configuration.
	default:
		break;
	}
	// Clear flag.
	(*EXTI_PR) |= (0b1 << line_idx);
}

/*** EXTI functions ***/

/* INIT EXTI PERIPHERAL.
 * @param:	None.
 * @return:	None.
 */
void EXTI_init(void) {
	// Enable peripheral clock.
	RCC -> APB2ENR |= (0b1 << 0); // SYSCFEN='1'.
	// Mask all sources by default.
	EXTI -> IMR1 = 0;
	EXTI -> IMR2 = 0;
	// Clear all flags.
	EXTI_clear_all_flags();
}

/* CONFIGURE A GPIO AS EXTERNAL INTERRUPT SOURCE.
 * @param gpio:		GPIO to be attached to EXTI peripheral.
 * @param trigger:	Interrupt edge trigger (see EXTI_trigger_t enum).
 * @return:			None.
 */
void EXTI_configure_gpio(const GPIO_pin_t* gpio, EXTI_trigger_t trigger) {
	// Compute registers and bit index.
	uint8_t line = (0b1 << (gpio -> pin));
	uint8_t line_idx = (line % EXTI_LINES_PER_REGISTER);
	_EXTI_SELECT_IMR();
	// Select GPIO port.
	SYSCFG -> EXTICR[((gpio -> pin) >> 2)] &= ~(0b1111 << (((gpio -> pin) % 4) << 2));
	SYSCFG -> EXTICR[((gpio -> pin) >> 2)] |= ((gpio -> port_index) << (((gpio -> pin) % 4) << 2));
	// Set mask.
	(*EXTI_IMR) |= (0b1 << line_idx); // IMx='1'.
	// Select triggers.
	_EXTI_set_trigger(trigger, line);
}

/* CONFIGURE A LINE AS INTERNAL INTERRUPT SOURCE.
 * @param line:		Line to configure (see EXTI_line_t enum).
 * @param trigger:	Interrupt edge trigger (see EXTI_trigger_t enum).
 * @return:			None.
 */
void EXTI_configure_line(EXTI_line_t line, EXTI_trigger_t trigger) {
	// Compute registers and bit index.
	_EXTI_SELECT_IMR();
	uint8_t line_idx = (line % EXTI_LINES_PER_REGISTER);
	// Set mask.
	(*EXTI_IMR) |= (0b1 << line_idx); // IMx='1'.
	// Select triggers.
	_EXTI_set_trigger(trigger, line);
}

/* CLEAR EXTI FLAG.
 * @param line:	Line to clear (see EXTI_line_t enum).
 * @return:		None.
 */
void EXTI_clear_flag(EXTI_line_t line) {
	// Compute registers and bit index.
	_EXTI_SELECT_PR();
	uint8_t line_idx = (line % EXTI_LINES_PER_REGISTER);
	// Clear flag.
	(*EXTI_PR) |= (0b1 << line_idx); // PIFx='1'.
}

/* CLEAR ALL EXTI FLAGS.
 * @param:	None.
 * @return:	None.
 */
void EXTI_clear_all_flags(void) {
	// Clear all flags.
	EXTI -> PR1 |= 0xE07BFFFF; // PIFx='1'.
	EXTI -> PR2 |= 0x00000303; // PIFx='1'.
}

