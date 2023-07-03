/*
 * error.h
 *
 *  Created on: 27 feb. 2022
 *      Author: Ludo
 */

#ifndef __ERROR_H__
#define __ERROR_H__

// Peripherals.
#include "adc.h"
#include "iwdg.h"
#include "lptim.h"
#include "lpuart.h"
#include "rcc.h"
#include "rtc.h"
// Utils.
#include "parser.h"
#include "string_custom.h"
// Nodes.
#include "lbus.h"
// Applicative.
#include "measure.h"

/*** ERROR structures ***/

typedef enum {
	SUCCESS = 0,
	// Peripherals.
	ERROR_BASE_ADC = 0x0100,
	ERROR_BASE_IWDG = (ERROR_BASE_ADC + ADC_ERROR_BASE_LAST),
	ERROR_BASE_LPTIM1 = (ERROR_BASE_IWDG + IWDG_ERROR_BASE_LAST),
	ERROR_BASE_LPUART1 = (ERROR_BASE_LPTIM1 + LPTIM_ERROR_BASE_LAST),
	ERROR_BASE_RCC = (ERROR_BASE_LPUART1 + LPUART_ERROR_BASE_LAST),
	ERROR_BASE_RTC = (ERROR_BASE_RCC + RCC_ERROR_BASE_LAST),
	// Utils.
	ERROR_BASE_PARSER = (ERROR_BASE_RTC + RTC_ERROR_BASE_LAST),
	ERROR_BASE_STRING = (ERROR_BASE_PARSER + PARSER_ERROR_BASE_LAST),
	// Nodes.
	ERROR_BASE_LBUS = (ERROR_BASE_STRING + STRING_ERROR_BASE_LAST),
	// Applicative.
	ERROR_BASE_MEASURE = (ERROR_BASE_LBUS + LBUS_ERROR_BASE_LAST),
	// Last index.
	ERROR_BASE_LAST = (ERROR_BASE_MEASURE + MEASURE_ERROR_BASE_LAST)
} ERROR_t;

/*** ERROR functions ***/

void ERROR_stack_init(void);
void ERROR_stack_add(ERROR_t code);
ERROR_t ERROR_stack_read(void);
uint8_t ERROR_stack_is_empty(void);

#define ERROR_status_check(status, success, error_base) { \
	if (status != success) { \
		ERROR_stack_add(error_base + status); \
	} \
}

#define ERROR_status_check_print(status, success, error_base) { \
	if (status != success) { \
		_AT_print_error(error_base + status); \
		goto errors; \
	} \
}

#endif /* __ERROR_H__ */
