/*
 * error.h
 *
 *  Created on: 27 feb. 2022
 *      Author: Ludo
 */

#ifndef __ERROR_H__
#define __ERROR_H__

// Peripherals.
#include "iwdg.h"
#include "rcc.h"

/*** ERROR structures ***/

typedef enum {
	SUCCESS = 0,
	ERROR_BUSY,
	ERROR_SIGFOX_RC,
	// Peripherals.
	ERROR_BASE_IWDG = 0x0100,
	ERROR_BASE_RCC = (ERROR_BASE_IWDG + IWDG_ERROR_BASE_LAST),
	// Last index.
	ERROR_BASE_LAST = (ERROR_BASE_RCC + RCC_ERROR_BASE_LAST)
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
