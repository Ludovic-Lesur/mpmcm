/*
 * lptim.h
 *
 *  Created on: 18 jun. 2023
 *      Author: Ludo
 */

#ifndef __LPTIM_H__
#define __LPTIM_H__

#include "types.h"

/*** LPTIM structures ***/

typedef enum {
	LPTIM_SUCCESS = 0,
	LPTIM_ERROR_DELAY_UNDERFLOW,
	LPTIM_ERROR_DELAY_OVERFLOW,
	LPTIM_ERROR_WRITE_ARR,
	LPTIM_ERROR_DELAY_MODE,
	LPTIM_ERROR_BASE_LAST = 0x0100
} LPTIM_status_t;

typedef enum {
	LPTIM_DELAY_MODE_ACTIVE = 0,
	LPTIM_DELAY_MODE_STOP,
	LPTIM_DELAY_MODE_LAST
} LPTIM_delay_mode_t;

/*** LPTIM functions ***/

void LPTIM1_init(void);
LPTIM_status_t LPTIM1_delay_milliseconds(uint32_t delay_ms, LPTIM_delay_mode_t delay_mode);

#define LPTIM1_status_check(error_base) { if (lptim1_status != LPTIM_SUCCESS) { status = error_base + lptim1_status; goto errors; }}
#define LPTIM1_error_check() { ERROR_status_check(lptim1_status, LPTIM_SUCCESS, ERROR_BASE_LPTIM1); }
#define LPTIM1_error_check_print() { ERROR_status_check_print(lptim1_status, LPTIM_SUCCESS, ERROR_BASE_LPTIM1); }

#endif /* __LPTIM_H__ */
