/*
 * rcc.h
 *
 *  Created on: 16 jun. 2023
 *      Author: Ludo
 */

#ifndef __RCC_H__
#define __RCC_H__

#include "types.h"

/*** RCC structures ***/

typedef enum {
	RCC_SUCCESS = 0,
	RCC_ERROR_HSE_READY,
	RCC_ERROR_PLL_READY,
	RCC_ERROR_PLL_SWITCH,
	RCC_ERROR_BASE_LAST = 0x0100
} RCC_status_t;

/*** RCC functions ***/

RCC_status_t RCC_init(void);

#define RCC_status_check(error_base) { if (rcc_status != RCC_SUCCESS) { status = error_base + rcc_status; goto errors; }}
#define RCC_error_check() { ERROR_status_check(rcc_status, RCC_SUCCESS, ERROR_BASE_RCC); }
#define RCC_error_check_print() { ERROR_status_check_print(rcc_status, RCC_SUCCESS, ERROR_BASE_RCC); }

#endif /* __RCC_H__ */
