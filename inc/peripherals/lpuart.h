/*
 * lpuart.h
 *
 *  Created on: 9 jul. 2019
 *      Author: Ludo
 */

#ifndef __LPUART_H__
#define __LPUART_H__

#include "node_common.h"
#include "types.h"

/*** LPUART structures ***/

typedef enum {
	LPUART_SUCCESS = 0,
	LPUART_ERROR_NULL_PARAMETER,
	LPUART_ERROR_MODE,
	LPUART_ERROR_LBUS_ADDRESS,
	LPUART_ERROR_TX_TIMEOUT,
	LPUART_ERROR_TC_TIMEOUT,
	LPUART_ERROR_STRING_SIZE,
	LPUART_ERROR_BASE_LAST = 0x0100
} LPUART_status_t;

/*** LPUART functions ***/

LPUART_status_t LPUART1_init(NODE_address_t self_address);
void LPUART1_enable_rx(void);
void LPUART1_disable_rx(void);
LPUART_status_t LPUART1_send(uint8_t* data, uint32_t data_size_bytes);

#define LPUART1_status_check(error_base) { if (lpuart1_status != LPUART_SUCCESS) { status = error_base + lpuart1_status; goto errors; }}
#define LPUART1_error_check() { ERROR_status_check(lpuart1_status, LPUART_SUCCESS, ERROR_BASE_LPUART1); }
#define LPUART1_error_check_print() { ERROR_status_check_print(lpuart1_status, LPUART_SUCCESS, ERROR_BASE_LPUART1); }

#endif /* __LPUART_H__ */
