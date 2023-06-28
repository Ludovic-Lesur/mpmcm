/*
 * lbus.h
 *
 *  Created on: 16 feb. 2023
 *      Author: Ludo
 */

#ifndef __LBUS_H__
#define __LBUS_H__

#include "lpuart.h"
#include "node_common.h"
#include "types.h"

/*** LBUS macros ***/

#define LBUS_ADDRESS_MASK	0x7F
#define LBUS_ADDRESS_LAST	LBUS_ADDRESS_MASK

/*** LBUS structures ***/

typedef enum {
	LBUS_SUCCESS = 0,
	LBUS_ERROR_ADDRESS,
	LBUS_ERROR_BASE_LPUART = 0x0100,
	LBUS_ERROR_BASE_LAST = (LBUS_ERROR_BASE_LPUART + LPUART_ERROR_BASE_LAST),
} LBUS_status_t;

/*** LBUS functions ***/

LBUS_status_t LBUS_init(NODE_address_t self_address);
LBUS_status_t LBUS_send(uint8_t* data, uint32_t data_size_bytes);
void LBUS_fill_rx_buffer(uint8_t rx_byte);

#define LBUS_status_check(error_base) { if (lbus_status != LBUS_SUCCESS) { status = error_base + lbus_status; goto errors; }}
#define LBUS_error_check() { ERROR_status_check(lbus_status, LBUS_SUCCESS, ERROR_BASE_LBUS); }
#define LBUS_error_check_print() { ERROR_status_check_print(lbus_status, LBUS_SUCCESS, ERROR_BASE_LBUS); }

#endif /* __LBUS_H__ */
