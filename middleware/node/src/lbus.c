/*
 * lbus.c
 *
 *  Created on: 16 feb. 2023
 *      Author: Ludo
 */

#include "lbus.h"

#include "dinfox.h"
#include "lpuart.h"
#include "mode.h"
#include "node_common.h"
#include "types.h"

/*** LBUS local macros ***/

#define LBUS_DESTINATION_ADDRESS_MARKER		0x80
#define LBUS_ADDRESS_SIZE_BYTES				1

/*** LBUS local structures ***/

/*******************************************************************/
typedef enum {
	LBUS_FRAME_FIELD_INDEX_DESTINATION_ADDRESS = 0,
	LBUS_FRAME_FIELD_INDEX_SOURCE_ADDRESS = (LBUS_FRAME_FIELD_INDEX_DESTINATION_ADDRESS + LBUS_ADDRESS_SIZE_BYTES),
	LBUS_FRAME_FIELD_INDEX_DATA = (LBUS_FRAME_FIELD_INDEX_SOURCE_ADDRESS + LBUS_ADDRESS_SIZE_BYTES)
} LBUS_frame_field_index_t;

/*******************************************************************/
typedef struct {
	NODE_address_t self_address;
	NODE_address_t master_address;
	uint8_t rx_byte_count;
	LBUS_rx_irq_cb rx_irq_callback;
} LBUS_context_t;

/*** LBUS local global variables ***/

static LBUS_context_t lbus_ctx;

/*** LBUS local functions ***/

/*******************************************************************/
static void _LBUS_fill_rx_buffer(uint8_t rx_byte) {
	// Check field index.
	switch (lbus_ctx.rx_byte_count) {
	case LBUS_FRAME_FIELD_INDEX_DESTINATION_ADDRESS:
		// Nothing to do.
		break;
	case LBUS_FRAME_FIELD_INDEX_SOURCE_ADDRESS:
		// Store source address for next reply.
		lbus_ctx.master_address = (rx_byte & LBUS_ADDRESS_MASK);
		break;
	default:
		// Transmit byte to upper layer.
		if (lbus_ctx.rx_irq_callback != NULL) {
			lbus_ctx.rx_irq_callback(rx_byte);
		}
		break;
	}
	// Increment byte count.
	lbus_ctx.rx_byte_count++;
}

/*** LBUS functions ***/

/*******************************************************************/
LBUS_status_t LBUS_init(NODE_address_t self_address, LBUS_rx_irq_cb irq_callback) {
	// Local variables.
	LBUS_status_t status = LBUS_SUCCESS;
	// Check address.
	if (self_address > LBUS_ADDRESS_LAST) {
		status = LBUS_ERROR_ADDRESS;
		goto errors;
	}
	// Init context.
	lbus_ctx.self_address = self_address;
	lbus_ctx.master_address = (DINFOX_NODE_ADDRESS_DMM | LBUS_DESTINATION_ADDRESS_MARKER);
	lbus_ctx.rx_byte_count = 0;
	lbus_ctx.rx_irq_callback = irq_callback;
	// Init LPUART.
	LPUART1_init(self_address, &_LBUS_fill_rx_buffer);
errors:
	return status;
}

/*******************************************************************/
void LBUS_enable_rx(void) {
	// Enable receiver.
	LPUART1_enable_rx();
}

/*******************************************************************/
void LBUS_disable_rx(void) {
	// Disable receiver.
	LPUART1_disable_rx();
}

/*******************************************************************/
LBUS_status_t LBUS_send(uint8_t* data, uint32_t data_size_bytes) {
	// Local variables.
	LBUS_status_t status = LBUS_SUCCESS;
	LPUART_status_t lpuart1_status = LPUART_SUCCESS;
#ifndef HIGH_SPEED_LOG
	uint8_t lbus_header[LBUS_FRAME_FIELD_INDEX_DATA];
	// Build address header.
	lbus_header[LBUS_FRAME_FIELD_INDEX_DESTINATION_ADDRESS] = (lbus_ctx.master_address | LBUS_DESTINATION_ADDRESS_MARKER);
	lbus_header[LBUS_FRAME_FIELD_INDEX_SOURCE_ADDRESS] = lbus_ctx.self_address;
	// Send header.
	lpuart1_status = LPUART1_write(lbus_header, LBUS_FRAME_FIELD_INDEX_DATA);
	LPUART1_exit_error(LBUS_ERROR_BASE_LPUART1);
#endif
	// Send command.
	lpuart1_status = LPUART1_write(data, data_size_bytes);
	LPUART1_exit_error(LBUS_ERROR_BASE_LPUART1);
errors:
	// Reset RX byte for next reception.
	lbus_ctx.rx_byte_count = 0;
	return status;
}
