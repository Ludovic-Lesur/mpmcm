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

/*!******************************************************************
 * \enum LBUS_status_t
 * \brief LBUS driver error codes.
 *******************************************************************/
typedef enum {
	// Driver errors.
	LBUS_SUCCESS = 0,
	LBUS_ERROR_ADDRESS,
	// Low level drivers errors.
	LBUS_ERROR_BASE_LPUART1 = 0x0100,
	// Last base value.
	LBUS_ERROR_BASE_LAST = (LBUS_ERROR_BASE_LPUART1 + LPUART_ERROR_BASE_LAST),
} LBUS_status_t;

/*!******************************************************************
 * \fn LBUS_rx_irq_cb
 * \brief LBUS RX interrupt callback.
 *******************************************************************/
typedef void (*LBUS_rx_irq_cb)(uint8_t data);

/*** LBUS functions ***/

/*!******************************************************************
 * \fn LBUS_status_t LBUS_init(NODE_address_t self_address, LBUS_rx_irq_cb irq_callback)
 * \brief Init LBUS interface.
 * \param[in]  	self_address: RS485 address of the node.
 * \param[in]  	irq_callback: Function to call on frame reception interrupt.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
LBUS_status_t LBUS_init(NODE_address_t self_address, LBUS_rx_irq_cb irq_callback);

/*!******************************************************************
 * \fn void LBUS_enable_rx(void)
 * \brief Enable LBUS receiver.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void LBUS_enable_rx(void);

/*!******************************************************************
 * \fn void LBUS_disable_rx(void)
 * \brief Disable LBUS receiver.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void LBUS_disable_rx(void);

/*!******************************************************************
 * \fn LBUS_status_t LBUS_send(uint8_t* data, uint32_t data_size_bytes)
 * \brief Send data over LBUS.
 * \param[in]	data: Byte array to send.
 * \param[in]	data_size_bytes: Number of bytes to send.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
LBUS_status_t LBUS_send(uint8_t* data, uint32_t data_size_bytes);

/*******************************************************************/
#define LBUS_exit_error(error_base) { if (lbus_status != LBUS_SUCCESS) { status = (error_base + lbus_status); goto errors; } }

/*******************************************************************/
#define LBUS_stack_error(void) { if (lbus_status != LBUS_SUCCESS) { ERROR_stack_add(ERROR_BASE_LBUS + lbus_status); } }

/*******************************************************************/
#define LBUS_stack_exit_error(error_code) { if (lbus_status != LBUS_SUCCESS) { ERROR_stack_add(ERROR_BASE_LBUS + lbus_status); status = error_code; goto errors; } }

#endif /* __LBUS_H__ */
