/*
 * node.h
 *
 *  Created on: 18 feb. 2023
 *      Author: Ludo
 */

#ifndef __NODE_H__
#define __NODE_H__

#include "adc.h"
#include "lptim.h"
#include "measure.h"
#include "node_common.h"
#include "power.h"
#include "types.h"

/*** NODE structures ***/

/*!******************************************************************
 * \enum NODE_status_t
 * \brief NODE driver error codes.
 *******************************************************************/
typedef enum {
	// Driver errors.
	NODE_SUCCESS = 0,
	NODE_ERROR_NULL_PARAMETER,
	NODE_ERROR_REGISTER_ADDRESS,
	NODE_ERROR_REGISTER_READ_ONLY,
	NODE_ERROR_RADIO_STATE,
	NODE_ERROR_RADIO_POWER,
	NODE_ERROR_SIGFOX_RF_API,
	NODE_ERROR_SIGFOX_EP_API,
	// Low level drivers errors.
	NODE_ERROR_BASE_ADC1 = 0x0100,
	NODE_ERROR_BASE_LPTIM1 = (NODE_ERROR_BASE_ADC1 + ADC_ERROR_BASE_LAST),
	NODE_ERROR_BASE_MEASURE = (NODE_ERROR_BASE_LPTIM1 + LPTIM_ERROR_BASE_LAST),
	// Last base value.
	NODE_ERROR_BASE_LAST = (NODE_ERROR_BASE_MEASURE + MEASURE_ERROR_BASE_LAST)
} NODE_status_t;

/*!******************************************************************
 * \enum NODE_request_source_t
 * \brief NODE request sources.
 *******************************************************************/
typedef enum {
	NODE_REQUEST_SOURCE_INTERNAL = 0,
	NODE_REQUEST_SOURCE_EXTERNAL,
	NODE_REQUEST_SOURCE_LAST
} NODE_request_source_t;

/*** NODE functions ***/

/*!******************************************************************
 * \fn void NODE_init(NODE_address_t self_address)
 * \brief Init node registers to their default value.
 * \param[in]  	self_address: RS485 address of the node.
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void NODE_init(NODE_address_t self_address);

/*!******************************************************************
 * \fn NODE_status_t NODE_read_register(NODE_request_source_t request_source, uint8_t reg_addr, uint32_t* reg_value)
 * \brief Read node register.
 * \param[in]  	request_source: Request source.
 * \param[in]	reg_addr: Address of the register to read.
 * \param[out] 	reg_value: Pointer to byte that will contain the register value.
 * \retval		Function execution status.
 *******************************************************************/
NODE_status_t NODE_read_register(NODE_request_source_t request_source, uint8_t reg_addr, uint32_t* reg_value);

/*!******************************************************************
 * \fn NODE_status_t NODE_read_byte_array(NODE_request_source_t request_source, uint8_t reg_addr_base, uint8_t* data, uint8_t data_size_byte)
 * \brief Read multiple registers in a byte array.
 * \param[in]  	request_source: Request source.
 * \param[in]	reg_addr_base: Address of the first register to read.
 * \param[in]	data_size_byte: Number of bytes to read.
 * \param[out]	data: Pointer to the registers value.
 * \retval		Function execution status.
 *******************************************************************/
NODE_status_t NODE_read_byte_array(NODE_request_source_t request_source, uint8_t reg_addr_base, uint8_t* data, uint8_t data_size_byte);

/*!******************************************************************
 * \fn NODE_status_t NODE_write_register(NODE_request_source_t request_source, uint8_t reg_addr, uint32_t reg_mask, uint32_t reg_value)
 * \brief Write node register.
 * \param[in]  	request_source: Request source.
 * \param[in]	reg_addr: Address of the register to write.
 * \param[in]	reg_mask: Writing operation mask.
 * \param[in] 	reg_value: Value to write in register.
 * \param[out]	none
 * \retval		Function execution status.
 *******************************************************************/
NODE_status_t NODE_write_register(NODE_request_source_t request_source, uint8_t reg_addr, uint32_t reg_mask, uint32_t reg_value);

/*!******************************************************************
 * \fn NODE_status_t NODE_write_byte_array(NODE_request_source_t request_source, uint8_t reg_addr_base, uint8_t* data, uint8_t data_size_byte)
 * \brief Write multiple registers.
 * \param[in]  	request_source: Request source.
 * \param[in]	reg_addr_base: Address of the first register to write.
 * \param[in]	data: Pointer to the registers value.
 * \param[in]	data_size_byte: Number of bytes to write.
 * \param[out]	none
 * \retval		Function execution status.
 *******************************************************************/
NODE_status_t NODE_write_byte_array(NODE_request_source_t request_source, uint8_t reg_addr_base, uint8_t* data, uint8_t data_size_byte);

/*******************************************************************/
#define NODE_exit_error(error_base) { if (node_status != NODE_SUCCESS) { status = (error_base + node_status); goto errors; } }

/*******************************************************************/
#define NODE_stack_error(void) { if (node_status != NODE_SUCCESS) { ERROR_stack_add(ERROR_BASE_NODE + node_status); } }

/*******************************************************************/
#define NODE_stack_exit_error(error_code) { if (node_status != NODE_SUCCESS) { ERROR_stack_add(ERROR_BASE_NODE + node_status); status = error_code; goto errors; } }

#endif /* __NODE_H__ */
