/*
 * node.c
 *
 *  Created on: 28 may. 2023
 *      Author: Ludo
 */

#include "node.h"

#include "common.h"
#include "dinfox.h"
#include "error.h"
#include "mpmcm.h"

/*** NODE local global variables ***/

static volatile uint32_t NODE_REGISTERS[NODE_REG_ADDR_LAST];

/*** NODE local functions ***/

/*******************************************************************/
static NODE_status_t _NODE_update_register(uint8_t reg_addr) {
	// Local variables.
	NODE_status_t status = NODE_SUCCESS;
	// Update common registers.
	status = COMMON_update_register(reg_addr);
	if (status != NODE_SUCCESS) goto errors;
	// Update specific registers.
	status = MPMCM_update_register(reg_addr);
errors:
	return status;
}

/*******************************************************************/
static NODE_status_t _NODE_check_register(uint8_t reg_addr) {
	// Local variables.
	NODE_status_t status = NODE_SUCCESS;
	// Check common registers.
	status = COMMON_check_register(reg_addr);
	if (status != NODE_SUCCESS) goto errors;
	// Check specific registers.
	status = MPMCM_check_register(reg_addr);
errors:
	return status;
}

/*** NODE functions ***/

/*******************************************************************/
void NODE_init(NODE_address_t self_address) {
	// Local variables.
	uint8_t idx = 0;
	// Clean all registers.
	for (idx=0 ; idx<NODE_REG_ADDR_LAST ; idx++) {
		NODE_REGISTERS[idx] = 0;
	}
	// Init common registers.
	COMMON_init_registers(self_address);
	// Init specific registers.
	MPMCM_init_registers();
}

/*******************************************************************/
NODE_status_t NODE_read_register(NODE_request_source_t request_source, uint8_t reg_addr, uint32_t* reg_value) {
	// Local variables.
	NODE_status_t status = NODE_SUCCESS;
	// Check parameters.
	if (reg_addr >= NODE_REG_ADDR_LAST) {
		status = NODE_ERROR_REGISTER_ADDRESS;
		goto errors;
	}
	if (reg_value == NULL) {
		status = NODE_ERROR_NULL_PARAMETER;
		goto errors;
	}
	// Check update type.
	if (request_source == NODE_REQUEST_SOURCE_EXTERNAL) {
		// Update register.
		status = _NODE_update_register(reg_addr);
		if (status != NODE_SUCCESS) goto errors;
	}
	// Read register.
	(*reg_value) = NODE_REGISTERS[reg_addr];
errors:
	return status;
}

/*******************************************************************/
NODE_status_t NODE_read_byte_array(NODE_request_source_t request_source, uint8_t reg_addr_base, uint8_t* data, uint8_t data_size_byte) {
	// Local variables.
	NODE_status_t status = NODE_SUCCESS;
	uint8_t idx = 0;
	uint32_t reg_addr = 0;
	uint32_t reg_value = 0;
	uint32_t reg_mask = 0;
	// Check parameters.
	if (data == NULL) {
		status = NODE_ERROR_NULL_PARAMETER;
		goto errors;
	}
	// Byte loop.
	for (idx=0 ; idx<data_size_byte ; idx++) {
		// Compute address and mask.
		reg_addr = (reg_addr_base + (idx / 4));
		reg_mask = (0xFF << (8 * (idx % 4)));
		// Read byte.
		status = NODE_read_register(request_source, reg_addr, &reg_value);
		// Fill data.
		data[idx] = DINFOX_read_field(reg_value, reg_mask);
	}
errors:
	return status;
}

/*******************************************************************/
NODE_status_t NODE_write_register(NODE_request_source_t request_source, uint8_t reg_addr, uint32_t reg_mask, uint32_t reg_value) {
	// Local variables.
	NODE_status_t status = NODE_SUCCESS;
	uint32_t temp = 0;
	// Check address.
	if (reg_addr >= NODE_REG_ADDR_LAST) {
		status = NODE_ERROR_REGISTER_ADDRESS;
		goto errors;
	}
	// Check access.
	if ((request_source == NODE_REQUEST_SOURCE_EXTERNAL) && (NODE_REG_ACCESS[reg_addr] == DINFOX_REG_ACCESS_READ_ONLY)) {
		status = NODE_ERROR_REGISTER_READ_ONLY;
		goto errors;
	}
	// Read register.
	temp = NODE_REGISTERS[reg_addr];
	// Compute new value.
	temp &= ~reg_mask;
	temp |= (reg_value & reg_mask);
	// Write register.
	NODE_REGISTERS[reg_addr] = temp;
	// Check actions.
	if (request_source == NODE_REQUEST_SOURCE_EXTERNAL) {
		// Check control bits.
		status = _NODE_check_register(reg_addr);
	}
errors:
	return status;
}

/*******************************************************************/
NODE_status_t NODE_write_byte_array(NODE_request_source_t request_source, uint8_t reg_addr_base, uint8_t* data, uint8_t data_size_byte) {
	// Local variables.
	NODE_status_t status = NODE_SUCCESS;
	uint8_t idx = 0;
	uint8_t shift = 0;
	uint32_t reg_addr = 0;
	uint32_t reg_value = 0;
	uint32_t reg_mask = 0;
	// Check parameters.
	if (data == NULL) {
		status = NODE_ERROR_NULL_PARAMETER;
		goto errors;
	}
	// Byte loop.
	for (idx=0 ; idx<data_size_byte ; idx++) {
		// Compute address, mask and value.
		reg_addr = (reg_addr_base + (idx / 4));
		shift = (8 * (idx % 4));
		reg_mask = (0xFF << shift);
		reg_value = (data[idx] << shift);
		// Write register.
		status = NODE_write_register(request_source, reg_addr, reg_mask, reg_value);
		if (status != NODE_SUCCESS) goto errors;
	}
errors:
	return status;
}
