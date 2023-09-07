/*
 * common.c
 *
 *  Created on: 03 sep. 2023
 *      Author: Ludo
 */

#include "common.h"

#include "adc.h"
#include "common_reg.h"
#include "dinfox.h"
#include "error.h"
#include "mpmcm.h"
#include "node.h"
#include "pwr.h"
#include "rcc_reg.h"
#include "types.h"
#include "version.h"

/*** COMMON local functions ***/

/*******************************************************************/
NODE_status_t _COMMON_mtrg_callback(void) {
	// Local variables.
	NODE_status_t status = NODE_SUCCESS;
	status = MPMCM_mtrg_callback();
	return status;
}

/*** COMMON functions ***/

/*******************************************************************/
void COMMON_init_registers(NODE_address_t self_address) {
	// Local variables.
	uint32_t node_id = 0;
	uint32_t node_id_mask = 0;
	uint32_t hw_version = 0;
	uint32_t hw_version_mask = 0;
	uint32_t sw_version_0 = 0;
	uint32_t sw_version_0_mask = 0;
	uint32_t sw_version_1 = 0;
	uint32_t sw_version_1_mask = 0;
	uint32_t reset_flags = 0;
	uint32_t reset_flags_mask = 0;
	// Node ID register.
	DINFOX_write_field(&node_id, &node_id_mask, (uint32_t) self_address, COMMON_REG_NODE_ID_MASK_NODE_ADDR);
	DINFOX_write_field(&node_id, &node_id_mask, (uint32_t) NODE_BOARD_ID, COMMON_REG_NODE_ID_MASK_BOARD_ID);
	// HW version register.
#ifdef HW1_0
	DINFOX_write_field(&hw_version, &hw_version_mask, 1, COMMON_REG_HW_VERSION_MASK_MAJOR);
	DINFOX_write_field(&hw_version, &hw_version_mask, 0, COMMON_REG_HW_VERSION_MASK_MINOR);
#endif
	// SW version register 0.
	DINFOX_write_field(&sw_version_0, &sw_version_0_mask, (uint32_t) GIT_MAJOR_VERSION, COMMON_REG_SW_VERSION_0_MASK_MAJOR);
	DINFOX_write_field(&sw_version_0, &sw_version_0_mask, (uint32_t) GIT_MINOR_VERSION, COMMON_REG_SW_VERSION_0_MASK_MINOR);
	DINFOX_write_field(&sw_version_0, &sw_version_0_mask, (uint32_t) GIT_COMMIT_INDEX, COMMON_REG_SW_VERSION_0_MASK_COMMIT_INDEX);
	DINFOX_write_field(&sw_version_0, &sw_version_0_mask, (uint32_t) GIT_DIRTY_FLAG, COMMON_REG_SW_VERSION_0_MASK_DTYF);
	// SW version register 1.
	DINFOX_write_field(&sw_version_1, &sw_version_1_mask, (uint32_t) GIT_COMMIT_ID, COMMON_REG_SW_VERSION_1_MASK_COMMIT_ID);
	// Reset flags registers.
	DINFOX_write_field(&reset_flags, &reset_flags_mask, (uint32_t) (((RCC -> CSR) >> 24) & 0xFF), COMMON_REG_RESET_FLAGS_MASK_ALL);
	// Write registers.
	NODE_write_register(NODE_REQUEST_SOURCE_INTERNAL, COMMON_REG_ADDR_NODE_ID, node_id_mask, node_id);
	NODE_write_register(NODE_REQUEST_SOURCE_INTERNAL, COMMON_REG_ADDR_HW_VERSION, hw_version_mask, hw_version);
	NODE_write_register(NODE_REQUEST_SOURCE_INTERNAL, COMMON_REG_ADDR_SW_VERSION_0, sw_version_0_mask, sw_version_0);
	NODE_write_register(NODE_REQUEST_SOURCE_INTERNAL, COMMON_REG_ADDR_SW_VERSION_1, sw_version_1_mask, sw_version_1);
	NODE_write_register(NODE_REQUEST_SOURCE_INTERNAL, COMMON_REG_ADDR_RESET_FLAGS, reset_flags_mask, reset_flags);
}

/*******************************************************************/
NODE_status_t COMMON_update_register(uint8_t reg_addr) {
	// Local variables.
	NODE_status_t status = NODE_SUCCESS;
	uint32_t reg_value = 0;
	uint32_t reg_mask = 0;
	// Check address.
	switch (reg_addr) {
	case COMMON_REG_ADDR_ERROR_STACK:
		// Unstack error.
		DINFOX_write_field(&reg_value, &reg_mask, (uint32_t) ERROR_stack_read(), COMMON_REG_ERROR_STACK_MASK_ERROR);
		break;
	default:
		// Nothing to do.
		break;
	}
	// Write register.
	NODE_write_register(NODE_REQUEST_SOURCE_INTERNAL, reg_addr, reg_mask, reg_value);
	return status;
}

/*******************************************************************/
NODE_status_t COMMON_check_register(uint8_t reg_addr) {
	// Local variables.
	NODE_status_t status = NODE_SUCCESS;
	uint32_t reg_value = 0;
	// Read register.
	NODE_read_register(NODE_REQUEST_SOURCE_INTERNAL, COMMON_REG_ADDR_STATUS_CONTROL_0, &reg_value);
	// Check address.
	switch (reg_addr) {
	case COMMON_REG_ADDR_STATUS_CONTROL_0:
		// Reset trigger bit.
		if ((DINFOX_read_field(reg_value, COMMON_REG_STATUS_CONTROL_0_MASK_RTRG)) != 0) {
			// Reset MCU.
			PWR_software_reset();
		}
		// Measure trigger bit.
		if ((DINFOX_read_field(reg_value, COMMON_REG_STATUS_CONTROL_0_MASK_MTRG)) != 0) {
			// Perform measurements.
			_COMMON_mtrg_callback();
		}
		break;
	default:
		// Nothing to do for other registers.
		break;
	}
	return status;
}
