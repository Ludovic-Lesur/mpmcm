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
static void _COMMON_reset_analog_data(void) {
	// Local variables.
	uint32_t reg_analog_data_0 = 0;
	uint32_t reg_analog_data_0_mask = 0;
	// VMCU and TMCU.
	DINFOX_write_field(&reg_analog_data_0, &reg_analog_data_0_mask, DINFOX_VOLTAGE_ERROR_VALUE, COMMON_REG_ANALOG_DATA_0_MASK_VMCU);
	DINFOX_write_field(&reg_analog_data_0, &reg_analog_data_0_mask, DINFOX_TEMPERATURE_ERROR_VALUE, COMMON_REG_ANALOG_DATA_0_MASK_TMCU);
	NODE_write_register(NODE_REQUEST_SOURCE_INTERNAL, COMMON_REG_ADDR_ANALOG_DATA_0, reg_analog_data_0_mask, reg_analog_data_0);
}

/*******************************************************************/
static NODE_status_t _COMMON_mtrg_callback(void) {
	// Local variables.
	NODE_status_t status = NODE_SUCCESS;
	status = MPMCM_mtrg_callback();
	return status;
}

/*** COMMON functions ***/

/*******************************************************************/
void COMMON_init_registers(NODE_address_t self_address) {
	// Local variables.
	uint32_t reg_node_id = 0;
	uint32_t reg_node_id_mask = 0;
	uint32_t reg_hw_version = 0;
	uint32_t reg_hw_version_mask = 0;
	uint32_t reg_sw_version_0 = 0;
	uint32_t reg_sw_version_0_mask = 0;
	uint32_t reg_sw_version_1 = 0;
	uint32_t reg_sw_version_1_mask = 0;
	uint32_t reg_status_0 = 0;
	uint32_t reg_status_0_mask = 0;
	// Node ID register.
	DINFOX_write_field(&reg_node_id, &reg_node_id_mask, (uint32_t) self_address, COMMON_REG_NODE_ID_MASK_NODE_ADDR);
	DINFOX_write_field(&reg_node_id, &reg_node_id_mask, (uint32_t) NODE_BOARD_ID, COMMON_REG_NODE_ID_MASK_BOARD_ID);
	// HW version register.
#ifdef HW1_0
	DINFOX_write_field(&reg_hw_version, &reg_hw_version_mask, 1, COMMON_REG_HW_VERSION_MASK_MAJOR);
	DINFOX_write_field(&reg_hw_version, &reg_hw_version_mask, 0, COMMON_REG_HW_VERSION_MASK_MINOR);
#endif
	// SW version register 0.
	DINFOX_write_field(&reg_sw_version_0, &reg_sw_version_0_mask, (uint32_t) GIT_MAJOR_VERSION, COMMON_REG_SW_VERSION_0_MASK_MAJOR);
	DINFOX_write_field(&reg_sw_version_0, &reg_sw_version_0_mask, (uint32_t) GIT_MINOR_VERSION, COMMON_REG_SW_VERSION_0_MASK_MINOR);
	DINFOX_write_field(&reg_sw_version_0, &reg_sw_version_0_mask, (uint32_t) GIT_COMMIT_INDEX, COMMON_REG_SW_VERSION_0_MASK_COMMIT_INDEX);
	DINFOX_write_field(&reg_sw_version_0, &reg_sw_version_0_mask, (uint32_t) GIT_DIRTY_FLAG, COMMON_REG_SW_VERSION_0_MASK_DTYF);
	// SW version register 1.
	DINFOX_write_field(&reg_sw_version_1, &reg_sw_version_1_mask, (uint32_t) GIT_COMMIT_ID, COMMON_REG_SW_VERSION_1_MASK_COMMIT_ID);
	// Reset flags registers.
	DINFOX_write_field(&reg_status_0, &reg_status_0_mask, (uint32_t) (((RCC -> CSR) >> 24) & 0xFF), COMMON_REG_STATUS_0_MASK_RESET_FLAGS);
	DINFOX_write_field(&reg_status_0, &reg_status_0_mask, 0b1, COMMON_REG_STATUS_0_MASK_BF);
	// Write registers.
	NODE_write_register(NODE_REQUEST_SOURCE_INTERNAL, COMMON_REG_ADDR_NODE_ID, reg_node_id_mask, reg_node_id);
	NODE_write_register(NODE_REQUEST_SOURCE_INTERNAL, COMMON_REG_ADDR_HW_VERSION, reg_hw_version_mask, reg_hw_version);
	NODE_write_register(NODE_REQUEST_SOURCE_INTERNAL, COMMON_REG_ADDR_SW_VERSION_0, reg_sw_version_0_mask, reg_sw_version_0);
	NODE_write_register(NODE_REQUEST_SOURCE_INTERNAL, COMMON_REG_ADDR_SW_VERSION_1, reg_sw_version_1_mask, reg_sw_version_1);
	NODE_write_register(NODE_REQUEST_SOURCE_INTERNAL, COMMON_REG_ADDR_STATUS_0, reg_status_0_mask, reg_status_0);
	// Load default values.
	_COMMON_reset_analog_data();
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
	case COMMON_REG_ADDR_STATUS_0:
		// Check error stack.
		DINFOX_write_field(&reg_value, &reg_mask, ((ERROR_stack_is_empty() == 0) ? 0b1 : 0b0), COMMON_REG_STATUS_0_MASK_ESF);
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
NODE_status_t COMMON_check_register(uint8_t reg_addr, uint32_t reg_mask) {
	// Local variables.
	NODE_status_t status = NODE_SUCCESS;
	uint32_t reg_value = 0;
	// Read register.
	status = NODE_read_register(NODE_REQUEST_SOURCE_INTERNAL, reg_addr, &reg_value);
	if (status != NODE_SUCCESS) goto errors;
	// Check address.
	switch (reg_addr) {
	case COMMON_REG_ADDR_CONTROL_0:
		// RTRG.
		if ((reg_mask & COMMON_REG_CONTROL_0_MASK_RTRG) != 0) {
			// Read bit.
			if ((DINFOX_read_field(reg_value, COMMON_REG_CONTROL_0_MASK_RTRG)) != 0) {
				// Reset MCU.
				PWR_software_reset();
			}
		}
		// MTRG.
		if ((reg_mask & COMMON_REG_CONTROL_0_MASK_MTRG) != 0) {
			// Read bit.
			if ((DINFOX_read_field(reg_value, COMMON_REG_CONTROL_0_MASK_MTRG)) != 0) {
				// Clear request.
				NODE_write_register(NODE_REQUEST_SOURCE_INTERNAL, COMMON_REG_ADDR_CONTROL_0, COMMON_REG_CONTROL_0_MASK_MTRG, 0b0);
				// Perform measurements.
				status = _COMMON_mtrg_callback();
				if (status != NODE_SUCCESS) goto errors;
			}
		}
		// BFC.
		if ((reg_mask & COMMON_REG_CONTROL_0_MASK_BFC) != 0) {
			// Read bit.
			if ((DINFOX_read_field(reg_value, COMMON_REG_CONTROL_0_MASK_BFC)) != 0) {
				// Clear request.
				NODE_write_register(NODE_REQUEST_SOURCE_INTERNAL, COMMON_REG_ADDR_CONTROL_0, COMMON_REG_CONTROL_0_MASK_BFC, 0b0);
				// Clear boot flag.
				NODE_write_register(NODE_REQUEST_SOURCE_INTERNAL, COMMON_REG_ADDR_STATUS_0, COMMON_REG_STATUS_0_MASK_BF, 0b0);
			}
		}
		break;
	default:
		// Nothing to do for other registers.
		break;
	}
errors:
	return status;
}
