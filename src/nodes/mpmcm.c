/*
 * ddrm.c
 *
 *  Created on: 03 sep. 2023
 *      Author: Ludo
 */

#include "mpmcm.h"

#include "adc.h"
#include "dinfox.h"
#include "error.h"
#include "measure.h"
#include "mpmcm_reg.h"
#include "node.h"

/*** MPMCM functions ***/

/*******************************************************************/
void MPMCM_init_registers(void) {
	// Local variables.
	uint8_t reg_addr = 0;
	// Load default values.
	for (reg_addr=MPMCM_REG_ADDR_STATUS_CONTROL_1 ; reg_addr<MPMCM_REG_ADDR_LAST ; reg_addr++) {
		NODE_write_register(NODE_REQUEST_SOURCE_INTERNAL, reg_addr, DINFOX_REG_MASK_ALL, 0);
	}
}

/*******************************************************************/
NODE_status_t MPMCM_update_register(uint8_t reg_addr) {
	// Local variables.
	NODE_status_t status = NODE_SUCCESS;
	MEASURE_status_t measure_status = MEASURE_SUCCESS;
	uint32_t reg_value = 0;
	uint32_t reg_mask = 0;
	uint8_t channel_idx = 0;
	uint8_t chxd = 0;
	// Check address.
	switch (reg_addr) {
	case MPMCM_REG_ADDR_STATUS_CONTROL_1:
		// Update detect flags.
		for (channel_idx=0 ; channel_idx<ADC_NUMBER_OF_ACI_CHANNELS ; channel_idx++) {
			// Read flag.
			measure_status = MEASURE_get_detect_flag(channel_idx, &chxd);
			MEASURE_exit_error(NODE_ERROR_BASE_MEASURE);
			// Update field.
			DINFOX_write_field(&reg_value, &reg_mask, (uint32_t) chxd, (0b1 << ((channel_idx << 1) + 1)));
		}
		break;
	// Check run registers.
	// TODO
	default:
		// Nothing to do for other registers.
		break;
	}
	// Write register.
	NODE_write_register(NODE_REQUEST_SOURCE_INTERNAL, reg_addr, reg_mask, reg_value);
errors:
	return status;
}

/*******************************************************************/
NODE_status_t MPMCM_check_register(uint8_t reg_addr) {
	// Local variables.
	NODE_status_t status = NODE_SUCCESS;
	MEASURE_status_t measure_status = MEASURE_SUCCESS;
	MEASURE_accumulated_channel_result_t channel_data;
	uint32_t reg_value = 0;
	uint32_t new_reg_value = 0;
	uint32_t new_reg_mask = 0;
	uint8_t channel_idx = 0;
	uint32_t data_reg_value = 0;
	uint32_t data_reg_mask = 0;
	uint8_t reg_offset = 0;
	// Read register.
	NODE_read_register(NODE_REQUEST_SOURCE_INTERNAL, reg_addr, &reg_value);
	// Check address.
	switch (reg_addr) {
	case MPMCM_REG_ADDR_STATUS_CONTROL_1:
		// Check frequency store control bit.
		// TODO
		// Check measurements store control bits.
		for (channel_idx=0 ; channel_idx<ADC_NUMBER_OF_ACI_CHANNELS ; channel_idx++) {
			// Check CHxS bit.
			if (DINFOX_read_field(reg_value, (0b1 << (channel_idx << 1))) != 0) {
				// Clear request.
				DINFOX_write_field(&new_reg_value, &new_reg_mask, 0, (0b1 << (channel_idx << 1)));
				// Read and reset measurements.
				measure_status = MEASURE_get_accumulated_data(channel_idx, &channel_data);
				MEASURE_exit_error(NODE_ERROR_BASE_MEASURE);
				// Compute registers offset.
				reg_offset = (MPMCM_NUMBER_OF_REG_PER_DATA * channel_idx);
				// Active power.
				data_reg_value = 0;
				data_reg_mask = 0;
				DINFOX_write_field(&data_reg_value, &data_reg_mask, DINFOX_convert_mw(channel_data.active_power_mw.min), MPMCM_REG_X_0_MIN_MASK);
				DINFOX_write_field(&data_reg_value, &data_reg_mask, DINFOX_convert_mw(channel_data.active_power_mw.max), MPMCM_REG_X_0_MAX_MASK);
				NODE_write_register(NODE_REQUEST_SOURCE_INTERNAL, (MPMCM_REG_ADDR_CH1_ACTIVE_POWER_0 + reg_offset), data_reg_mask, data_reg_value);
				data_reg_value = 0;
				data_reg_mask = 0;
				DINFOX_write_field(&data_reg_value, &data_reg_mask, DINFOX_convert_mw(channel_data.active_power_mw.rolling_mean), MPMCM_REG_X_1_MEAN_MASK);
				NODE_write_register(NODE_REQUEST_SOURCE_INTERNAL, (MPMCM_REG_ADDR_CH1_ACTIVE_POWER_1 + reg_offset), data_reg_mask, data_reg_value);
				// RMS voltage.
				data_reg_value = 0;
				data_reg_mask = 0;
				DINFOX_write_field(&data_reg_value, &data_reg_mask, DINFOX_convert_mv(channel_data.rms_voltage_mv.min), MPMCM_REG_X_0_MIN_MASK);
				DINFOX_write_field(&data_reg_value, &data_reg_mask, DINFOX_convert_mv(channel_data.rms_voltage_mv.max), MPMCM_REG_X_0_MAX_MASK);
				NODE_write_register(NODE_REQUEST_SOURCE_INTERNAL, (MPMCM_REG_ADDR_CH1_RMS_VOLTAGE_0 + reg_offset), data_reg_mask, data_reg_value);
				data_reg_value = 0;
				data_reg_mask = 0;
				DINFOX_write_field(&data_reg_value, &data_reg_mask, DINFOX_convert_mv(channel_data.rms_voltage_mv.rolling_mean), MPMCM_REG_X_1_MEAN_MASK);
				NODE_write_register(NODE_REQUEST_SOURCE_INTERNAL, (MPMCM_REG_ADDR_CH1_RMS_VOLTAGE_1 + reg_offset), data_reg_mask, data_reg_value);
				// RMS current.
				data_reg_value = 0;
				data_reg_mask = 0;
				DINFOX_write_field(&data_reg_value, &data_reg_mask, DINFOX_convert_ua(channel_data.rms_current_ma.min * 1000), MPMCM_REG_X_0_MIN_MASK);
				DINFOX_write_field(&data_reg_value, &data_reg_mask, DINFOX_convert_ua(channel_data.rms_current_ma.max * 1000), MPMCM_REG_X_0_MAX_MASK);
				NODE_write_register(NODE_REQUEST_SOURCE_INTERNAL, (MPMCM_REG_ADDR_CH1_RMS_CURRENT_0 + reg_offset), data_reg_mask, data_reg_value);
				data_reg_value = 0;
				data_reg_mask = 0;
				DINFOX_write_field(&data_reg_value, &data_reg_mask, DINFOX_convert_ua(channel_data.rms_current_ma.rolling_mean * 1000), MPMCM_REG_X_1_MEAN_MASK);
				NODE_write_register(NODE_REQUEST_SOURCE_INTERNAL, (MPMCM_REG_ADDR_CH1_RMS_CURRENT_1 + reg_offset), data_reg_mask, data_reg_value);
				// Apparent power.
				data_reg_value = 0;
				data_reg_mask = 0;
				DINFOX_write_field(&data_reg_value, &data_reg_mask, DINFOX_convert_mw(channel_data.apparent_power_mva.min), MPMCM_REG_X_0_MIN_MASK);
				DINFOX_write_field(&data_reg_value, &data_reg_mask, DINFOX_convert_mw(channel_data.apparent_power_mva.max), MPMCM_REG_X_0_MAX_MASK);
				NODE_write_register(NODE_REQUEST_SOURCE_INTERNAL, (MPMCM_REG_ADDR_CH1_APPARENT_POWER_0 + reg_offset), data_reg_mask, data_reg_value);
				data_reg_value = 0;
				data_reg_mask = 0;
				DINFOX_write_field(&data_reg_value, &data_reg_mask, DINFOX_convert_mw(channel_data.apparent_power_mva.rolling_mean), MPMCM_REG_X_1_MEAN_MASK);
				NODE_write_register(NODE_REQUEST_SOURCE_INTERNAL, (MPMCM_REG_ADDR_CH1_APPARENT_POWER_1 + reg_offset), data_reg_mask, data_reg_value);
				// Power factor.
				data_reg_value = 0;
				data_reg_mask = 0;
				DINFOX_write_field(&data_reg_value, &data_reg_mask, DINFOX_convert_power_factor(channel_data.power_factor.min), MPMCM_REG_X_0_MIN_MASK);
				DINFOX_write_field(&data_reg_value, &data_reg_mask, DINFOX_convert_power_factor(channel_data.power_factor.max), MPMCM_REG_X_0_MAX_MASK);
				NODE_write_register(NODE_REQUEST_SOURCE_INTERNAL, (MPMCM_REG_ADDR_CH1_POWER_FACTOR_0 + reg_offset), data_reg_mask, data_reg_value);
				data_reg_value = 0;
				data_reg_mask = 0;
				DINFOX_write_field(&data_reg_value, &data_reg_mask, DINFOX_convert_power_factor(channel_data.power_factor.rolling_mean), MPMCM_REG_X_1_MEAN_MASK);
				NODE_write_register(NODE_REQUEST_SOURCE_INTERNAL, (MPMCM_REG_ADDR_CH1_POWER_FACTOR_1 + reg_offset), data_reg_mask, data_reg_value);
			}
		}
		break;
	default:
		break;
	}
errors:
	NODE_write_register(NODE_REQUEST_SOURCE_INTERNAL, reg_addr, new_reg_mask, new_reg_value);
	return status;
}

/*******************************************************************/
NODE_status_t MPMCM_mtrg_callback(void) {
	// Local variables.
	NODE_status_t status = NODE_SUCCESS;
	// Not implemented on MPMCM.
	return status;
}
