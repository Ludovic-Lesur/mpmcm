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
#include "mode.h"
#include "mpmcm_reg.h"
#include "node.h"

/*** MPMCM functions ***/

/*******************************************************************/
void MPMCM_init_registers(void) {
	// Local variables.
	uint8_t reg_addr = 0;
	// Read init state.
	MPMCM_update_register(MPMCM_REG_ADDR_CONFIGURATION_0);
	MPMCM_update_register(MPMCM_REG_ADDR_CH1_CONFIGURATION);
	MPMCM_update_register(MPMCM_REG_ADDR_CH2_CONFIGURATION);
	MPMCM_update_register(MPMCM_REG_ADDR_CH3_CONFIGURATION);
	MPMCM_update_register(MPMCM_REG_ADDR_CH4_CONFIGURATION);
	MPMCM_update_register(MPMCM_REG_ADDR_STATUS_1);
	// Load default values.
	for (reg_addr=MPMCM_REG_ADDR_CONTROL_1 ; reg_addr<MPMCM_REG_ADDR_LAST ; reg_addr++) {
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
	uint8_t generic_u8 = 0;
	// Check address.
	switch (reg_addr) {
	case MPMCM_REG_ADDR_CONFIGURATION_0:
		// Transformer gain and attenuation.
		DINFOX_write_field(&reg_value, &reg_mask, MPMCM_TRANSFORMER_GAIN,  MPMCM_REG_MASK_GAIN);
		DINFOX_write_field(&reg_value, &reg_mask, MPMCM_TRANSFORMER_ATTEN, MPMCM_REG_MASK_ATTEN);
		break;
	case MPMCM_REG_ADDR_CH1_CONFIGURATION:
		// SCT013 gain and attenuation.
		DINFOX_write_field(&reg_value, &reg_mask, (uint32_t) MPMCM_SCT013_GAIN[0],  MPMCM_REG_MASK_GAIN);
		DINFOX_write_field(&reg_value, &reg_mask, (uint32_t) MPMCM_SCT013_ATTEN[0], MPMCM_REG_MASK_ATTEN);
		break;
	case MPMCM_REG_ADDR_CH2_CONFIGURATION:
		// SCT013 gain and attenuation.
		DINFOX_write_field(&reg_value, &reg_mask, (uint32_t) MPMCM_SCT013_GAIN[1],  MPMCM_REG_MASK_GAIN);
		DINFOX_write_field(&reg_value, &reg_mask, (uint32_t) MPMCM_SCT013_ATTEN[1], MPMCM_REG_MASK_ATTEN);
		break;
	case MPMCM_REG_ADDR_CH3_CONFIGURATION:
		// SCT013 gain and attenuation.
		DINFOX_write_field(&reg_value, &reg_mask, (uint32_t) MPMCM_SCT013_GAIN[2],  MPMCM_REG_MASK_GAIN);
		DINFOX_write_field(&reg_value, &reg_mask, (uint32_t) MPMCM_SCT013_ATTEN[2], MPMCM_REG_MASK_ATTEN);
		break;
	case MPMCM_REG_ADDR_CH4_CONFIGURATION:
		// SCT013 gain and attenuation.
		DINFOX_write_field(&reg_value, &reg_mask, (uint32_t) MPMCM_SCT013_GAIN[3],  MPMCM_REG_MASK_GAIN);
		DINFOX_write_field(&reg_value, &reg_mask, (uint32_t) MPMCM_SCT013_ATTEN[3], MPMCM_REG_MASK_ATTEN);
		break;
	case MPMCM_REG_ADDR_STATUS_1:
		// Update probe detect flags.
		for (channel_idx=0 ; channel_idx<ADC_NUMBER_OF_ACI_CHANNELS ; channel_idx++) {
			// Read flag.
			measure_status = MEASURE_get_probe_detect_flag(channel_idx, &generic_u8);
			MEASURE_exit_error(NODE_ERROR_BASE_MEASURE);
			// Update field.
			DINFOX_write_field(&reg_value, &reg_mask, (uint32_t) generic_u8, (0b1 << channel_idx));
		}
		// Update mains detect flag.
		measure_status = MEASURE_get_mains_detect_flag(&generic_u8);
		MEASURE_exit_error(NODE_ERROR_BASE_MEASURE);
		// Update field.
		DINFOX_write_field(&reg_value, &reg_mask, (uint32_t) generic_u8, MPMCM_REG_STATUS_1_MASK_MVD);
		break;
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
NODE_status_t MPMCM_check_register(uint8_t reg_addr, uint32_t reg_mask) {
	// Local variables.
	NODE_status_t status = NODE_SUCCESS;
	MEASURE_status_t measure_status = MEASURE_SUCCESS;
	MEASURE_accumulated_data_t data;
	MEASURE_channel_accumulated_data_t channel_data;
	uint8_t channel_idx = 0;
	uint8_t reg_offset = 0;
	uint32_t reg_value = 0;
	uint32_t field_value = 0;
	uint32_t new_reg_value = 0;
	uint32_t new_reg_mask = 0;
	uint32_t data_reg_value = 0;
	uint32_t data_reg_mask = 0;
	// Read register.
	NODE_read_register(NODE_REQUEST_SOURCE_INTERNAL, reg_addr, &reg_value);
	// Check address.
	switch (reg_addr) {
	case MPMCM_REG_ADDR_CONTROL_1:
		// FRQS.
		if ((reg_mask & MPMCM_REG_CONTROL_1_MASK_FRQS) != 0) {
			// Check bit.
			if (DINFOX_read_field(reg_value, MPMCM_REG_CONTROL_1_MASK_FRQS) != 0) {
				// Clear request.
				DINFOX_write_field(&new_reg_value, &new_reg_mask, 0b0, MPMCM_REG_CONTROL_1_MASK_FRQS);
				// Read and reset measurements.
				measure_status = MEASURE_get_accumulated_data(MEASURE_DATA_TYPE_MAINS_FREQUENCY, &data);
				MEASURE_exit_error(NODE_ERROR_BASE_MEASURE);
				// Write registers.
				data_reg_value = 0;
				data_reg_mask = 0;
				field_value = (data.number_of_samples > 0) ? (uint32_t) (data.rolling_mean / 10) : DINFOX_MAINS_FREQUENCY_ERROR_VALUE;
				DINFOX_write_field(&data_reg_value, &data_reg_mask, field_value, MPMCM_REG_MASK_MEAN);
				NODE_write_register(NODE_REQUEST_SOURCE_INTERNAL, MPMCM_REG_ADDR_MAINS_FREQUENCY_0, data_reg_mask, data_reg_value);
				data_reg_value = 0;
				data_reg_mask = 0;
				field_value = (data.number_of_samples > 0) ? (uint32_t) (data.min / 10) : DINFOX_MAINS_FREQUENCY_ERROR_VALUE;
				DINFOX_write_field(&data_reg_value, &data_reg_mask, field_value, MPMCM_REG_MASK_MIN);
				field_value = (data.number_of_samples > 0) ? (uint32_t) (data.max / 10) : DINFOX_MAINS_FREQUENCY_ERROR_VALUE;
				DINFOX_write_field(&data_reg_value, &data_reg_mask, field_value, MPMCM_REG_MASK_MAX);
				NODE_write_register(NODE_REQUEST_SOURCE_INTERNAL, MPMCM_REG_ADDR_MAINS_FREQUENCY_1, data_reg_mask, data_reg_value);
			}
		}
		// CHxS.
		for (channel_idx=0 ; channel_idx<ADC_NUMBER_OF_ACI_CHANNELS ; channel_idx++) {
			//  Check mask.
			if ((reg_mask & MPMCM_REG_CONTROL_1_MASK_FRQS) != 0) {
				// Check bit.
				if (DINFOX_read_field(reg_value, (0b1 << channel_idx)) != 0) {
					// Clear request.
					DINFOX_write_field(&new_reg_value, &new_reg_mask, 0b0, (0b1 << channel_idx));
					// Read and reset measurements.
					measure_status = MEASURE_get_channel_accumulated_data(channel_idx, &channel_data);
					MEASURE_exit_error(NODE_ERROR_BASE_MEASURE);
					// Compute registers offset.
					reg_offset = (MPMCM_NUMBER_OF_REG_PER_DATA * channel_idx);
					// Active power.
					data_reg_value = 0;
					data_reg_mask = 0;
					field_value = (channel_data.active_power_mw.number_of_samples > 0) ? DINFOX_convert_mw_mva(channel_data.active_power_mw.rolling_mean) : DINFOX_ELECTRICAL_POWER_ERROR_VALUE;
					DINFOX_write_field(&data_reg_value, &data_reg_mask, field_value, MPMCM_REG_MASK_MEAN);
					NODE_write_register(NODE_REQUEST_SOURCE_INTERNAL, (MPMCM_REG_ADDR_CH1_ACTIVE_POWER_0 + reg_offset), data_reg_mask, data_reg_value);
					data_reg_value = 0;
					data_reg_mask = 0;
					field_value = (channel_data.active_power_mw.number_of_samples > 0) ? DINFOX_convert_mw_mva(channel_data.active_power_mw.min) : DINFOX_ELECTRICAL_POWER_ERROR_VALUE;
					DINFOX_write_field(&data_reg_value, &data_reg_mask, field_value, MPMCM_REG_MASK_MIN);
					field_value = (channel_data.active_power_mw.number_of_samples > 0) ? DINFOX_convert_mw_mva(channel_data.active_power_mw.max) : DINFOX_ELECTRICAL_POWER_ERROR_VALUE;
					DINFOX_write_field(&data_reg_value, &data_reg_mask, field_value, MPMCM_REG_MASK_MAX);
					NODE_write_register(NODE_REQUEST_SOURCE_INTERNAL, (MPMCM_REG_ADDR_CH1_ACTIVE_POWER_1 + reg_offset), data_reg_mask, data_reg_value);
					// RMS voltage.
					data_reg_value = 0;
					data_reg_mask = 0;
					field_value = (channel_data.rms_voltage_mv.number_of_samples > 0) ? DINFOX_convert_mv(channel_data.rms_voltage_mv.rolling_mean) : DINFOX_VOLTAGE_ERROR_VALUE;
					DINFOX_write_field(&data_reg_value, &data_reg_mask, field_value, MPMCM_REG_MASK_MEAN);
					NODE_write_register(NODE_REQUEST_SOURCE_INTERNAL, (MPMCM_REG_ADDR_CH1_RMS_VOLTAGE_0 + reg_offset), data_reg_mask, data_reg_value);
					data_reg_value = 0;
					data_reg_mask = 0;
					field_value = (channel_data.rms_voltage_mv.number_of_samples > 0) ? DINFOX_convert_mv(channel_data.rms_voltage_mv.min) : DINFOX_VOLTAGE_ERROR_VALUE;
					DINFOX_write_field(&data_reg_value, &data_reg_mask, field_value, MPMCM_REG_MASK_MIN);
					field_value = (channel_data.rms_voltage_mv.number_of_samples > 0) ? DINFOX_convert_mv(channel_data.rms_voltage_mv.max) : DINFOX_VOLTAGE_ERROR_VALUE;
					DINFOX_write_field(&data_reg_value, &data_reg_mask, field_value, MPMCM_REG_MASK_MAX);
					NODE_write_register(NODE_REQUEST_SOURCE_INTERNAL, (MPMCM_REG_ADDR_CH1_RMS_VOLTAGE_1 + reg_offset), data_reg_mask, data_reg_value);
					// RMS current.
					data_reg_value = 0;
					data_reg_mask = 0;
					field_value = (channel_data.rms_current_ma.number_of_samples > 0) ? DINFOX_convert_ua(channel_data.rms_current_ma.rolling_mean * 1000) : DINFOX_CURRENT_ERROR_VALUE;
					DINFOX_write_field(&data_reg_value, &data_reg_mask, field_value, MPMCM_REG_MASK_MEAN);
					NODE_write_register(NODE_REQUEST_SOURCE_INTERNAL, (MPMCM_REG_ADDR_CH1_RMS_CURRENT_0 + reg_offset), data_reg_mask, data_reg_value);
					data_reg_value = 0;
					data_reg_mask = 0;
					field_value = (channel_data.rms_current_ma.number_of_samples > 0) ? DINFOX_convert_ua(channel_data.rms_current_ma.min * 1000) : DINFOX_CURRENT_ERROR_VALUE;
					DINFOX_write_field(&data_reg_value, &data_reg_mask, field_value, MPMCM_REG_MASK_MIN);
					field_value = (channel_data.rms_current_ma.number_of_samples > 0) ? DINFOX_convert_ua(channel_data.rms_current_ma.max * 1000) : DINFOX_CURRENT_ERROR_VALUE;
					DINFOX_write_field(&data_reg_value, &data_reg_mask, field_value, MPMCM_REG_MASK_MAX);
					NODE_write_register(NODE_REQUEST_SOURCE_INTERNAL, (MPMCM_REG_ADDR_CH1_RMS_CURRENT_1 + reg_offset), data_reg_mask, data_reg_value);
					// Apparent power.
					data_reg_value = 0;
					data_reg_mask = 0;
					field_value = (channel_data.apparent_power_mva.number_of_samples > 0) ? DINFOX_convert_mw_mva(channel_data.apparent_power_mva.rolling_mean) : DINFOX_ELECTRICAL_POWER_ERROR_VALUE;
					DINFOX_write_field(&data_reg_value, &data_reg_mask, field_value, MPMCM_REG_MASK_MEAN);
					NODE_write_register(NODE_REQUEST_SOURCE_INTERNAL, (MPMCM_REG_ADDR_CH1_APPARENT_POWER_0 + reg_offset), data_reg_mask, data_reg_value);
					data_reg_value = 0;
					data_reg_mask = 0;
					field_value = (channel_data.apparent_power_mva.number_of_samples > 0) ? DINFOX_convert_mw_mva(channel_data.apparent_power_mva.min) : DINFOX_ELECTRICAL_POWER_ERROR_VALUE;
					DINFOX_write_field(&data_reg_value, &data_reg_mask, field_value, MPMCM_REG_MASK_MIN);
					field_value = (channel_data.apparent_power_mva.number_of_samples > 0) ? DINFOX_convert_mw_mva(channel_data.apparent_power_mva.max) : DINFOX_ELECTRICAL_POWER_ERROR_VALUE;
					DINFOX_write_field(&data_reg_value, &data_reg_mask, field_value, MPMCM_REG_MASK_MAX);
					NODE_write_register(NODE_REQUEST_SOURCE_INTERNAL, (MPMCM_REG_ADDR_CH1_APPARENT_POWER_1 + reg_offset), data_reg_mask, data_reg_value);
					// Power factor.
					data_reg_value = 0;
					data_reg_mask = 0;
					field_value = (channel_data.power_factor.number_of_samples > 0) ? DINFOX_convert_power_factor(channel_data.power_factor.rolling_mean) : DINFOX_POWER_FACTOR_ERROR_VALUE;
					DINFOX_write_field(&data_reg_value, &data_reg_mask, field_value, MPMCM_REG_MASK_MEAN);
					NODE_write_register(NODE_REQUEST_SOURCE_INTERNAL, (MPMCM_REG_ADDR_CH1_POWER_FACTOR_0 + reg_offset), data_reg_mask, data_reg_value);
					data_reg_value = 0;
					data_reg_mask = 0;
					field_value = (channel_data.power_factor.number_of_samples > 0) ? DINFOX_convert_power_factor(channel_data.power_factor.min) : DINFOX_POWER_FACTOR_ERROR_VALUE;
					DINFOX_write_field(&data_reg_value, &data_reg_mask, field_value, MPMCM_REG_MASK_MIN);
					field_value = (channel_data.power_factor.number_of_samples > 0) ? DINFOX_convert_power_factor(channel_data.power_factor.max) : DINFOX_POWER_FACTOR_ERROR_VALUE;
					DINFOX_write_field(&data_reg_value, &data_reg_mask, field_value, MPMCM_REG_MASK_MAX);
					NODE_write_register(NODE_REQUEST_SOURCE_INTERNAL, (MPMCM_REG_ADDR_CH1_POWER_FACTOR_1 + reg_offset), data_reg_mask, data_reg_value);
					// Active and apparent energy.
					data_reg_value = 0;
					data_reg_mask = 0;
					DINFOX_write_field(&data_reg_value, &data_reg_mask, DINFOX_convert_mwh_mvah(channel_data.active_energy_mwh), MPMCM_REG_MASK_ACTIVE_ENERGY);
					DINFOX_write_field(&data_reg_value, &data_reg_mask, DINFOX_convert_mwh_mvah(channel_data.apparent_energy_mvah), MPMCM_REG_MASK_APPARENT_ENERGY);
					NODE_write_register(NODE_REQUEST_SOURCE_INTERNAL, (MPMCM_REG_ADDR_CH1_ENERGY + reg_offset), data_reg_mask, data_reg_value);
				}
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
	MEASURE_status_t measure_status = MEASURE_SUCCESS;
	MEASURE_data_t data;
	MEASURE_channel_data_t channel_data;
	uint32_t field_value = 0;
	uint8_t reg_offset = 0;
	uint32_t data_reg_value = 0;
	uint32_t data_reg_mask = 0;
	uint8_t channel_idx = 0;
	// Read run data.
	measure_status = MEASURE_get_run_data(MEASURE_DATA_TYPE_MAINS_FREQUENCY, &data);
	MEASURE_exit_error(NODE_ERROR_BASE_MEASURE);
	// Write register.
	data_reg_value = 0;
	data_reg_mask = 0;
	field_value = (data.number_of_samples > 0) ? (uint32_t) (data.value / 10) : DINFOX_MAINS_FREQUENCY_ERROR_VALUE;
	DINFOX_write_field(&data_reg_value, &data_reg_mask, field_value, MPMCM_REG_MASK_RUN);
	NODE_write_register(NODE_REQUEST_SOURCE_INTERNAL, MPMCM_REG_ADDR_MAINS_FREQUENCY_0, data_reg_mask, data_reg_value);
	// Update run registers for all channels.
	for (channel_idx=0 ; channel_idx<ADC_NUMBER_OF_ACI_CHANNELS ; channel_idx++) {
		// Read run data.
		measure_status = MEASURE_get_channel_run_data(channel_idx, &channel_data);
		MEASURE_exit_error(NODE_ERROR_BASE_MEASURE);
		// Compute registers offset.
		reg_offset = (MPMCM_NUMBER_OF_REG_PER_DATA * channel_idx);
		// Active power.
		data_reg_value = 0;
		data_reg_mask = 0;
		field_value = (channel_data.active_power_mw.number_of_samples > 0) ? DINFOX_convert_mw_mva(channel_data.active_power_mw.value) : DINFOX_ELECTRICAL_POWER_ERROR_VALUE;
		DINFOX_write_field(&data_reg_value, &data_reg_mask, field_value, MPMCM_REG_MASK_RUN);
		NODE_write_register(NODE_REQUEST_SOURCE_INTERNAL, (MPMCM_REG_ADDR_CH1_ACTIVE_POWER_0 + reg_offset), data_reg_mask, data_reg_value);
		// RMS voltage.
		data_reg_value = 0;
		data_reg_mask = 0;
		field_value = (channel_data.rms_voltage_mv.number_of_samples > 0) ? DINFOX_convert_mv(channel_data.rms_voltage_mv.value) : DINFOX_VOLTAGE_ERROR_VALUE;
		DINFOX_write_field(&data_reg_value, &data_reg_mask, field_value, MPMCM_REG_MASK_RUN);
		NODE_write_register(NODE_REQUEST_SOURCE_INTERNAL, (MPMCM_REG_ADDR_CH1_RMS_VOLTAGE_0 + reg_offset), data_reg_mask, data_reg_value);
		// RMS current.
		data_reg_value = 0;
		data_reg_mask = 0;
		field_value = (channel_data.rms_current_ma.number_of_samples > 0) ? DINFOX_convert_ua(channel_data.rms_current_ma.value * 1000) : DINFOX_CURRENT_ERROR_VALUE;
		DINFOX_write_field(&data_reg_value, &data_reg_mask, field_value, MPMCM_REG_MASK_RUN);
		NODE_write_register(NODE_REQUEST_SOURCE_INTERNAL, (MPMCM_REG_ADDR_CH1_RMS_CURRENT_0 + reg_offset), data_reg_mask, data_reg_value);
		// Apparent power.
		data_reg_value = 0;
		data_reg_mask = 0;
		field_value = (channel_data.apparent_power_mva.number_of_samples > 0) ? DINFOX_convert_mw_mva(channel_data.apparent_power_mva.value) : DINFOX_ELECTRICAL_POWER_ERROR_VALUE;
		DINFOX_write_field(&data_reg_value, &data_reg_mask, field_value, MPMCM_REG_MASK_RUN);
		NODE_write_register(NODE_REQUEST_SOURCE_INTERNAL, (MPMCM_REG_ADDR_CH1_APPARENT_POWER_0 + reg_offset), data_reg_mask, data_reg_value);
		// Power factor.
		data_reg_value = 0;
		data_reg_mask = 0;
		field_value = (channel_data.power_factor.number_of_samples > 0) ? DINFOX_convert_power_factor(channel_data.power_factor.value) : DINFOX_POWER_FACTOR_ERROR_VALUE;
		DINFOX_write_field(&data_reg_value, &data_reg_mask, field_value, MPMCM_REG_MASK_RUN);
		NODE_write_register(NODE_REQUEST_SOURCE_INTERNAL, (MPMCM_REG_ADDR_CH1_POWER_FACTOR_0 + reg_offset), data_reg_mask, data_reg_value);
	}
errors:
	return status;
}
