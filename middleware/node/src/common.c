/*
 * common.c
 *
 *  Created on: 04 jun. 2023
 *      Author: Ludo
 */

#include "common.h"

#include "analog.h"
#include "common_registers.h"
#include "error.h"
#include "mpmcm.h"
#include "node.h"
#include "nvm.h"
#include "pwr.h"
#include "swreg.h"
#include "types.h"
#include "version.h"
#include "una.h"

/*** COMMON local functions ***/

/*******************************************************************/
static void _COMMON_reset_analog_data(void) {
    // Local variables.
    uint32_t reg_analog_data_0 = 0;
    uint32_t reg_analog_data_0_mask = 0;
    // VMCU and TMCU.
    SWREG_write_field(&reg_analog_data_0, &reg_analog_data_0_mask, UNA_VOLTAGE_ERROR_VALUE, COMMON_REGISTER_ANALOG_DATA_0_MASK_VMCU);
    SWREG_write_field(&reg_analog_data_0, &reg_analog_data_0_mask, UNA_TEMPERATURE_ERROR_VALUE, COMMON_REGISTER_ANALOG_DATA_0_MASK_TMCU);
    NODE_write_register(NODE_REQUEST_SOURCE_INTERNAL, COMMON_REGISTER_ADDRESS_ANALOG_DATA_0, reg_analog_data_0, reg_analog_data_0_mask);
}

/*******************************************************************/
static NODE_status_t _COMMON_mtrg_callback(void) {
    // Local variables.
    NODE_status_t status = NODE_SUCCESS;
    ANALOG_status_t analog_status = ANALOG_SUCCESS;
    int32_t vmcu_mv = 0;
    int32_t tmcu_degrees = 0;
    uint32_t reg_analog_data_0 = 0;
    uint32_t reg_analog_data_0_mask = 0;
    // Common analog data.
    _COMMON_reset_analog_data();
    // Turn analog front-end on.
    POWER_enable(POWER_REQUESTER_ID_COMMON, POWER_DOMAIN_ANALOG, LPTIM_DELAY_MODE_ACTIVE);
    // MCU voltage.
    analog_status = ANALOG_convert_channel(ANALOG_CHANNEL_VMCU_MV, &vmcu_mv);
    ANALOG_exit_error(NODE_ERROR_BASE_ANALOG);
    SWREG_write_field(&reg_analog_data_0, &reg_analog_data_0_mask, (uint32_t) UNA_convert_mv(vmcu_mv), COMMON_REGISTER_ANALOG_DATA_0_MASK_VMCU);
    // MCU temperature.
    analog_status = ANALOG_convert_channel(ANALOG_CHANNEL_TMCU_DEGREES, &tmcu_degrees);
    ANALOG_exit_error(NODE_ERROR_BASE_ANALOG);
    SWREG_write_field(&reg_analog_data_0, &reg_analog_data_0_mask, (uint32_t) UNA_convert_degrees(tmcu_degrees), COMMON_REGISTER_ANALOG_DATA_0_MASK_TMCU);
    // Write register.
    NODE_write_register(NODE_REQUEST_SOURCE_INTERNAL, COMMON_REGISTER_ADDRESS_ANALOG_DATA_0, reg_analog_data_0, reg_analog_data_0_mask);
    // Specific analog data.
    status = MPMCM_mtrg_callback();
    if (status != NODE_SUCCESS) goto errors;
errors:
    POWER_disable(POWER_REQUESTER_ID_COMMON, POWER_DOMAIN_ANALOG);
    return status;
}

/*** COMMON functions ***/

/*******************************************************************/
NODE_status_t COMMON_init_registers(UNA_node_address_t self_address) {
    // Local variables.
    NODE_status_t status = NODE_SUCCESS;
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
    SWREG_write_field(&reg_node_id, &reg_node_id_mask, (uint32_t) self_address, COMMON_REGISTER_NODE_ID_MASK_NODE_ADDR);
    SWREG_write_field(&reg_node_id, &reg_node_id_mask, (uint32_t) UNA_BOARD_ID_MPMCM, COMMON_REGISTER_NODE_ID_MASK_BOARD_ID);
    // HW version register.
    SWREG_write_field(&reg_hw_version, &reg_hw_version_mask, 1, COMMON_REGISTER_HW_VERSION_MASK_MAJOR);
    SWREG_write_field(&reg_hw_version, &reg_hw_version_mask, 0, COMMON_REGISTER_HW_VERSION_MASK_MINOR);
    // SW version register 0.
    SWREG_write_field(&reg_sw_version_0, &reg_sw_version_0_mask, (uint32_t) GIT_MAJOR_VERSION, COMMON_REGISTER_SW_VERSION_0_MASK_MAJOR);
    SWREG_write_field(&reg_sw_version_0, &reg_sw_version_0_mask, (uint32_t) GIT_MINOR_VERSION, COMMON_REGISTER_SW_VERSION_0_MASK_MINOR);
    SWREG_write_field(&reg_sw_version_0, &reg_sw_version_0_mask, (uint32_t) GIT_COMMIT_INDEX, COMMON_REGISTER_SW_VERSION_0_MASK_COMMIT_INDEX);
    SWREG_write_field(&reg_sw_version_0, &reg_sw_version_0_mask, (uint32_t) GIT_DIRTY_FLAG, COMMON_REGISTER_SW_VERSION_0_MASK_DTYF);
    // SW version register 1.
    SWREG_write_field(&reg_sw_version_1, &reg_sw_version_1_mask, (uint32_t) GIT_COMMIT_ID, COMMON_REGISTER_SW_VERSION_1_MASK_COMMIT_ID);
    // Reset flags registers.
    SWREG_write_field(&reg_status_0, &reg_status_0_mask, (uint32_t) PWR_get_reset_flags(), COMMON_REGISTER_STATUS_0_MASK_RESET_FLAGS);
    SWREG_write_field(&reg_status_0, &reg_status_0_mask, 0b1, COMMON_REGISTER_STATUS_0_MASK_BF);
    // Write registers.
    NODE_write_register(NODE_REQUEST_SOURCE_INTERNAL, COMMON_REGISTER_ADDRESS_NODE_ID, reg_node_id, reg_node_id_mask);
    NODE_write_register(NODE_REQUEST_SOURCE_INTERNAL, COMMON_REGISTER_ADDRESS_HW_VERSION, reg_hw_version, reg_hw_version_mask);
    NODE_write_register(NODE_REQUEST_SOURCE_INTERNAL, COMMON_REGISTER_ADDRESS_SW_VERSION_0, reg_sw_version_0, reg_sw_version_0_mask);
    NODE_write_register(NODE_REQUEST_SOURCE_INTERNAL, COMMON_REGISTER_ADDRESS_SW_VERSION_1, reg_sw_version_1, reg_sw_version_1_mask);
    NODE_write_register(NODE_REQUEST_SOURCE_INTERNAL, COMMON_REGISTER_ADDRESS_STATUS_0, reg_status_0, reg_status_0_mask);
    // Load default values.
    _COMMON_reset_analog_data();
    return status;
}

/*******************************************************************/
NODE_status_t COMMON_update_register(uint8_t reg_addr) {
    // Local variables.
    NODE_status_t status = NODE_SUCCESS;
    uint32_t reg_value = 0;
    uint32_t reg_mask = 0;
    // Check address.
    switch (reg_addr) {
    case COMMON_REGISTER_ADDRESS_ERROR_STACK:
        // Unstack error.
        SWREG_write_field(&reg_value, &reg_mask, (uint32_t) ERROR_stack_read(), COMMON_REGISTER_ERROR_STACK_MASK_ERROR);
        break;
    case COMMON_REGISTER_ADDRESS_STATUS_0:
        // Check error stack.
        SWREG_write_field(&reg_value, &reg_mask, ((ERROR_stack_is_empty() == 0) ? 0b1 : 0b0), COMMON_REGISTER_STATUS_0_MASK_ESF);
        break;
    default:
        // Nothing to do.
        break;
    }
    // Write register.
    NODE_write_register(NODE_REQUEST_SOURCE_INTERNAL, reg_addr, reg_value, reg_mask);
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
    case COMMON_REGISTER_ADDRESS_CONTROL_0:
        // RTRG.
        if ((reg_mask & COMMON_REGISTER_CONTROL_0_MASK_RTRG) != 0) {
            // Read bit.
            if ((SWREG_read_field(reg_value, COMMON_REGISTER_CONTROL_0_MASK_RTRG)) != 0) {
                // Reset MCU.
                PWR_software_reset();
            }
        }
        // MTRG.
        if ((reg_mask & COMMON_REGISTER_CONTROL_0_MASK_MTRG) != 0) {
            // Read bit.
            if ((SWREG_read_field(reg_value, COMMON_REGISTER_CONTROL_0_MASK_MTRG)) != 0) {
                // Clear request.
                NODE_write_register(NODE_REQUEST_SOURCE_INTERNAL, COMMON_REGISTER_ADDRESS_CONTROL_0, 0b0, COMMON_REGISTER_CONTROL_0_MASK_MTRG);
                // Perform measurements.
                status = _COMMON_mtrg_callback();
                if (status != NODE_SUCCESS) goto errors;
            }
        }
        // BFC.
        if ((reg_mask & COMMON_REGISTER_CONTROL_0_MASK_BFC) != 0) {
            // Read bit.
            if ((SWREG_read_field(reg_value, COMMON_REGISTER_CONTROL_0_MASK_BFC)) != 0) {
                // Clear request and boot flag.
                NODE_write_register(NODE_REQUEST_SOURCE_INTERNAL, COMMON_REGISTER_ADDRESS_CONTROL_0, 0b0, COMMON_REGISTER_CONTROL_0_MASK_BFC);
                NODE_write_register(NODE_REQUEST_SOURCE_INTERNAL, COMMON_REGISTER_ADDRESS_STATUS_0, 0b0, COMMON_REGISTER_STATUS_0_MASK_BF);
                // Clear MCU reset flags.
                PWR_clear_reset_flags();
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
