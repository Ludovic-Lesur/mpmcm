/*
 * node.c
 *
 *  Created on: 28 may. 2023
 *      Author: Ludo
 */

#include "node.h"

#include "common.h"
#include "error.h"
#include "error_base.h"
#include "gpsm_registers.h"
#include "led.h"
#include "mpmcm.h"
#include "mpmcm_flags.h"
#include "mpmcm_registers.h"
#include "nvm.h"
#include "nvm_address.h"
#include "power.h"
#include "rtc.h"
#include "swreg.h"
#include "una.h"

/*** NODE local structures ***/

/*******************************************************************/
typedef struct {
    volatile uint32_t registers[NODE_REGISTER_ADDRESS_LAST];
    NODE_state_t state;
} NODE_context_t;

/*** NODE local global variables ***/

static NODE_context_t node_ctx;

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
static NODE_status_t _NODE_check_register(uint8_t reg_addr, uint32_t reg_mask) {
    // Local variables.
    NODE_status_t status = NODE_SUCCESS;
    // Check common registers.
    status = COMMON_check_register(reg_addr, reg_mask);
    if (status != NODE_SUCCESS) goto errors;
    // Check specific registers.
    status = MPMCM_check_register(reg_addr, reg_mask);
errors:
    return status;
}

/*** NODE functions ***/

/*******************************************************************/
NODE_status_t NODE_init(void) {
    // Local variables.
    NODE_status_t status = NODE_SUCCESS;
    NVM_status_t nvm_status = NVM_SUCCESS;
    uint32_t self_address = 0;
    uint8_t idx = 0;
    // Init context.
    for (idx = 0; idx < NODE_REGISTER_ADDRESS_LAST; idx++) {
        node_ctx.registers[idx] = 0;
    }
    node_ctx.state = NODE_STATE_IDLE;
#ifdef MPMCM_NVM_FACTORY_RESET
    nvm_status = NVM_erase();
    NVM_exit_error(NODE_ERROR_BASE_NVM);
    nvm_status = NVM_write_word(NVM_ADDRESS_SELF_ADDRESS, (uint32_t) MPMCM_NODE_ADDRESS);
    NVM_exit_error(NODE_ERROR_BASE_NVM);
#endif
    // Read self address in NVM.
    nvm_status = NVM_read_word(NVM_ADDRESS_SELF_ADDRESS, &self_address);
    NVM_exit_error(NODE_ERROR_BASE_NVM);
    // Init common registers.
    status = COMMON_init_registers((UNA_node_address_t) self_address);
    if (status != NODE_SUCCESS) goto errors;
    // Init specific registers.
    status = MPMCM_init_registers();
    if (status != NODE_SUCCESS) goto errors;
errors:
    return status;
}

/*******************************************************************/
NODE_status_t NODE_de_init(void) {
    // Local variables.
    NODE_status_t status = NODE_SUCCESS;
    return status;
}

/*******************************************************************/
NODE_status_t NODE_process(void) {
    // Local variables.
    NODE_status_t status = NODE_SUCCESS;
    // Reset state to default.
    node_ctx.state = NODE_STATE_IDLE;
    return status;
}

/*******************************************************************/
NODE_state_t NODE_get_state(void) {
    return (node_ctx.state);
}

/*******************************************************************/
NODE_status_t NODE_write_register(NODE_request_source_t request_source, uint8_t reg_addr, uint32_t reg_value, uint32_t reg_mask) {
    // Local variables.
    NODE_status_t status = NODE_SUCCESS;
    // Check address.
    if (reg_addr >= NODE_REGISTER_ADDRESS_LAST) {
        status = NODE_ERROR_REGISTER_ADDRESS;
        goto errors;
    }
    // Check access.
    if ((request_source == NODE_REQUEST_SOURCE_EXTERNAL) && (NODE_REGISTER_ACCESS[reg_addr] == UNA_REGISTER_ACCESS_READ_ONLY)) {
        status = NODE_ERROR_REGISTER_READ_ONLY;
        goto errors;
    }
    // Write register.
    SWREG_modify_register((uint32_t*) &(node_ctx.registers[reg_addr]), reg_value, reg_mask);
    // Check actions.
    if (request_source == NODE_REQUEST_SOURCE_EXTERNAL) {
        // Check control bits.
        status = _NODE_check_register(reg_addr, reg_mask);
        if (status != NODE_SUCCESS) goto errors;
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
    uint8_t reg_addr = 0;
    uint32_t reg_value = 0;
    uint32_t reg_mask = 0;
    // Check parameters.
    if (data == NULL) {
        status = NODE_ERROR_NULL_PARAMETER;
        goto errors;
    }
    // Byte loop.
    for (idx = 0; idx < data_size_byte; idx++) {
        // Compute address, mask and value.
        reg_addr = (reg_addr_base + (idx >> 2));
        shift = ((idx % 4) << 3);
        reg_mask = (0xFF << shift);
        reg_value = (data[idx] << shift);
        // Write register.
        status = NODE_write_register(request_source, reg_addr, reg_value, reg_mask);
        if (status != NODE_SUCCESS) goto errors;
    }
errors:
    return status;
}

/*******************************************************************/
NODE_status_t NODE_write_nvm(uint8_t reg_addr, uint32_t reg_value) {
    // Local variables.
    NODE_status_t status = NODE_SUCCESS;
    NVM_status_t nvm_status = NVM_SUCCESS;
    // Write NVM.
    nvm_status = NVM_write_word((NVM_ADDRESS_REGISTERS + reg_addr), reg_value);
    NVM_exit_error(NODE_ERROR_BASE_NVM);
errors:
    return status;
}

/*******************************************************************/
NODE_status_t NODE_read_register(NODE_request_source_t request_source, uint8_t reg_addr, uint32_t* reg_value) {
    // Local variables.
    NODE_status_t status = NODE_SUCCESS;
    // Check parameters.
    if (reg_addr >= NODE_REGISTER_ADDRESS_LAST) {
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
    (*reg_value) = node_ctx.registers[reg_addr];
errors:
    return status;
}

/*******************************************************************/
NODE_status_t NODE_read_byte_array(NODE_request_source_t request_source, uint8_t reg_addr_base, uint8_t* data, uint8_t data_size_byte) {
    // Local variables.
    NODE_status_t status = NODE_SUCCESS;
    uint8_t idx = 0;
    uint8_t reg_addr = 0;
    uint32_t reg_value = 0;
    uint32_t reg_mask = 0;
    // Check parameters.
    if (data == NULL) {
        status = NODE_ERROR_NULL_PARAMETER;
        goto errors;
    }
    // Byte loop.
    for (idx = 0; idx < data_size_byte; idx++) {
        // Compute address and mask.
        reg_addr = (reg_addr_base + (idx >> 2));
        reg_mask = (0xFF << ((idx % 4) << 3));
        // Read byte.
        status = NODE_read_register(request_source, reg_addr, &reg_value);
        // Fill data.
        data[idx] = (uint8_t) SWREG_read_field(reg_value, reg_mask);
    }
errors:
    return status;
}

/*******************************************************************/
NODE_status_t NODE_read_nvm(uint8_t reg_addr, uint32_t* reg_value) {
    // Local variables.
    NODE_status_t status = NODE_SUCCESS;
    NVM_status_t nvm_status = NVM_SUCCESS;
    // Read NVM.
    nvm_status = NVM_read_word((NVM_ADDRESS_REGISTERS + reg_addr), reg_value);
    NVM_exit_error(NODE_ERROR_BASE_NVM);
errors:
    return status;
}
