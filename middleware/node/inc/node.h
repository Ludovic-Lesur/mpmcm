/*
 * node.h
 *
 *  Created on: 18 feb. 2023
 *      Author: Ludo
 */

#ifndef __NODE_H__
#define __NODE_H__

#include "analog.h"
#include "error.h"
#include "lptim.h"
#include "measure.h"
#include "nvm.h"
#include "power.h"
#include "tic.h"
#include "types.h"
#include "una.h"

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
    NODE_ERROR_REGISTER_FIELD_RANGE,
    NODE_ERROR_RADIO_STATE,
    NODE_ERROR_RADIO_POWER,
    NODE_ERROR_FORCED_HARDWARE,
    NODE_ERROR_FORCED_SOFTWARE,
    NODE_ERROR_SIGFOX_MCU_API,
    NODE_ERROR_SIGFOX_RF_API,
    NODE_ERROR_SIGFOX_EP_API,
    // Low level drivers errors.
    NODE_ERROR_BASE_NVM = ERROR_BASE_STEP,
    NODE_ERROR_BASE_LPTIM = (NODE_ERROR_BASE_NVM + NVM_ERROR_BASE_LAST),
    NODE_ERROR_BASE_ANALOG = (NODE_ERROR_BASE_LPTIM + LPTIM_ERROR_BASE_LAST),
    NODE_ERROR_BASE_MEASURE = (NODE_ERROR_BASE_ANALOG + ANALOG_ERROR_BASE_LAST),
    NODE_ERROR_BASE_TIC = (NODE_ERROR_BASE_MEASURE + MEASURE_ERROR_BASE_LAST),
    // Last base value.
    NODE_ERROR_BASE_LAST = (NODE_ERROR_BASE_TIC + TIC_ERROR_BASE_LAST)
} NODE_status_t;

/*!******************************************************************
 * \enum NODE_state_t
 * \brief NODE states list.
 *******************************************************************/
typedef enum {
    NODE_STATE_IDLE = 0,
    NODE_STATE_RUNNING,
    NODE_STATE_LAST
} NODE_state_t;

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
 * \fn NODE_status_t NODE_init(void)
 * \brief Init node driver.
 * \param[in]   none
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
NODE_status_t NODE_init(void);

/*!******************************************************************
 * \fn NODE_status_t NODE_de_init(void)
 * \brief Release node driver.
 * \param[in]   none
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
NODE_status_t NODE_de_init(void);

/*!******************************************************************
 * \fn NODE_status_t NODE_process(void)
 * \brief Execute node tasks.
 * \param[in]   none
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
NODE_status_t NODE_process(void);

/*!******************************************************************
 * \fn NODE_state_t NODE_get_state(void)
 * \brief Get node state.
 * \param[in]   none
 * \param[out]  none
 * \retval      Current node state.
 *******************************************************************/
NODE_state_t NODE_get_state(void);

/*!******************************************************************
 * \fn NODE_status_t NODE_write_register(NODE_request_source_t request_source, uint8_t reg_addr, uint32_t reg_value, uint32_t reg_mask)
 * \brief Write node register.
 * \param[in]   request_source: Request source.
 * \param[in]   reg_addr: Address of the register to write.
 * \param[in]   reg_value: Value to write in register.
 * \param[in]   reg_mask: Writing operation mask.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
NODE_status_t NODE_write_register(NODE_request_source_t request_source, uint8_t reg_addr, uint32_t reg_value, uint32_t reg_mask);

/*!******************************************************************
 * \fn NODE_status_t NODE_write_byte_array(NODE_request_source_t request_source, uint8_t reg_addr_base, uint8_t* data, uint8_t data_size_byte)
 * \brief Write multiple registers.
 * \param[in]   request_source: Request source.
 * \param[in]   reg_addr_base: Address of the first register to write.
 * \param[in]   data: Pointer to the registers value.
 * \param[in]   data_size_byte: Number of bytes to write.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
NODE_status_t NODE_write_byte_array(NODE_request_source_t request_source, uint8_t reg_addr_base, uint8_t* data, uint8_t data_size_byte);

/*!******************************************************************
 * \fn NODE_status_t NODE_write_nvm(uint8_t reg_addr, uint32_t reg_value)
 * \brief Write register in NVM.
 * \param[in]   reg_addr: Address of the register to write.
 * \param[in]   reg_value: Value to write in NVM.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
NODE_status_t NODE_write_nvm(uint8_t reg_addr, uint32_t reg_value);

/*!******************************************************************
 * \fn NODE_status_t NODE_read_register(NODE_request_source_t request_source, uint8_t reg_addr, uint32_t* reg_value)
 * \brief Read node register.
 * \param[in]   request_source: Request source.
 * \param[in]   reg_addr: Address of the register to read.
 * \param[out]  reg_value: Pointer to the register value.
 * \retval      Function execution status.
 *******************************************************************/
NODE_status_t NODE_read_register(NODE_request_source_t request_source, uint8_t reg_addr, uint32_t* reg_value);

/*!******************************************************************
 * \fn NODE_status_t NODE_read_byte_array(NODE_request_source_t request_source, uint8_t reg_addr_base, uint8_t* data, uint8_t data_size_byte)
 * \brief Read multiple registers in a byte array.
 * \param[in]   request_source: Request source.
 * \param[in]   reg_addr_base: Address of the first register to read.
 * \param[in]   data_size_byte: Number of bytes to read.
 * \param[out]  data: Pointer to the registers value.
 * \retval      Function execution status.
 *******************************************************************/
NODE_status_t NODE_read_byte_array(NODE_request_source_t request_source, uint8_t reg_addr_base, uint8_t* data, uint8_t data_size_byte);

/*!******************************************************************
 * \fn NODE_status_t NODE_read_nvm(uint8_t reg_addr, uint32_t* reg_value)
 * \brief Write register in NVM.
 * \param[in]   reg_addr: Address of the register to read.
 * \param[out]  reg_value: Pointer to the register value.
 * \retval      Function execution status.
 *******************************************************************/
NODE_status_t NODE_read_nvm(uint8_t reg_addr, uint32_t* reg_value);

/*******************************************************************/
#define NODE_exit_error(base) { ERROR_check_exit(node_status, NODE_SUCCESS, base) }

/*******************************************************************/
#define NODE_stack_error(base) { ERROR_check_stack(node_status, NODE_SUCCESS, base) }

/*******************************************************************/
#define NODE_stack_exit_error(base, code) { ERROR_check_stack_exit(node_status, NODE_SUCCESS, base, code) }

#endif /* __NODE_H__ */
