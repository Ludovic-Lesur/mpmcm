/*
 * common.h
 *
 *  Created on: 03 sep. 2023
 *      Author: Ludo
 */

#ifndef __COMMON_H__
#define __COMMON_H__

#include "node.h"
#include "types.h"

/*** COMMON functions ***/

/*!******************************************************************
 * \fn void COMMON_init_registers(void)
 * \brief Init common registers to their default value.
 * \param[in]  	self_address: RS485 address of the node.
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void COMMON_init_registers(NODE_address_t self_address);

/*!******************************************************************
 * \fn NODE_status_t COMMON_update_register(uint8_t reg_addr)
 * \brief Update common register.
 * \param[in]  	reg_addr: Address of the register to update.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
NODE_status_t COMMON_update_register(uint8_t reg_addr);

/*!******************************************************************
 * \fn NODE_status_t COMMON_check_register(uint8_t reg_addr)
 * \brief Check common register.
 * \param[in]  	reg_addr: Address of the register to check.
 * \param[in]	reg_mask: Mask of the bits to check.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
NODE_status_t COMMON_check_register(uint8_t reg_addr, uint32_t reg_mask);

#endif /* __COMMON_H__ */
