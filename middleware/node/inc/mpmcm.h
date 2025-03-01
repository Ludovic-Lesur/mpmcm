/*
 * mpmcm.h
 *
 *  Created on: 03 sep. 2023
 *      Author: Ludo
 */

#ifndef __MPMCM_H__
#define __MPMCM_H__

#include "mpmcm_registers.h"
#include "node.h"
#include "una.h"

/*** MPMCM macros ***/

#define NODE_BOARD_ID               UNA_BOARD_ID_MPMCM
#define NODE_REGISTER_ADDRESS_LAST  MPMCM_REGISTER_ADDRESS_LAST
#define NODE_REGISTER_ACCESS        MPMCM_REGISTER_ACCESS

/*** MPMCM functions ***/

/*!******************************************************************
 * \fn NODE_status_t MPMCM_init_registers(void)
 * \brief Init MPMCM registers to their default value.
 * \param[in]   none
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
NODE_status_t MPMCM_init_registers(void);

/*!******************************************************************
 * \fn NODE_status_t MPMCM_update_register(uint8_t reg_addr)
 * \brief Update MPMCM register.
 * \param[in]   reg_addr: Address of the register to update.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
NODE_status_t MPMCM_update_register(uint8_t reg_addr);

/*!******************************************************************
 * \fn NODE_status_t MPMCM_check_register(uint8_t reg_addr)
 * \brief Check MPMCM register.
 * \param[in]   reg_addr: Address of the register to check.
 * \param[in]   reg_mask: Mask of the bits to check.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
NODE_status_t MPMCM_check_register(uint8_t reg_addr, uint32_t reg_mask);

/*!******************************************************************
 * \fn NODE_status_t MPMCM_mtrg_callback(none)
 * \brief MPMCM measurements callback.
 * \param[in]   none
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
NODE_status_t MPMCM_mtrg_callback(void);

#endif /* __MPMCM_H__ */
