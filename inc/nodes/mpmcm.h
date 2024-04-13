/*
 * ddrm.h
 *
 *  Created on: 03 sep. 2023
 *      Author: Ludo
 */

#ifndef __MPMCM_H__
#define __MPMCM_H__

#include "mpmcm_reg.h"
#include "common.h"
#include "common_reg.h"
#include "dinfox.h"
#include "node.h"

/*** MPMCM macros ***/

#define NODE_BOARD_ID		DINFOX_BOARD_ID_MPMCM
#define NODE_REG_ADDR_LAST	MPMCM_REG_ADDR_LAST

/*******************************************************************/
#define MPMCM_DATA_REG_ACCESS    \
	DINFOX_REG_ACCESS_READ_ONLY, \
	DINFOX_REG_ACCESS_READ_ONLY,

/*******************************************************************/
#define MPMCM_CHANNEL_REG_ACCESS  \
	MPMCM_DATA_REG_ACCESS         \
	MPMCM_DATA_REG_ACCESS         \
	MPMCM_DATA_REG_ACCESS         \
	MPMCM_DATA_REG_ACCESS         \
	MPMCM_DATA_REG_ACCESS		  \
	DINFOX_REG_ACCESS_READ_ONLY,  \

/*** MPMCM global variables ***/

static const DINFOX_register_access_t NODE_REG_ACCESS[MPMCM_REG_ADDR_LAST] = {
	COMMON_REG_ACCESS
	DINFOX_REG_ACCESS_READ_ONLY,
	DINFOX_REG_ACCESS_READ_ONLY,
	DINFOX_REG_ACCESS_READ_WRITE,
	DINFOX_REG_ACCESS_READ_WRITE,
	DINFOX_REG_ACCESS_READ_WRITE,
	DINFOX_REG_ACCESS_READ_WRITE,
	DINFOX_REG_ACCESS_READ_ONLY,
	DINFOX_REG_ACCESS_READ_WRITE,
	MPMCM_DATA_REG_ACCESS
	MPMCM_CHANNEL_REG_ACCESS
	MPMCM_CHANNEL_REG_ACCESS
	MPMCM_CHANNEL_REG_ACCESS
	MPMCM_CHANNEL_REG_ACCESS
	MPMCM_CHANNEL_REG_ACCESS
};

/*** MPMCM functions ***/

/*!******************************************************************
 * \fn void MPMCM_init_registers(void)
 * \brief Init MPMCM registers to their default value.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void MPMCM_init_registers(void);

/*!******************************************************************
 * \fn NODE_status_t MPMCM_update_register(uint8_t reg_addr)
 * \brief Update MPMCM register.
 * \param[in]  	reg_addr: Address of the register to update.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
NODE_status_t MPMCM_update_register(uint8_t reg_addr);

/*!******************************************************************
 * \fn NODE_status_t MPMCM_check_register(uint8_t reg_addr)
 * \brief Check MPMCM register.
 * \param[in]  	reg_addr: Address of the register to check.
 * \param[in]	reg_mask: Mask of the bits to check.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
NODE_status_t MPMCM_check_register(uint8_t reg_addr, uint32_t reg_mask);

/*!******************************************************************
 * \fn NODE_status_t MPMCM_mtrg_callback(none)
 * \brief MPMCM measurements callback.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
NODE_status_t MPMCM_mtrg_callback(void);

#endif /* __MPMCM_H__ */
