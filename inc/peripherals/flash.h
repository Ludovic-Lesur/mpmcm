/*
 * flash.h
 *
 *  Created on: 03 sep. 2023
 *      Author: Ludo
 */

#ifndef __FLASH_H__
#define __FLASH_H__

#include "types.h"

/*** FLASH structures ***/

/*!******************************************************************
 * \enum FLASH_status_t
 * \brief FLASH driver error codes.
 *******************************************************************/
typedef enum {
	// Driver errors.
	FLASH_SUCCESS = 0,
	FLASH_ERROR_NULL_PARAMETER,
	FLASH_ERROR_LATENCY,
	FLASH_ERROR_TIMEOUT,
	FLASH_ERROR_ADDRESS,
	FLASH_ERROR_UNLOCK,
	FLASH_ERROR_LOCK,
	FLASH_ERROR_ERASE,
	FLASH_ERROR_READ,
	FLASH_ERROR_WRITE,
	// Last base value.
	FLASH_ERROR_BASE_LAST = 0x0100
} FLASH_status_t;

/*!******************************************************************
 * \enum FLASH_address_t
 * \brief FLASH address mapping.
 *******************************************************************/
typedef enum {
	FLASH_ADDRESS_SELF_ADDRESS = 0,
	FLASH_ADDRESS_REGISTERS = 0x40,
	FLASH_ADDRESS_LAST = 0xFF
} FLASH_address_t;

/*** FLASH functions ***/

/*!******************************************************************
 * \fn FLASH_status_t FLASH_set_latency(uint8_t wait_states)
 * \brief Set FLASH latency.
 * \param[in]  	wait_states: Number of wait states to set.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
FLASH_status_t FLASH_set_latency(uint8_t wait_states);

/*!******************************************************************
 * \fn FLASH_status_t FLASH_read_word(FLASH_address_t address, uint32_t* data)
 * \brief Read 32-bits value in flash.
 * \param[in]  	address: Address to read.
 * \param[out] 	data: Pointer to word that will contain the read value.
 * \retval		Function execution status.
 *******************************************************************/
FLASH_status_t FLASH_read_word(FLASH_address_t address, uint32_t* data);

/*!******************************************************************
 * \fn FLASH_status_t FLASH_write_word(FLASH_address_t address, uint32_t data)
 * \brief Write a 32-bits value in flash.
 * \param[in]  	address: Address to write.
 * \param[out] 	data: Byte to write.
 * \retval		Function execution status.
 *******************************************************************/
FLASH_status_t FLASH_write_word(FLASH_address_t address, uint32_t data);

/*******************************************************************/
#define FLASH_exit_error(error_base) { if (flash_status != FLASH_SUCCESS) { status = (error_base + flash_status); goto errors; } }

/*******************************************************************/
#define FLASH_stack_error(void) { if (flash_status != FLASH_SUCCESS) { ERROR_stack_add(ERROR_BASE_FLASH + flash_status); } }

/*******************************************************************/
#define FLASH_stack_exit_error(error_code) { if (flash_status != FLASH_SUCCESS) { ERROR_stack_add(ERROR_BASE_FLASH + flash_status); status = error_code; goto errors; } }

#endif /* __FLASH_H__ */
