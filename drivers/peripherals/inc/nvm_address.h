/*
 * nvm_address.h
 *
 *  Created on: 20 feb. 2025
 *      Author: Ludo
 */

#ifndef __NVM_ADDRESS_H__
#define __NVM_ADDRESS_H__

#include "mpmcm_registers.h"

/*!******************************************************************
 * \enum NVM_address_t
 * \brief NVM address mapping.
 *******************************************************************/
typedef enum {
    NVM_ADDRESS_SELF_ADDRESS = 0,
    NVM_ADDRESS_REGISTERS = 0x40,
} NVM_address_t;

#endif /* __NVM_ADDRESS_H__ */
