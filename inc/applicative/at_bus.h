/*
 * at_bus.h
 *
 *  Created on: 18 apr. 2020
 *      Author: Ludo
 */

#ifndef __AT_BUS_H__
#define __AT_BUS_H__

#include "node_common.h"
#include "types.h"

/*** AT functions ***/

void AT_BUS_init(NODE_address_t self_address);
void AT_BUS_task(void);
void AT_BUS_fill_rx_buffer(uint8_t rx_byte);
#ifdef HIGH_SPEED_LOG
void AT_BUS_high_speed_log(void);
#endif

#endif /* __AT_BUS_H__ */
