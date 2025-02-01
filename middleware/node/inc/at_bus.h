/*
 * at_bus.h
 *
 *  Created on: 18 apr. 2020
 *      Author: Ludo
 */

#ifndef __AT_BUS_H__
#define __AT_BUS_H__

#include "mode.h"
#include "node_common.h"
#include "types.h"

/*** AT functions ***/

/*!******************************************************************
 * \fn void AT_BUS_init(void)
 * \brief Init AT BUS interface.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void AT_BUS_init(void);

/*!******************************************************************
 * \fn void AT_BUS_task(void)
 * \brief AT BUS interface task.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void AT_BUS_task(void);

#ifdef HIGH_SPEED_LOG
/*!******************************************************************
 * \fn void AT_BUS_high_speed_log(void)
 * \brief Print measure results on AT BUS interface.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void AT_BUS_high_speed_log(void);
#endif

#endif /* __AT_BUS_H__ */
