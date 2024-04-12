/*
 * pwr.h
 *
 *  Created on: 18 jun. 2023
 *      Author: Ludo
 */

#ifndef __PWR_H__
#define __PWR_H__

/*** PWR functions ***/

/*!******************************************************************
 * \fn void PWR_init(void)
 * \brief Init PWR interface.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void PWR_init(void);

/*!******************************************************************
 * \fn void PWR_enter_sleep_mode(void)
 * \brief Enter sleep mode.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void PWR_enter_sleep_mode(void);

/*!******************************************************************
 * \fn void PWR_enter_stop_mode_1(void)
 * \brief Enter stop mode 1.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void PWR_enter_stop_mode_1(void);

/*!******************************************************************
 * \fn void PWR_software_reset(void)
 * \brief Reset the MCU.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void PWR_software_reset(void);

#endif /* __PWR_H__ */
