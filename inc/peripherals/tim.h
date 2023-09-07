/*
 * tim.h
 *
 *  Created on: 17 jun. 2023
 *      Author: Ludo
 */

#ifndef __TIM_H__
#define __TIM_H__

/*** TIM functions ***/

/*!******************************************************************
 * \fn void TIM6_init(void)
 * \brief Init TIM6 peripheral for ADC trigger operation.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void TIM6_init(void);

/*!******************************************************************
 * \fn void TIM6_start(void)
 * \brief Start TIM6.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void TIM6_start(void);

/*!******************************************************************
 * \fn void TIM6_stop(void)
 * \brief Stop TIM6.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void TIM6_stop(void);

#endif /* __TIM_H__ */
