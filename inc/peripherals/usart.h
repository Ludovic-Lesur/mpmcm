/*
 * usart.h
 *
 *  Created on: 09 apr. 2024
 *      Author: Ludo
 */

#ifndef __USART_H__
#define __USART_H__

#include "rcc.h"
#include "types.h"

/*** USART structures ***/

/*!******************************************************************
 * \enum USART_status_t
 * \brief USART driver error codes.
 *******************************************************************/
typedef enum {
	// Driver errors.
	USART_SUCCESS = 0,
	USART_ERROR_NULL_PARAMETER,
	USART_ERROR_TX_TIMEOUT,
	// Low level drivers errors.
	USART_ERROR_BASE_RCC = 0x0100,
	// Last base value.
	USART_ERROR_BASE_LAST = (USART_ERROR_BASE_RCC + RCC_ERROR_BASE_LAST)
} USART_status_t;

/*!******************************************************************
 * \fn USART_character_match_irq_cb_t
 * \brief USART character match interrupt callback.
 *******************************************************************/
typedef void (*USART_character_match_irq_cb_t)(void);

/*** USART functions ***/

/*!******************************************************************
 * \fn USART_status_t USART2_init(USART_rx_irq_cb_t irq_callback)
 * \brief Init USART2 peripheral.
 * \param[in]	baud_rate: USART baud rate.
 * \param[in]	character_match: Character used to trigger interrupt.
 * \param[in]  	irq_callback: Function to call on interrupt.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
USART_status_t USART2_init(uint32_t baud_rate, char_t character_match, USART_character_match_irq_cb_t irq_callback);

/*!******************************************************************
 * \fn void USART2_de_init(void)
 * \brief Release USART2 peripheral.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void USART2_de_init(void);

/*!******************************************************************
 * \fn void USART2_enable_rx(void)
 * \brief Enable USART2 RX operation.
 * \param[in]   none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void USART2_enable_rx(void);

/*!******************************************************************
 * \fn void USART2_disable_rx(void)
 * \brief Disable USART2 RX operation.
 * \param[in]   none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void USART2_disable_rx(void);

/*******************************************************************/
#define USART2_exit_error(error_base) { if (usart2_status != USART_SUCCESS) { status = (error_base + usart2_status); goto errors; } }

/*******************************************************************/
#define USART2_stack_error(void) { if (usart2_status != USART_SUCCESS) { ERROR_stack_add(ERROR_BASE_USART2 + usart2_status); } }

/*******************************************************************/
#define USART2_stack_exit_error(error_code) { if (usart2_status != USART_SUCCESS) { ERROR_stack_add(ERROR_BASE_USART2 + usart2_status); status = error_code; goto errors; } }

#endif /* __USART_H__ */
