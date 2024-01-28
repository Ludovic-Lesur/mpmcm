/*
 * error.h
 *
 *  Created on: 27 feb. 2022
 *      Author: Ludo
 */

#ifndef __ERROR_H__
#define __ERROR_H__

// Peripherals.
#include "adc.h"
#include "flash.h"
#include "iwdg.h"
#include "lptim.h"
#include "lpuart.h"
#include "rcc.h"
#include "rtc.h"
// Utils.
#include "parser.h"
#include "string_custom.h"
// Components.
#include "measure.h"
#include "power.h"
// Nodes.
#include "lbus.h"
#include "node.h"

/*** ERROR structures ***/

/*!******************************************************************
 * \enum ERROR_base_t
 * \brief Board error bases.
 *******************************************************************/
typedef enum {
	SUCCESS = 0,
	// Peripherals.
	ERROR_BASE_ADC = 0x0100,
	ERROR_BASE_FLASH = (ERROR_BASE_ADC + ADC_ERROR_BASE_LAST),
	ERROR_BASE_IWDG = (ERROR_BASE_FLASH + FLASH_ERROR_BASE_LAST),
	ERROR_BASE_LPTIM1 = (ERROR_BASE_IWDG + IWDG_ERROR_BASE_LAST),
	ERROR_BASE_LPUART1 = (ERROR_BASE_LPTIM1 + LPTIM_ERROR_BASE_LAST),
	ERROR_BASE_RCC = (ERROR_BASE_LPUART1 + LPUART_ERROR_BASE_LAST),
	ERROR_BASE_RTC = (ERROR_BASE_RCC + RCC_ERROR_BASE_LAST),
	ERROR_BASE_TIM2 = (ERROR_BASE_RTC + RTC_ERROR_BASE_LAST),
	ERROR_BASE_TIM4 = (ERROR_BASE_TIM2 + TIM_ERROR_BASE_LAST),
	ERROR_BASE_TIM6 = (ERROR_BASE_TIM4 + TIM_ERROR_BASE_LAST),
	ERROR_BASE_TIM17 = (ERROR_BASE_TIM6 + TIM_ERROR_BASE_LAST),
	// Utils.
	ERROR_BASE_PARSER = (ERROR_BASE_TIM17 + TIM_ERROR_BASE_LAST),
	ERROR_BASE_STRING = (ERROR_BASE_PARSER + PARSER_ERROR_BASE_LAST),
	// Components.
	ERROR_BASE_MEASURE = (ERROR_BASE_STRING + STRING_ERROR_BASE_LAST),
	ERROR_BASE_POWER = (ERROR_BASE_MEASURE + MEASURE_ERROR_BASE_LAST),
	// Nodes.
	ERROR_BASE_LBUS = (ERROR_BASE_POWER + POWER_ERROR_BASE_LAST),
	ERROR_BASE_NODE = (ERROR_BASE_LBUS + LBUS_ERROR_BASE_LAST),
	// Last base value.
	ERROR_BASE_LAST = (ERROR_BASE_NODE + NODE_ERROR_BASE_LAST)
} ERROR_base_t;

/*!******************************************************************
 * \enum ERROR_code_t
 * \brief Board error code type.
 *******************************************************************/
typedef uint16_t ERROR_code_t;

/*** ERROR functions ***/

/*!******************************************************************
 * \fn void ERROR_stack_init(void)
 * \brief Init error stack.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void ERROR_stack_init(void);

/*!******************************************************************
 * \fn void ERROR_stack_add(ERROR_code_t code)
 * \brief Add error to stack.
 * \param[in]  	code: Error to stack.
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void ERROR_stack_add(ERROR_code_t code);

/*!******************************************************************
 * \fn ERROR_code_t ERROR_stack_read(void)
 * \brief Read error stack.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		Last error code stored.
 *******************************************************************/
ERROR_code_t ERROR_stack_read(void);

/*!******************************************************************
 * \fn uint8_t ERROR_stack_is_empty(void)
 * \brief Check if error stack is empty.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		1 if the error stack is empty, 0 otherwise.
 *******************************************************************/
uint8_t ERROR_stack_is_empty(void);

#endif /* __ERROR_H__ */
