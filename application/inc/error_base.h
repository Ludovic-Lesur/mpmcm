/*
 * error_base.h
 *
 *  Created on: 27 feb. 2022
 *      Author: Ludo
 */

#ifndef __ERROR_BASE_H__
#define __ERROR_BASE_H__

// Peripherals.
#include "adc.h"
#include "dma.h"
#include "flash.h"
#include "iwdg.h"
#include "lptim.h"
#include "lpuart.h"
#include "rcc.h"
#include "rtc.h"
#include "usart.h"
// Utils.
#include "maths.h"
#include "parser.h"
#include "strings.h"
// Components.
#include "led.h"
#include "measure.h"
#include "power.h"
#include "tic.h"
// Nodes.
#include "una_at.h"
#include "cli.h"
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
	ERROR_BASE_DMA = (ERROR_BASE_ADC + ADC_ERROR_BASE_LAST),
	ERROR_BASE_FLASH = (ERROR_BASE_DMA + DMA_ERROR_BASE_LAST),
	ERROR_BASE_IWDG = (ERROR_BASE_FLASH + FLASH_ERROR_BASE_LAST),
	ERROR_BASE_LPTIM = (ERROR_BASE_IWDG + IWDG_ERROR_BASE_LAST),
	ERROR_BASE_LPUART1 = (ERROR_BASE_LPTIM + LPTIM_ERROR_BASE_LAST),
	ERROR_BASE_RCC = (ERROR_BASE_LPUART1 + LPUART_ERROR_BASE_LAST),
	ERROR_BASE_RTC = (ERROR_BASE_RCC + RCC_ERROR_BASE_LAST),
	ERROR_BASE_TIM2 = (ERROR_BASE_RTC + RTC_ERROR_BASE_LAST),
	ERROR_BASE_TIM_LED = (ERROR_BASE_TIM2 + TIM_ERROR_BASE_LAST),
	ERROR_BASE_TIM6 = (ERROR_BASE_TIM_LED + TIM_ERROR_BASE_LAST),
	ERROR_BASE_TIM17 = (ERROR_BASE_TIM6 + TIM_ERROR_BASE_LAST),
	ERROR_BASE_USART2 = (ERROR_BASE_TIM17 + TIM_ERROR_BASE_LAST),
	// Utils.
	ERROR_BASE_MATH = (ERROR_BASE_USART2 + USART_ERROR_BASE_LAST),
	ERROR_BASE_PARSER = (ERROR_BASE_MATH + MATH_ERROR_BASE_LAST),
	ERROR_BASE_STRING = (ERROR_BASE_PARSER + PARSER_ERROR_BASE_LAST),
	// Components.
	ERROR_BASE_LED = (ERROR_BASE_STRING + STRING_ERROR_BASE_LAST),
	ERROR_BASE_MEASURE = (ERROR_BASE_LED + LED_ERROR_BASE_LAST),
	ERROR_BASE_POWER = (ERROR_BASE_MEASURE + MEASURE_ERROR_BASE_LAST),
	ERROR_BASE_TIC = (ERROR_BASE_POWER + POWER_ERROR_BASE_LAST),
	// Nodes.
	ERROR_BASE_UNA_AT = (ERROR_BASE_TIC + TIC_ERROR_BASE_LAST),
	ERROR_BASE_CLI = (ERROR_BASE_UNA_AT + UNA_AT_ERROR_BASE_LAST),
	ERROR_BASE_NODE = (ERROR_BASE_CLI + CLI_ERROR_BASE_LAST),
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

#endif /* __ERROR_BASE_H__ */
