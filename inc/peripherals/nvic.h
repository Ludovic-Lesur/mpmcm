/*
 * nvic.h
 *
 *  Created on: 17 jun. 2023
 *      Author: Ludo
 */

#ifndef __NVIC_H__
#define __NVIC_H__

#include "types.h"

/*** NVIC structures ***/

/*!******************************************************************
 * \enum NVIC_interrupt_t
 * \brief NVIC interrupt vector.
 *******************************************************************/
typedef enum {
	NVIC_INTERRUPT_WWDT = 0,
	NVIC_INTERRUPT_PVD_PVM,
	NVIC_INTERRUPT_RTC_TAMP_CSS,
	NVIC_INTERRUPT_RTC_WKUP,
	NVIC_INTERRUPT_FLASH,
	NVIC_INTERRUPT_RCC,
	NVIC_INTERRUPT_EXTI0,
	NVIC_INTERRUPT_EXTI1,
	NVIC_INTERRUPT_EXTI2,
	NVIC_INTERRUPT_EXTI3,
	NVIC_INTERRUPT_EXTI4,
	NVIC_INTERRUPT_DMA1_CH1,
	NVIC_INTERRUPT_DMA1_CH2,
	NVIC_INTERRUPT_DMA1_CH3,
	NVIC_INTERRUPT_DMA1_CH4,
	NVIC_INTERRUPT_DMA1_CH5,
	NVIC_INTERRUPT_DMA1_CH6,
	NVIC_INTERRUPT_DMA1_CH7,
	NVIC_INTERRUPT_ADC1_2,
	NVIC_INTERRUPT_USB_HP,
	NVIC_INTERRUPT_USB_LP,
	NVIC_INTERRUPT_FDCAN1_IT0,
	NVIC_INTERRUPT_FDCAN1_IT1,
	NVIC_INTERRUPT_EXTI5_9,
	NVIC_INTERRUPT_TIM1_BRK_TIM15,
	NVIC_INTERRUPT_TIM1_UP_TIM16,
	NVIC_INTERRUPT_TIM1_TRG_COM_DIR_IDX_TIM17,
	NVIC_INTERRUPT_TIM1_CC,
	NVIC_INTERRUPT_TIM2,
	NVIC_INTERRUPT_TIM3,
	NVIC_INTERRUPT_TIM4,
	NVIC_INTERRUPT_I2C1_EV,
	NVIC_INTERRUPT_I2C1_ER,
	NVIC_INTERRUPT_I2C2_EV,
	NVIC_INTERRUPT_I2C2_ER,
	NVIC_INTERRUPT_SPI1,
	NVIC_INTERRUPT_SPI2,
	NVIC_INTERRUPT_USART1,
	NVIC_INTERRUPT_USART2,
	NVIC_INTERRUPT_USART3,
	NVIC_INTERRUPT_EXTI10_15,
	NVIC_INTERRUPT_RTC_ALARM,
	NVIC_INTERRUPT_USB_WKUP,
	NVIC_INTERRUPT_TIM8_BRK_TER_IERR,
	NVIC_INTERRUPT_TIM8_UP,
	NVIC_INTERRUPT_TIM8_TRG_COM_DIR_IDX,
	NVIC_INTERRUPT_TIM8_CC,
	NVIC_INTERRUPT_ADC3,
	NVIC_INTERRUPT_FSMC,
	NVIC_INTERRUPT_LPTIM1,
	NVIC_INTERRUPT_TIM5,
	NVIC_INTERRUPT_SPI3,
	NVIC_INTERRUPT_UART4,
	NVIC_INTERRUPT_UART5,
	NVIC_INTERRUPT_TIM6_DAC1_3,
	NVIC_INTERRUPT_TIM7_DAC2_4,
	NVIC_INTERRUPT_DMA2_CH1,
	NVIC_INTERRUPT_DMA2_CH2,
	NVIC_INTERRUPT_DMA2_CH3,
	NVIC_INTERRUPT_DMA2_CH4,
	NVIC_INTERRUPT_DMA2_CH5,
	NVIC_INTERRUPT_ADC4,
	NVIC_INTERRUPT_ADC5,
	NVIC_INTERRUPT_UCPD1,
	NVIC_INTERRUPT_COMP1_2_3,
	NVIC_INTERRUPT_COMP4_5_6,
	NVIC_INTERRUPT_COMP7,
	NVIC_INTERRUPT_HRTIM_MASTER,
	NVIC_INTERRUPT_HRTIM_TIMA,
	NVIC_INTERRUPT_HRTIM_TIMB,
	NVIC_INTERRUPT_HRTIM_TIMC,
	NVIC_INTERRUPT_HRTIM_TIMD,
	NVIC_INTERRUPT_HRTIM_TIME,
	NVIC_INTERRUPT_HRTIM_TIM_FLT,
	NVIC_INTERRUPT_HRTIM_TIMF,
	NVIC_INTERRUPT_CRS,
	NVIC_INTERRUPT_SAI,
	NVIC_INTERRUPT_TIM20_BRK_TERR_IERR,
	NVIC_INTERRUPT_TIM20_UP,
	NVIC_INTERRUPT_TIM20_TRG_COM_DIR_IDX,
	NVIC_INTERRUPT_TIM20_CC,
	NVIC_INTERRUPT_FPU,
	NVIC_INTERRUPT_I2C4_EV,
	NVIC_INTERRUPT_I2C4_ER,
	NVIC_INTERRUPT_SPI4,
	NVIC_INTERRUPT_AES,
	NVIC_INTERRUPT_FDCAN2_IT0,
	NVIC_INTERRUPT_FDCAN2_IT1,
	NVIC_INTERRUPT_FDCAN3_IT0,
	NVIC_INTERRUPT_FDCAN3_IT1,
	NVIC_INTERRUPT_RNG,
	NVIC_INTERRUPT_LPUART1,
	NVIC_INTERRUPT_I2C3_EV,
	NVIC_INTERRUPT_I2C3_ER,
	NVIC_INTERRUPT_DMAMUX,
	NVIC_INTERRUPT_QUADSPI,
	NVIC_INTERRUPT_DMA1_CH8,
	NVIC_INTERRUPT_DMA2_CH6,
	NVIC_INTERRUPT_DMA2_CH7,
	NVIC_INTERRUPT_DMA2_CH8,
	NVIC_INTERRUPT_CORDIC,
	NVIC_INTERRUPT_FMAC
} NVIC_interrupt_t;

/*!******************************************************************
 * \enum NVIC_priority_list_t
 * \brief NVIC interrupt priorities list.
 *******************************************************************/
typedef enum {
	NVIC_PRIORITY_EXTI2 = 0,
	NVIC_PRIORITY_DMA1_CH1,
	NVIC_PRIORITY_DMA1_CH2,
	NVIC_PRIORITY_LPUART1,
	NVIC_PRIORITY_LPTIM1,
	NVIC_PRIORITY_TIM17,
	NVIC_PRIORITY_RTC_WKUP,
	NVIC_PRIORITY_RCC,
	NVIC_PRIORITY_USART2,
	NVIC_PRIORITY_DMA1_CH4,
	NVIC_PRIORITY_TIM4
} NVIC_priority_list_t;

/*** NVIC functions ***/

/*!******************************************************************
 * \fn void NVIC_init(void)
 * \brief Init interrupts vector.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void NVIC_init(void);

/*!******************************************************************
 * \fn void NVIC_enable_interrupt(NVIC_interrupt_t irq_index, uint8_t priority)
 * \brief Enable interrupt.
 * \param[in]  	irq_index: Interrupt to enable.
 * \param[in]	priority: Interrupt priority to set.
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void NVIC_enable_interrupt(NVIC_interrupt_t irq_index, uint8_t priority);

/*!******************************************************************
 * \fn void NVIC_disable_interrupt(NVIC_interrupt_t irq_index)
 * \brief Disable interrupt.
 * \param[in]  	irq_index: Interrupt to enable.
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void NVIC_disable_interrupt(NVIC_interrupt_t irq_index);

#endif /* __NVIC_H__ */
