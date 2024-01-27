/*
 * dma.h
 *
 *  Created on: 18 jun. 2023
 *      Author: Ludo
 */

#ifndef __DMA_H__
#define __DMA_H__

#include "types.h"

/*** DMA structures ***/

/*!******************************************************************
 * \fn DMA_transfer_complete_irq_cb
 * \brief DMA transfer complete interrupt callback.
 *******************************************************************/
typedef void (*DMA_transfer_complete_irq_cb_t)(void);

/*** DMA functions ***/

/*!******************************************************************
 * \fn void DMA1_adcx_init(DMA_transfer_complete_irq_cb_t irq_callback)
 * \brief Init DMA1 for ADC samples transfer.
 * \param[in]  	irq_callback: Function to call on transfer complete interrupt.
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void DMA1_adcx_init(DMA_transfer_complete_irq_cb_t irq_callback);

/*!******************************************************************
 * \fn void DMA1_adcx_set_destination_address(uint32_t adc1_buffer_address, uint32_t adc2_buffer_address, uint16_t adc_buffer_size)
 * \brief Set DMA1 ADC1 and ADC2 destination buffers address.
 * \param[in]  	adc1_buffer_address: ADC1 samples destination buffer address.
 * \param[in]  	adc2_buffer_address: ADC2 samples destination buffer address.
 * \param[in] 	adc_buffer_size: Destination buffer size (number of bytes to transfer).
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void DMA1_adcx_set_destination_address(uint32_t adc1_buffer_address, uint32_t adc2_buffer_address, uint16_t adc_buffer_size);

/*!******************************************************************
 * \fn void DMA1_adcx_start(void)
 * \brief Start DMA1 ADC1 and ADC2 transfer.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void DMA1_adcx_start(void);

/*!******************************************************************
 * \fn void DMA1_adcx_stop(void)
 * \brief Stop DMA1 ADC1 and ADC2 transfer.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void DMA1_adcx_stop(void);

/*!******************************************************************
 * \fn void DMA1_adcx_get_number_of_transfered_data(uint16_t* adc1_buffer_size, uint16_t* adc2_buffer_size)
 * \brief Get the current transfer status.
 * \param[in]  	none
 * \param[out] 	adc1_buffer_size: Current number of ADC1 samples transfered.
 * \param[out] 	adc2_buffer_size: Current number of ADC2 samples transfered.
 * \retval		none
 *******************************************************************/
void DMA1_adcx_get_number_of_transfered_data(uint16_t* adc1_buffer_size, uint16_t* adc2_buffer_size);

/*!******************************************************************
 * \fn void DMA1_tim2_init(void)
 * \brief Init DMA1 for TIM2 capture samples transfer.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void DMA1_tim2_init(void);

/*!******************************************************************
 * \fn void DMA1_tim2_set_destination_address(uint32_t tim2_buffer_address, uint16_t tim2_buffer_size)
 * \brief Set DMA1 TIM2 destination buffers address.
 * \param[in]  	tim2_buffer_address: TIM2 samples destination buffer address.
 * \param[in] 	tim2_buffer_size: Destination buffer size (number of bytes to transfer).
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void DMA1_tim2_set_destination_address(uint32_t tim2_buffer_address, uint16_t tim2_buffer_size);

/*!******************************************************************
 * \fn void DMA1_tim2_start(void)
 * \brief Start DMA1 TIM2 transfer.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void DMA1_tim2_start(void);

/*!******************************************************************
 * \fn void DMA1_tim2_stop(void)
 * \brief Stop DMA1 TIM2 transfer.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void DMA1_tim2_stop(void);

#endif /* __DMA_H__ */
