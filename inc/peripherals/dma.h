/*
 * dma.h
 *
 *  Created on: 18 jun. 2023
 *      Author: Ludo
 */

#ifndef __DMA_H__
#define __DMA_H__

#include "types.h"

/*** DMA functions ***/

void DMA1_init(void);
void DMA1_set_destination_address(uint32_t adc1_buffer_address, uint32_t adc2_buffer_address, uint16_t adc_buffer_size);
void DMA1_start(void);
void DMA1_stop(void);
void DMA1_get_number_of_transfered_data(uint16_t* adc1_buffer_size, uint16_t* adc2_buffer_size);

#endif /* __DMA_H__ */
