/*
 * dma.c
 *
 *  Created on: Jun 18, 2023
 *      Author: ludo
 */

#include "dma.h"

#include "adc_reg.h"
#include "dma_reg.h"
#include "dmamux_reg.h"
#include "nvic.h"
#include "rcc_reg.h"
#include "types.h"

/*** DMA local structures ***/

/*******************************************************************/
typedef enum {
	DMA_CHANNEL_INDEX_ADC1 = 0,
	DMA_CHANNEL_INDEX_ADC2 = 1
} DMA_channel_index_t;

/*******************************************************************/
typedef struct {
	uint16_t buffer_size;
	DMA_transfer_complete_irq_cb_t tc_irq_callback;
} DMA_context_t;

/*** DMA local global variables ***/

static DMA_context_t dma_ctx = {
	.buffer_size = 0,
	.tc_irq_callback = NULL
};

/*** DMA local functions ***/

/*******************************************************************/
void __attribute__((optimize("-O0"))) DMA1_CH1_IRQHandler(void) {
	// Execute callback.
	if (dma_ctx.tc_irq_callback != NULL) {
		dma_ctx.tc_irq_callback();
	}
	// Clear flags.
	DMA1 -> IFCR |= 0x0000000F;
}

/*******************************************************************/
void __attribute__((optimize("-O0"))) DMA1_CH2_IRQHandler(void) {
	// Exectue callback.
	if (dma_ctx.tc_irq_callback != NULL) {
		dma_ctx.tc_irq_callback();
	}
	// Clear flags.
	DMA1 -> IFCR |= 0x000000F0;
}

/*** DMA functions ***/

/*******************************************************************/
void DMA1_init(DMA_transfer_complete_irq_cb_t irq_callback) {
	// Enable peripheral clock.
	RCC -> AHB1ENR |= (0b101 << 0);
	// Memory and peripheral sizes = 16bits.
	// Memory increment enabled.
	// Read from peripheral.
	// Transfer complete interrupt enable.
	// Configure ADC1 channel.
	(DMA1 -> CHx[DMA_CHANNEL_INDEX_ADC1]).CCR |= (0b01 << 10) | (0b01 << 8) | (0b1 << 7) | (0b1 << 1);
	(DMA1 -> CHx[DMA_CHANNEL_INDEX_ADC1]).CPAR = (uint32_t) &(ADC1 -> DR);
	// Configure ADC1 channel.
	(DMA1 -> CHx[DMA_CHANNEL_INDEX_ADC2]).CCR |= (0b01 << 10) | (0b01 << 8) | (0b1 << 7) | (0b1 << 1);
	(DMA1 -> CHx[DMA_CHANNEL_INDEX_ADC2]).CPAR = (uint32_t) &(ADC2 -> DR);
	// Configure DMA multiplexer.
	DMAMUX -> CxCR[DMA_CHANNEL_INDEX_ADC1] = 5;
	DMAMUX -> CxCR[DMA_CHANNEL_INDEX_ADC2] = 36;
	// Register callback.
	dma_ctx.tc_irq_callback = irq_callback;
}

/*******************************************************************/
void DMA1_set_destination_address(uint32_t adc1_buffer_address, uint32_t adc2_buffer_address, uint16_t adc_buffer_size) {
	// Set voltage buffer address for ADC1.
	(DMA1 -> CHx[DMA_CHANNEL_INDEX_ADC1]).CNDTR = adc_buffer_size;
	(DMA1 -> CHx[DMA_CHANNEL_INDEX_ADC1]).CMAR = adc1_buffer_address;
	// Set current buffer address for ADC2.
	(DMA1 -> CHx[DMA_CHANNEL_INDEX_ADC2]).CNDTR = adc_buffer_size;
	(DMA1 -> CHx[DMA_CHANNEL_INDEX_ADC2]).CMAR = adc2_buffer_address;
	// Store buffer size locally.
	dma_ctx.buffer_size = adc_buffer_size;
}

/*******************************************************************/
void DMA1_start(void) {
	// Clear all flags.
	DMA1 -> IFCR |= 0x000000FF;
	// Enable interrupts.
	NVIC_enable_interrupt(NVIC_INTERRUPT_DMA1_CH1, NVIC_PRIORITY_DMA1);
	NVIC_enable_interrupt(NVIC_INTERRUPT_DMA1_CH2, NVIC_PRIORITY_DMA1);
	// Start transfer.
	(DMA1 -> CHx[DMA_CHANNEL_INDEX_ADC1]).CCR |= (0b1 << 0); // EN='1'.
	(DMA1 -> CHx[DMA_CHANNEL_INDEX_ADC2]).CCR |= (0b1 << 0); // EN='1'.
}

/*******************************************************************/
void DMA1_stop(void) {
	// Stop transfer.
	(DMA1 -> CHx[DMA_CHANNEL_INDEX_ADC1]).CCR &= ~(0b1 << 0); // EN='0'.
	(DMA1 -> CHx[DMA_CHANNEL_INDEX_ADC2]).CCR &= ~(0b1 << 0); // EN='0'.
	// Enable interrupts.
	NVIC_disable_interrupt(NVIC_INTERRUPT_DMA1_CH1);
	NVIC_disable_interrupt(NVIC_INTERRUPT_DMA1_CH2);
	// Clear all flags.
	DMA1 -> IFCR |= 0x000000FF;
}

/*******************************************************************/
void DMA1_get_number_of_transfered_data(uint16_t* adc1_buffer_size, uint16_t* adc2_buffer_size) {
	// Read registers.
	(*adc1_buffer_size) = (dma_ctx.buffer_size - ((DMA1 -> CHx[DMA_CHANNEL_INDEX_ADC1]).CNDTR));
	(*adc2_buffer_size) = (dma_ctx.buffer_size - ((DMA1 -> CHx[DMA_CHANNEL_INDEX_ADC2]).CNDTR));
}
