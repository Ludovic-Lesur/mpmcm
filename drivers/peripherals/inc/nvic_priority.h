/*
 * nvic_priority.h
 *
 *  Created on: 20 feb. 2025
 *      Author: Ludo
 */

#ifndef __NVIC_PRIORITY_H__
#define __NVIC_PRIORITY_H__

/*!******************************************************************
 * \enum NVIC_priority_list_t
 * \brief NVIC interrupt priorities list.
 *******************************************************************/
typedef enum {
    // Analog measure.
    NVIC_PRIORITY_ZERO_CROSS = 0,
    NVIC_PRIORITY_DMA_ACV_SAMPLING,
    NVIC_PRIORITY_DMA_ACI_SAMPLING,
    NVIC_PRIORITY_DMA_ACV_FREQUENCY,
    // TIC interface.
    NVIC_PRIORITY_TIC,
    NVIC_PRIORITY_DMA_TIC,
    // RS485 interface.
    NVIC_PRIORITY_RS485,
    // Common.
    NVIC_PRIORITY_CLOCK,
    NVIC_PRIORITY_CLOCK_CALIBRATION,
    NVIC_PRIORITY_DELAY,
    NVIC_PRIORITY_RTC,
    // Simulation timer.
    NVIC_PRIORITY_SIMULATION,
    // Unused lines.
    NVIC_PRIORITY_ADC_TRIGGER
} NVIC_priority_list_t;

#endif /* __NVIC_PRIORITY_H__ */
