/*
 * simulation.h
 *
 *  Created on: 20 apr. 2024
 *      Author: Ludo
 */

#ifndef __SIMULATION_H__
#define __SIMULATION_H__

#include "adc.h"
#include "mode.h"
#include "types.h"

#ifdef ANALOG_SIMULATION

/*** SIMULATION macros ***/

#define MEASURE_MAINS_PERIOD_US		20000
#define MEASURE_PERIOD_BUFFER_SIZE	(MEASURE_MAINS_PERIOD_US / ADC_SAMPLING_PERIOD_US)

#define SIMULATION_BUFFER_SIZE		(ADC_NUMBER_OF_ACI_CHANNELS * MEASURE_PERIOD_BUFFER_SIZE)

/*** SIMULATION global variables ***/

extern const uint8_t SIMULATION_GPIO_ACI_DETECT[ADC_NUMBER_OF_ACI_CHANNELS];

extern const int16_t SIMULATION_ACV_BUFFER[SIMULATION_BUFFER_SIZE];
extern const int16_t SIMULATION_ACI_BUFFER[SIMULATION_BUFFER_SIZE];

#endif /* ANALOG_SIMULATION */

#endif /* __SIMULATION_H__ */
