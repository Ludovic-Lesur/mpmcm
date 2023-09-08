/*
 * mode.h
 *
 *  Created on: 21 mar. 2023
 *      Author: Ludo
 */

#ifndef __MODE_H__
#define __MODE_H__

#include "adc.h"
#include "types.h"

/*** Specific modes ***/

//#define ATM
//#define HIGH_SPEED_LOG	// Use 115200 raw transmission on RS485 bus.

/*** Current sensors selection ***/

static const uint8_t SCT013_GAIN[ADC_NUMBER_OF_ACI_CHANNELS] = {5, 5, 10, 20};

/*** Debug mode ***/

//#define DEBUG				// Disable watchdog for debug purpose if defined.

#endif /* __MODE_H__ */
