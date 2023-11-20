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

/*** MPMCM options ***/

// Transformer selection.
//#define BLOCK_VC_10_2_6
#define BLOCK_VB_2_1_6

// Current sensors selection.
static const uint8_t MPMCM_SCT013_GAIN[ADC_NUMBER_OF_ACI_CHANNELS] = {5, 5, 10, 20};

// Specific modes.
//#define ATM
//#define HIGH_SPEED_LOG

// Debug mode.
//#define DEBUG

#endif /* __MODE_H__ */
