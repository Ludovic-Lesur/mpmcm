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

/*** Board modes ***/

//#define ATM
//#define HIGH_SPEED_LOG
//#define DEBUG

/*** Board options ***/

// Transformer selection.
//#define BLOCK_VC_10_2_6
#define BLOCK_VB_2_1_6

// Transformer settings.
#ifdef BLOCK_VC_10_2_6
#define MPMCM_TRANSFORMER_ATTEN		11 		// Unit V/V.
#endif
#ifdef BLOCK_VB_2_1_6
#define MPMCM_TRANSFORMER_ATTEN		15 		// Unit V/V.
#endif

// Current sensors settings.
static const uint8_t MPMCM_SCT013_ATTEN[ADC_NUMBER_OF_ACI_CHANNELS] = {1, 1, 1, 1};		// Unit V/V.

#endif /* __MODE_H__ */
