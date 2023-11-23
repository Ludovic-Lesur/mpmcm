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

// Transformer settings.
#ifdef BLOCK_VC_10_2_6
#define MPMCM_TRANSFORMER_GAIN		300		// Unit (10 * V/V).
#define MPMCM_TRANSFORMER_ATTEN		11 		// Unit V/V.
#endif
#ifdef BLOCK_VB_2_1_6
#define MPMCM_TRANSFORMER_GAIN		236 	// Unit (10 * V/V).
#define MPMCM_TRANSFORMER_ATTEN		15 		// Unit V/V.
#endif

// Current sensors settings.
static const uint8_t MPMCM_SCT013_GAIN[ADC_NUMBER_OF_ACI_CHANNELS] = {5, 5, 10, 20};	// Unit A/V.
static const uint8_t MPMCM_SCT013_ATTEN[ADC_NUMBER_OF_ACI_CHANNELS] = {1, 1, 1, 1};		// Unit V/V.

// Specific modes.
//#define ATM
//#define HIGH_SPEED_LOG

// Debug mode.
//#define DEBUG

#endif /* __MODE_H__ */
