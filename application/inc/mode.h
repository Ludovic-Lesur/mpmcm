/*
 * mode.h
 *
 *  Created on: 21 mar. 2023
 *      Author: Ludo
 */

#ifndef __MODE_H__
#define __MODE_H__

/*** Board modes ***/

//#define DEBUG
//#define NVM_FACTORY_RESET
//#define ANALOG_SIMULATION

/*** Board options ***/

#ifdef NVM_FACTORY_RESET
#define NODE_ADDRESS                0x1C
#endif

#define ANALOG_MEASURE_ENABLE
//#define LINKY_TIC_ENABLE

// Linky TIC mode.
#define LINKY_TIC_MODE_HISTORIC
//#define LINKY_TIC_MODE_STANDARD

// Transformer selection.
//#define BLOCK_VC_10_2_6
#define BLOCK_VB_2_1_6

// Transformer settings.
#ifdef BLOCK_VC_10_2_6
#define MPMCM_TRANSFORMER_ATTEN     11 // Unit V/V.
#ifdef NVM_FACTORY_RESET
#define MPMCM_TRANSFORMER_GAIN      300 // Unit (10 * V/V).
#endif
#endif
#ifdef BLOCK_VB_2_1_6
#define MPMCM_TRANSFORMER_ATTEN     15 // Unit V/V.
#ifdef NVM_FACTORY_RESET
#define MPMCM_TRANSFORMER_GAIN      236 // Unit (10 * V/V).
#endif
#endif

// Current sensors settings.
#define MPMCM_SCT013_ATTEN          { 1, 1, 1, 1 } // Unit V/V.
#ifdef NVM_FACTORY_RESET
#define MPMCM_SCT013_GAIN           { 50, 50, 100, 200 } // Unit (10 * A/V).
#endif

#endif /* __MODE_H__ */
