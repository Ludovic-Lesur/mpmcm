/*
 * mpmcm_flags.h
 *
 *  Created on: 21 mar. 2023
 *      Author: Ludo
 */

#ifndef __MPMCM_FLAGS_H__
#define __MPMCM_FLAGS_H__

/*** Board modes ***/

//#define MPMCM_DEBUG
//#define MPMCM_NVM_FACTORY_RESET
//#define MPMCM_ANALOG_SIMULATION

/*** Board options ***/

#ifdef MPMCM_NVM_FACTORY_RESET
#define MPMCM_NODE_ADDRESS                0x1C
#endif

#define MPMCM_ANALOG_MEASURE_ENABLE
//#define MPMCM_LINKY_TIC_ENABLE

// Linky TIC mode.
#define MPMCM_LINKY_TIC_MODE_HISTORIC
//#define MPMCM_LINKY_TIC_MODE_STANDARD

// Transformer selection.
//#define MPMCM_TRANSFORMER_BLOCK_VC_10_2_6
#define MPMCM_TRANSFORMER_BLOCK_VB_2_1_6

// Transformer settings.
#ifdef MPMCM_TRANSFORMER_BLOCK_VC_10_2_6
#define MPMCM_TRANSFORMER_ATTEN     11 // Unit V/V.
#ifdef MPMCM_NVM_FACTORY_RESET
#define MPMCM_TRANSFORMER_GAIN      300 // Unit (10 * V/V).
#endif
#endif
#ifdef MPMCM_TRANSFORMER_BLOCK_VB_2_1_6
#define MPMCM_TRANSFORMER_ATTEN     15 // Unit V/V.
#ifdef MPMCM_NVM_FACTORY_RESET
#define MPMCM_TRANSFORMER_GAIN      236 // Unit (10 * V/V).
#endif
#endif

// Current sensors settings.
#define MPMCM_SCT013_ATTEN          { 1, 1, 1, 1 } // Unit V/V.
#ifdef MPMCM_NVM_FACTORY_RESET
#define MPMCM_SCT013_GAIN           { 50, 50, 100, 200 } // Unit (10 * A/V).
#endif

#endif /* __MPMCM_FLAGS_H__ */
