/*
 * fault.c
 *
 *  Created on: 03 sep. 2023
 *      Author: Ludo
 */

#include "pwr.h"

/*******************************************************************/
void __attribute__((optimize("-O0"))) NMI_Handler(void) {
	// Trigger software reset.
	PWR_software_reset();
}

/*******************************************************************/
void __attribute__((optimize("-O0"))) HardFault_Handler(void) {
	// Trigger software reset.
	PWR_software_reset();
}

/*******************************************************************/
void __attribute__((optimize("-O0"))) MemManage_Handler(void) {
	// Trigger software reset.
	PWR_software_reset();
}

/*******************************************************************/
void __attribute__((optimize("-O0"))) BusFault_Handler(void) {
	// Trigger software reset.
	PWR_software_reset();
}
