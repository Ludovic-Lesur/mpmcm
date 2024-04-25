/*
 * fpu.c
 *
 *  Created on: 25 apr. 2024
 *      Author: Ludo
 */

#include "fpu.h"

#include "fpu_reg.h"

/*** FPU functions ***/

/*******************************************************************/
void FPU_init(void) {
	// Enable FPU access for all cores.
	FPU -> CPACR |= (0b11 << 22) | (0b11 << 20);
}
