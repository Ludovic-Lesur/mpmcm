/*
 * pwr.c
 *
 *  Created on: Jun 18, 2023
 *      Author: ludo
 */

#include "pwr.h"

#include "pwr_reg.h"
#include "rcc_reg.h"

/*** PWR functions ***/

/* INIT PWR INTERFACE.
 * @param:	None.
 * @return:	None.
 */
void PWR_init(void) {
	// Enable power interface clock.
	RCC -> APB1ENR1 |= (0b1 << 28); // PWREN='1'.
	// Unlock back-up registers (DBP bit).
	PWR -> CR1 |= (0b1 << 8);
}
