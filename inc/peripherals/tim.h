/*
 * tim.h
 *
 *  Created on: 17 jun. 2023
 *      Author: Ludo
 */

#ifndef __TIM_H__
#define __TIM_H__

/*** TIM structures ***/

typedef enum {
	TIM_SUCCESS = 0,
	TIM_ERROR_BASE_LAST = 0x0100
} TIM_status_t;

/*** TIM functions ***/

void TIM6_init(void);
void TIM6_start(void);
void TIM6_stop(void);

#endif /* __TIM_H__ */
