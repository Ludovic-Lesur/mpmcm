/*
 * error_base.h
 *
 *  Created on: 27 feb. 2022
 *      Author: Ludo
 */

#ifndef __ERROR_BASE_H__
#define __ERROR_BASE_H__

// Peripherals.
#include "iwdg.h"
#include "lptim.h"
#include "rcc.h"
#include "rtc.h"
// MAC.
#include "lmac.h"
// Utils.
#include "error.h"
// Components.
#include "led.h"
#include "tic.h"
// Middleware.
#include "analog.h"
#include "cli.h"
#include "measure.h"
#include "node.h"
#include "power.h"

/*** ERROR structures ***/

/*!******************************************************************
 * \enum ERROR_base_t
 * \brief Board error bases.
 *******************************************************************/
typedef enum {
    SUCCESS = 0,
    // Peripherals.
    ERROR_BASE_IWDG = ERROR_BASE_STEP,
    ERROR_BASE_LPTIM = (ERROR_BASE_IWDG + IWDG_ERROR_BASE_LAST),
    ERROR_BASE_RCC = (ERROR_BASE_LPTIM + LPTIM_ERROR_BASE_LAST),
    ERROR_BASE_RTC = (ERROR_BASE_RCC + RCC_ERROR_BASE_LAST),
    // MAC.
    ERROR_BASE_LMAC = (ERROR_BASE_RTC + RTC_ERROR_BASE_LAST),
    // Components.
    ERROR_BASE_LED = (ERROR_BASE_LMAC + LMAC_ERROR_BASE_LAST),
    ERROR_BASE_TIC = (ERROR_BASE_LED + LED_ERROR_BASE_LAST),
    // Middleware.
    ERROR_BASE_ANALOG = (ERROR_BASE_TIC + TIC_ERROR_BASE_LAST),
    ERROR_BASE_CLI = (ERROR_BASE_ANALOG + ANALOG_ERROR_BASE_LAST),
    ERROR_BASE_MEASURE = (ERROR_BASE_CLI + CLI_ERROR_BASE_LAST),
    ERROR_BASE_NODE = (ERROR_BASE_MEASURE + MEASURE_ERROR_BASE_LAST),
    ERROR_BASE_POWER = (ERROR_BASE_NODE + NODE_ERROR_BASE_LAST),
    // Last base value.
    ERROR_BASE_LAST = (ERROR_BASE_POWER + POWER_ERROR_BASE_LAST),
} ERROR_base_t;

#endif /* __ERROR_BASE_H__ */
