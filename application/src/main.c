/*
 * main.c
 *
 *  Created on: 6 mar. 2023
 *      Author: Ludo
 */

// Peripherals.
#include "adc.h"
#include "error.h"
#include "error_base.h"
#include "exti.h"
#include "fpu.h"
#include "gpio.h"
#include "iwdg.h"
#include "lptim.h"
#include "lpuart.h"
#include "mcu_mapping.h"
#include "nvic.h"
#include "nvic_priority.h"
#include "pwr.h"
#include "rcc.h"
#include "rtc.h"
#include "tim.h"
#include "types.h"
// Components.
#include "measure.h"
#include "tic.h"
// Middleware.
#include "cli.h"
#include "node.h"
// Applicative.
#include "mpmcm_flags.h"

#include "rcc_registers.h"

/*** MAIN local structures ***/

/*******************************************************************/
typedef struct {
    volatile uint8_t rtc_wakeup_timer_flag;
} MPMCM_context_t;

/*** MAIN local global variables ***/

static MPMCM_context_t mpmcm_ctx = {
    .rtc_wakeup_timer_flag = 0
};

/*** MAIN local functions ***/

/*******************************************************************/
static void _MPMCM_rtc_wakeup_timer_irq_callback(void) {
    mpmcm_ctx.rtc_wakeup_timer_flag = 1;
}

/*******************************************************************/
static void _MPMCM_init_hw(void) {
    // Local variables.
    RCC_status_t rcc_status = RCC_SUCCESS;
    RTC_status_t rtc_status = RTC_SUCCESS;
    LPTIM_status_t lptim_status = LPTIM_SUCCESS;
    MEASURE_status_t measure_status = MEASURE_SUCCESS;
    TIC_status_t tic_status = TIC_SUCCESS;
    LED_status_t led_status = LED_SUCCESS;
    NODE_status_t node_status = NODE_SUCCESS;
    CLI_status_t cli_status = CLI_SUCCESS;
#ifndef MPMCM_DEBUG
    IWDG_status_t iwdg_status = IWDG_SUCCESS;
#endif
    // Init error stack
    ERROR_stack_init();
    // Init memory.
    NVIC_init();
    // Enable FPU.
    FPU_init();
    // Init power module and clock tree.
    PWR_init();
    rcc_status = RCC_init(NVIC_PRIORITY_CLOCK);
    RCC_stack_error(ERROR_BASE_RCC);
    // Init GPIOs.
    GPIO_init();
    EXTI_init();
#ifndef MPMCM_DEBUG
    // Start independent watchdog.
    iwdg_status = IWDG_init();
    IWDG_stack_error(ERROR_BASE_IWDG);
    IWDG_reload();
#endif
    // Calibrate clocks.
    rcc_status = RCC_calibrate_internal_clocks(NVIC_PRIORITY_CLOCK_CALIBRATION);
    RCC_stack_error(ERROR_BASE_RCC);
    // Init RTC.
    rtc_status = RTC_init(&_MPMCM_rtc_wakeup_timer_irq_callback, NVIC_PRIORITY_RTC);
    RTC_stack_error(ERROR_BASE_RTC);
    // Init delay timer.
    lptim_status = LPTIM_init(NVIC_PRIORITY_DELAY);
    LPTIM_stack_error(ERROR_BASE_LPTIM);
    // Init components.
    POWER_init();
    led_status = LED_init();
    LED_stack_error(ERROR_BASE_LED);
    measure_status = MEASURE_init();
    MEASURE_stack_error(ERROR_BASE_MEASURE);
    tic_status = TIC_init();
    TIC_stack_error(ERROR_BASE_TIC);
    // Init node.
    node_status = NODE_init();
    NODE_stack_error(ERROR_BASE_NODE);
    cli_status = CLI_init();
    CLI_stack_error(ERROR_BASE_CLI);
}

/*** MAIN function ***/

/*******************************************************************/
int main(void) {
    // Init board.
    _MPMCM_init_hw();
    // Local variables.
#ifdef MPMCM_ANALOG_MEASURE_ENABLE
    MEASURE_status_t measure_status = MEASURE_SUCCESS;
#endif
    NODE_status_t node_status = NODE_SUCCESS;
    CLI_status_t cli_status = CLI_SUCCESS;
#ifdef MPMCM_LINKY_TIC_ENABLE
    TIC_status_t tic_status = TIC_SUCCESS;
#endif
    // Main loop.
    while (1) {
        // Enter sleep mode.
        IWDG_reload();
#ifndef MPMCM_DEBUG
        // Check modules state.
        if ((TIC_get_state() == TIC_STATE_OFF) && (MEASURE_get_state() == MEASURE_STATE_OFF) && (LED_get_state() == LED_STATE_OFF)) {
            PWR_enter_deepsleep_mode(PWR_DEEPSLEEP_MODE_STOP_1);
        }
        else {
            PWR_enter_sleep_mode(PWR_SLEEP_MODE_NORMAL);
        }
        IWDG_reload();
#endif
        // Check RTC flag.
        if (mpmcm_ctx.rtc_wakeup_timer_flag != 0) {
            // Clear flag.
            mpmcm_ctx.rtc_wakeup_timer_flag = 0;
#ifdef MPMCM_ANALOG_MEASURE_ENABLE
            // Call measure tick.
            measure_status = MEASURE_tick_second();
            MEASURE_stack_error(ERROR_BASE_MEASURE);
#endif
#ifdef MPMCM_LINKY_TIC_ENABLE
            // Call TIC interface tick.
            TIC_tick_second();
#endif
        }
#ifdef MPMCM_LINKY_TIC_ENABLE
        // Process TIC interface.
        tic_status = TIC_process();
        TIC_stack_error(ERROR_BASE_TIC);
#endif
        // Perform node tasks.
        node_status = NODE_process();
        NODE_stack_error(ERROR_BASE_NODE);
        // Perform command task.
        cli_status = CLI_process();
        CLI_stack_error(ERROR_BASE_CLI);
    }
    return 0;
}
