/*
 * measure.h
 *
 *  Created on: 18 jun. 2023
 *      Author: Ludo
 */

#ifndef __MEASURE_H__
#define __MEASURE_H__

#include "adc.h"
#include "data.h"
#include "dma.h"
#include "led.h"
#include "maths.h"
#include "mpmcm_flags.h"
#include "power.h"
#include "rcc.h"
#include "tim.h"
#include "types.h"

/*** MEASURE macros ***/

#define MEASURE_MAINS_PERIOD_US             20000

#define MEASURE_NUMBER_OF_ACI_CHANNELS      4
#define MEASURE_ACV_ACI_SAMPLING_PERIOD_US  200

#define MEASURE_PERIOD_BUFFER_SIZE          (MEASURE_MAINS_PERIOD_US / MEASURE_ACV_ACI_SAMPLING_PERIOD_US)

/*** MEASURE global variables ***/

extern const uint8_t MEASURE_SCT013_ATTEN[MEASURE_NUMBER_OF_ACI_CHANNELS];

/*** MEASURE structures ***/

/*!******************************************************************
 * \enum MEASURE_status_t
 * \brief MEASURE driver error codes.
 *******************************************************************/
typedef enum {
    // Driver errors.
    MEASURE_SUCCESS = 0,
    MEASURE_ERROR_NULL_PARAMETER,
    MEASURE_ERROR_STATE,
    MEASURE_ERROR_ZERO_CROSS_TIMEOUT,
    MEASURE_ERROR_DATA_TYPE,
    MEASURE_ERROR_AC_CHANNEL,
    // Low level drivers errors.
    MEASURE_ERROR_BASE_ADC = 0x0100,
    MEASURE_ERROR_BASE_DMA = (MEASURE_ERROR_BASE_ADC + ADC_ERROR_BASE_LAST),
    MEASURE_ERROR_BASE_TIM = (MEASURE_ERROR_BASE_DMA + DMA_ERROR_BASE_LAST),
    MEASURE_ERROR_BASE_RCC = (MEASURE_ERROR_BASE_TIM + TIM_ERROR_BASE_LAST),
    MEASURE_ERROR_BASE_MATH = (MEASURE_ERROR_BASE_RCC + RCC_ERROR_BASE_LAST),
    MEASURE_ERROR_BASE_POWER = (MEASURE_ERROR_BASE_MATH + MATH_ERROR_BASE_LAST),
    MEASURE_ERROR_BASE_LED = (MEASURE_ERROR_BASE_POWER + POWER_ERROR_BASE_LAST),
    // Last base value.
    MEASURE_ERROR_BASE_LAST = (MEASURE_ERROR_BASE_LED + LED_ERROR_BASE_LAST)
} MEASURE_status_t;

/*!******************************************************************
 * \enum MEASURE_state_t
 * \brief MEASURE states list.
 *******************************************************************/
typedef enum {
    MEASURE_STATE_OFF = 0,
    MEASURE_STATE_ACTIVE,
    MEASURE_STATE_LAST
} MEASURE_state_t;

/*!******************************************************************
 * \enum MEASURE_data_index_t
 * \brief MEASURE single data list.
 *******************************************************************/
typedef enum {
    MEASURE_DATA_INDEX_MAINS_FREQUENCY_MHZ = 0,
    MEASURE_DATA_INDEX_LAST
} MEASURE_data_index_t;

/*** MEASURE functions ***/

/*!******************************************************************
 * \fn MEASURE_status_t MEASURE_init(void)
 * \brief Init MEASURE driver.
 * \param[in]   none
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
MEASURE_status_t MEASURE_init(void);

/*!******************************************************************
 * \fn MEASURE_state_t MEASURE_get_state(void)
 * \brief Read MEASURE driver state.
 * \param[in]   none
 * \param[out]  none
 * \retval      MEASURE driver state.
 *******************************************************************/
MEASURE_state_t MEASURE_get_state(void);

/*!******************************************************************
 * \fn void MEASURE_set_gains(uint16_t transformer_gain, uint16_t current_sensors_gain[MEASURE_NUMBER_OF_ACI_CHANNELS])
 * \brief Set ACV and ACI measurements gains.
 * \param[in]   transformer_gain: Transformer gain in (10 * V/V).
 * \param[in]   current_sensors_gain: Current sensors gain table in (10 * A/V).
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
MEASURE_status_t MEASURE_set_gains(uint16_t transformer_gain, uint16_t current_sensors_gain[MEASURE_NUMBER_OF_ACI_CHANNELS]);

#ifdef MPMCM_ANALOG_MEASURE_ENABLE
/*!******************************************************************
 * \fn MEASURE_status_t MEASURE_tick_second(void)
 * \brief Function to call every second.
 * \param[in]   none
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
MEASURE_status_t MEASURE_tick_second(void);
#endif

/*!******************************************************************
 * \fn MEASURE_status_t MEASURE_get_probe_detect_flag(uint8_t channel_index, uint8_t* current_probe_connected)
 * \brief Get AC channel detect flag.
 * \param[in]   channel_index: AC channel index to read.
 * \param[out]  current_probe_connected: Pointer to the current probe detection flag.
 * \retval      Function execution status.
 *******************************************************************/
MEASURE_status_t MEASURE_get_probe_detect_flag(uint8_t channel_index, uint8_t* current_probe_connected);

/*!******************************************************************
 * \fn MEASURE_status_t MEASURE_get_mains_detect_flag(uint8_t mains_voltage_detected)
 * \brief Get mains voltage detect flag.
 * \param[in]   none
 * \param[out]  mains_voltage_detected: Pointer to the mains voltage detection flag.
 * \retval      Function execution status.
 *******************************************************************/
MEASURE_status_t MEASURE_get_mains_detect_flag(uint8_t* mains_voltage_detected);

/*!******************************************************************
 * \fn MEASURE_status_t MEASURE_get_run_data(MEASURE_data_index_t data_type, DATA_run_t* run_data)
 * \brief Get run data.
 * \param[in]   data_index: Data to read.
 * \param[out]  run_data: Pointer to the run data.
 * \retval      Function execution status.
 *******************************************************************/
MEASURE_status_t MEASURE_get_run_data(MEASURE_data_index_t data_index, DATA_run_t* run_data);

/*!******************************************************************
 * \fn MEASURE_status_t MEASURE_get_accumulated_data(MEASURE_data_index_t data_type, DATA_accumulated_t* accumulated_data)
 * \brief Get accumulated data.
 * \param[in]   data_index: Data to read.
 * \param[out]  accumulated_data: Pointer to the accumulated data.
 * \retval      Function execution status.
 *******************************************************************/
MEASURE_status_t MEASURE_get_accumulated_data(MEASURE_data_index_t data_index, DATA_accumulated_t* accumulated_data);

/*!******************************************************************
 * \fn MEASURE_status_t MEASURE_get_channel_run_data(uint8_t channel, DATA_run_channel_t* channel_run_data)
 * \brief Get AC channel run data.
 * \param[in]   channel_index: AC channel index to read.
 * \param[out]  channel_data: Pointer to the channel results.
 * \retval      Function execution status.
 *******************************************************************/
MEASURE_status_t MEASURE_get_channel_run_data(uint8_t channel, DATA_run_channel_t* channel_run_data);

/*!******************************************************************
 * \fn MEASURE_status_t MEASURE_get_channel_accumulated_data(uint8_t channel, DATA_accumulated_channel_t* channel_accumulated_data)
 * \brief Get AC channel accumulated data.
 * \param[in]   channel_index: AC channel index to read.
 * \param[out]  channel_data: Pointer to the channel results.
 * \retval      Function execution status.
 *******************************************************************/
MEASURE_status_t MEASURE_get_channel_accumulated_data(uint8_t channel, DATA_accumulated_channel_t* channel_accumulated_data);

/*******************************************************************/
#define MEASURE_exit_error(base) { ERROR_check_exit(measure_status, MEASURE_SUCCESS, base) }

/*******************************************************************/
#define MEASURE_stack_error(base) { ERROR_check_stack(measure_status, MEASURE_SUCCESS, base) }

/*******************************************************************/
#define MEASURE_stack_exit_error(base, code) { ERROR_check_stack_exit(measure_status, MEASURE_SUCCESS, base, code) }

#endif /* __MEASURE_H__ */
