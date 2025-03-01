/*
 * analog.c
 *
 *  Created on: 20 feb. 2025
 *      Author: Ludo
 */

#include "analog.h"

#include "adc.h"
#include "error.h"
#include "gpio_mapping.h"
#include "mode.h"
#include "types.h"

/*** ANALOG local macros ***/

#define ANALOG_ADC_INSTANCE                 ADC_INSTANCE_ADC1

#define ANALOG_VMCU_MV_DEFAULT              3000
#define ANALOG_TMCU_DEGREES_DEFAULT         25

#define ANALOG_ERROR_VALUE                  0xFFFF

/*** ANALOG local structures ***/

#ifndef ANALOG_MEASURE_ENABLE
/*******************************************************************/
typedef struct {
    int32_t vmcu_mv;
} ANALOG_context_t;
#endif

/*** ANALOG local global variables ***/

#ifndef ANALOG_MEASURE_ENABLE
static ANALOG_context_t analog_ctx = { .vmcu_mv = ANALOG_VMCU_MV_DEFAULT };
#endif

/*** ANALOG functions ***/

/*******************************************************************/
ANALOG_status_t ANALOG_init(void) {
    // Local variables.
    ANALOG_status_t status = ANALOG_SUCCESS;
#ifndef ANALOG_MEASURE_ENABLE
    ADC_status_t adc_status = ADC_SUCCESS;
    ADC_SGL_configuration_t adc_config;
    // Init context.
    analog_ctx.vmcu_mv = ANALOG_VMCU_MV_DEFAULT;
    // Init internal ADC.
    adc_config.clock = ADC_CLOCK_SYSCLK;
    adc_config.clock_prescaler = ADC_CLOCK_PRESCALER_NONE;
    adc_status = ADC_SGL_init(ANALOG_ADC_INSTANCE, NULL, &adc_config);
    ADC_exit_error(ANALOG_ERROR_BASE_ADC);
errors:
#endif
    return status;
}

/*******************************************************************/
ANALOG_status_t ANALOG_de_init(void) {
    // Local variables.
    ANALOG_status_t status = ANALOG_SUCCESS;
#ifndef ANALOG_MEASURE_ENABLE
    ADC_status_t adc_status = ADC_SUCCESS;
    // Release internal ADC.
    adc_status = ADC_SGL_de_init(ANALOG_ADC_INSTANCE);
    ADC_exit_error(ANALOG_ERROR_BASE_ADC);
errors:
#endif
    return status;
}

/*******************************************************************/
ANALOG_status_t ANALOG_convert_channel(ANALOG_channel_t channel, int32_t* analog_data) {
    // Local variables.
    ANALOG_status_t status = ANALOG_SUCCESS;
#ifndef ANALOG_MEASURE_ENABLE
    ADC_status_t adc_status = ADC_SUCCESS;
    int32_t adc_data_12bits = 0;
#endif
    // Check parameter.
    if (analog_data == NULL) {
        status = ANALOG_ERROR_NULL_PARAMETER;
        goto errors;
    }
    // Check channel.
    switch (channel) {
    case ANALOG_CHANNEL_VMCU_MV:
        // MCU voltage.
#ifndef ANALOG_MEASURE_ENABLE
        adc_status = ADC_SGL_convert_channel(ANALOG_ADC_INSTANCE, ADC_CHANNEL_VBAT, &adc_data_12bits);
        ADC_exit_error(ANALOG_ERROR_BASE_ADC);
        // Convert to mV.
        adc_status = ADC_compute_vmcu(adc_data_12bits, analog_data);
        ADC_exit_error(ANALOG_ERROR_BASE_ADC);
        // Update local value for temperature computation.
        analog_ctx.vmcu_mv = (*analog_data);
#else
        (*analog_data) = ANALOG_VMCU_MV_DEFAULT;
#endif
        break;
    case ANALOG_CHANNEL_TMCU_DEGREES:
        // MCU temperature.
#ifndef ANALOG_MEASURE_ENABLE
        adc_status = ADC_SGL_convert_channel(ANALOG_ADC_INSTANCE, ADC_CHANNEL_TEMPERATURE_SENSOR, &adc_data_12bits);
        ADC_exit_error(ANALOG_ERROR_BASE_ADC);
        // Convert to degrees.
        adc_status = ADC_compute_tmcu(adc_data_12bits, analog_data);
        ADC_exit_error(ANALOG_ERROR_BASE_ADC);
#else
        (*analog_data) = ANALOG_TMCU_DEGREES_DEFAULT;
#endif
        break;
    default:
        status = ANALOG_ERROR_CHANNEL;
        goto errors;
    }
errors:
    return status;
}
