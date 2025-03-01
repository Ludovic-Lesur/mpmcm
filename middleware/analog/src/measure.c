/*
 * measure.c
 *
 *  Created on: 18 jun. 2023
 *      Author: Ludo
 */

#include "measure.h"

#ifndef STM32G4XX_DRIVERS_DISABLE_FLAGS_FILE
#include "stm32g4xx_drivers_flags.h"
#endif
#include "adc.h"
#include "data.h"
#include "dma.h"
#include "dma_channel.h"
#include "dmamux.h"
#include "dsp/basic_math_functions.h"
#include "dsp/statistics_functions.h"
#include "error.h"
#include "error_base.h"
#include "exti.h"
#include "gpio.h"
#include "gpio_mapping.h"
#include "led.h"
#include "maths.h"
#include "mpmcm_flags.h"
#include "nvic.h"
#include "nvic_priority.h"
#include "power.h"
#include "rcc.h"
#include "simulation.h"
#include "tim.h"
#include "types.h"

/*** MEASURE local macros ***/

#define MEASURE_ACV_ACI_ADC_INSTANCE                    ADC_INSTANCE_ADC1
#define MEASURE_ACV_ACI_SAMPLING_FREQUENCY_HZ           (MATH_POWER_10[6] / MEASURE_ACV_ACI_SAMPLING_PERIOD_US)
#define MEASURE_ACV_ACI_TIM_INSTANCE                    TIM_INSTANCE_TIM6

#define MEASURE_ADC_OFFSET_12_BITS                      ((ADC_FULL_SCALE >> 1) + 1)

#define MEASURE_ACV_FREQUENCY_TIM_INSTANCE              TIM_INSTANCE_TIM2
#define MEASURE_ACV_FREQUENCY_SAMPLING_HZ               1000000

#define MEASURE_ZERO_CROSS_PER_PERIOD                   2

// Wait for 1 second of mains voltage presence before sampling.
#define MEASURE_ZERO_CROSS_START_THRESHOLD              ((MATH_POWER_10[6] * MEASURE_ZERO_CROSS_PER_PERIOD) / (MEASURE_MAINS_PERIOD_US))

#define MEASURE_TRANSFORMER_GAIN_FACTOR                 10 // For (10 * V/V) input unit.
#define MEASURE_CURRENT_SENSOR_GAIN_FACTOR              10 // For (10 * A/V) input unit.

// Note: this factor is used to add a margin to the buffer length (more than 1 mains periods long).
// Buffer switch then is triggered by zero cross detection instead of a fixed number of samples.
#define MEASURE_PERIOD_PER_BUFFER                       2
#define MEASURE_PERIOD_ADCX_BUFFER_SIZE                 (MEASURE_PERIOD_PER_BUFFER * MEASURE_PERIOD_BUFFER_SIZE)
#define MEASURE_PERIOD_ADCX_DMA_BUFFER_SIZE             (MEASURE_NUMBER_OF_ACI_CHANNELS * MEASURE_PERIOD_ADCX_BUFFER_SIZE)
#define MEASURE_PERIOD_ADCX_DMA_BUFFER_DEPTH            2
#define MEASURE_PERIOD_TIMX_DMA_BUFFER_SIZE             3

// Wait for 1 second of sampling before computing run and accumulated data.
#define MEASURE_SAMPLED_PERIOD_START_THRESHOLD          (MATH_POWER_10[6] / MEASURE_MAINS_PERIOD_US)

// ADC buffers with size out of this range are not computed.
// Warning: it limits the acceptable input frequency range around 50Hz.
#define MEASURE_PERIOD_ADCX_BUFFER_SIZE_ERROR_PERCENT   20

#define MEASURE_POWER_FACTOR_MULTIPLIER                 100

#define MEASURE_LED_PULSE_DURATION_MS                   50
#define MEASURE_LED_PULSE_PERIOD_SECONDS                5

#define MEASURE_SIMULATION_TIM_INSTANCE                 TIM_INSTANCE_TIM15

/*** MEASURE local structures ***/

/*******************************************************************/
typedef struct {
    int16_t data[MEASURE_PERIOD_ADCX_DMA_BUFFER_SIZE];
    uint16_t size;
} MEASURE_buffer_t;

/*******************************************************************/
typedef struct {
    // Raw buffers filled by ADC and DMA for 1 period.
    MEASURE_buffer_t acv[MEASURE_PERIOD_ADCX_DMA_BUFFER_DEPTH];
    uint8_t acv_read_idx;
    uint8_t acv_write_idx;
    MEASURE_buffer_t aci[MEASURE_PERIOD_ADCX_DMA_BUFFER_DEPTH];
    uint8_t aci_read_idx;
    uint8_t aci_write_idx;
    // Raw buffer filled by timer and DMA.
    uint32_t acv_frequency_capture[MEASURE_PERIOD_TIMX_DMA_BUFFER_SIZE];
} MEASURE_sampling_t;

/*******************************************************************/
typedef struct {
    // Factors.
    float64_t acv_factor_num;
    float64_t acv_factor_den;
    float64_t aci_factor_num[MEASURE_NUMBER_OF_ACI_CHANNELS];
    float64_t aci_factor_den;
    float64_t acp_factor_num[MEASURE_NUMBER_OF_ACI_CHANNELS];
    float64_t acp_factor_den;
    // Temporary variables for individual channel processing on 1 period.
    float32_t period_acvx_buffer_f32[MEASURE_PERIOD_ADCX_BUFFER_SIZE];
    float32_t period_acix_buffer_f32[MEASURE_PERIOD_ADCX_BUFFER_SIZE];
    float32_t period_acpx_buffer_f32[MEASURE_PERIOD_ADCX_BUFFER_SIZE];
    uint32_t period_acxx_buffer_size;
    uint32_t period_acxx_buffer_size_low_limit;
    uint32_t period_acxx_buffer_size_high_limit;
    float32_t period_active_power_f32;
    float32_t period_rms_voltage_f32;
    float32_t period_rms_current_f32;
    float32_t period_apparent_power_f32;
    float32_t period_power_factor_f32;
    // AC channels results.
    DATA_run_channel_t chx_rolling_mean[MEASURE_NUMBER_OF_ACI_CHANNELS];
    DATA_run_channel_t chx_run_data[MEASURE_NUMBER_OF_ACI_CHANNELS];
    DATA_accumulated_channel_t chx_accumulated_data[MEASURE_NUMBER_OF_ACI_CHANNELS];
    DATA_run_t active_energy_mws_sum[MEASURE_NUMBER_OF_ACI_CHANNELS];
    DATA_run_t apparent_energy_mvas_sum[MEASURE_NUMBER_OF_ACI_CHANNELS];
    // Mains frequency.
    DATA_run_t acv_frequency_rolling_mean;
    DATA_run_t acv_frequency_run_data;
    DATA_accumulated_t acv_frequency_accumulated_data;
} MEASURE_data_t;

/*******************************************************************/
typedef struct {
    MEASURE_state_t state;
    uint8_t processing_enable;
    uint32_t zero_cross_count;
    uint8_t dma_transfer_end_flag;
    uint32_t sampled_period_count;
    uint8_t period_compute_enable;
    uint32_t tick_led_seconds_count;
#ifdef MPMCM_ANALOG_SIMULATION
    uint8_t random_divider;
#endif
} MEASURE_context_t;

/*** MEASURE global variables ***/

const uint8_t MEASURE_SCT013_ATTEN[MEASURE_NUMBER_OF_ACI_CHANNELS] = MPMCM_SCT013_ATTEN;

/*** MEASURE local global variables ***/

#ifdef MPMCM_ANALOG_MEASURE_ENABLE
static const ADC_channel_configuration_t MEASURE_ADC_MASTER_SEQUENCE[MEASURE_NUMBER_OF_ACI_CHANNELS] = {
    { GPIO_ADC_CHANNEL_ACV_SAMPLING, 0 },
    { GPIO_ADC_CHANNEL_ACV_SAMPLING, 0 },
    { GPIO_ADC_CHANNEL_ACV_SAMPLING, 0 },
    { GPIO_ADC_CHANNEL_ACV_SAMPLING, 0 }
};

static const ADC_channel_configuration_t MEASURE_ADC_SLAVE_SEQUENCE[MEASURE_NUMBER_OF_ACI_CHANNELS] = {
    { GPIO_ADC_CHANNEL_ACI1_SAMPLING, MEASURE_ADC_OFFSET_12_BITS },
    { GPIO_ADC_CHANNEL_ACI2_SAMPLING, MEASURE_ADC_OFFSET_12_BITS },
    { GPIO_ADC_CHANNEL_ACI3_SAMPLING, MEASURE_ADC_OFFSET_12_BITS },
    { GPIO_ADC_CHANNEL_ACI4_SAMPLING, MEASURE_ADC_OFFSET_12_BITS }
};

static const GPIO_pin_t* MEASURE_GPIO_ACI_DETECT[MEASURE_NUMBER_OF_ACI_CHANNELS] = {
    &GPIO_ACI1_DETECT,
    &GPIO_ACI2_DETECT,
    &GPIO_ACI3_DETECT,
    &GPIO_ACI4_DETECT
};
#endif

static volatile MEASURE_sampling_t measure_sampling;
static volatile MEASURE_data_t measure_data __attribute__((section(".bss_ccmsram")));
static volatile MEASURE_context_t measure_ctx;

/*** MEASURE local functions ***/

/*******************************************************************/
static void _MEASURE_reset(void) {
    // Local variables.
    uint8_t chx_idx = 0;
    uint32_t idx0 = 0;
    uint32_t idx1 = 0;
    // Reset indexes.
    measure_sampling.acv_write_idx = 0;
    measure_sampling.acv_read_idx = 0;
    measure_sampling.aci_write_idx = 0;
    measure_sampling.aci_read_idx = 0;
    // Reset flags.
    measure_ctx.processing_enable = 0;
    measure_ctx.zero_cross_count = 0;
    measure_ctx.dma_transfer_end_flag = 0;
    measure_ctx.sampled_period_count = 0;
    measure_ctx.period_compute_enable = 0;
    measure_ctx.tick_led_seconds_count = 0;
#ifdef MPMCM_ANALOG_SIMULATION
    measure_ctx.random_divider = 1;
#endif
    // Reset sampling buffers.
    for (idx0 = 0; idx0 < MEASURE_PERIOD_ADCX_DMA_BUFFER_DEPTH; idx0++) {
        for (idx1 = 0; idx1 < MEASURE_PERIOD_ADCX_DMA_BUFFER_SIZE; idx1++) {
            measure_sampling.acv[idx0].data[idx1] = 0;
            measure_sampling.aci[idx0].data[idx1] = 0;
        }
    }
    // Reset channels data.
    for (chx_idx = 0; chx_idx < MEASURE_NUMBER_OF_ACI_CHANNELS; chx_idx++) {
        // Clear all data.
        DATA_reset_run_channel(measure_data.chx_rolling_mean[chx_idx]);
        DATA_reset_run_channel(measure_data.chx_run_data[chx_idx]);
        DATA_reset_accumulated_channel(measure_data.chx_accumulated_data[chx_idx]);
        DATA_reset_run(measure_data.active_energy_mws_sum[chx_idx]);
        DATA_reset_run(measure_data.apparent_energy_mvas_sum[chx_idx]);
    }
    // Reset frequency data.
    DATA_reset_run(measure_data.acv_frequency_rolling_mean);
    DATA_reset_run(measure_data.acv_frequency_run_data);
    DATA_reset_accumulated(measure_data.acv_frequency_accumulated_data);
    // Reset sampling buffers.
    for (idx1 = 0; idx1 < MEASURE_PERIOD_TIMX_DMA_BUFFER_SIZE; idx1++) {
        measure_sampling.acv_frequency_capture[idx1] = 0;
    }
}

#ifdef MPMCM_ANALOG_MEASURE_ENABLE
/*******************************************************************/
static MEASURE_status_t _MEASURE_start_analog_transfer(void) {
    // Local variables.
    MEASURE_status_t status = MEASURE_SUCCESS;
    ADC_status_t adc_status = ADC_SUCCESS;
    DMA_status_t dma_status = DMA_SUCCESS;
    TIM_status_t tim_status = TIM_SUCCESS;
    // Start ADC.
    adc_status = ADC_SQC_start(MEASURE_ACV_ACI_ADC_INSTANCE);
    ADC_exit_error(MEASURE_ERROR_BASE_ADC);
    // Start DMA.
    dma_status = DMA_start(DMA_INSTANCE_ACV_SAMPLING, DMA_CHANNEL_ACV_SAMPLING);
    DMA_exit_error(MEASURE_ERROR_BASE_DMA);
    dma_status = DMA_start(DMA_INSTANCE_ACI_SAMPLING, DMA_CHANNEL_ACI_SAMPLING);
    DMA_exit_error(MEASURE_ERROR_BASE_DMA);
    // Start trigger.
    tim_status = TIM_STD_start(MEASURE_ACV_ACI_TIM_INSTANCE, RCC_CLOCK_SYSTEM, MEASURE_ACV_ACI_SAMPLING_PERIOD_US, TIM_UNIT_US, NULL);
    TIM_exit_error(MEASURE_ERROR_BASE_TIM);
errors:
    return status;
}
#endif

#ifdef MPMCM_ANALOG_MEASURE_ENABLE
/*******************************************************************/
static MEASURE_status_t _MEASURE_stop_analog_transfer(void) {
    // Local variables.
    MEASURE_status_t status = MEASURE_SUCCESS;
    ADC_status_t adc_status = ADC_SUCCESS;
    DMA_status_t dma_status = DMA_SUCCESS;
    TIM_status_t tim_status = TIM_SUCCESS;
    // Stop trigger.
    tim_status = TIM_STD_stop(MEASURE_ACV_ACI_TIM_INSTANCE);
    TIM_stack_error(ERROR_BASE_MEASURE + MEASURE_ERROR_BASE_TIM);
    // Stop DMA.
    dma_status = DMA_stop(DMA_INSTANCE_ACV_SAMPLING, DMA_CHANNEL_ACV_SAMPLING);
    DMA_stack_error(ERROR_BASE_MEASURE + MEASURE_ERROR_BASE_DMA);
    dma_status = DMA_stop(DMA_INSTANCE_ACI_SAMPLING, DMA_CHANNEL_ACI_SAMPLING);
    DMA_stack_error(ERROR_BASE_MEASURE + MEASURE_ERROR_BASE_DMA);
    // Stop ADC.
    adc_status = ADC_SQC_stop(MEASURE_ACV_ACI_ADC_INSTANCE);
    ADC_stack_error(ERROR_BASE_MEASURE + MEASURE_ERROR_BASE_ADC);
    return status;
}
#endif

#ifdef MPMCM_ANALOG_MEASURE_ENABLE
/*******************************************************************/
static MEASURE_status_t _MEASURE_start(void) {
    // Local variables.
    MEASURE_status_t status = MEASURE_SUCCESS;
    RCC_status_t rcc_status = RCC_SUCCESS;
    ADC_status_t adc_status = ADC_SUCCESS;
    DMA_status_t dma_status = DMA_SUCCESS;
    LED_status_t led_status = LED_SUCCESS;
    TIM_status_t tim_status = TIM_SUCCESS;
    RCC_pll_configuration_t pll_config;
    ADC_SQC_configuration_t adc_config;
    // Turn TCXO on.
    POWER_enable(POWER_REQUESTER_ID_MEASURE, POWER_DOMAIN_MCU_TCXO, LPTIM_DELAY_MODE_ACTIVE);
    // Switch to PLL (system clock 120MHz, ADC clock 8MHz).
    pll_config.source = RCC_CLOCK_HSE;
    pll_config.hse_mode = RCC_HSE_MODE_BYPASS;
    pll_config.m = 2;
    pll_config.n = 30;
    pll_config.r = RCC_PLL_RQ_2;
    pll_config.p = 30;
    pll_config.q = RCC_PLL_RQ_8;
    rcc_status = RCC_switch_to_pll(&pll_config);
    RCC_exit_error(MEASURE_ERROR_BASE_RCC);
    // Init ADC.
    adc_config.clock = ADC_CLOCK_PLL;
    adc_config.clock_prescaler = ADC_CLOCK_PRESCALER_NONE;
    adc_config.master_sequence = (ADC_channel_configuration_t*) MEASURE_ADC_MASTER_SEQUENCE;
    adc_config.slave_sequence = (ADC_channel_configuration_t*) MEASURE_ADC_SLAVE_SEQUENCE;
    adc_config.sequence_length = MEASURE_NUMBER_OF_ACI_CHANNELS;
    adc_config.sampling_frequency_hz = MEASURE_ACV_ACI_SAMPLING_FREQUENCY_HZ;
    adc_config.trigger = ADC_TRIGGER_TIM6_TRGO;
    adc_config.trigger_detection = ADC_TRIGGER_DETECTION_RISING_EDGE;
    adc_status = ADC_SQC_init(MEASURE_ACV_ACI_ADC_INSTANCE, &GPIO_ADC, &adc_config);
    ADC_exit_error(MEASURE_ERROR_BASE_ADC);
    // Init frequency measurement timer and ADC timer.
    tim_status = TIM_IC_init(MEASURE_ACV_FREQUENCY_TIM_INSTANCE, (TIM_gpio_t*) &GPIO_ACV_FREQUENCY_TIM);
    TIM_exit_error(MEASURE_ERROR_BASE_TIM);
    tim_status = TIM_STD_init(MEASURE_ACV_ACI_TIM_INSTANCE, NVIC_PRIORITY_ADC_TRIGGER);
    TIM_exit_error(MEASURE_ERROR_BASE_TIM);
    // Re-init LED to update clock frequency.
    led_status = LED_de_init();
    LED_exit_error(MEASURE_ERROR_BASE_LED);
    led_status = LED_init();
    LED_exit_error(MEASURE_ERROR_BASE_LED);
    // Start frequency measurement timer.
    tim_status = TIM_IC_start_channel(MEASURE_ACV_FREQUENCY_TIM_INSTANCE, GPIO_TIM_CHANNEL_ACV_FREQUENCY, MEASURE_ACV_FREQUENCY_SAMPLING_HZ, TIM_CAPTURE_PRESCALER_2);
    TIM_exit_error(MEASURE_ERROR_BASE_TIM);
    dma_status = DMA_start(DMA_INSTANCE_ACV_FREQUENCY, DMA_CHANNEL_ACV_FREQUENCY);
    DMA_exit_error(MEASURE_ERROR_BASE_DMA);
    // Start analog measurements.
    status = _MEASURE_start_analog_transfer();
    if (status != MEASURE_SUCCESS) goto errors;
    // Update flag.
    measure_ctx.processing_enable = 1;
errors:
    return status;
}
#endif

#ifdef MPMCM_ANALOG_MEASURE_ENABLE
/*******************************************************************/
static MEASURE_status_t _MEASURE_stop(void) {
    // Local variables.
    MEASURE_status_t status = MEASURE_SUCCESS;
    MEASURE_status_t measure_status = MEASURE_SUCCESS;
    RCC_status_t rcc_status = RCC_SUCCESS;
    ADC_status_t adc_status = ADC_SUCCESS;
    DMA_status_t dma_status = DMA_SUCCESS;
    TIM_status_t tim_status = TIM_SUCCESS;
    LED_status_t led_status = LED_SUCCESS;
    // Update flag.
    measure_ctx.processing_enable = 0;
    // Start analog measurements.
    measure_status = _MEASURE_stop_analog_transfer();
    MEASURE_stack_error(ERROR_BASE_MEASURE);
    // Stop frequency measurement timer.
    dma_status = DMA_stop(DMA_INSTANCE_ACV_FREQUENCY, DMA_CHANNEL_ACV_FREQUENCY);
    DMA_stack_error(ERROR_BASE_MEASURE + MEASURE_ERROR_BASE_DMA);
    tim_status = TIM_IC_stop_channel(MEASURE_ACV_FREQUENCY_TIM_INSTANCE, GPIO_TIM_CHANNEL_ACV_FREQUENCY);
    TIM_stack_error(ERROR_BASE_MEASURE + MEASURE_ERROR_BASE_TIM);
    // Release ADC.
    adc_status = ADC_SQC_de_init(MEASURE_ACV_ACI_ADC_INSTANCE);
    ADC_stack_error(ERROR_BASE_MEASURE + MEASURE_ERROR_BASE_ADC);
    // Release timers.
    tim_status = TIM_IC_de_init(MEASURE_ACV_FREQUENCY_TIM_INSTANCE, (TIM_gpio_t*) &GPIO_ACV_FREQUENCY_TIM);
    TIM_stack_error(ERROR_BASE_MEASURE + MEASURE_ERROR_BASE_TIM);
    tim_status = TIM_STD_de_init(MEASURE_ACV_ACI_TIM_INSTANCE);
    TIM_stack_error(ERROR_BASE_MEASURE + MEASURE_ERROR_BASE_TIM);
    // Switch to HSI.
    rcc_status = RCC_switch_to_hsi();
    RCC_stack_error(ERROR_BASE_MEASURE + MEASURE_ERROR_BASE_RCC);
    // Turn TCXO off.
    POWER_disable(POWER_REQUESTER_ID_MEASURE, POWER_DOMAIN_MCU_TCXO);
    // Re-init LED to update clock frequency.
    led_status = LED_de_init();
    LED_stack_error(ERROR_BASE_MEASURE + MEASURE_ERROR_BASE_LED);
    led_status = LED_init();
    LED_stack_error(ERROR_BASE_MEASURE + MEASURE_ERROR_BASE_LED);
    return status;
}
#endif

#ifdef MPMCM_ANALOG_MEASURE_ENABLE
/*******************************************************************/
static MEASURE_status_t _MEASURE_switch_dma_buffer(void) {
    // Local variables.
    MEASURE_status_t status = MEASURE_SUCCESS;
    DMA_status_t dma_status = DMA_SUCCESS;
    // Stop ADC and DMA.
    status = _MEASURE_stop_analog_transfer();
    if (status != MEASURE_SUCCESS) goto errors;
    // Retrieve number of transfered data.
    dma_status = DMA_get_number_of_transfered_data(DMA_INSTANCE_ACV_SAMPLING, DMA_CHANNEL_ACV_SAMPLING, (uint16_t*) &(measure_sampling.acv[measure_sampling.acv_write_idx].size));
    DMA_stack_error(ERROR_BASE_MEASURE + ERROR_BASE_DMA);
    dma_status = DMA_get_number_of_transfered_data(DMA_INSTANCE_ACI_SAMPLING, DMA_CHANNEL_ACI_SAMPLING, (uint16_t*) &(measure_sampling.aci[measure_sampling.aci_write_idx].size));
    DMA_stack_error(ERROR_BASE_MEASURE + ERROR_BASE_DMA);
    // Update write indexes.
    measure_sampling.acv_write_idx = ((measure_sampling.acv_write_idx + 1) % MEASURE_PERIOD_ADCX_DMA_BUFFER_DEPTH);
    measure_sampling.aci_write_idx = ((measure_sampling.aci_write_idx + 1) % MEASURE_PERIOD_ADCX_DMA_BUFFER_DEPTH);
    // Set new address.
    dma_status = DMA_set_memory_address(DMA_INSTANCE_ACV_SAMPLING, DMA_CHANNEL_ACV_SAMPLING, (uint32_t) &(measure_sampling.acv[measure_sampling.acv_write_idx].data), MEASURE_PERIOD_ADCX_DMA_BUFFER_SIZE);
    DMA_stack_error(ERROR_BASE_MEASURE + ERROR_BASE_DMA);
    dma_status = DMA_set_memory_address(DMA_INSTANCE_ACI_SAMPLING, DMA_CHANNEL_ACI_SAMPLING, (uint32_t) &(measure_sampling.aci[measure_sampling.aci_write_idx].data), MEASURE_PERIOD_ADCX_DMA_BUFFER_SIZE);
    DMA_stack_error(ERROR_BASE_MEASURE + ERROR_BASE_DMA);
    // Restart DMA.
    status = _MEASURE_start_analog_transfer();
errors:
    return status;
}
#endif

#ifdef MPMCM_ANALOG_MEASURE_ENABLE
/*******************************************************************/
static void _MEASURE_compute_period_data(void) {
    // Local variables.
    uint32_t acv_buffer_size = 0;
    uint32_t aci_buffer_size = 0;
    float32_t mean_voltage_f32 = 0.0;
    float32_t mean_current_f32 = 0.0;
    float64_t active_power_mw = 0.0;
    float64_t rms_voltage_mv = 0.0;
    float64_t rms_current_ma = 0.0;
    float64_t apparent_power_mva = 0.0;
    float64_t power_factor = 0.0;
    uint32_t acv_frequency_capture_delta = 0;
    float64_t frequency_mhz = 0.0;
    float64_t temp_f64 = 0.0;
    uint8_t chx_idx = 0;
    uint32_t sample_idx = 0;
    uint32_t idx = 0;
    // Check enable flag.
    if (measure_ctx.processing_enable == 0) goto errors;
    // Check compute flag.
    if (measure_ctx.period_compute_enable == 0) {
        // Increment sampled period count.
        measure_ctx.sampled_period_count++;
        // Enable period computing after the given number of sampled periods.
        if (measure_ctx.sampled_period_count >= MEASURE_SAMPLED_PERIOD_START_THRESHOLD) {
            measure_ctx.period_compute_enable = 1;
        }
        else {
            goto errors;
        }
    }
    // Get size.
#ifdef MPMCM_ANALOG_SIMULATION
    acv_buffer_size = (SIMULATION_BUFFER_SIZE / MEASURE_NUMBER_OF_ACI_CHANNELS);
    aci_buffer_size = (SIMULATION_BUFFER_SIZE / MEASURE_NUMBER_OF_ACI_CHANNELS);
#else
    acv_buffer_size = (uint32_t) ((measure_sampling.acv[measure_sampling.acv_read_idx].size) / (MEASURE_NUMBER_OF_ACI_CHANNELS));
    aci_buffer_size = (uint32_t) ((measure_sampling.aci[measure_sampling.acv_read_idx].size) / (MEASURE_NUMBER_OF_ACI_CHANNELS));
#endif
    // Take the minimum size between voltage and current.
    measure_data.period_acxx_buffer_size = (acv_buffer_size < aci_buffer_size) ? acv_buffer_size : aci_buffer_size;
    // Check size.
    if ((measure_data.period_acxx_buffer_size < measure_data.period_acxx_buffer_size_low_limit) || (measure_data.period_acxx_buffer_size > measure_data.period_acxx_buffer_size_high_limit)) {
        goto errors;
    }
    // Processing each channel.
    for (chx_idx = 0; chx_idx < MEASURE_NUMBER_OF_ACI_CHANNELS; chx_idx++) {
        // Compute channel buffer.
        for (idx = 0; idx < (measure_data.period_acxx_buffer_size); idx++) {
            // Copy samples by channel and convert to Q31 type.
            sample_idx = (MEASURE_NUMBER_OF_ACI_CHANNELS * idx) + chx_idx;
#ifdef MPMCM_ANALOG_SIMULATION
            measure_data.period_acvx_buffer_f32[idx] = (float32_t) (SIMULATION_ACV_BUFFER[sample_idx]);
            measure_data.period_acix_buffer_f32[idx] = (float32_t) (SIMULATION_ACI_BUFFER[sample_idx] / measure_ctx.random_divider);
#else
            measure_data.period_acvx_buffer_f32[idx] = (float32_t) (measure_sampling.acv[measure_sampling.acv_read_idx].data[sample_idx]);
            measure_data.period_acix_buffer_f32[idx] = (float32_t) (measure_sampling.aci[measure_sampling.aci_read_idx].data[sample_idx]);
            // Force current to 0 if sensor is not connected.
            if (GPIO_read(MEASURE_GPIO_ACI_DETECT[chx_idx]) == 0) {
                measure_data.period_acix_buffer_f32[idx] = 0.0;
            }
#endif
        }
        // Mean voltage and current.
        arm_mean_f32((float32_t*) measure_data.period_acvx_buffer_f32, measure_data.period_acxx_buffer_size, &mean_voltage_f32);
        arm_mean_f32((float32_t*) measure_data.period_acix_buffer_f32, measure_data.period_acxx_buffer_size, &mean_current_f32);
        // DC removal.
        for (idx = 0; idx < (measure_data.period_acxx_buffer_size); idx++) {
            measure_data.period_acvx_buffer_f32[idx] -= mean_voltage_f32;
            measure_data.period_acix_buffer_f32[idx] -= mean_current_f32;
        }
        // Instantaneous power.
        arm_mult_f32((float32_t*) measure_data.period_acvx_buffer_f32, (float32_t*) measure_data.period_acix_buffer_f32, (float32_t*) measure_data.period_acpx_buffer_f32, measure_data.period_acxx_buffer_size);
        // Active power.
        arm_mean_f32((float32_t*) measure_data.period_acpx_buffer_f32, measure_data.period_acxx_buffer_size, (float32_t*) &(measure_data.period_active_power_f32));
        temp_f64 = (float64_t) measure_data.period_active_power_f32;
        temp_f64 *= measure_data.acp_factor_num[chx_idx];
        active_power_mw = (temp_f64 / measure_data.acp_factor_den);
        // RMS voltage.
        arm_rms_f32((float32_t*) measure_data.period_acvx_buffer_f32, measure_data.period_acxx_buffer_size, (float32_t*) &(measure_data.period_rms_voltage_f32));
        temp_f64 = measure_data.acv_factor_num * ((float64_t) measure_data.period_rms_voltage_f32);
        rms_voltage_mv = (temp_f64 / measure_data.acv_factor_den);
        // RMS current.
        arm_rms_f32((float32_t*) measure_data.period_acix_buffer_f32, measure_data.period_acxx_buffer_size, (float32_t*) &(measure_data.period_rms_current_f32));
        temp_f64 = measure_data.aci_factor_num[chx_idx] * ((float64_t) measure_data.period_rms_current_f32);
        rms_current_ma = (temp_f64 / measure_data.aci_factor_den);
        // Apparent power.
        temp_f64 = (rms_voltage_mv * rms_current_ma);
        apparent_power_mva = ((temp_f64) / ((float64_t) 1000.0));
        if (((active_power_mw > 0.0) && (apparent_power_mva < 0.0)) || ((active_power_mw < 0.0) && (apparent_power_mva > 0.0))) {
            apparent_power_mva *= (-1.0);
        }
        // Power factor.
        temp_f64 = (active_power_mw * ((float64_t) MEASURE_POWER_FACTOR_MULTIPLIER));
        power_factor = (apparent_power_mva != 0.0) ? (temp_f64 / apparent_power_mva) : 0;
        if (((active_power_mw > 0.0) && (power_factor < 0.0)) || ((active_power_mw < 0.0) && (power_factor > 0.0))) {
            power_factor *= (-1.0);
        }
        // Update accumulated data.
        DATA_add_run_channel_sample(measure_data.chx_rolling_mean[chx_idx], active_power_mw, active_power_mw);
        DATA_add_run_channel_sample(measure_data.chx_rolling_mean[chx_idx], rms_voltage_mv, rms_voltage_mv);
        DATA_add_run_channel_sample(measure_data.chx_rolling_mean[chx_idx], rms_current_ma, rms_current_ma);
        DATA_add_run_channel_sample(measure_data.chx_rolling_mean[chx_idx], apparent_power_mva, apparent_power_mva);
        DATA_add_run_channel_sample(measure_data.chx_rolling_mean[chx_idx], power_factor, power_factor);
    }
    // Compute mains frequency.
    for (idx = 0; idx < MEASURE_PERIOD_TIMX_DMA_BUFFER_SIZE; idx++) {
        // Search two valid consecutive samples.
        if (measure_sampling.acv_frequency_capture[idx] < measure_sampling.acv_frequency_capture[(idx + 1) % MEASURE_PERIOD_TIMX_DMA_BUFFER_SIZE]) {
            // Compute delta.
            acv_frequency_capture_delta = measure_sampling.acv_frequency_capture[(idx + 1) % MEASURE_PERIOD_TIMX_DMA_BUFFER_SIZE] - measure_sampling.acv_frequency_capture[idx];
            // Avoid rollover case (clamp to 1Hz).
            if (acv_frequency_capture_delta < MEASURE_ACV_FREQUENCY_SAMPLING_HZ) break;
        }
    }
    // Check index.
    if (idx < MEASURE_PERIOD_TIMX_DMA_BUFFER_SIZE) {
        // Compute mains frequency.
        frequency_mhz = (((float64_t) (MEASURE_ACV_FREQUENCY_SAMPLING_HZ * 1000)) / ((float64_t) acv_frequency_capture_delta));
        // Update accumulated data.
        DATA_add_run_sample(measure_data.acv_frequency_rolling_mean, frequency_mhz);
    }
errors:
    // Update read indexes.
    measure_sampling.acv_read_idx = ((measure_sampling.acv_read_idx + 1) % MEASURE_PERIOD_ADCX_DMA_BUFFER_DEPTH);
    measure_sampling.aci_read_idx = ((measure_sampling.aci_read_idx + 1) % MEASURE_PERIOD_ADCX_DMA_BUFFER_DEPTH);
}
#endif

#ifdef MPMCM_ANALOG_MEASURE_ENABLE
/*******************************************************************/
static void _MEASURE_compute_run_data(void) {
    // Local variables.
    uint8_t chx_idx = 0;
    // Compute AC channels run data.
    for (chx_idx = 0; chx_idx < MEASURE_NUMBER_OF_ACI_CHANNELS; chx_idx++) {
        // Copy all rolling means and reset.
        DATA_copy_run_channel(measure_data.chx_rolling_mean[chx_idx], measure_data.chx_run_data[chx_idx]);
        DATA_reset_run_channel(measure_data.chx_rolling_mean[chx_idx]);
    }
    // Compute frequency run data and reset.
    DATA_copy_run(measure_data.acv_frequency_rolling_mean, measure_data.acv_frequency_run_data);
    DATA_reset_run(measure_data.acv_frequency_rolling_mean);
}
#endif

#ifdef MPMCM_ANALOG_MEASURE_ENABLE
/*******************************************************************/
static void _MEASURE_compute_accumulated_data(void) {
    // Local variables.
    float64_t sample_abs = 0.0;
    float64_t ref_abs = 0.0;
    uint8_t chx_idx = 0;
    // Compute AC channels accumulated data.
    for (chx_idx = 0; chx_idx < MEASURE_NUMBER_OF_ACI_CHANNELS; chx_idx++) {
        // Copy all rolling means.
        DATA_add_accumulated_channel_sample(measure_data.chx_accumulated_data[chx_idx], active_power_mw, measure_data.chx_run_data[chx_idx].active_power_mw);
        DATA_add_accumulated_channel_sample(measure_data.chx_accumulated_data[chx_idx], rms_voltage_mv, measure_data.chx_run_data[chx_idx].rms_voltage_mv);
        DATA_add_accumulated_channel_sample(measure_data.chx_accumulated_data[chx_idx], rms_current_ma, measure_data.chx_run_data[chx_idx].rms_current_ma);
        DATA_add_accumulated_channel_sample(measure_data.chx_accumulated_data[chx_idx], apparent_power_mva, measure_data.chx_run_data[chx_idx].apparent_power_mva);
        DATA_add_accumulated_channel_sample(measure_data.chx_accumulated_data[chx_idx], power_factor, measure_data.chx_run_data[chx_idx].power_factor);
        // Increase active energy.
        measure_data.active_energy_mws_sum[chx_idx].value += (measure_data.chx_run_data[chx_idx].active_power_mw.value);
        measure_data.active_energy_mws_sum[chx_idx].number_of_samples++;
        // Increase apparent energy.
        measure_data.apparent_energy_mvas_sum[chx_idx].value += (measure_data.chx_run_data[chx_idx].apparent_power_mva.value);
        measure_data.apparent_energy_mvas_sum[chx_idx].number_of_samples++;
        // Reset results.
        DATA_reset_run_channel(measure_data.chx_rolling_mean[chx_idx]);
    }
    // Compute frequency accumulated data.
    DATA_add_accumulated_sample(measure_data.acv_frequency_accumulated_data, measure_data.acv_frequency_run_data);
}
#endif

#ifdef MPMCM_ANALOG_MEASURE_ENABLE
/*******************************************************************/
static MEASURE_status_t _MEASURE_led_single_pulse(void) {
    // Local variables.
    MEASURE_status_t status = MEASURE_SUCCESS;
    LED_status_t led_status = LED_SUCCESS;
    LED_color_t led_color = LED_COLOR_OFF;
    // Check LED period.
    if (measure_ctx.tick_led_seconds_count >= MEASURE_LED_PULSE_PERIOD_SECONDS) {
        // Reset count.
        measure_ctx.tick_led_seconds_count = 0;
        // Compute LED color according to state.
        if (measure_ctx.state == MEASURE_STATE_OFF) {
            // Check current number of samples (CH1 RMS voltage as reference).
            led_color = (measure_data.chx_accumulated_data[0].rms_voltage_mv.number_of_samples == 0) ? LED_COLOR_RED : LED_COLOR_YELLOW;
        }
        else {
            led_color = LED_COLOR_GREEN;
        }
        // Perform LED pulse.
        led_status = LED_single_pulse(MEASURE_LED_PULSE_DURATION_MS, led_color);
        LED_exit_error(MEASURE_ERROR_BASE_LED);
    }
errors:
    return status;
}
#endif

#ifdef MPMCM_ANALOG_MEASURE_ENABLE
/*******************************************************************/
static MEASURE_status_t _MEASURE_internal_process(void) {
    // Local variables.
    MEASURE_status_t status = MEASURE_SUCCESS;
    // Perform state machine.
    switch (measure_ctx.state) {
    case MEASURE_STATE_OFF:
        // Synchronize on zero cross.
        if (measure_ctx.zero_cross_count >= MEASURE_ZERO_CROSS_START_THRESHOLD) {
            // Reset context.
            _MEASURE_reset();
            // Start measure.
            status = _MEASURE_start();
            if (status != MEASURE_SUCCESS) goto errors;
            // Update state.
            measure_ctx.state = MEASURE_STATE_ACTIVE;
        }
        break;
    case MEASURE_STATE_ACTIVE:
        // Check zero cross count.
        if (measure_ctx.zero_cross_count >= MEASURE_ZERO_CROSS_PER_PERIOD) {
            // Clear counters.
            measure_ctx.zero_cross_count = 0;
            measure_ctx.dma_transfer_end_flag = 0;
            // Switch to next buffer.
            status = _MEASURE_switch_dma_buffer();
            if (status != MEASURE_SUCCESS) goto errors;
            // Compute data.
            _MEASURE_compute_period_data();
        }
        // Check DMA transfer end flag.
        if (measure_ctx.dma_transfer_end_flag != 0) {
            // Clear counters and flags.
            measure_ctx.zero_cross_count = 0;
            measure_ctx.dma_transfer_end_flag = 0;
            measure_ctx.sampled_period_count = 0;
            measure_ctx.period_compute_enable = 0;
            // Update state.
            measure_ctx.state = MEASURE_STATE_OFF;
            // Stop measure.
            status = _MEASURE_stop();
            if (status != MEASURE_SUCCESS) goto errors;
        }
        break;
    default:
        status = MEASURE_ERROR_STATE;
        goto errors;
    }
errors:
    return status;
}
#endif

#ifdef MPMCM_ANALOG_MEASURE_ENABLE
/*******************************************************************/
static void _MEASURE_increment_zero_cross_count(void) {
    // Local variables.
    MEASURE_status_t measure_status = MEASURE_SUCCESS;
    // Increment counts.
    measure_ctx.zero_cross_count++;
    // Process measure.
    measure_status = _MEASURE_internal_process();
    MEASURE_stack_error(ERROR_BASE_MEASURE);
}
#endif

#ifdef MPMCM_ANALOG_MEASURE_ENABLE
/*******************************************************************/
static void _MEASURE_set_dma_transfer_end_flag(void) {
    // Local variables.
    MEASURE_status_t measure_status = MEASURE_SUCCESS;
    // Set local flag.
    measure_ctx.dma_transfer_end_flag = 1;
    // Process measure.
    measure_status = _MEASURE_internal_process();
    MEASURE_stack_error(ERROR_BASE_MEASURE);
}
#endif

/*** MEASURE functions ***/

/*******************************************************************/
MEASURE_status_t MEASURE_init(void) {
    // Local variables.
    MEASURE_status_t status = MEASURE_SUCCESS;
#ifdef MPMCM_ANALOG_MEASURE_ENABLE
    DMA_status_t dma_status = DMA_SUCCESS;
    DMA_configuration_t dma_config;
    uint8_t chx_idx = 0;
#endif
    // Init context.
    measure_ctx.state = MEASURE_STATE_OFF;
    measure_ctx.tick_led_seconds_count = 0;
    // Reset data.
    _MEASURE_reset();
#ifdef MPMCM_ANALOG_MEASURE_ENABLE
    // Init detect pins.
    for (chx_idx = 0; chx_idx < MEASURE_NUMBER_OF_ACI_CHANNELS; chx_idx++) {
        GPIO_configure(MEASURE_GPIO_ACI_DETECT[chx_idx], GPIO_MODE_INPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    }
    // Init DMA for master ADC.
    dma_config.direction = DMA_DIRECTION_PERIPHERAL_TO_MEMORY;
    dma_config.flags.all = 0;
    dma_config.flags.memory_increment = 1;
    dma_config.memory_address = (uint32_t) &(measure_sampling.acv[measure_sampling.acv_write_idx].data);
    dma_config.memory_data_size = DMA_DATA_SIZE_16_BITS;
    dma_config.peripheral_address = ADC_get_master_dr_register_address(MEASURE_ACV_ACI_ADC_INSTANCE);
    dma_config.peripheral_data_size = DMA_DATA_SIZE_16_BITS;
    dma_config.number_of_data = MEASURE_PERIOD_ADCX_DMA_BUFFER_SIZE;
    dma_config.priority = DMA_PRIORITY_VERY_HIGH;
    dma_config.request_id = DMAMUX_PERIPHERAL_REQUEST_ADC1;
    dma_config.tc_irq_callback = &_MEASURE_set_dma_transfer_end_flag;
    dma_config.nvic_priority = NVIC_PRIORITY_DMA_ACV_SAMPLING;
    dma_status = DMA_init(DMA_INSTANCE_ACV_SAMPLING, DMA_CHANNEL_ACV_SAMPLING, &dma_config);
    DMA_exit_error(MEASURE_ERROR_BASE_DMA);
    // Init DMA for slave ADC.
    dma_config.memory_address = (uint32_t) &(measure_sampling.aci[measure_sampling.aci_write_idx].data);
    dma_config.peripheral_address = ADC_get_slave_dr_register_address(MEASURE_ACV_ACI_ADC_INSTANCE);
    dma_config.number_of_data = MEASURE_PERIOD_ADCX_DMA_BUFFER_SIZE;
    dma_config.priority = DMA_PRIORITY_HIGH;
    dma_config.request_id = DMAMUX_PERIPHERAL_REQUEST_ADC2;
    dma_config.nvic_priority = NVIC_PRIORITY_DMA_ACI_SAMPLING;
    dma_status = DMA_init(DMA_INSTANCE_ACI_SAMPLING, DMA_CHANNEL_ACI_SAMPLING, &dma_config);
    DMA_exit_error(MEASURE_ERROR_BASE_DMA);
    // Init DMA for ACV frequency capture timer.
    dma_config.flags.all = 0;
    dma_config.flags.memory_increment = 1;
    dma_config.flags.circular_mode = 1;
    dma_config.memory_address = (uint32_t) &(measure_sampling.acv_frequency_capture);
    dma_config.memory_data_size = DMA_DATA_SIZE_32_BITS;
    dma_config.peripheral_address = TIM_get_ccr_register_address(MEASURE_ACV_FREQUENCY_TIM_INSTANCE, GPIO_TIM_CHANNEL_ACV_FREQUENCY);
    dma_config.peripheral_data_size = DMA_DATA_SIZE_32_BITS;
    dma_config.number_of_data = MEASURE_PERIOD_TIMX_DMA_BUFFER_SIZE;
    dma_config.priority = DMA_PRIORITY_MEDIUM;
    dma_config.request_id = DMAMUX_PERIPHERAL_REQUEST_TIM2_CH1;
    dma_config.tc_irq_callback = NULL;
    dma_config.nvic_priority = NVIC_PRIORITY_DMA_ACV_FREQUENCY;
    dma_status = DMA_init(DMA_INSTANCE_ACV_FREQUENCY, DMA_CHANNEL_ACV_FREQUENCY, &dma_config);
    DMA_exit_error(MEASURE_ERROR_BASE_DMA);
    // Turn analog front-end on to have VREF+ for ADC calibration.
    POWER_enable(POWER_REQUESTER_ID_MEASURE, POWER_DOMAIN_ANALOG, LPTIM_DELAY_MODE_SLEEP);
#ifdef MPMCM_ANALOG_SIMULATION
    // Init zero cross emulation timer.
    TIM_STD_init(MEASURE_SIMULATION_TIM_INSTANCE, NVIC_PRIORITY_SIMULATION);
    TIM_STD_start(MEASURE_SIMULATION_TIM_INSTANCE, RCC_CLOCK_LSE, (MEASURE_MAINS_PERIOD_US / MEASURE_ZERO_CROSS_PER_PERIOD), TIM_UNIT_US, &_MEASURE_increment_zero_cross_count);
#else
    // Init zero cross interrupt from external circuit.
    EXTI_configure_gpio(&GPIO_ZERO_CROSS_PULSE, GPIO_PULL_DOWN, EXTI_TRIGGER_RISING_EDGE, &_MEASURE_increment_zero_cross_count, NVIC_PRIORITY_ZERO_CROSS);
    EXTI_enable_gpio_interrupt(&GPIO_ZERO_CROSS_PULSE);
#endif
errors:
#endif
    return status;
}

/*******************************************************************/
MEASURE_state_t MEASURE_get_state(void) {
#ifdef MPMCM_ANALOG_SIMULATION
    return MEASURE_STATE_ACTIVE;
#else
    return (measure_ctx.state);
#endif
}

/*******************************************************************/
MEASURE_status_t MEASURE_set_gains(uint16_t transformer_gain, uint16_t current_sensors_gain[MEASURE_NUMBER_OF_ACI_CHANNELS]) {
    // Local variables.
    MEASURE_status_t status = MEASURE_SUCCESS;
    uint8_t chx_idx = 0;
    // Check parameters.
    if (current_sensors_gain == NULL) {
        status = MEASURE_ERROR_NULL_PARAMETER;
        goto errors;
    }
    // ACV.
    measure_data.acv_factor_num = ((float64_t) transformer_gain * (float64_t) MPMCM_TRANSFORMER_ATTEN * (float64_t) STM32G4XX_DRIVERS_ADC_VREF_MV);
    measure_data.acv_factor_den = ((float64_t) MEASURE_TRANSFORMER_GAIN_FACTOR * (float64_t) ADC_FULL_SCALE);
    // ACI.
    for (chx_idx = 0; chx_idx < MEASURE_NUMBER_OF_ACI_CHANNELS; chx_idx++) {
        measure_data.aci_factor_num[chx_idx] = ((float64_t) current_sensors_gain[chx_idx] * (float64_t) MEASURE_SCT013_ATTEN[chx_idx] * (float64_t) STM32G4XX_DRIVERS_ADC_VREF_MV);
    }
    measure_data.aci_factor_den = ((float64_t) MEASURE_CURRENT_SENSOR_GAIN_FACTOR * (float64_t) ADC_FULL_SCALE);
    // ACP.
    for (chx_idx = 0; chx_idx < MEASURE_NUMBER_OF_ACI_CHANNELS; chx_idx++) {
        // Note: 1000 factor is used to get mW from mV and mA.
        // Conversion is done here to limit numerator value and avoid overflow during power computation.
        // There is no precision loss since ACV and ACI factors multiplication is necessarily a multiple of 1000 thanks to ADC_VREF_MV.
        measure_data.acp_factor_num[chx_idx] = (measure_data.acv_factor_num * measure_data.aci_factor_num[chx_idx]) / ((float64_t) 1000);
    }
    measure_data.acp_factor_den = (measure_data.acv_factor_den * measure_data.aci_factor_den);
    // Buffer size limits.
    measure_data.period_acxx_buffer_size_low_limit = ((100 - MEASURE_PERIOD_ADCX_BUFFER_SIZE_ERROR_PERCENT) * MEASURE_PERIOD_BUFFER_SIZE) / (100);
    measure_data.period_acxx_buffer_size_high_limit = ((100 + MEASURE_PERIOD_ADCX_BUFFER_SIZE_ERROR_PERCENT) * MEASURE_PERIOD_BUFFER_SIZE) / (100);
errors:
    return status;
}

#ifdef MPMCM_ANALOG_MEASURE_ENABLE
/*******************************************************************/
MEASURE_status_t MEASURE_tick_second(void) {
    // Local variables.
    MEASURE_status_t status = MEASURE_SUCCESS;
    // Increment seconds count.
    measure_ctx.tick_led_seconds_count++;
#ifdef MPMCM_ANALOG_SIMULATION
    measure_ctx.random_divider = 1 + ((measure_ctx.random_divider + 1) % 100);
#endif
    // Check state.
    if ((measure_ctx.state != MEASURE_STATE_OFF) && (measure_ctx.period_compute_enable != 0)) {
        // Compute run data from last second.
        measure_ctx.processing_enable = 0;
        _MEASURE_compute_run_data();
        measure_ctx.processing_enable = 1;
        // Compute accumulated data.
        _MEASURE_compute_accumulated_data();
    }
    // Manage LED.
    status = _MEASURE_led_single_pulse();
    return status;
}
#endif

/*******************************************************************/
MEASURE_status_t MEASURE_get_probe_detect_flag(uint8_t channel_index, uint8_t* current_sensor_connected) {
    // Local variables.
    MEASURE_status_t status = MEASURE_SUCCESS;
    // Check parameters.
    if (channel_index >= MEASURE_NUMBER_OF_ACI_CHANNELS) {
        status = MEASURE_ERROR_AC_CHANNEL;
        goto errors;
    }
    if (current_sensor_connected == NULL) {
        status = MEASURE_ERROR_NULL_PARAMETER;
        goto errors;
    }
    // Update flag.
#ifdef MPMCM_ANALOG_MEASURE_ENABLE
#ifdef MPMCM_ANALOG_SIMULATION
    (*current_sensor_connected) = SIMULATION_GPIO_ACI_DETECT[channel_index];
#else
    (*current_sensor_connected) = GPIO_read(MEASURE_GPIO_ACI_DETECT[channel_index]);
#endif
#else
    (*current_sensor_connected) = 0;
#endif
errors:
    return status;
}

/*******************************************************************/
MEASURE_status_t MEASURE_get_mains_detect_flag(uint8_t* mains_voltage_detected) {
    // Local variables.
    MEASURE_status_t status = MEASURE_SUCCESS;
    // Check parameters.
    if (mains_voltage_detected == NULL) {
        status = MEASURE_ERROR_NULL_PARAMETER;
        goto errors;
    }
    // Update flag.
    (*mains_voltage_detected) = (measure_ctx.state == MEASURE_STATE_OFF) ? 0 : 1;
errors:
    return status;
}

/*******************************************************************/
MEASURE_status_t MEASURE_get_run_data(MEASURE_data_index_t data_index, DATA_run_t* run_data) {
    // Local variables.
    MEASURE_status_t status = MEASURE_SUCCESS;
    // Check parameters.
    if (run_data == NULL) {
        status = MEASURE_ERROR_NULL_PARAMETER;
        goto errors;
    }
    // Check data type.
    switch (data_index) {
    case MEASURE_DATA_INDEX_MAINS_FREQUENCY_MHZ:
        // Copy data.
        DATA_copy_run(measure_data.acv_frequency_run_data, (*run_data));
        break;
    default:
        status = MEASURE_ERROR_DATA_TYPE;
        goto errors;
    }
errors:
    return status;
}

/*******************************************************************/
MEASURE_status_t MEASURE_get_accumulated_data(MEASURE_data_index_t data_index, DATA_accumulated_t* accumulated_data) {
    // Local variables.
    MEASURE_status_t status = MEASURE_SUCCESS;
    // Check parameters.
    if (accumulated_data == NULL) {
        status = MEASURE_ERROR_NULL_PARAMETER;
        goto errors;
    }
    // Check data type.
    switch (data_index) {
    case MEASURE_DATA_INDEX_MAINS_FREQUENCY_MHZ:
        // Copy and reset data.
        DATA_copy_accumulated(measure_data.acv_frequency_accumulated_data, (*accumulated_data));
        DATA_reset_accumulated(measure_data.acv_frequency_accumulated_data);
        break;
    default:
        status = MEASURE_ERROR_DATA_TYPE;
        goto errors;
    }
errors:
    return status;
}

/*******************************************************************/
MEASURE_status_t MEASURE_get_channel_run_data(uint8_t channel, DATA_run_channel_t* channel_run_data) {
    // Local variables.
    MEASURE_status_t status = MEASURE_SUCCESS;
    // Check parameters.
    if (channel_run_data == NULL) {
        status = MEASURE_ERROR_NULL_PARAMETER;
        goto errors;
    }
    if (channel >= MEASURE_NUMBER_OF_ACI_CHANNELS) {
        status = MEASURE_ERROR_AC_CHANNEL;
        goto errors;
    }
    // Copy data.
    DATA_copy_run_channel(measure_data.chx_run_data[channel], (*channel_run_data));
errors:
    return status;
}

/*******************************************************************/
MEASURE_status_t MEASURE_get_channel_accumulated_data(uint8_t channel, DATA_accumulated_channel_t* channel_accumulated_data) {
    // Local variables.
    MEASURE_status_t status = MEASURE_SUCCESS;
    // Check parameters.
    if (channel_accumulated_data == NULL) {
        status = MEASURE_ERROR_NULL_PARAMETER;
        goto errors;
    }
    if (channel >= MEASURE_NUMBER_OF_ACI_CHANNELS) {
        status = MEASURE_ERROR_AC_CHANNEL;
        goto errors;
    }
    // Compute active energy.
    measure_data.chx_accumulated_data[channel].active_energy_mwh.value = ((measure_data.active_energy_mws_sum[channel].value) / ((float64_t) DATA_SECONDS_PER_HOUR));
    measure_data.chx_accumulated_data[channel].active_energy_mwh.number_of_samples = measure_data.active_energy_mws_sum[channel].number_of_samples;
    // Compute apparent energy.
    measure_data.chx_accumulated_data[channel].apparent_energy_mvah.value = ((measure_data.apparent_energy_mvas_sum[channel].value) / ((float64_t) DATA_SECONDS_PER_HOUR));
    measure_data.chx_accumulated_data[channel].apparent_energy_mvah.number_of_samples = measure_data.apparent_energy_mvas_sum[channel].number_of_samples;
    // Copy data.
    DATA_copy_accumulated_channel(measure_data.chx_accumulated_data[channel], (*channel_accumulated_data));
    // Reset data.
    DATA_reset_accumulated_channel(measure_data.chx_accumulated_data[channel]);
    DATA_reset_run(measure_data.active_energy_mws_sum[channel]);
    DATA_reset_run(measure_data.apparent_energy_mvas_sum[channel]);
#ifdef MPMCM_ANALOG_SIMULATION
    measure_ctx.random_divider = 1;
#endif
errors:
    return status;
}
