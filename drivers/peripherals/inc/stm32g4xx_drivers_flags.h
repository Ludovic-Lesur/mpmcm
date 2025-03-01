/*
 * stm32g4xx_drivers_flags.h
 *
 *  Created on: 02 feb. 2025
 *      Author: Ludo
 */

#ifndef __STM32G4XX_DRIVERS_FLAGS_H__
#define __STM32G4XX_DRIVERS_FLAGS_H__

#include "nvm_address.h"

/*** STM32G4xx drivers compilation flags ***/

#define STM32G4XX_DRIVERS_ADC_MODE_MASK                 0x03
#define STM32G4XX_DRIVERS_ADC_VREF_MV                   2500

#define STM32G4XX_DRIVERS_DMA_CHANNEL_MASK              0x000F

#define STM32G4XX_DRIVERS_EXTI_GPIO_MASK                0x0004

#define STM32G4XX_DRIVERS_LPUART_RS485
#define STM32G4XX_DRIVERS_LPUART_DISABLE_TX_0

#define STM32G4XX_DRIVERS_RCC_HSE_ENABLE
#define STM32G4XX_DRIVERS_RCC_HSE_FREQUENCY_HZ          16000000
#define STM32G4XX_DRIVERS_RCC_LSE_MODE                  2
#define STM32G4XX_DRIVERS_RCC_LSE_FREQUENCY_HZ          32768

#define STM32G4XX_DRIVERS_RTC_WAKEUP_PERIOD_SECONDS     1
#define STM32G4XX_DRIVERS_RTC_ALARM_MASK                0x00

#define STM32G4XX_DRIVERS_TIM_MODE_MASK                 0x3F

//#define STM32G4XX_DRIVERS_USART_RS485

#endif /* __STM32G4XX_DRIVERS_FLAGS_H__ */
