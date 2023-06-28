/*
 * mpmcm_startup.c
 *
 *  Created on: 26 apr. 2018
 *      Author: ARM
 */

#include "types.h"

/*----------------------------------------------------------------------------
  Linker generated Symbols
 *----------------------------------------------------------------------------*/
extern uint32_t __etext;
extern uint32_t __data_sram1_start__;
extern uint32_t __data_sram1_end__;
#ifdef __STARTUP_COPY_MULTIPLE
extern uint32_t __copy_table_start__;
extern uint32_t __copy_table_end__;
extern uint32_t __zero_table_start__;
extern uint32_t __zero_table_end__;
#endif
extern uint32_t __bss_start__;
extern uint32_t __bss_end__;
extern uint32_t __StackTop;

/*----------------------------------------------------------------------------
  Exception / Interrupt Handler Function Prototype
 *----------------------------------------------------------------------------*/
typedef void( *pFunc )( void );

/*----------------------------------------------------------------------------
  External References
 *----------------------------------------------------------------------------*/
#ifndef __START
extern void  _start(void) __attribute__((noreturn));    /* PreeMain (C library entry point) */
#else
extern int  __START(void) __attribute__((noreturn));    /* main entry point */
#endif

#ifndef __NO_SYSTEM_INIT
extern void SystemInit (void);							/* CMSIS System Initialization      */
#endif

/*----------------------------------------------------------------------------
  Internal References
 *----------------------------------------------------------------------------*/
void Default_Handler(void);								/* Default empty handler */
void Reset_Handler(void);								/* Reset Handler */

/*----------------------------------------------------------------------------
  User Initial Stack & Heap
 *----------------------------------------------------------------------------*/
#ifndef __STACK_SIZE
#define	__STACK_SIZE  0x00000400
#endif
static uint8_t stack[__STACK_SIZE] __attribute__ ((aligned(8), used, section(".stack")));

#ifndef __HEAP_SIZE
#define	__HEAP_SIZE   0x00000C00
#endif
#if __HEAP_SIZE > 0
static uint8_t heap[__HEAP_SIZE]   __attribute__ ((aligned(8), used, section(".heap")));
#endif

/*----------------------------------------------------------------------------
  Exception / Interrupt Handler
 *----------------------------------------------------------------------------*/
/* Cortex-M4 Processor Exceptions */
void NMI_Handler							(void) __attribute__ ((weak, alias("Default_Handler")));
void HardFault_Handler						(void) __attribute__ ((weak, alias("Default_Handler")));
void MemManage_Handler						(void) __attribute__ ((weak, alias("Default_Handler")));
void BusFault_Handler						(void) __attribute__ ((weak, alias("Default_Handler")));
void UsageFault_Handler						(void) __attribute__ ((weak, alias("Default_Handler")));
void SVC_Handler							(void) __attribute__ ((weak, alias("Default_Handler")));
void DebugMon_Handler						(void) __attribute__ ((weak, alias("Default_Handler")));
void PendSV_Handler							(void) __attribute__ ((weak, alias("Default_Handler")));
void SysTick_Handler						(void) __attribute__ ((weak, alias("Default_Handler")));

/* ARMCM4 Specific Interrupts */
void WWDT_IRQHandler      					(void) __attribute__ ((weak, alias("Default_Handler")));
void PVD_PVM_IRQHandler						(void) __attribute__ ((weak, alias("Default_Handler")));
void RTC_TAMP_CSS_IRQHandler				(void) __attribute__ ((weak, alias("Default_Handler")));
void RTC_WKUP_IRQHandler					(void) __attribute__ ((weak, alias("Default_Handler")));
void FLASH_IRQHandler						(void) __attribute__ ((weak, alias("Default_Handler")));
void RCC_IRQHandler     					(void) __attribute__ ((weak, alias("Default_Handler")));
void EXTI0_IRQHandler						(void) __attribute__ ((weak, alias("Default_Handler")));
void EXTI1_IRQHandler						(void) __attribute__ ((weak, alias("Default_Handler")));
void EXTI2_IRQHandler						(void) __attribute__ ((weak, alias("Default_Handler")));
void EXTI3_IRQHandler						(void) __attribute__ ((weak, alias("Default_Handler")));
void EXTI4_IRQHandler						(void) __attribute__ ((weak, alias("Default_Handler")));
void DMA1_CH1_IRQHandler					(void) __attribute__ ((weak, alias("Default_Handler")));
void DMA1_CH2_IRQHandler					(void) __attribute__ ((weak, alias("Default_Handler")));
void DMA1_CH3_IRQHandler					(void) __attribute__ ((weak, alias("Default_Handler")));
void DMA1_CH4_IRQHandler					(void) __attribute__ ((weak, alias("Default_Handler")));
void DMA1_CH5_IRQHandler					(void) __attribute__ ((weak, alias("Default_Handler")));
void DMA1_CH6_IRQHandler					(void) __attribute__ ((weak, alias("Default_Handler")));
void DMA1_CH7_IRQHandler					(void) __attribute__ ((weak, alias("Default_Handler")));
void ADC1_2_IRQHandler						(void) __attribute__ ((weak, alias("Default_Handler")));
void USB_HP_IRQHandler						(void) __attribute__ ((weak, alias("Default_Handler")));
void USB_LP_IRQHandler						(void) __attribute__ ((weak, alias("Default_Handler")));
void FDCAN1_IT0_IRQHandler					(void) __attribute__ ((weak, alias("Default_Handler")));
void FDCAN1_IT1_IRQHandler					(void) __attribute__ ((weak, alias("Default_Handler")));
void EXTI5_9_IRQHandler						(void) __attribute__ ((weak, alias("Default_Handler")));
void TIM1_BRK_TIM15_IRQHandler				(void) __attribute__ ((weak, alias("Default_Handler")));
void TIM1_UP_TIM16_IRQHandler				(void) __attribute__ ((weak, alias("Default_Handler")));
void TIM1_TRG_COM_DIR_IDX_TIM17_IRQHandler	(void) __attribute__ ((weak, alias("Default_Handler")));
void TIM1_CC_IRQHandler						(void) __attribute__ ((weak, alias("Default_Handler")));
void TIM2_IRQHandler						(void) __attribute__ ((weak, alias("Default_Handler")));
void TIM3_IRQHandler						(void) __attribute__ ((weak, alias("Default_Handler")));
void TIM4_IRQHandler						(void) __attribute__ ((weak, alias("Default_Handler")));
void I2C1_EV_IRQHandler						(void) __attribute__ ((weak, alias("Default_Handler")));
void I2C1_ER_IRQHandler						(void) __attribute__ ((weak, alias("Default_Handler")));
void I2C2_EV_IRQHandler						(void) __attribute__ ((weak, alias("Default_Handler")));
void I2C2_ER_IRQHandler						(void) __attribute__ ((weak, alias("Default_Handler")));
void SPI1_IRQHandler						(void) __attribute__ ((weak, alias("Default_Handler")));
void SPI2_IRQHandler						(void) __attribute__ ((weak, alias("Default_Handler")));
void USART1_IRQHandler						(void) __attribute__ ((weak, alias("Default_Handler")));
void USART2_IRQHandler						(void) __attribute__ ((weak, alias("Default_Handler")));
void USART3_IRQHandler						(void) __attribute__ ((weak, alias("Default_Handler")));
void EXTI10_15_IRQHandler					(void) __attribute__ ((weak, alias("Default_Handler")));
void RTC_ALARM_IRQHandler					(void) __attribute__ ((weak, alias("Default_Handler")));
void USB_WKUP_IRQHandler					(void) __attribute__ ((weak, alias("Default_Handler")));
void TIM8_BRK_TER_IERR_IRQHandler			(void) __attribute__ ((weak, alias("Default_Handler")));
void TIM8_UP_IRQHandler						(void) __attribute__ ((weak, alias("Default_Handler")));
void TIM8_TRG_COM_DIR_IDX_IRQHandler		(void) __attribute__ ((weak, alias("Default_Handler")));
void TIM8_CC_IRQHandler						(void) __attribute__ ((weak, alias("Default_Handler")));
void ADC3_IRQHandler						(void) __attribute__ ((weak, alias("Default_Handler")));
void FSMC_IRQHandler						(void) __attribute__ ((weak, alias("Default_Handler")));
void LPTIM1_IRQHandler						(void) __attribute__ ((weak, alias("Default_Handler")));
void TIM5_IRQHandler						(void) __attribute__ ((weak, alias("Default_Handler")));
void SPI3_IRQHandler						(void) __attribute__ ((weak, alias("Default_Handler")));
void UART4_IRQHandler						(void) __attribute__ ((weak, alias("Default_Handler")));
void UART5_IRQHandler						(void) __attribute__ ((weak, alias("Default_Handler")));
void TIM6_DAC1_3_IRQHandler					(void) __attribute__ ((weak, alias("Default_Handler")));
void TIM7_DAC2_4_IRQHandler					(void) __attribute__ ((weak, alias("Default_Handler")));
void DMA2_CH1_IRQHandler					(void) __attribute__ ((weak, alias("Default_Handler")));
void DMA2_CH2_IRQHandler					(void) __attribute__ ((weak, alias("Default_Handler")));
void DMA2_CH3_IRQHandler					(void) __attribute__ ((weak, alias("Default_Handler")));
void DMA2_CH4_IRQHandler					(void) __attribute__ ((weak, alias("Default_Handler")));
void DMA2_CH5_IRQHandler					(void) __attribute__ ((weak, alias("Default_Handler")));
void ADC4_IRQHandler						(void) __attribute__ ((weak, alias("Default_Handler")));
void ADC5_IRQHandler						(void) __attribute__ ((weak, alias("Default_Handler")));
void UCPD1_IRQHandler						(void) __attribute__ ((weak, alias("Default_Handler")));
void COMP1_2_3_IRQHandler					(void) __attribute__ ((weak, alias("Default_Handler")));
void COMP4_5_6_IRQHandler					(void) __attribute__ ((weak, alias("Default_Handler")));
void COMP7_IRQHandler						(void) __attribute__ ((weak, alias("Default_Handler")));
void HRTIM_MASTER_IRQHandler				(void) __attribute__ ((weak, alias("Default_Handler")));
void HRTIM_TIMA_IRQHandler					(void) __attribute__ ((weak, alias("Default_Handler")));
void HRTIM_TIMB_IRQHandler					(void) __attribute__ ((weak, alias("Default_Handler")));
void HRTIM_TIMC_IRQHandler					(void) __attribute__ ((weak, alias("Default_Handler")));
void HRTIM_TIMD_IRQHandler					(void) __attribute__ ((weak, alias("Default_Handler")));
void HRTIM_TIME_IRQHandler					(void) __attribute__ ((weak, alias("Default_Handler")));
void HRTIM_TIM_FLT_IRQHandler				(void) __attribute__ ((weak, alias("Default_Handler")));
void HRTIM_TIMF_IRQHandler					(void) __attribute__ ((weak, alias("Default_Handler")));
void CRS_IRQHandler							(void) __attribute__ ((weak, alias("Default_Handler")));
void SAI_IRQHandler							(void) __attribute__ ((weak, alias("Default_Handler")));
void TIM20_BRK_TERR_IERR_IRQHandler			(void) __attribute__ ((weak, alias("Default_Handler")));
void TIM20_UP_IRQHandler					(void) __attribute__ ((weak, alias("Default_Handler")));
void TIM20_TRG_COM_DIR_IDX_IRQHandler		(void) __attribute__ ((weak, alias("Default_Handler")));
void TIM20_CC_IRQHandler					(void) __attribute__ ((weak, alias("Default_Handler")));
void FPU_IRQHandler							(void) __attribute__ ((weak, alias("Default_Handler")));
void I2C4_EV_IRQHandler						(void) __attribute__ ((weak, alias("Default_Handler")));
void I2C4_ER_IRQHandler						(void) __attribute__ ((weak, alias("Default_Handler")));
void SPI4_IRQHandler						(void) __attribute__ ((weak, alias("Default_Handler")));
void AES_IRQHandler							(void) __attribute__ ((weak, alias("Default_Handler")));
void FDCAN2_IT0_IRQHandler					(void) __attribute__ ((weak, alias("Default_Handler")));
void FDCAN2_IT1_IRQHandler					(void) __attribute__ ((weak, alias("Default_Handler")));
void FDCAN3_IT0_IRQHandler					(void) __attribute__ ((weak, alias("Default_Handler")));
void FDCAN3_IT1_IRQHandler					(void) __attribute__ ((weak, alias("Default_Handler")));
void RNG_IRQHandler							(void) __attribute__ ((weak, alias("Default_Handler")));
void LPUART1_IRQHandler						(void) __attribute__ ((weak, alias("Default_Handler")));
void I2C3_EV_IRQHandler						(void) __attribute__ ((weak, alias("Default_Handler")));
void I2C3_ER_IRQHandler						(void) __attribute__ ((weak, alias("Default_Handler")));
void DMAMUX_IRQHandler						(void) __attribute__ ((weak, alias("Default_Handler")));
void QUADSPI_IRQHandler						(void) __attribute__ ((weak, alias("Default_Handler")));
void DMA1_CH8_IRQHandler					(void) __attribute__ ((weak, alias("Default_Handler")));
void DMA2_CH6_IRQHandler					(void) __attribute__ ((weak, alias("Default_Handler")));
void DMA2_CH7_IRQHandler					(void) __attribute__ ((weak, alias("Default_Handler")));
void DMA2_CH8_IRQHandler					(void) __attribute__ ((weak, alias("Default_Handler")));
void CORDIC_IRQHandler						(void) __attribute__ ((weak, alias("Default_Handler")));
void FMAC_IRQHandler						(void) __attribute__ ((weak, alias("Default_Handler")));

/*----------------------------------------------------------------------------
  Exception / Interrupt Vector table
 *----------------------------------------------------------------------------*/
const pFunc __Vectors[] __attribute__ ((section(".vectors"))) = {
  /* Cortex-M4 Exceptions Handler */
  (pFunc)&__StackTop,                       /*      Initial Stack Pointer     */
  Reset_Handler,                            /*      Reset Handler             */
  NMI_Handler,                              /*      NMI Handler               */
  HardFault_Handler,                        /*      Hard Fault Handler        */
  MemManage_Handler,                        /*      MPU Fault Handler         */
  BusFault_Handler,                         /*      Bus Fault Handler         */
  UsageFault_Handler,                       /*      Usage Fault Handler       */
  0,                                        /*      Reserved                  */
  0,                                        /*      Reserved                  */
  0,                                        /*      Reserved                  */
  0,                                        /*      Reserved                  */
  SVC_Handler,                              /*      SVCall Handler            */
  DebugMon_Handler,                         /*      Debug Monitor Handler     */
  0,                                        /*      Reserved                  */
  PendSV_Handler,                           /*      PendSV Handler            */
  SysTick_Handler,                          /*      SysTick Handler           */

  /* External interrupts */
  WWDT_IRQHandler,
  PVD_PVM_IRQHandler,
  RTC_TAMP_CSS_IRQHandler,
  RTC_WKUP_IRQHandler,
  FLASH_IRQHandler,
  RCC_IRQHandler,
  EXTI0_IRQHandler,
  EXTI1_IRQHandler,
  EXTI2_IRQHandler,
  EXTI3_IRQHandler,
  EXTI4_IRQHandler,
  DMA1_CH1_IRQHandler,
  DMA1_CH2_IRQHandler,
  DMA1_CH3_IRQHandler,
  DMA1_CH4_IRQHandler,
  DMA1_CH5_IRQHandler,
  DMA1_CH6_IRQHandler,
  DMA1_CH7_IRQHandler,
  ADC1_2_IRQHandler,
  USB_HP_IRQHandler,
  USB_LP_IRQHandler,
  FDCAN1_IT0_IRQHandler,
  FDCAN1_IT1_IRQHandler,
  EXTI5_9_IRQHandler,
  TIM1_BRK_TIM15_IRQHandler,
  TIM1_UP_TIM16_IRQHandler,
  TIM1_TRG_COM_DIR_IDX_TIM17_IRQHandler,
  TIM1_CC_IRQHandler,
  TIM2_IRQHandler,
  TIM3_IRQHandler,
  TIM4_IRQHandler,
  I2C1_EV_IRQHandler,
  I2C1_ER_IRQHandler,
  I2C2_EV_IRQHandler,
  I2C2_ER_IRQHandler,
  SPI1_IRQHandler,
  SPI2_IRQHandler,
  USART1_IRQHandler,
  USART2_IRQHandler,
  USART3_IRQHandler,
  EXTI10_15_IRQHandler,
  RTC_ALARM_IRQHandler,
  USB_WKUP_IRQHandler,
  TIM8_BRK_TER_IERR_IRQHandler,
  TIM8_UP_IRQHandler,
  TIM8_TRG_COM_DIR_IDX_IRQHandler,
  TIM8_CC_IRQHandler,
  ADC3_IRQHandler,
  FSMC_IRQHandler,
  LPTIM1_IRQHandler,
  TIM5_IRQHandler,
  SPI3_IRQHandler,
  UART4_IRQHandler,
  UART5_IRQHandler,
  TIM6_DAC1_3_IRQHandler,
  TIM7_DAC2_4_IRQHandler,
  DMA2_CH1_IRQHandler,
  DMA2_CH2_IRQHandler,
  DMA2_CH3_IRQHandler,
  DMA2_CH4_IRQHandler,
  DMA2_CH5_IRQHandler,
  ADC4_IRQHandler,
  ADC5_IRQHandler,
  UCPD1_IRQHandler,
  COMP1_2_3_IRQHandler,
  COMP4_5_6_IRQHandler,
  COMP7_IRQHandler,
  HRTIM_MASTER_IRQHandler,
  HRTIM_TIMA_IRQHandler,
  HRTIM_TIMB_IRQHandler,
  HRTIM_TIMC_IRQHandler,
  HRTIM_TIMD_IRQHandler,
  HRTIM_TIME_IRQHandler,
  HRTIM_TIM_FLT_IRQHandler,
  HRTIM_TIMF_IRQHandler,
  CRS_IRQHandler,
  SAI_IRQHandler,
  TIM20_BRK_TERR_IERR_IRQHandler,
  TIM20_UP_IRQHandler,
  TIM20_TRG_COM_DIR_IDX_IRQHandler,
  TIM20_CC_IRQHandler,
  FPU_IRQHandler,
  I2C4_EV_IRQHandler,
  I2C4_ER_IRQHandler,
  SPI4_IRQHandler,
  AES_IRQHandler,
  FDCAN2_IT0_IRQHandler,
  FDCAN2_IT1_IRQHandler,
  FDCAN3_IT0_IRQHandler,
  FDCAN3_IT1_IRQHandler,
  RNG_IRQHandler,
  LPUART1_IRQHandler,
  I2C3_EV_IRQHandler,
  I2C3_ER_IRQHandler,
  DMAMUX_IRQHandler,
  QUADSPI_IRQHandler,
  DMA1_CH8_IRQHandler,
  DMA2_CH6_IRQHandler,
  DMA2_CH7_IRQHandler,
  DMA2_CH8_IRQHandler,
  CORDIC_IRQHandler,
  FMAC_IRQHandler
};

/*----------------------------------------------------------------------------
  Reset Handler called on controller reset
 *----------------------------------------------------------------------------*/
void Reset_Handler(void) {
  uint32_t *pSrc, *pDest;
  uint32_t *pTable __attribute__((unused));

/*  Firstly it copies data from read only memory to RAM. There are two schemes
 *  to copy. One can copy more than one sections. Another can only copy
 *  one section.  The former scheme needs more instructions and read-only
 *  data to implement than the latter.
 *  Macro __STARTUP_COPY_MULTIPLE is used to choose between two schemes.  */

#ifdef __STARTUP_COPY_MULTIPLE
/*  Multiple sections scheme.
 *
 *  Between symbol address __copy_table_start__ and __copy_table_end__,
 *  there are array of triplets, each of which specify:
 *    offset 0: LMA of start of a section to copy from
 *    offset 4: VMA of start of a section to copy to
 *    offset 8: size of the section to copy. Must be multiply of 4
 *
 *  All addresses must be aligned to 4 bytes boundary.
 */
  pTable = &__copy_table_start__;

  for (; pTable < &__copy_table_end__; pTable = pTable + 3) {
		pSrc  = (uint32_t*)*(pTable + 0);
		pDest = (uint32_t*)*(pTable + 1);
		for (; pDest < (uint32_t*)(*(pTable + 1) + *(pTable + 2)) ; ) {
      *pDest++ = *pSrc++;
		}
	}
#else
/*  Single section scheme.
 *
 *  The ranges of copy from/to are specified by following symbols
 *    __etext: LMA of start of the section to copy from. Usually end of text
 *    __data_start__: VMA of start of the section to copy to
 *    __data_end__: VMA of end of the section to copy to
 *
 *  All addresses must be aligned to 4 bytes boundary.
 */
  pSrc  = &__etext;
  pDest = &__data_sram1_start__;

  for ( ; pDest < &__data_sram1_end__ ; ) {
    *pDest++ = *pSrc++;
  }
#endif /*__STARTUP_COPY_MULTIPLE */

/*  This part of work usually is done in C library startup code. Otherwise,
 *  define this macro to enable it in this startup.
 *
 *  There are two schemes too. One can clear multiple BSS sections. Another
 *  can only clear one section. The former is more size expensive than the
 *  latter.
 *
 *  Define macro __STARTUP_CLEAR_BSS_MULTIPLE to choose the former.
 *  Otherwise efine macro __STARTUP_CLEAR_BSS to choose the later.
 */
#ifdef __STARTUP_CLEAR_BSS_MULTIPLE
/*  Multiple sections scheme.
 *
 *  Between symbol address __copy_table_start__ and __copy_table_end__,
 *  there are array of tuples specifying:
 *    offset 0: Start of a BSS section
 *    offset 4: Size of this BSS section. Must be multiply of 4
 */
  pTable = &__zero_table_start__;

  for (; pTable < &__zero_table_end__; pTable = pTable + 2) {
		pDest = (uint32_t*)*(pTable + 0);
		for (; pDest < (uint32_t*)(*(pTable + 0) + *(pTable + 1)) ; ) {
      *pDest++ = 0;
		}
	}
#elif defined (__STARTUP_CLEAR_BSS)
/*  Single BSS section scheme.
 *
 *  The BSS section is specified by following symbols
 *    __bss_start__: start of the BSS section.
 *    __bss_end__: end of the BSS section.
 *
 *  Both addresses must be aligned to 4 bytes boundary.
 */
  pDest = &__bss_start__;

  for ( ; pDest < &__bss_end__ ; ) {
    *pDest++ = 0ul;
  }
#endif /* __STARTUP_CLEAR_BSS_MULTIPLE || __STARTUP_CLEAR_BSS */

#ifndef __NO_SYSTEM_INIT
	SystemInit();
#endif

#ifndef __START
#define __START _start
#endif
	__START();
}

/*----------------------------------------------------------------------------
  Default Handler for Exceptions / Interrupts
 *----------------------------------------------------------------------------*/
void Default_Handler(void) {
	// Enter sleep mode.
	while (1) {
		__asm volatile ("wfi");
	}
}
