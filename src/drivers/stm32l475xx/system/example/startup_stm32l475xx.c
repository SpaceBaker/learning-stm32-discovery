/******************************************************************************
 * @file     startup_stm32l475xx.c
 * @brief    CMSIS-Core(M) Device Startup File for
 *           Device STM32L475xx
 * @version  V1.0.1
 * @date     30. April 2024
 ******************************************************************************/
/*
 * Copyright (c) 2009-2021 Arm Limited. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "../stm32l4xx.h"   // IWYU pragma: keep
#include "../ivt.h"
#include "../irq.h"
#include "linker_symbols.h"


/*----------------------------------------------------------------------------
  Exception / Interrupt Vector table
 *----------------------------------------------------------------------------*/
ivt_t isr_vector_table __attribute__((used, section(".isr_vector"))) = {
    &__stack_start__,                    /*     Initial Stack Pointer */
    Reset_Handler,                       /*     Reset Handler */
    NMI_Handler,                         /* -14 NMI Handler */
    HardFault_Handler,                   /* -13 Hard Fault Handler */
    MemManage_Handler,                   /* -12 MPU Fault Handler */
    BusFault_Handler,                    /* -11 Bus Fault Handler */
    UsageFault_Handler,                  /* -10 Usage Fault Handler */
    {0, 0, 0, 0},                        /*     Reserved */
    SVC_Handler,                         /*  -5 SVCall Handler */
    DebugMon_Handler,                    /*  -4 Debug Monitor Handler */
    0,                                   /*     Reserved */
    PendSV_Handler,                      /*  -2 PendSV Handler */
    SysTick_Handler,                     /*  -1 SysTick Handler */
    /* Device specific interrupt vectors */
    {
        WWDG_IRQHandler,
        PVD_PVM_IRQHandler,
        TAMP_STAMP_IRQHandler,
        RTC_WKUP_IRQHandler,
        FLASH_IRQHandler,
        RCC_IRQHandler,
        EXTI0_IRQHandler,
        EXTI1_IRQHandler,
        EXTI2_IRQHandler,
        EXTI3_IRQHandler,
        EXTI4_IRQHandler,
        DMA1_Channel1_IRQHandler,
        DMA1_Channel2_IRQHandler,
        DMA1_Channel3_IRQHandler,
        DMA1_Channel4_IRQHandler,
        DMA1_Channel5_IRQHandler,
        DMA1_Channel6_IRQHandler,
        DMA1_Channel7_IRQHandler,
        ADC1_2_IRQHandler,
        CAN1_TX_IRQHandler,
        CAN1_RX0_IRQHandler,
        CAN1_RX1_IRQHandler,
        CAN1_SCE_IRQHandler,
        EXTI9_5_IRQHandler,
        TIM1_BRK_TIM15_IRQHandler,
        TIM1_UP_TIM16_IRQHandler,
        TIM1_TRG_COM_TIM17_IRQHandler,
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
        EXTI15_10_IRQHandler,
        RTC_Alarm_IRQHandler,
        DFSDM1_FLT3_IRQHandler,
        TIM8_BRK_IRQHandler,
        TIM8_UP_IRQHandler,
        TIM8_TRG_COM_IRQHandler,
        TIM8_CC_IRQHandler,
        ADC3_IRQHandler,
        FMC_IRQHandler,
        SDMMC1_IRQHandler,
        TIM5_IRQHandler,
        SPI3_IRQHandler,
        UART4_IRQHandler,
        UART5_IRQHandler,
        TIM6_DAC_IRQHandler,
        TIM7_IRQHandler,
        DMA2_Channel1_IRQHandler,
        DMA2_Channel2_IRQHandler,
        DMA2_Channel3_IRQHandler,
        DMA2_Channel4_IRQHandler,
        DMA2_Channel5_IRQHandler,
        DFSDM1_FLT0_IRQHandler,
        DFSDM1_FLT1_IRQHandler,
        DFSDM1_FLT2_IRQHandler,
        COMP_IRQHandler,
        LPTIM1_IRQHandler,
        LPTIM2_IRQHandler,
        OTG_FS_IRQHandler,
        DMA2_Channel6_IRQHandler,
        DMA2_Channel7_IRQHandler,
        LPUART1_IRQHandler,
        QUADSPI_IRQHandler,
        I2C3_EV_IRQHandler,
        I2C3_ER_IRQHandler,
        SAI1_IRQHandler,
        SAI2_IRQHandler,
        SWPMI1_IRQHandler,
        TSC_IRQHandler,
        0,                                   /*     Reserved */
        0,                                   /*     Reserved */
        RNG_IRQHandler,
        FPU_IRQHandler
    }
};

/*---------------------------------------------------------------------------
  Reset Handler called upon reset
 *---------------------------------------------------------------------------*/
void Reset_Handler(void)
{
    extern int main(void);

    /* CMSIS System Initialization */
    /* FPU settings ------------------------------------------------------------*/
    #if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
    SCB->CPACR |= ((3UL << 20U)|(3UL << 22U));  /* set CP10 and CP11 Full Access */
    #endif

    // Copy .data from FLASH to SRAM
    unsigned int *flash_data = &__text_end__;
    unsigned int *sram_data  = &__data_start__;
    for (; sram_data < &__data_end__; sram_data++, flash_data++) {
        *sram_data = *flash_data;
    }

    // Zero-fill .bss section in SRAM
    unsigned int *bss = &__bss_start__;
    for (; bss < &__bss_end__; bss++) {
        *bss = 0;
    }

    // __libc_init_array();

    // Go to main
    main();

    // If main exit
    while (1);
}

/*---------------------------------------------------------------------------
  Default Handler for Exceptions / Interrupts
 *---------------------------------------------------------------------------*/
void Default_Handler(void)
{
    while (1);
}
