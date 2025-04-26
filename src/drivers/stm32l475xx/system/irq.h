/**
 * @file irq.h
 * @author SpaceBaker (emile.forcier@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2025-04-26
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef __IRQ_H
#define __IRQ_H


void Default_Handler(void);

/*---------------------------------------------------------------------------
  Exception / Interrupt Handler
 *---------------------------------------------------------------------------*/
/* Exceptions */
void Reset_Handler                 (void);
void NMI_Handler                   (void) __attribute__ ((weak, alias("Default_Handler")));
void HardFault_Handler             (void) __attribute__ ((weak, alias("Default_Handler")));
void MemManage_Handler             (void) __attribute__ ((weak, alias("Default_Handler")));
void BusFault_Handler              (void) __attribute__ ((weak, alias("Default_Handler")));
void UsageFault_Handler            (void) __attribute__ ((weak, alias("Default_Handler")));
void SecureFault_Handler           (void) __attribute__ ((weak, alias("Default_Handler")));
void SVC_Handler                   (void) __attribute__ ((weak, alias("Default_Handler")));
void DebugMon_Handler              (void) __attribute__ ((weak, alias("Default_Handler")));
void PendSV_Handler                (void) __attribute__ ((weak, alias("Default_Handler")));
void SysTick_Handler               (void) __attribute__ ((weak, alias("Default_Handler")));
/* Device specific interrupt handler */
void WWDG_IRQHandler               (void) __attribute__ ((weak, alias("Default_Handler")));
void PVD_PVM_IRQHandler            (void) __attribute__ ((weak, alias("Default_Handler")));
void TAMP_STAMP_IRQHandler         (void) __attribute__ ((weak, alias("Default_Handler")));
void RTC_WKUP_IRQHandler           (void) __attribute__ ((weak, alias("Default_Handler")));
void FLASH_IRQHandler              (void) __attribute__ ((weak, alias("Default_Handler")));
void RCC_IRQHandler                (void) __attribute__ ((weak, alias("Default_Handler")));
void EXTI0_IRQHandler              (void) __attribute__ ((weak, alias("Default_Handler")));
void EXTI1_IRQHandler              (void) __attribute__ ((weak, alias("Default_Handler")));
void EXTI2_IRQHandler              (void) __attribute__ ((weak, alias("Default_Handler")));
void EXTI3_IRQHandler              (void) __attribute__ ((weak, alias("Default_Handler")));
void EXTI4_IRQHandler              (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA1_Channel1_IRQHandler      (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA1_Channel2_IRQHandler      (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA1_Channel3_IRQHandler      (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA1_Channel4_IRQHandler      (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA1_Channel5_IRQHandler      (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA1_Channel6_IRQHandler      (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA1_Channel7_IRQHandler      (void) __attribute__ ((weak, alias("Default_Handler")));
void ADC1_2_IRQHandler             (void) __attribute__ ((weak, alias("Default_Handler")));
void CAN1_TX_IRQHandler            (void) __attribute__ ((weak, alias("Default_Handler")));
void CAN1_RX0_IRQHandler           (void) __attribute__ ((weak, alias("Default_Handler")));
void CAN1_RX1_IRQHandler           (void) __attribute__ ((weak, alias("Default_Handler")));
void CAN1_SCE_IRQHandler           (void) __attribute__ ((weak, alias("Default_Handler")));
void EXTI9_5_IRQHandler            (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM1_BRK_TIM15_IRQHandler     (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM1_UP_TIM16_IRQHandler      (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM1_TRG_COM_TIM17_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM1_CC_IRQHandler            (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM2_IRQHandler               (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM3_IRQHandler               (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM4_IRQHandler               (void) __attribute__ ((weak, alias("Default_Handler")));
void I2C1_EV_IRQHandler            (void) __attribute__ ((weak, alias("Default_Handler")));
void I2C1_ER_IRQHandler            (void) __attribute__ ((weak, alias("Default_Handler")));
void I2C2_EV_IRQHandler            (void) __attribute__ ((weak, alias("Default_Handler")));
void I2C2_ER_IRQHandler            (void) __attribute__ ((weak, alias("Default_Handler")));
void SPI1_IRQHandler               (void) __attribute__ ((weak, alias("Default_Handler")));
void SPI2_IRQHandler               (void) __attribute__ ((weak, alias("Default_Handler")));
void USART1_IRQHandler             (void) __attribute__ ((weak, alias("Default_Handler")));
void USART2_IRQHandler             (void) __attribute__ ((weak, alias("Default_Handler")));
void USART3_IRQHandler             (void) __attribute__ ((weak, alias("Default_Handler")));
void EXTI15_10_IRQHandler          (void) __attribute__ ((weak, alias("Default_Handler")));
void RTC_Alarm_IRQHandler          (void) __attribute__ ((weak, alias("Default_Handler")));
void DFSDM1_FLT3_IRQHandler        (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM8_BRK_IRQHandler           (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM8_UP_IRQHandler            (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM8_TRG_COM_IRQHandler       (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM8_CC_IRQHandler            (void) __attribute__ ((weak, alias("Default_Handler")));
void ADC3_IRQHandler               (void) __attribute__ ((weak, alias("Default_Handler")));
void FMC_IRQHandler                (void) __attribute__ ((weak, alias("Default_Handler")));
void SDMMC1_IRQHandler             (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM5_IRQHandler               (void) __attribute__ ((weak, alias("Default_Handler")));
void SPI3_IRQHandler               (void) __attribute__ ((weak, alias("Default_Handler")));
void UART4_IRQHandler              (void) __attribute__ ((weak, alias("Default_Handler")));
void UART5_IRQHandler              (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM6_DAC_IRQHandler           (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM7_IRQHandler               (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA2_Channel1_IRQHandler      (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA2_Channel2_IRQHandler      (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA2_Channel3_IRQHandler      (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA2_Channel4_IRQHandler      (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA2_Channel5_IRQHandler      (void) __attribute__ ((weak, alias("Default_Handler")));
void DFSDM1_FLT0_IRQHandler        (void) __attribute__ ((weak, alias("Default_Handler")));
void DFSDM1_FLT1_IRQHandler        (void) __attribute__ ((weak, alias("Default_Handler")));
void DFSDM1_FLT2_IRQHandler        (void) __attribute__ ((weak, alias("Default_Handler")));
void COMP_IRQHandler	           (void) __attribute__ ((weak, alias("Default_Handler")));
void LPTIM1_IRQHandler	           (void) __attribute__ ((weak, alias("Default_Handler")));
void LPTIM2_IRQHandler             (void) __attribute__ ((weak, alias("Default_Handler")));
void OTG_FS_IRQHandler             (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA2_Channel6_IRQHandler      (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA2_Channel7_IRQHandler      (void) __attribute__ ((weak, alias("Default_Handler")));
void LPUART1_IRQHandler            (void) __attribute__ ((weak, alias("Default_Handler")));
void QUADSPI_IRQHandler            (void) __attribute__ ((weak, alias("Default_Handler")));
void I2C3_EV_IRQHandler            (void) __attribute__ ((weak, alias("Default_Handler")));
void I2C3_ER_IRQHandler            (void) __attribute__ ((weak, alias("Default_Handler")));
void SAI1_IRQHandler	           (void) __attribute__ ((weak, alias("Default_Handler")));
void SAI2_IRQHandler	           (void) __attribute__ ((weak, alias("Default_Handler")));
void SWPMI1_IRQHandler	           (void) __attribute__ ((weak, alias("Default_Handler")));
void TSC_IRQHandler	               (void) __attribute__ ((weak, alias("Default_Handler")));
void RNG_IRQHandler	               (void) __attribute__ ((weak, alias("Default_Handler")));
void FPU_IRQHandler                (void) __attribute__ ((weak, alias("Default_Handler")));


#endif // __IRQ_H