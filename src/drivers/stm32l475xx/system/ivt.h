/**
 * @file ivt.h
 * @author SpaceBaker (emile.forcier@gmail.com)
 * @brief An interrupt vector table (IVT) is a data structure that associates a list of interrupt handlers 
 * with a list of interrupt requests in a table of interrupt vectors. Each entry of the interrupt vector table, 
 * called an interrupt vector, is the address of an interrupt handler (also known as ISR).
 * @version 0.1
 * @date 2025-04-26
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef __IVT_H
#define __IVT_H


#include "drivers/stm32l475xx/stm32l475xx.h"
#include "common/macro_magic.h"

typedef void (*ivt_entry_t)(void);

typedef struct {
    unsigned int *initial_stack_pointer;
    ivt_entry_t Reset_Handler;
    ivt_entry_t NMI_Handler;
    ivt_entry_t HardFault_Handler;
    ivt_entry_t MemManage_Handler;
    ivt_entry_t BusFault_Handler;
    ivt_entry_t UsageFault_Handler;
    ivt_entry_t RESERVED[4];
    ivt_entry_t SVC_Handler;
    ivt_entry_t DebugMon_Handler;
    ivt_entry_t RESERVED;
    ivt_entry_t PendSV_Handler;
    ivt_entry_t SysTick_Handler;
    ivt_entry_t irq[IRQn_Max];
} ivt_t;


#endif // __IVT_H
