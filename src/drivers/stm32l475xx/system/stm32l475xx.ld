/*
******************************************************************************
**
** @file        : stm32l475xx.ld
**
** @author      : Modified from STM32CubeIDE Auto-Generator by SpaceBaker
**
** @brief       : Linker script for STM32L475VGTx Device from STM32L4 series
**                      1024Kbytes ROM
**                      96Kbytes   SRAM1
**                      32Kbytes   SRAM2
**
**                Set heap size, stack size and stack location according
**                to application requirements.
**
**                Set memory bank area and size if external memory is used
**
**  Target      : STMicroelectronics STM32
**
**  Distribution: The file is distributed as is, without any warranty
**                of any kind.
**
******************************************************************************
** @attention
**
** Copyright (c) 2022 STMicroelectronics.
** All rights reserved.
**
** This software is licensed under terms that can be found in the LICENSE file
** in the root directory of this software component.
** If no LICENSE file comes with this software, it is provided AS-IS.
**
******************************************************************************
*/
/* Memory sections visualization
 *
 *     Code memory (FLASH)                                         Data memory (SRAM)
 *   ----------------------                                      ---------------------- ----> __stack_end__
 *   | Unused code memory |                                      |       Stack        | 
 *   |                    |                                      |                    | |
 *   |                    |                                      |                    | v
 *   |--------------------| ----> __data_end__                   |--------------------| ----> __stack_start__ 
 *   |      .data         |                                      |    Unused SRAM     |
 *   |--------------------| ----> __data_start__                 |--------------------| ^ --> __heap_end__
 *   |     .rodata        |                                      |       Heap         | |
 *   |--------------------| ----> __text_end__                   |                    | 
 *   |      .text         |                                      |--------------------| ----> __heap_start__
 *   |                    |                                      |       .bss         |
 *   |                    |                                      |                    |
 *   |--------------------|                                      |--------------------|
 *   |   .vector_table    |                                      |       .data        |
 *   ---------------------- ----> ROM(FLASH) ORIGIN              ---------------------- ----> RAM(SRAM) ORIGIN
 *
 *
 * Note 1 : The .data section of ROM needs to be copied to the .data section of RAM
 * Note 2 : Location counter operator '.' gives the current location (addr) in VMA (virtual memory address)
*/


/* Stack and Heap configuration */
__STACK_SIZE = 0x00000400;
__HEAP_SIZE  = 0x00000C00;


/* Memories definition */
MEMORY
{
  SRAM1  (xrw) : ORIGIN = 0x20000000, LENGTH = 96K
  SRAM2  (xrw) : ORIGIN = 0x10000000, LENGTH = 32K
  FLASH   (rx) : ORIGIN = 0x08000000, LENGTH = 1024K
}


/* Highest address of the user mode stack */
__stack_start__ = ORIGIN(SRAM1) + LENGTH(SRAM1) - __STACK_SIZE;



/* Entry Point */
ENTRY(Reset_Handler)


/* Sections */
SECTIONS
{
  /* The program code and other rodata into "ROM" type memory */
  .text :
  {
    . = ALIGN(4);
    __text_start__ = .;

    /* The vector table defines the initial stack pointer and interrupt/exception routines */
    KEEP(*(.isr_vector))

    /* Include code and read-only data sections from all input files. /*
    *(.text .text* .gnu.linkonce.t.*)
    *(.rodata .rodata* .gnu.linkonce.r.*)
  
    /* Compiler generated helper functions */
    *(.glue_7 .glue_7t)

    /* Data used to unwind the stack when an exception is thrown */
    *(.eh_frame)

    /* C++ Runtime: initializers for static variables.
       C Runtime: designated constructors */
    . = ALIGN(4);
    KEEP(*(.init))
    . = ALIGN(4);
    __preinit_array_start = .;
    KEEP (*(.preinit_array))
    __preinit_array_end = .;

    . = ALIGN(4);
    __init_array_start = .;
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array))
    __init_array_end = .;

    /* C++ runtime: destructors for static variables.
       C runtime: designated finializers */
    . = ALIGN(4);
    KEEP(*(.fini))

    . = ALIGN(4);
    __fini_array_start = .;
    KEEP (*(.fini_array))
    KEEP (*(SORT(.fini_array.*)))
    __fini_array_end = .;

    /* C++ runtime: static constructors */
    . = ALIGN(4);
    KEEP (*crtbegin.o(.ctors))
    KEEP (*(EXCLUDE_FILE (*crtend.o) .ctors))
    KEEP (*(SORT(.ctors.*)))
    KEEP (*crtend.o(.ctors))

    /* C++ runtime: static destructors and atexit() */
    . = ALIGN(4);
    KEEP (*crtbegin.o(.dtors))
    KEEP (*(EXCLUDE_FILE (*crtend.o) .dtors))
    KEEP (*(SORT(.dtors.*)))
    KEEP (*crtend.o(.dtors))

    . = ALIGN(4);
  } >FLASH

  /* ARM special sections for exception handling */
  .ARM.extab :
  {
    *(.ARM.extab* .gnu.linkonce.armextab.*)
  } >FLASH

  .ARM.exidx :
  {
    PROVIDE (__exidx_start = .);
    *(.ARM.exidx* .gnu.linkonce.armexidx.*)
    PROVIDE (__exidx_end = .);
  } >FLASH

  /* Initialized data sections into "RAM" type memory */
  .data :
  {
    . = ALIGN(4);
    __data_start__ = .;
    *(.data .data*)
    *(.RamFunc .RamFunc*)
    . = ALIGN(4);
    __data_end__ = .;
  } >SRAM1 AT>FLASH

  __text_end__ = LOADADDR(.data);

  /* Uninitialized data section into "RAM" type memory */
  .bss (NOLOAD):
  {
    . = ALIGN(4);
    __bss_start__ = .;
    *(.bss .bss*)
    *(COMMON)
    . = ALIGN(4);
    __bss_end__ = .;
  } >SRAM1

  /* Heap grows up */
  .heap (NOLOAD) :
  {
    . = ALIGN(4);
    __heap_start__ = .;
    . = . + __HEAP_SIZE;
    . = ALIGN(4);
    __heap_end__ = .;
    PROVIDE(__HeapLimit = .);
  } > SRAM1

  /* Stack grows down */
  .stack (__stack_start__) (NOLOAD) :
  {
    /* The ARM procedure call standard (AAPCS) requires the stack to be aligned on an eight byte boundary */
    . = ALIGN(8);
    PROVIDE(__StackLimit = .);
    . = . + __STACK_SIZE;
    . = ALIGN(4);
    PROVIDE(__StackTop = .);
    __stack_end__ = .;
  } > SRAM1

  /* Check if data + heap + stack exceeds SRAM1 limit */
  ASSERT(__StackLimit >= __HeapLimit, "region SRAM1 overflowed with stack")
}