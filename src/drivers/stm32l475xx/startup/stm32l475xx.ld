/*
******************************************************************************
**
** @file        : stm32l475xx.ld
**
** @author      : Modified from STM32CubeIDE Auto-Generator by SpaceBaker
**
** @brief       : Linker script for STM32L475VGTx Device from STM32L4 series
**                      1024Kbytes ROM
**                      96Kbytes   RAM
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
 *   ----------------------                                      ----------------------
 *   | Unused code memory |                                      |       Stack        | 
 *   |                    |                                      |                    | |
 *   |                    |                                      |                    | v
 *   |--------------------| ----> _edata                         |--------------------|
 *   |      .data         |                                      |    Unused SRAM     |
 *   |--------------------| ----> _sdata                         |--------------------| ^
 *   |     .rodata        |                                      |       Heap         | |
 *   |--------------------| ----> _etext                         |                    | 
 *   |      .text         |                                      |--------------------|
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


__STACK_SIZE = 0x00000400;
__HEAP_SIZE  = 0x00000C00;

/* ARMv8-M stack sealing:
   to use ARMv8-M stack sealing set __STACKSEAL_SIZE to 8 otherwise keep 0
 */
__STACKSEAL_SIZE = 0;


/* Memories definition */
MEMORY
{
  SRAM1  (xrw) : ORIGIN = 0x20000000, LENGTH = 96K
  SRAM2  (xrw) : ORIGIN = 0x10000000, LENGTH = 32K
  FLASH   (rx) : ORIGIN = 0x08000000, LENGTH = 1024K
}

/* Highest address of the user mode stack */
_estack = ORIGIN(SRAM1) + LENGTH(SRAM1) - __STACK_SIZE - __STACKSEAL_SIZE;

/* Entry Point */
ENTRY(Reset_Handler)


/* Sections */
SECTIONS
{
  /* The program code and other data into "ROM" type memory */
  .text :
  {
    . = ALIGN(4);
    *(.isr_vector)     /* .isr_vector sections (startup code) */
    *(.text)           /* .text sections (code) */
    *(.text*)          /* .text* sections (code) */
    *(.glue_7)         /* glue arm to thumb code */
    *(.glue_7t)        /* glue thumb to arm code */
    *(.eh_frame)
    . = ALIGN(4);
    _etext = .;        /* end of text global symbol */
  } >FLASH

  /* Constant data into "ROM" type memory */
  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)         /* .rodata sections (constants, strings, etc.) */
    *(.rodata*)        /* .rodata* sections (constants, strings, etc.) */
    . = ALIGN(4);
  } >FLASH

  /* Initialized data sections into "RAM" type memory */
  .data :
  {
    . = ALIGN(4);
    _sdata = .;        /* start of data section global symbol */
    *(.data)           /* .data sections */
    *(.data*)          /* .data* sections */
    *(.RamFunc)        /* .RamFunc sections */
    *(.RamFunc*)       /* .RamFunc* sections */
    . = ALIGN(4);
    _edata = .;        /* end of data section global symbol */
  } >SRAM1 AT>FLASH

  /* Uninitialized data section into "RAM" type memory */
  . = ALIGN(4);
  .bss :
  {
    /* This is used by the startup in order to initialize the .bss section */
    _sbss = .;         /* start of bss section global symbol */
    *(.bss)
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;         /* end of bss section global symbol */
  } >SRAM1

  .heap (NOLOAD) :
  {
    . = ALIGN(4);
    __end__ = .;
    PROVIDE(end = .);
    . = . + __HEAP_SIZE;
    . = ALIGN(4);
    __HeapLimit = .;
  } > SRAM1

  .stack (_estack) (NOLOAD) :
  {
    . = ALIGN(4);
    __StackLimit = .;
    . = . + __STACK_SIZE;
    . = ALIGN(4);
    __StackTop = .;
  } > SRAM1
  PROVIDE(__stack = __StackTop);

    /* ARMv8-M stack sealing:
     to use ARMv8-M stack sealing uncomment '.stackseal' section
   */
/*
  .stackseal (ORIGIN(SRAM1) + LENGTH(SRAM1) - __STACKSEAL_SIZE) (NOLOAD) :
  {
    . = ALIGN(8);
    __StackSeal = .;
    . = . + 8;
    . = ALIGN(8);
  } > SRAM1
*/

  /* Check if data + heap + stack exceeds SRAM1 limit */
  ASSERT(__StackLimit >= __HeapLimit, "region SRAM1 overflowed with stack")
}