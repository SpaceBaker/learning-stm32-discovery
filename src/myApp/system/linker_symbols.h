/**
 * @file linker_symbols.h
 * @author SpaceBaker (emile.forcier@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2025-04-26
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef __LINKER_SYMBOLS_H
#define __LINKER_SYMBOLS_H


extern unsigned int __preinit_array_start;
extern unsigned int __preinit_array_end;

extern unsigned int __init_array_start;
extern unsigned int __init_array_end;

extern unsigned int __fini_array_start;
extern unsigned int __fini_array_end;

extern unsigned int __text_start__;
extern unsigned int __text_end__;

extern unsigned int __data_start__;
extern unsigned int __data_end__;

extern unsigned int __bss_start__;
extern unsigned int __bss_end__;

extern unsigned int __heap_start__;
extern unsigned int __heap_end__;
extern unsigned int __HEAP_SIZE;

extern unsigned int __stack_start__;
extern unsigned int __stack_end__;
extern unsigned int __STACK_SIZE;


#endif // __LINKER_SYMBOLS_H
