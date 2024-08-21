/**
 *  This code is specifically made for uart4 of STM32l475
 *
 */

#ifndef UART_H
#define UART_H


#include <stdint.h>

/**
 *  Need to setup system/peripheral clock and gpio before calling this init
 */
uint8_t uart_init(uint32_t baudrate);
void uart_enable(void);
void uart_disable(void);
void uart_putchar(uint8_t data);
uint8_t uart_getchar(void);


#endif /* UART_H */