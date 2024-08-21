/**
 *  This code is specifically made for uart4 of STM32l475
 *
 */

#ifndef UART_H
#define UART_H


#include <stdint.h>
#include <stddef.h>

/**
 *  Need to setup system/peripheral clock and gpio before calling this init
 */
uint8_t uart_init(uint32_t baudrate);
void uart_enable(void);
void uart_disable(void);
void uart_putchar(uint8_t data);
uint8_t uart_getchar(void);
void uart_transmit(char * tx_buf, size_t size);


#endif /* UART_H */