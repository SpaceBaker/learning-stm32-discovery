/**
 *  This code is specifically made for uart4 of STM32l475
 *
 */

#ifndef UART_H
#define UART_H


#include "stm32l4xx.h"   // IWYU pragma: export
#include <stdint.h>
#include <stdbool.h>


#define UART_BUFFER_LENGTH 64
// #define USART1_INT_MODE_ENABLE
// #define USART2_INT_MODE_ENABLE
// #define USART3_INT_MODE_ENABLE
#define UART4_INT_MODE_ENABLE
// #define UART5_INT_MODE_ENABLE

typedef enum {
    UART_WORD_LENGTH_7 = USART_CR1_M1,
    UART_WORD_LENGTH_8 = 0,
    UART_WORD_LENGTH_9 = USART_CR1_M0
} uart_word_length_t;

typedef enum {
    UART_OVERSAMPLING_16 = 0,
    UART_OVERSAMPLING_8  = USART_CR1_OVER8
} uart_oversampling_t;

typedef enum {
    UART_PARITY_DISABLED = 0,
    UART_PARITY_EVEN     = USART_CR1_PCE,
    UART_PARITY_ODD      = (USART_CR1_PCE | USART_CR1_PS)
} uart_parity_t;

typedef enum {
    UART_STOP_BITS_0d5 = USART_CR2_STOP_0,
    UART_STOP_BITS_1   = 0,
    UART_STOP_BITS_1d5 = USART_CR2_STOP,
    UART_STOP_BITS_2   = USART_CR2_STOP_1,
} uart_stop_bits_t;

typedef enum {
    UART_RX_TIMEOUT_DISABLED = 0,
    UART_RX_TIMEOUT_ENABLED
} uart_rx_timeout_t;

typedef enum {
    UART_LSB_FIRST = 0,
    UART_MSB_FIRST = USART_CR2_MSBFIRST
} uart_bit_endianness_t;

typedef enum {
    UART_RX     = USART_CR1_RE,
    UART_TX     = USART_CR1_TE,
    UART_TX_RX  = (USART_CR1_RE | USART_CR1_TE),
} uart_direction_t;


typedef struct {
    uint32_t baudrate;
    uart_word_length_t word_length;
    uart_oversampling_t oversampling;
    uart_parity_t parity;
    uart_stop_bits_t stop_bits;
    uart_rx_timeout_t rx_timeout;
    uart_bit_endianness_t bit_endianness;
    uart_direction_t direction;
} uart_config_t;


#define UART4_CONFIG_DEFAULT { \
    .baudrate = 115200u,\
    .word_length = UART_WORD_LENGTH_8,\
    .oversampling = UART_OVERSAMPLING_16,\
    .parity = UART_PARITY_DISABLED,\
    .stop_bits = UART_STOP_BITS_1,\
    .rx_timeout = UART_RX_TIMEOUT_DISABLED,\
    .bit_endianness = UART_LSB_FIRST, \
    .direction = UART_TX_RX \
}


/**
 *  Need to setup system/peripheral clock and gpio before calling this init
 */
void uart_init(USART_TypeDef * self, uart_config_t config);
void uart_deinit(USART_TypeDef * self);
void uart_enable(USART_TypeDef * self);
void uart_disable(USART_TypeDef * self);
void uart_putchar(USART_TypeDef * self, char c);
void uart_puts(USART_TypeDef * self, char * s);
char uart_getchar(USART_TypeDef * self);
uint16_t uart_gets(USART_TypeDef * self, char * buffer, uint16_t length);
void uart_send(USART_TypeDef * self, char * buffer, uint16_t length);
void uart_listen(USART_TypeDef * self);
uint8_t uart_msgReceived(USART_TypeDef * self);


#endif /* UART_H */ 