/**
 *  This code is specifically made for uart4 of STM32l475
 *
 */

#ifndef UART_H
#define UART_H


#include "../stm32l4xx.h"   // IWYU pragma: keep
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "uart_error.h"
#include "../dma/dma.h"
#include "common/ringbuffer.h"


#define UART_BUFFER_LENGTH 64


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
    UART_PARITY_ODD      = (USART_CR1_PCE | USART_CR1_PS),
    UART_PARITY_Msk      = (USART_CR1_PCE | USART_CR1_PS)
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

typedef struct _uart_handler_t uart_handler_t;


#define UART_CONFIG_DEFAULT { \
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

/**
 * @brief Initialize the uart peripheral
 * 
 * @param[in] self      The handler to initialize (required)
 * @param[in] config    The configuration desired (required)
 * @param[in] rb_tx     The TX ringbuffer (optional pass NULL if not used, otherwise required if using tx interrupt mode)
 * @param[in] rb_rx     The RX ringbuffer (optional pass NULL if not used, otherwise required if using rx interrupt mode)
 * @param[in] dma_tx    The DMA tx (optional pass NULL if not used, otherwise required if using DMA mode)
 * @param[in] dma_rx    The DMA rx (optional pass NULL if not used, otherwise required if using DMA mode)
 *
 * @pre rb_tx, rb_rx, dma_tx and dma_rx need to be initialzied
 * @result The UART registers are initialized according to the provided configuration and the peripheral is enabled
 * @note It is not required to enable the peripheral as this function already enables it
*/
uart_handler_t * uart_init(USART_TypeDef * usartx, const uart_config_t config, ringbuffer_t * rb_tx, ringbuffer_t * rb_rx, dma_handler_t * dma_tx, dma_handler_t * dma_rx);
uart_error_t uart_deinit(uart_handler_t * self);
uart_error_t uart_enable(uart_handler_t * self);
uart_error_t uart_disable(uart_handler_t * self);
uart_error_t uart_putchar(uart_handler_t * self, char c);
uart_error_t uart_puts(uart_handler_t * self, char * s);
char uart_getchar(uart_handler_t * self);
uint16_t uart_gets(uart_handler_t * self, char * buffer, uint16_t length);
uart_error_t uart_send(uart_handler_t * self, char * buffer, uint16_t length);
uart_error_t uart_send_dma(uart_handler_t * self, const char * buffer, const uint16_t length);
uart_error_t uart_listen(uart_handler_t * self);
uart_error_t uart_listen_dma(uart_handler_t * self, char * buffer, const uint16_t length);
uint16_t uart_read(uart_handler_t * self, char * buffer, const uint16_t buffer_size);


#endif /* UART_H */ 