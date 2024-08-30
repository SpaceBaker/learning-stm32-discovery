/**
 *  This code is specifically made for uart4 of STM32l475
 *
 */

#ifndef UART_H
#define UART_H


#include "stm32l4xx.h"
#include <stdint.h>

typedef enum {
    UART_ID_1 = 0,
    UART_ID_2,
    UART_ID_3,
    UART_ID_4,
    UART_ID_5,
    UART_ID_MAX
} uart_id_t;

typedef enum {
    UART_WORD_LENGTH_7 = 0,
    UART_WORD_LENGTH_8,
    UART_WORD_LENGTH_9
} uart_word_length_t;

typedef enum {
    UART_OVERSAMPLING_16 = 0,
    UART_OVERSAMPLING_8
} uart_oversampling_t;

typedef enum {
    UART_PARITY_DISABLED = 0,
    UART_PARITY_EVEN,
    UART_PARITY_ODD
} uart_parity_t;

typedef enum {
    UART_STOP_BITS_0d5 = 0,
    UART_STOP_BITS_1,
    UART_STOP_BITS_1d5,
    UART_STOP_BITS_2,
} uart_stop_bits_t;

typedef enum {
    UART_MUTE_MODE_DISABLED = 0,
    UART_MUTE_MODE_WAKE_ON_IDLE,
    UART_MUTE_MODE_WAKE_ON_ADDR_MARK
} uart_mute_mode_t;

typedef enum {
    UART_RX_TIMEOUT_DISABLED = 0,
    UART_RX_TIMEOUT_ENABLED
} uart_rx_timeout_t;

typedef enum {
    UART_LSB_FIRST = 0,
    UART_MSB_FIRST
} uart_bit_endianness_t;

typedef union {
    struct {
        uint16_t eob : 1;       /* End of Block interrupt */
        uint16_t rto : 1;       /* Receiver timeout interrupt */
        uint16_t cm : 1;        /* Character match interrupt */
        uint16_t pe : 1;        /* PE interrupt */
        uint16_t txe : 1;       /* Transmitter Enabled interrupt */
        uint16_t tc : 1;        /* Transmission completed interrupt */
        uint16_t rxne : 1;      /* RXNE interrupt */
        uint16_t idle : 1;      /* Idle line interrupt */
        uint16_t linbrk : 1;    /* LIN break detection interrupt */
        uint16_t wake : 1;      /* Wake-up from stop mode interrupt */
        uint16_t cts : 1;       /* CTS interrupt */
        uint16_t error : 1;     /* Error interrupt */
        uint16_t UNUSED : 4;
    } bit;
    uint16_t reg;
} uart_int_en_t;

typedef struct {
    uint32_t baudrate;
    uart_word_length_t word_length;
    uart_oversampling_t oversampling;
    uart_parity_t parity;
    uart_stop_bits_t stop_bits;
    uart_mute_mode_t mute_mode;
    uart_rx_timeout_t rx_timeout;
    uart_bit_endianness_t bit_endianness;
    uart_int_en_t interrupt_enable;
} uart_config_t;

typedef struct {
    const uart_id_t id;
    uart_config_t config;
    struct {
        char * rx;
        char * tx;
    } buffer;
} uart_handle_t;


#define UART4_CONFIG_DEFAULT {\
    .id = UART_ID_4, \
    .config.baudrate = 9600u,\
    .config.word_length = UART_WORD_LENGTH_8,\
    .config.oversampling = UART_OVERSAMPLING_16,\
    .config.parity = UART_PARITY_DISABLED,\
    .config.stop_bits = UART_STOP_BITS_1,\
    .config.mute_mode = UART_MUTE_MODE_DISABLED,\
    .config.rx_timeout = UART_RX_TIMEOUT_DISABLED,\
    .config.bit_endianness = UART_LSB_FIRST,\
    .config.interrupt_enable.reg = 0,\
    .buffer.rx = NULL,\
    .buffer.tx = NULL\
}


/**
 *  Need to setup system/peripheral clock and gpio before calling this init
 */
void uart_init(uart_handle_t * uart_handle);
void uart_enable(uart_handle_t * uart_handle);
void uart_disable(uart_handle_t * uart_handle);
void uart_putchar(uart_handle_t * uart_handle, char c);
void uart_puts(uart_handle_t * uart_handle, char * s);
char uart_getchar(uart_handle_t * uart_handle);


#endif /* UART_H */ 