/**
 *  This code is specifically made for uart4 of STM32l475
 *
 */

#ifndef UART_H
#define UART_H


#include "stm32l4xx.h"
#include <stdint.h>
#include <stdbool.h>

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
        uint16_t tx_data_reg_empty;             // TXEIE
        uint16_t clear_to_send;                 // CTSIE
        uint16_t transmission_complete;         // TCIE
        uint16_t rx_data_reg_not_empty;         // RXNEIE
        uint16_t overrun_error;                 // RXNEIE
        uint16_t idle_line_detected;            // IDLEIE
        uint16_t parity_error;                  // PEIE
        uint16_t lin_break;                     // LBDIE
        uint16_t nf_ore_fe;                     // EIE
        uint16_t character_match;               // CMIE
        uint16_t rx_timeout;                    // RTOIE
        uint16_t rx_timeoutend_of_block;        // EOBIE
        uint16_t wake_from_stop;                // WUFIE
        uint16_t tx_complete_before_guard_time; //TCBGTIE
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
    bool echo;
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
    .config.echo = true,\
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
uint16_t uart_gets(uart_handle_t * uart_handle, char * buffer, uint16_t length);


#endif /* UART_H */ 