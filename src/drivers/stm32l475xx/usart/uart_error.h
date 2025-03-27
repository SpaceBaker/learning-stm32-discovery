#ifndef UART_ERROR_H
#define UART_ERROR_H

typedef enum uart_error {
    UART_OK = 0,
    UART_INVALID_PARAMETER,
    UART_INVALID_CONFIG,
    UART_UNKNOWN_ERROR,
} uart_error_t;

#endif /* UART_ERROR_H */ 