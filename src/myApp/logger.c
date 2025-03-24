#include "logger.h"
#include "bsp/gpiomap.h"
#include "usart/uart.h"


ringbuffer_t uartLoggerTxRingbuffer;
uint8_t uartLoggerTxbuffer[UART_BUFFER_LENGTH];
uart_handler_t * uartHandler;

void logger_init(void)
{
    uart_config_t config = UART_CONFIG_DEFAULT;
	ringbuffer_init(&uartLoggerTxRingbuffer, uartLoggerTxbuffer, (sizeof(uartLoggerTxbuffer)/sizeof(uartLoggerTxbuffer[0])));
    uartHandler = uart_init(LOGGER_UART, config, &uartLoggerTxRingbuffer, NULL, NULL, NULL);
    if (NULL == uartHandler) {
        __BKPT();
    }
}

void logger_write(char * str, uint16_t length)
{
    if (NULL == uartHandler) {
        __BKPT();
    }
    uart_send(uartHandler, str, length);
    // uart_puts(uartLogger, uart_blocking_msg);
}