#include "logger.h"
#include "bsp/gpiomap.h"
#include "usart/uart.h"
#include "dma/dma.h"


// ringbuffer_t uartLoggerTxRingbuffer;
uint8_t uartLoggerTxbuffer[UART_BUFFER_LENGTH];
dma_handler_t * dmaHandler;
uart_handler_t * uartHandler;


void logger_init(void)
{
    uart_config_t uartConfig = UART_CONFIG_DEFAULT;
	// ringbuffer_init(&uartLoggerTxRingbuffer, uartLoggerTxbuffer, (sizeof(uartLoggerTxbuffer)/sizeof(uartLoggerTxbuffer[0])));
    // uartHandler = uart_init(LOGGER_UART, config, &uartLoggerTxRingbuffer, NULL, NULL, NULL);
    dma_config_t dmaConfig = {
        .request = DMA_REQUEST_2,
        .sw_priority = DMA_SW_PRIORITY_LOW,
        .data_size = DMA_DATA_SIZE_8b,
        .xfer_mode = DMA_XFER_MODE_NORMAL,
        .direction = DMA_DIRECTION_MEM_TO_PERIPH,
        .memAddrAutoInc = true,
        .periphAddrAutoInc = false
    };
    dmaHandler = dma_init(DMA2_Channel3, dmaConfig);
    if (NULL == dmaHandler) {
        __BKPT();
    }
    uartHandler = uart_init(LOGGER_UART, uartConfig, NULL, NULL, dmaHandler, NULL);
    if (NULL == uartHandler) {
        __BKPT();
    }
}

void logger_write(const char * str, const uint16_t length)
{
    if (NULL == uartHandler) {
        __BKPT();
    }
    // uart_puts(uartLogger, uart_blocking_msg);
    // uart_send(uartHandler, str, length);
    uart_send_dma(uartHandler, str, length);
}