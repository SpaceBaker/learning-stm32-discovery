#include "stm32l4xx.h"
#include "system_stm32l4xx.h"
#include <stdbool.h>
#include <stddef.h>

#include "uart.h"
// #include "common/ringbuffer.h"

/*------------------------- Defines -------------------------------*/
// #define UART_OVER_SAMPLING_8
// #define UART_BUFFER_SIZE_MAX 64

/*------------------------- Global private variables -------------------------------*/
// static uint8_t tx_buffer[UART_BUFFER_SIZE_MAX];
// static uint8_t rx_buffer[UART_BUFFER_SIZE_MAX];
// static ringbuffer_t tx_ringbuffer;
// static ringbuffer_t rx_ringbuffer;

/*------------------------- Private functions -------------------------------*/
static inline bool _uart_is_busy(void)
{
    return (UART4->ISR & USART_ISR_BUSY);
}

static inline bool _uart_tx_data_reg_empty(void)
{
    return (UART4->ISR & USART_ISR_TXE);
}

static inline bool _uart_tx_complete(void)
{
    return (UART4->ISR & USART_ISR_TC);
}

static inline bool _uart_rx_data_reg_not_empty(void)
{
    return (UART4->ISR & USART_ISR_RXNE);
}

static inline bool _uart_overrun(void)
{
    return (UART4->ISR & USART_ISR_ORE);
}

static inline bool _uart_noise_error(void)
{
    return (UART4->ISR & USART_ISR_NE);
}

static inline bool _uart_framing_error(void)
{
    return (UART4->ISR & USART_ISR_FE);
}

static inline bool _uart_parity_error(void)
{
    return (UART4->ISR & USART_ISR_ORE);
}


/*------------------------- Public functions -------------------------------*/
uint8_t uart_init(uint32_t baudrate)
{
    uint32_t brr_value;

    /* Reset UART */
    uart_disable();
    SET_BIT(RCC->APB1RSTR1, RCC_APB1RSTR1_UART4RST);
    CLEAR_BIT(RCC->APB1RSTR1, RCC_APB1RSTR1_UART4RST);

    /* Word length - 8 data bits */
    CLEAR_BIT(UART4->CR1, (USART_CR1_M1 | USART_CR1_M0));

    /* Oversampling mode - 16 (default) */
    #ifdef UART_OVER_SAMPLING_8
    SET_BIT(UART4->CR1, USART_CR1_OVER8);
    #endif

    /* Parity control - disabled (default) */
    // CLEAR_BIT(UART4->CR1, USART_CR1_PCE);
    /* Parity selection - even (default) */
    // CLEAR_BIT(UART4->CR1, USART_CR1_PS);

    /* Interrupts enable - disable (default) */
    // PE interrupt
    // TX interrupt
    // Transmission complete interrupt
    // RXNE interrupt
    // IDLE interrupt
    // Receiver timeout interrupt (duration programmed in the RTOR)
    // Error interrupt

    /* Bit endianness - Tx/Rx bit0 first (default) */
    // CLEAR_BIT(UART4->CR2, USART_CR2_MSBFIRST);

    /* Logic level inversion - normal (default) */
    // CLEAR_BIT(UART4->CR2, USART_CR2_TXINV);
    // CLEAR_BIT(UART4->CR2, USART_CR2_RXINV);

    /* STOP bits - 1 (default) */
    // CLEAR_BIT(UART4->CR2, USART_CR2_STOP);

    /* DMA enable - Tx/Rx disabled (default) */
    // CLEAR_BIT(UART4->CR3, USART_CR3_DMAT);
    // CLEAR_BIT(UART4->CR3, USART_CR3_DMAR);

    /* UART baudrate - 0 (default) */
    #ifdef UART_OVER_SAMPLING_8
    brr_value = 2 * f_clk_hz / baudrate;
    MODIFY_REG(brr_value, USART_BRR_DIV_FRACTION_Msk, ((brr_value & USART_BRR_DIV_FRACTION_Msk) >> 1));    
    #else
    brr_value = SystemCoreClock / baudrate;
    #endif
    if (brr_value > 0xFFFF) {
        return -1;
    }
    UART4->BRR = brr_value;



    // ring_buffer_init(&tx_ringbuffer, tx_buffer, (sizeof(tx_buffer)/sizeof(tx_buffer[0])));
    // ring_buffer_init(&rx_ringbuffer, rx_buffer, (sizeof(rx_buffer)/sizeof(rx_buffer[0])));

    return 0;
}

void uart_enable(void)
{
    SET_BIT(UART4->CR1, (USART_CR1_TE | USART_CR1_RE | USART_CR1_UE));
}

void uart_disable(void)
{
    /* Disable Tx and Rx */
    CLEAR_BIT(UART4->CR1, (USART_CR1_TE | USART_CR1_RE));

    /* Wait for Transmission Complete bit */
    while (!(UART4->ISR & USART_ISR_TC));

    // TODO : DMA channel must be disabled before resetting the UE bit

    /* Disable UART */
    CLEAR_BIT(UART4->CR1, USART_CR1_UE);
}

void uart_putchar(char c)
{
    while (!_uart_tx_data_reg_empty());
    UART4->TDR = c;
}

void uart_puts(char * s)
{
    if (NULL == s){
        return;
    }

    while(*s != '\0') {
        uart_putchar(*s);
        s++;
    }
}

char uart_getchar(void)
{
    char c;

    while (!_uart_rx_data_reg_not_empty());
    c = UART4->RDR;
    return c;
}