#include "stm32l4xx.h"
#include "system_stm32l4xx.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "uart.h"
// #include "common/ringbuffer.h"

/*------------------------- Defines -------------------------------*/
// #define UART_OVER_SAMPLING_8
// #define UART_BUFFER_SIZE_MAX 64

/*------------------------- Global private variables -------------------------------*/
static uint8_t uart_init_status = 0;
// static uint8_t tx_buffer[UART_BUFFER_SIZE_MAX];
// static uint8_t rx_buffer[UART_BUFFER_SIZE_MAX];
// static ringbuffer_t tx_ringbuffer;
// static ringbuffer_t rx_ringbuffer;

/*------------------------- Private functions -------------------------------*/

static inline USART_TypeDef * _uart_get_base_addr(uart_id_t id)
{
    switch (id) {
        case UART_ID_1: return USART1;
        case UART_ID_2: return USART2;
        case UART_ID_3: return USART3;
        case UART_ID_4: return UART4;
        case UART_ID_5: return UART5;
        default:        return NULL;
    }
}

static inline bool _uart_is_busy(USART_TypeDef * usart_reg)
{
    return (usart_reg->ISR & USART_ISR_BUSY);
}

static inline bool _uart_tx_data_reg_empty(USART_TypeDef * usart_reg)
{
    return (usart_reg->ISR & USART_ISR_TXE);
}

static inline bool _uart_tx_complete(USART_TypeDef * usart_reg)
{
    return (usart_reg->ISR & USART_ISR_TC);
}

static inline bool _uart_rx_data_reg_not_empty(USART_TypeDef * usart_reg)
{
    return (usart_reg->ISR & USART_ISR_RXNE);
}

static inline bool _uart_overrun(USART_TypeDef * usart_reg)
{
    return (usart_reg->ISR & USART_ISR_ORE);
}

static inline bool _uart_noise_error(USART_TypeDef * usart_reg)
{
    return (usart_reg->ISR & USART_ISR_NE);
}

static inline bool _uart_framing_error(USART_TypeDef * usart_reg)
{
    return (usart_reg->ISR & USART_ISR_FE);
}

static inline bool _uart_parity_error(USART_TypeDef * usart_reg)
{
    return (usart_reg->ISR & USART_ISR_ORE);
}


/*------------------------- Public functions -------------------------------*/
void uart_init(uart_handle_t * uart_handle)
{
    USART_TypeDef * usart_reg;
    uint32_t brr_value;

    if ((NULL == uart_handle) || 
        (UART_ID_MAX < uart_handle->id) ||
        (uart_init_status & (1 << uart_handle->id))) {
        return;
    }

    usart_reg = _uart_get_base_addr(uart_handle->id);

    /* Reset UART */
    uart_disable(uart_handle);
    switch (uart_handle->id) {
        case UART_ID_1:
            SET_BIT(RCC->APB2RSTR, RCC_APB2RSTR_USART1RST);
            CLEAR_BIT(RCC->APB2RSTR, RCC_APB2RSTR_USART1RST);
            break;
        case UART_ID_2:
            SET_BIT(RCC->APB1RSTR1, RCC_APB1RSTR1_USART2RST);
            CLEAR_BIT(RCC->APB1RSTR1, RCC_APB1RSTR1_USART2RST);
            break;
        case UART_ID_3:
            SET_BIT(RCC->APB1RSTR1, RCC_APB1RSTR1_USART3RST);
            CLEAR_BIT(RCC->APB1RSTR1, RCC_APB1RSTR1_USART3RST);
            break;
        case UART_ID_4:
            SET_BIT(RCC->APB1RSTR1, RCC_APB1RSTR1_UART4RST);
            CLEAR_BIT(RCC->APB1RSTR1, RCC_APB1RSTR1_UART4RST);
            break;
        case UART_ID_5:
            SET_BIT(RCC->APB1RSTR1, RCC_APB1RSTR1_UART5RST);
            CLEAR_BIT(RCC->APB1RSTR1, RCC_APB1RSTR1_UART5RST);
            break;
        default: return;
    }

    /* Word length - 8 data bits (default) */
    switch (uart_handle->config.word_length) {
        case UART_WORD_LENGTH_7:
            CLEAR_BIT(usart_reg->CR1, USART_CR1_M0);
            SET_BIT(usart_reg->CR1, USART_CR1_M1);
            break;
        case UART_WORD_LENGTH_9:
            SET_BIT(usart_reg->CR1, USART_CR1_M0);
            CLEAR_BIT(usart_reg->CR1, USART_CR1_M1);
            break;
        case UART_WORD_LENGTH_8:
        default:
            CLEAR_BIT(usart_reg->CR1, (USART_CR1_M1 | USART_CR1_M0));
            break;
    }

    /* Oversampling mode - 16 (default) */ 
    switch (uart_handle->config.oversampling) {
        case UART_OVERSAMPLING_8:
            SET_BIT(usart_reg->CR1, USART_CR1_OVER8);
            brr_value = 2 * SystemCoreClock / uart_handle->config.baudrate;
            break;
        case UART_OVERSAMPLING_16:
        default:
            CLEAR_BIT(usart_reg->CR1, USART_CR1_OVER8);
            brr_value = SystemCoreClock / uart_handle->config.baudrate;
            break;
    }
    
    /* UART baudrate - 0 (default) */
    usart_reg->BRR = brr_value;

    /* Parity control - disabled (default) */
    switch (uart_handle->config.parity) {
        case UART_PARITY_EVEN:
            SET_BIT(usart_reg->CR1, USART_CR1_PCE);
            CLEAR_BIT(usart_reg->CR1, USART_CR1_PS);
            break;
        case UART_PARITY_ODD:
            SET_BIT(usart_reg->CR1, USART_CR1_PCE);
            SET_BIT(usart_reg->CR1, USART_CR1_PS);
            break;
        case UART_PARITY_DISABLED:
        default:
            CLEAR_BIT(usart_reg->CR1, USART_CR1_PCE);
            break;
    }

    /* Bit endianness - Tx/Rx bit0 first (default) */
    switch (uart_handle->config.bit_endianness) {
        case UART_MSB_FIRST:
            SET_BIT(usart_reg->CR2, USART_CR2_MSBFIRST);
            break;
        case UART_LSB_FIRST:
        default:
            CLEAR_BIT(usart_reg->CR2, USART_CR2_MSBFIRST);
            break;
    }

    /* Logic level inversion - normal (default) */
    // CLEAR_BIT(usart_reg->CR2, USART_CR2_TXINV);
    // CLEAR_BIT(usart_reg->CR2, USART_CR2_RXINV);

    /* STOP bits - 1 (default) */
    switch (uart_handle->config.stop_bits) {
        case UART_STOP_BITS_0d5 :
            MODIFY_REG(usart_reg->CR2, USART_CR2_STOP_Msk, USART_CR2_STOP_0);
            break;
        case UART_STOP_BITS_1d5 :
            SET_BIT(usart_reg->CR2, USART_CR2_STOP);
            break;
        case UART_STOP_BITS_2 :
            MODIFY_REG(usart_reg->CR2, USART_CR2_STOP_Msk, USART_CR2_STOP_1);
            break;
        case UART_STOP_BITS_1 :
        default:
            CLEAR_BIT(usart_reg->CR2, USART_CR2_STOP);
            break;
    }

    /* DMA enable - Tx/Rx disabled (default) */
    // TBC : need to be configured after USART ENABLE ?
    // CLEAR_BIT(usart_reg->CR3, USART_CR3_DMAT);
    // CLEAR_BIT(usart_reg->CR3, USART_CR3_DMAR);

    /* Interrupts enable - disable (default) */
    // Character Match
    MODIFY_REG(usart_reg->CR1, USART_CR1_CMIE_Msk, 
        uart_handle->config.interrupt_enable.bit.character_match << USART_CR1_CMIE_Pos);
    // PE interrupt
    MODIFY_REG(usart_reg->CR1, USART_CR1_PEIE_Msk, 
        uart_handle->config.interrupt_enable.bit.parity_error << USART_CR1_PEIE_Pos);
    // TX interrupt
    MODIFY_REG(usart_reg->CR1, USART_CR1_TXEIE_Msk, 
        uart_handle->config.interrupt_enable.bit.tx_data_reg_empty << USART_CR1_TXEIE_Pos);
    // Transmission complete interrupt
    MODIFY_REG(usart_reg->CR1, USART_CR1_TCIE_Msk, 
        uart_handle->config.interrupt_enable.bit.transmission_complete << USART_CR1_TCIE_Pos);
    // RXNE interrupt
    MODIFY_REG(usart_reg->CR1, USART_CR1_RXNEIE_Msk, 
        uart_handle->config.interrupt_enable.bit.rx_data_reg_not_empty << USART_CR1_RXNEIE_Pos);
    // IDLE interrupt
    MODIFY_REG(usart_reg->CR1, USART_CR1_IDLEIE_Msk, 
        uart_handle->config.interrupt_enable.bit.idle_line_detected << USART_CR1_IDLEIE_Pos);
    // Receiver timeout interrupt (duration programmed in the RTOR)
    // TODO : write timeout value in RTOR and enable Receiver timeout (CR2_RTOEN)
    MODIFY_REG(usart_reg->CR1, USART_CR1_RTOIE_Msk, 
        uart_handle->config.interrupt_enable.bit.rx_timeout << USART_CR1_RTOIE_Pos);
    // Error interrupt
    MODIFY_REG(usart_reg->CR3, USART_CR3_EIE_Msk, 
        uart_handle->config.interrupt_enable.bit.nf_ore_fe << USART_CR3_EIE_Pos);

    // ring_buffer_init(&tx_ringbuffer, tx_buffer, (sizeof(tx_buffer)/sizeof(tx_buffer[0])));
    // ring_buffer_init(&rx_ringbuffer, rx_buffer, (sizeof(rx_buffer)/sizeof(rx_buffer[0])));

    uart_init_status |= 1 << uart_handle->id;
}

void uart_enable(uart_handle_t * uart_handle)
{
    USART_TypeDef * usart_reg;

    if (uart_handle == NULL) {
        return;
    }

    usart_reg = _uart_get_base_addr(uart_handle->id);

    SET_BIT(usart_reg->CR1, (USART_CR1_TE | USART_CR1_RE | USART_CR1_UE));
}

void uart_disable(uart_handle_t * uart_handle)
{
    USART_TypeDef * usart_reg;

    if (uart_handle == NULL) {
        return;
    }

    usart_reg = _uart_get_base_addr(uart_handle->id);

    /* Disable Tx and Rx */
    CLEAR_BIT(usart_reg->CR1, (USART_CR1_TE | USART_CR1_RE));

    /* Wait for Transmission Complete bit */
    while (!(usart_reg->ISR & USART_ISR_TC));

    // TODO : DMA channel must be disabled before resetting the UE bit

    /* Disable UART */
    CLEAR_BIT(usart_reg->CR1, USART_CR1_UE);
}

void uart_putchar(uart_handle_t * uart_handle, char c)
{
    USART_TypeDef * usart_reg;

    if (uart_handle == NULL) {
        return;
    }

    usart_reg = _uart_get_base_addr(uart_handle->id);

    while (!_uart_tx_data_reg_empty(usart_reg));
    usart_reg->TDR = c;
}

void uart_puts(uart_handle_t * uart_handle, char * s)
{
    if ((uart_handle == NULL) || (NULL == s)) {
        return;
    }

    while (*s != '\0') {
        uart_putchar(uart_handle, *s);
        s++;
    }
}

char uart_getchar(uart_handle_t * uart_handle)
{
    char c;
    USART_TypeDef * usart_reg;

    if (uart_handle == NULL) {
        return UINT8_MAX;
    }

    usart_reg = _uart_get_base_addr(uart_handle->id);

    while (!_uart_rx_data_reg_not_empty(usart_reg));
    c = usart_reg->RDR;

    if (uart_handle->config.echo) {
        uart_putchar(uart_handle, c);
    }

    return c;
}

uint16_t uart_gets(uart_handle_t * uart_handle, char * buffer, uint16_t length)
{
    uint16_t count = 0;

    if ((uart_handle == NULL) || (buffer == NULL) || (length == 0)) {
        return 0;
    }

    do {
        buffer[count] = uart_getchar(uart_handle);
        if ((buffer[count] == '\n') || (buffer[count] == '\r')) {
            count++;
            break;
        }
        count++;
    } while (count < (length-1));
    buffer[count] = 0;  // NUL terminating string

    if (uart_handle->config.echo) {
        uart_puts(uart_handle, "\r\n");
    }

    return count;
}