#include "uart.h"
#include <stddef.h>
#include <stdint.h>
#include "common/ringbuffer.h"
#include "stm32l4xx.h"

/*------------------------- Defines -------------------------------*/
#define UART_ISR_ERROR_MASK (USART_ISR_PE | USART_ISR_FE | USART_ISR_ORE | USART_ISR_NE | USART_ISR_RTOF)

/*------------------------- Global private variables -------------------------------*/
uint8_t uart4_tx_buffer[UART_BUFFER_LENGTH] = {0};
ringbuffer_t uart4_ringbuffer_rx;
uint8_t uart4_rx_buffer[UART_BUFFER_LENGTH] = {0};
ringbuffer_t uart4_ringbuffer_tx;

/*------------------------- Private functions -------------------------------*/
__attribute__((unused)) static inline bool _uart_is_busy(USART_TypeDef * usart_reg)
{
    return (usart_reg->ISR & USART_ISR_BUSY);
}

__attribute__((unused)) static inline bool _uart_is_idle(USART_TypeDef * usart_reg)
{
    return (usart_reg->ISR & USART_ISR_IDLE);
}

__attribute__((unused)) static inline bool _uart_tx_data_reg_empty(USART_TypeDef * usart_reg)
{
    return (usart_reg->ISR & USART_ISR_TXE);
}

__attribute__((unused)) static inline bool _uart_tx_complete(USART_TypeDef * usart_reg)
{
    return (usart_reg->ISR & USART_ISR_TC);
}

__attribute__((unused)) static inline bool _uart_rx_data_reg_not_empty(USART_TypeDef * usart_reg)
{
    return (usart_reg->ISR & USART_ISR_RXNE);
}

__attribute__((unused)) static inline bool _uart_overrun(USART_TypeDef * usart_reg)
{
    return (usart_reg->ISR & USART_ISR_ORE);
}

__attribute__((unused)) static inline bool _uart_noise_error(USART_TypeDef * usart_reg)
{
    return (usart_reg->ISR & USART_ISR_NE);
}

__attribute__((unused)) static inline bool _uart_framing_error(USART_TypeDef * usart_reg)
{
    return (usart_reg->ISR & USART_ISR_FE);
}

__attribute__((unused)) static inline bool _uart_parity_error(USART_TypeDef * usart_reg)
{
    return (usart_reg->ISR & USART_ISR_ORE);
}

__attribute__((unused)) static inline bool _uart_reset(USART_TypeDef * self)
{
    if (USART1 == self) {
        ATOMIC_SET_BIT(RCC->APB2RSTR, RCC_APB2RSTR_USART1RST);
        ATOMIC_CLEAR_BIT(RCC->APB2RSTR, RCC_APB2RSTR_USART1RST);
    }
    else if (USART2 == self) {
        ATOMIC_SET_BIT(RCC->APB1RSTR1, RCC_APB1RSTR1_USART2RST);
        ATOMIC_CLEAR_BIT(RCC->APB1RSTR1, RCC_APB1RSTR1_USART2RST);
    }
    else if (USART3 == self) {
        ATOMIC_SET_BIT(RCC->APB1RSTR1, RCC_APB1RSTR1_USART3RST);
        ATOMIC_CLEAR_BIT(RCC->APB1RSTR1, RCC_APB1RSTR1_USART3RST);
    }
    else if (UART4 == self) {
        ATOMIC_SET_BIT(RCC->APB1RSTR1, RCC_APB1RSTR1_UART4RST);
        ATOMIC_CLEAR_BIT(RCC->APB1RSTR1, RCC_APB1RSTR1_UART4RST);
    }
    else if (UART5 == self) {
        ATOMIC_SET_BIT(RCC->APB1RSTR1, RCC_APB1RSTR1_UART5RST);
        ATOMIC_CLEAR_BIT(RCC->APB1RSTR1, RCC_APB1RSTR1_UART5RST);
    }
    else {
        return true;
    }

    return false;
}

static inline uint16_t _uart_baudrateToBrrValue(int32_t periphClk, uart_oversampling_t oversampling, uint32_t baudrate)
{
    uint32_t brrtemp;
    
    if (oversampling == UART_OVERSAMPLING_8) {
        uint32_t usartdiv;
        usartdiv = (uint16_t)(((periphClk * 2) + (baudrate / 2)) / baudrate);
        brrtemp = usartdiv & 0xFFF0U;
        brrtemp |= (uint16_t)((usartdiv & (uint16_t)0x000FU) >> 1U);
    }
    else {
        brrtemp = (uint16_t)((periphClk + (baudrate / 2)) / baudrate);
    }

    return brrtemp;
}

// TODO : timeout
static void _uart_waitReady(USART_TypeDef * self)
{
    if ((self->CR1 & USART_CR1_TE) == USART_CR1_TE) {
        while ((self->ISR & USART_ISR_TEACK) != USART_ISR_TEACK);
        // if timeout occured, clear interrupts
        // ATOMIC_CLEAR_BIT(self->CR1, (USART_CR1_TXEIE));
    }

    if ((self->CR1 & USART_CR1_RE) == USART_CR1_RE) {
        while ((self->ISR & USART_ISR_REACK) != USART_ISR_REACK) {
            // TODO : check Overrun and Receiver Timeout flag
        }
        // if timeout occured, clear interrupts
        // ATOMIC_CLEAR_BIT(self->CR1, (USART_CR1_RXNEIE | USART_CR1_PEIE));
        // ATOMIC_CLEAR_BIT(self->CR3, USART_CR3_EIE);
    }
}

static inline IRQn_Type _uart_getIRQn(USART_TypeDef * self)
{
    switch ((unsigned long)self) {
        case (unsigned long)USART1: return USART1_IRQn;
        case (unsigned long)USART2: return USART2_IRQn;
        case (unsigned long)USART3: return USART3_IRQn;
        case (unsigned long)UART4:  return UART4_IRQn;
        case (unsigned long)UART5:  return UART5_IRQn;
        default: {
            __BKPT();
            return USART1_IRQn;
        }
    }
}

static inline ringbuffer_t * _uart_getRingbufferTx(USART_TypeDef * self)
{    
    switch ((unsigned long)self) {
        case (unsigned long)USART1: {
            #ifdef UASRT1_INT_MODE_ENABLE
            return &usart1_ringbuffer_tx;
            #else
            return NULL;
            #endif
        }
        case (unsigned long)USART2: {
            #ifdef USART2_INT_MODE_ENABLE
            return &usart2_ringbuffer_tx;
            #else
            return NULL;
            #endif
        }
        case (unsigned long)USART3: {
            #ifdef USART3_INT_MODE_ENABLE
            return &usart3_ringbuffer_tx;
            #else
            return NULL;
            #endif
        }
        case (unsigned long)UART4:  {
            #ifdef UART4_INT_MODE_ENABLE
            return &uart4_ringbuffer_tx;
            #else
            return NULL;
            #endif
        }
        case (unsigned long)UART5:  {
            #ifdef UART5_INT_MODE_ENABLE
            return &uart5_ringbuffer_tx;
            #else
            return NULL;
            #endif
        }
        default: {
            __BKPT();
            return NULL;
        }
    }
}

static inline ringbuffer_t * _uart_getRingbufferRx(USART_TypeDef * self)
{    
    switch ((unsigned long)self) {
        case (unsigned long)USART1: {
            #ifdef UASRT1_INT_MODE_ENABLE
            return &usart1_ringbuffer_rx;
            #else
            return NULL;
            #endif
        }
        case (unsigned long)USART2: {
            #ifdef USART2_INT_MODE_ENABLE
            return &usart2_ringbuffer_rx;
            #else
            return NULL;
            #endif
        }
        case (unsigned long)USART3: {
            #ifdef USART3_INT_MODE_ENABLE
            return &usart3_ringbuffer_rx;
            #else
            return NULL;
            #endif
        }
        case (unsigned long)UART4:  {
            #ifdef UART4_INT_MODE_ENABLE
            return &uart4_ringbuffer_rx;
            #else
            return NULL;
            #endif
        }
        case (unsigned long)UART5:  {
            #ifdef UART5_INT_MODE_ENABLE
            return &uart5_ringbuffer_rx;
            #else
            return NULL;
            #endif
        }
        default: {
            __BKPT();
            return NULL;
        }
    }
}


/*------------------------- Public functions -------------------------------*/
void uart_init(USART_TypeDef * self, uart_config_t config)
{
    if (NULL == self) {
        return;
    }

    // Disable and reset usart
    uart_deinit(self);

    /*------------- USARTx CR1 Configuration -------------*/
    /* Word length - 8 data bits (default) */
    MODIFY_REG(self->CR1, USART_CR1_M_Msk, config.word_length);
    /* Parity control - disabled (default) */
    MODIFY_REG(self->CR1, (USART_CR1_PS_Msk | USART_CR1_PCE_Msk), config.parity);
    /* Direction Tx and/or Rx - disabled (default) */
    MODIFY_REG(self->CR1, (USART_CR1_TE_Msk | USART_CR1_RE_Msk), config.direction);
    /* Oversampling mode - 16 (default) */
    MODIFY_REG(self->CR1, USART_CR1_OVER8_Msk, config.oversampling);

    /*------------- USARTx CR2 Configuration -------------*/
    /* STOP bits - 1 (default) */
    MODIFY_REG(self->CR2, (USART_CR2_STOP_Msk), config.bit_endianness);
    /* Bit endianness - Tx/Rx bit0 first (default) */
    MODIFY_REG(self->CR2, (USART_CR2_MSBFIRST_Msk), config.bit_endianness);

    /*------------- USARTx CR3 Configuration -------------*/
    /* Hardware Flow Controle - none (default) */
    // MODIFY_REG(self->CR3, (USART_CR3_CTSE_Msk | USART_CR3_RTSE_Msk), config.flow_control);
    
    /*------------- USARTx BRR Configuration -------------*/
    /* UART baudrate - 0 (default) */
    // TODO : get USARTx periph clock_source freq
    self->BRR = _uart_baudrateToBrrValue(SystemCoreClock, config.oversampling, config.baudrate);

    ringbuffer_init(&uart4_ringbuffer_rx, uart4_rx_buffer, sizeof(uart4_rx_buffer)/sizeof(uart4_rx_buffer[0]));
    ringbuffer_init(&uart4_ringbuffer_tx, uart4_tx_buffer, sizeof(uart4_tx_buffer)/sizeof(uart4_tx_buffer[0]));

    // In asynchronous mode, the following bits must be kept cleared:
    ATOMIC_CLEAR_BIT(self->CR2, (USART_CR2_LINEN | USART_CR2_CLKEN));
    ATOMIC_CLEAR_BIT(self->CR3, (USART_CR3_SCEN | USART_CR3_HDSEL | USART_CR3_IREN));

    uart_enable(self);

}

void uart_deinit(USART_TypeDef * self)
{
    if (NULL == self) {
        return;
    }

    uart_disable(self);

    _uart_reset(self);
}

void uart_enable(USART_TypeDef * self)
{
    if (self == NULL) {
        return;
    }

    ATOMIC_SET_BIT(self->CR1, USART_CR1_UE);
    _uart_waitReady(self);
}

void uart_disable(USART_TypeDef * self)
{
    if (self == NULL) {
        return;
    }

    /* Disable Tx and Rx */
    ATOMIC_CLEAR_BIT(self->CR1, (USART_CR1_TE | USART_CR1_RE));

    /* Wait for Transmission Complete bit */
    while (!(self->ISR & USART_ISR_TC)){};

    // TODO : DMA channel must be disabled before resetting the UE bit

    // Disable UART irq
    IRQn_Type irqn = _uart_getIRQn(self);
    NVIC_ClearPendingIRQ(irqn);
    NVIC_DisableIRQ(irqn);

    /* Disable UART */
    ATOMIC_CLEAR_BIT(self->CR1, USART_CR1_UE);
}

void uart_putchar(USART_TypeDef * self, char c)
{
    if (self == NULL) {
        return;
    }

    while (!_uart_tx_data_reg_empty(self)){};
    self->TDR = c;
}

void uart_puts(USART_TypeDef * self, char * s)
{
    if ((self == NULL) || (NULL == s)) {
        return;
    }

    IRQn_Type irqn = _uart_getIRQn(self);
    NVIC_DisableIRQ(irqn);
    while (*s != '\0') {
        uart_putchar(self, *s);
        s++;
    }
    NVIC_EnableIRQ(irqn);
}

char uart_getchar(USART_TypeDef * self)
{
    char c;

    if (self == NULL) {
        return UINT8_MAX;
    }

    while (!_uart_rx_data_reg_not_empty(self)){};
    c = self->RDR;

    return c;
}

uint16_t uart_gets(USART_TypeDef * self, char * buffer, uint16_t length)
{
    uint16_t count = 0;

    if ((self == NULL) || (buffer == NULL) || (length == 0)) {
        return 0;
    }

    do {
        buffer[count] = uart_getchar(self);
        if ((buffer[count] == '\n') || (buffer[count] == '\r')) {
            count++;
            break;
        }
        count++;
    } while (count < (length-1));
    buffer[count] = 0;  // NUL terminating string

    return count;
}

void uart_send(USART_TypeDef * self, char * buffer, uint16_t length)
{
    if ((self == NULL) || (buffer == NULL) || (length == 0)) {
        return;
    }

    // TODO : Check for ongoing transmission (and handle it if true)

    IRQn_Type irqn = _uart_getIRQn(self);
    ringbuffer_t * buffPtr = _uart_getRingbufferTx(self);

    // TODO : Is it necessary to protect buffer memcpy ?
    // memcpy
    NVIC_DisableIRQ(irqn);
    for (uint8_t i = 0; i < (length*sizeof(buffer[0])); i++) {
        ringbuffer_put(buffPtr, buffer[i]);
    }
    NVIC_EnableIRQ(irqn);

    // Enable TX interrupt
    ATOMIC_SET_BIT(self->CR1, USART_CR1_TXEIE);
}

void uart_listen(USART_TypeDef * self)
{
    if (self == NULL) {
        return;
    }

    IRQn_Type irqn = _uart_getIRQn(self);

    ATOMIC_SET_BIT(self->CR3, USART_CR3_EIE);
    // Enable RX interrupt
    ATOMIC_SET_BIT(self->CR1, USART_CR1_RXNEIE);
    NVIC_EnableIRQ(irqn);
}

uint8_t uart_msgReceived(USART_TypeDef * self)
{
    if (self == NULL) {
        return 0;
    }

    ringbuffer_t * buffPtr = _uart_getRingbufferRx(self);

    return !ringbuffer_isEmpty(buffPtr);
}

static void _rxIsr(USART_TypeDef * self, ringbuffer_t * rb)
{
    uint8_t rb_error = ringbuffer_put(rb, self->RDR);
    if (rb_error != 0) {
        // TODO : Handling of ringbuffer full
        ATOMIC_CLEAR_BIT(self->CR1, USART_CR1_RXNEIE);
    }
}

static void _txIsr(USART_TypeDef * self, ringbuffer_t * rb)
{
    if (ringbuffer_isEmpty(rb)) {
        // Disable the TX interrupt
        ATOMIC_CLEAR_BIT(self->CR1, USART_CR1_TXEIE);
        // Enable the UART Transmit Complete Interrupt
        ATOMIC_SET_BIT(self->CR1, USART_CR1_TCIE);
    }
    else {
        self->TDR = ringbuffer_get(rb);
    }
}

static void _errorIsr(USART_TypeDef * self, uint32_t cr1RegCpy, uint32_t cr3RegCpy, uint32_t isrRegCpy)
{
    /* UART parity error interrupt occurred */
    if ((isrRegCpy & USART_ISR_PE) && (cr1RegCpy & USART_CR1_PEIE))
    {
        // TODO : error handling
        ATOMIC_SET_BIT(self->ICR, USART_ICR_PECF);
    }

    /* UART frame error interrupt occurred */
    if ((isrRegCpy & USART_ISR_FE) && (cr3RegCpy & USART_CR3_EIE))
    {
        // TODO : error handling
        ATOMIC_SET_BIT(self->ICR, USART_ICR_FECF);
    }

    /* UART noise error interrupt occurred */
    if ((isrRegCpy & USART_ISR_NE) && (cr3RegCpy & USART_CR3_EIE))
    {
        // TODO : error handling
        ATOMIC_SET_BIT(self->ICR, USART_ICR_NECF);
    }

    /* UART Overrun interrupt occurred */
    if ((isrRegCpy & USART_ISR_ORE) && (
        (cr1RegCpy & USART_CR1_RXNEIE) || (cr3RegCpy & USART_CR3_EIE)))
    {
        // TODO : error handling
        ATOMIC_SET_BIT(self->ICR, USART_ICR_ORECF);
    }

    /* UART Receiver Timeout interrupt occurred */
    if ((isrRegCpy & USART_ISR_RTOF) && (cr1RegCpy & USART_CR1_RTOIE))
    {
        // TODO : error handling
        ATOMIC_SET_BIT(self->ICR, USART_ICR_RTOCF);
    }
}

void UART4_IRQHandler(void)
{
    uint32_t isrFlags = READ_REG(UART4->ISR);
    uint32_t cr1      = READ_REG(UART4->CR1);
    uint32_t cr3      = READ_REG(UART4->CR3);


    /* UART RECEIVE INTERRUPT HANDLING */
    if ((0 == (isrFlags & UART_ISR_ERROR_MASK)) && (cr1 & USART_CR1_RXNEIE) && (isrFlags & USART_ISR_RXNE)) {
        _rxIsr(UART4, &uart4_ringbuffer_tx);
        return;
    }

    /* UART ERROR INTERRUPT HANDLING */
    if ((0 != (isrFlags & UART_ISR_ERROR_MASK)) && ((cr3 & USART_CR3_EIE) || (cr1 & (USART_CR1_RXNEIE | USART_CR1_PEIE | USART_CR1_RTOIE)))) {
        // Handling of errors
        _errorIsr(UART4, cr1, cr3, isrFlags);
        // Handling of RX with error if RX Data Register Not Empty
        if ((isrFlags & USART_ISR_RXNE) && (cr1 & USART_CR1_RXNEIE)) {
            _rxIsr(UART4, &uart4_ringbuffer_tx);
        }
        return;
    }

    /* UART IDLE EVENT INTERRUPT HANDLING */
    if ((cr1 & USART_CR1_IDLEIE) && (isrFlags & USART_ISR_IDLE)) {
        // TODO : Handling of IDLE
        ATOMIC_CLEAR_BIT(UART4->ICR, USART_ICR_IDLECF);
        return;
    }

    /* UART WAKEUP FROM STOP MODE INTERRUPT HANDLING */
    if ((cr3 & USART_CR3_WUFIE) && (isrFlags & USART_ISR_WUF)) {
        // TODO : Handling of WAKEUP
        ATOMIC_CLEAR_BIT(UART4->ICR, USART_ICR_IDLECF);
        return;
    }

    /* UART TRANSMIT INTERRUPT HANDLING */
    // TX Data Register Empty
    if ((cr1 & USART_CR1_TXEIE) && (isrFlags & USART_ISR_TXE)) {
        _txIsr(UART4, &uart4_ringbuffer_tx);
        return;
    }
    // Transmission Complete
    if ((cr1 & USART_CR1_TCIE) & (isrFlags & USART_ISR_TC)) {
        // Disable the UART Transmit Complete Interrupt
        ATOMIC_CLEAR_BIT(UART4->CR1, (USART_CR1_TXEIE | USART_CR1_TCIE));
        // TODO : Handling of Transmit Complete
    }
}