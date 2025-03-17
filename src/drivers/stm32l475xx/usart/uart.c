#include "uart.h"
#include <stddef.h>
#include <stdint.h>
#include "common/ringbuffer.h"
#include "stm32l475xx.h"
#include "stm32l4xx.h"

/*------------------------- Defines -------------------------------*/
#define UART_ISR_ERROR_MASK (USART_ISR_PE | USART_ISR_FE | USART_ISR_ORE | USART_ISR_NE | USART_ISR_RTOF)

/*------------------------- Global private variables -------------------------------*/
volatile uint32_t errorFlags = 0;
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


/*------------------------- Public functions -------------------------------*/
void uart_init(USART_TypeDef * self, uart_config_t config)
{
    uint32_t brr_value;

    if (NULL == self) {
        return;
    }

    // Disable and reset usart
    uart_deinit(self);

    /* Word length - 8 data bits (default) */
    // TODO : param assert
    MODIFY_REG(self->CR1, USART_CR1_M_Msk, config.word_length);

    /* Oversampling mode - 16 (default) */ 
    switch (config.oversampling) {
        case UART_OVERSAMPLING_8:
            ATOMIC_SET_BIT(self->CR1, USART_CR1_OVER8);
            brr_value = 2 * SystemCoreClock / config.baudrate;
            break;
        case UART_OVERSAMPLING_16:
        default:
            ATOMIC_CLEAR_BIT(self->CR1, USART_CR1_OVER8);
            brr_value = SystemCoreClock / config.baudrate;
            break;
    }
    
    /* UART baudrate - 0 (default) */
    self->BRR = brr_value;

    /* Parity control - disabled (default) */
    // TODO : param assert
    MODIFY_REG(self->CR1, (USART_CR1_PS_Msk | USART_CR1_PCE_Msk), config.parity);

    /* Bit endianness - Tx/Rx bit0 first (default) */
    // TODO : param assert
    MODIFY_REG(self->CR2, (USART_CR2_MSBFIRST_Msk), config.bit_endianness);

    /* Logic level inversion - normal (default) */
    // CLEAR_BIT(self->CR2, USART_CR2_TXINV);
    // ATOMIC_CLEAR_BIT(self->CR2, USART_CR2_RXINV);

    /* STOP bits - 1 (default) */
    // TODO : param assert
    MODIFY_REG(self->CR2, (USART_CR2_STOP_Msk), config.bit_endianness);

    /* DMA enable - Tx/Rx disabled (default) */
    // TBC : need to be configured after USART ENABLE ?
    // ATOMIC_CLEAR_BIT(self->CR3, USART_CR3_DMAT);
    // ATOMIC_CLEAR_BIT(self->CR3, USART_CR3_DMAR);

    ringbuffer_init(&uart4_ringbuffer_rx, uart4_rx_buffer);
    ringbuffer_init(&uart4_ringbuffer_tx, uart4_tx_buffer);
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

    // In asynchronous mode, the following bits must be kept cleared:
    ATOMIC_CLEAR_BIT(self->CR2, (USART_CR2_LINEN | USART_CR2_CLKEN));
    ATOMIC_CLEAR_BIT(self->CR3, (USART_CR3_SCEN | USART_CR3_HDSEL | USART_CR3_IREN));

    ATOMIC_SET_BIT(self->CR1, (USART_CR1_UE | USART_CR1_TE));
    while (USART_ISR_TEACK != (self->ISR & USART_ISR_TEACK)){};
}

void uart_disable(USART_TypeDef * self)
{
    int uart_irqn;

    if (self == NULL) {
        return;
    }

    /* Disable Tx and Rx */
    ATOMIC_CLEAR_BIT(self->CR1, (USART_CR1_TE | USART_CR1_RE));

    /* Wait for Transmission Complete bit */
    while (!(self->ISR & USART_ISR_TC)){};

    // TODO : DMA channel must be disabled before resetting the UE bit

    // Disable UART irq
    switch ((unsigned long)self) {
        case (unsigned long)USART1: uart_irqn = USART1_IRQn; break;
        case (unsigned long)USART2: uart_irqn = USART2_IRQn; break;
        case (unsigned long)USART3: uart_irqn = USART3_IRQn; break;
        case (unsigned long)UART4:  uart_irqn = UART4_IRQn;  break;
        case (unsigned long)UART5:  uart_irqn = UART5_IRQn;  break;
        default: return;
    }
    NVIC_ClearPendingIRQ(uart_irqn);
    NVIC_DisableIRQ(uart_irqn);

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

    while (*s != '\0') {
        uart_putchar(self, *s);
        s++;
    }
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

    IRQn_Type irqn;
    ringbuffer_t * buffPtr;
    if (USART1 == self) {
        #ifdef UASRT1_INT_MODE_ENABLE
        irqn = USART1_IRQn;
        buffPtr = &usart1_ringbuffer_tx;
        #else
        return;
        #endif
    }
    else if (USART2 == self) {
        #ifdef USART2_INT_MODE_ENABLE
        irqn = USART2_IRQn;
        buffPtr = &usart2_ringbuffer_tx;
        #else
        return;
        #endif
    }
    else if (USART3 == self) {
        #ifdef USART3_INT_MODE_ENABLE
        irqn = USART3_IRQn;
        buffPtr = &usart3_ringbuffer_tx;
        #else
        return;
        #endif
    }
    else if (UART4 == self) {
        #ifdef UART4_INT_MODE_ENABLE
        irqn = UART4_IRQn;
        buffPtr = &uart4_ringbuffer_tx;
        #else
        return;
        #endif
    }
    else if (UART5 == self) {
        #ifdef UART5_INT_MODE_ENABLE
        irqn = UART5_IRQn;
        buffPtr = &uart5_ringbuffer_tx;
        #else
        return;
        #endif
    }
    else {
        return;
    }

    NVIC_DisableIRQ(irqn);

    // memcpy
    for (uint8_t i = 0; i < (length*sizeof(buffer[0])); i++) {
        ringbuffer_put(buffPtr, buffer[i]);
    }
    
    // Enable TX interrupt
    ATOMIC_SET_BIT(self->CR1, USART_CR1_TXEIE);
    
    // Send the first byte
    self->TDR = ringbuffer_get(buffPtr);

    NVIC_EnableIRQ(irqn);
}

void uart_listen(USART_TypeDef * self)
{
    if (self == NULL) {
        return;
    }

    IRQn_Type irqn;
    if (USART1 == self) {
        #ifdef UASRT1_INT_MODE_ENABLE
        irqn = USART1_IRQn;
        #else
        return;
        #endif
    }
    else if (USART2 == self) {
        #ifdef USART2_INT_MODE_ENABLE
        irqn = USART2_IRQn;
        #else
        return;
        #endif
    }
    else if (USART3 == self) {
        #ifdef USART3_INT_MODE_ENABLE
        irqn = USART3_IRQn;
        #else
        return;
        #endif
    }
    else if (UART4 == self) {
        #ifdef UART4_INT_MODE_ENABLE
        irqn = UART4_IRQn;
        #else
        return;
        #endif
    }
    else if (UART5 == self) {
        #ifdef UART5_INT_MODE_ENABLE
        irqn = UART5_IRQn;
        #else
        return;
        #endif
    }
    else {
        return;
    }

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

    ringbuffer_t * buffPtr;
    switch ((unsigned long)self) {
        case (unsigned long)USART1:
            #ifdef UASRT1_INT_MODE_ENABLE
            buffPtr = &usart1_ringbuffer_rx;
            break;
            #else
            return 0;
            #endif
        case (unsigned long)USART2:
            #ifdef USART2_INT_MODE_ENABLE
            buffPtr = &usart2_ringbuffer_rx;
            break;
            #else
            return 0;
            #endif
        case (unsigned long)USART3:
            #ifdef USART3_INT_MODE_ENABLE
            buffPtr = &usart3_ringbuffer_rx;
            break;
            #else
            return 0;
            #endif
        case (unsigned long)UART4:
            #ifdef UART4_INT_MODE_ENABLE
            buffPtr = &uart4_ringbuffer_rx;
            break;
            #else
            return 0;
            #endif
        case (unsigned long)UART5:
            #ifdef UART5_INT_MODE_ENABLE
            buffPtr = &uart5_ringbuffer_rx;
            break;
            #else
            return 0;
            #endif
        default: return 0;
    }

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

#ifdef UART4_INT_MODE_ENABLE
void UART4_IRQHandler(void)
{
    uint32_t isrflags = READ_REG(UART4->ISR);
    uint32_t cr1      = READ_REG(UART4->CR1);
    uint32_t cr3   = READ_REG(UART4->CR3);

    errorFlags |= (isrflags & UART_ISR_ERROR_MASK);

    /* UART RECEIVE INTERRUPT HANDLING */
    if (0 == errorFlags) {
        // RX Data Register Not Empty
        if ((cr1 & USART_CR1_RXNEIE) && (isrflags & USART_ISR_RXNE)) {
            _rxIsr(UART4, &uart4_ringbuffer_tx);
            return;
        }
    }

    /* UART ERROR INTERRUPT HANDLING */
    if (errorFlags && (
        (cr3 & USART_CR3_EIE) ||
        (cr1 & (USART_CR1_RXNEIE | USART_CR1_PEIE | USART_CR1_RTOIE)))) {

        /* UART parity error interrupt occurred */
        if ((isrflags & USART_ISR_PE) && (cr1 & USART_CR1_PEIE))
        {
            // TODO : error handling
            ATOMIC_SET_BIT(UART4->ICR, USART_ICR_PECF);
        }

        /* UART frame error interrupt occurred */
        if ((isrflags & USART_ISR_FE) && (cr3 & USART_CR3_EIE))
        {
            // TODO : error handling
            ATOMIC_SET_BIT(UART4->ICR, USART_ICR_FECF);
        }

        /* UART noise error interrupt occurred */
        if ((isrflags & USART_ISR_NE) && (cr3 & USART_CR3_EIE))
        {
            // TODO : error handling
            ATOMIC_SET_BIT(UART4->ICR, USART_ICR_NECF);
        }

        /* UART Overrun interrupt occurred */
        if ((isrflags & USART_ISR_ORE) && (
            (cr1 & USART_CR1_RXNEIE) || (cr3 & USART_CR3_EIE)))
        {
            // TODO : error handling
            ATOMIC_SET_BIT(UART4->ICR, USART_ICR_ORECF);
        }

        /* UART Receiver Timeout interrupt occurred */
        if ((isrflags & USART_ISR_RTOF) && (cr1 & USART_CR1_RTOIE))
        {
            // TODO : error handling
            ATOMIC_SET_BIT(UART4->ICR, USART_ICR_RTOCF);
        }

        // Handling of RX with error
        // RX Data Register Not Empty
        if ((isrflags & USART_ISR_RXNE) && (cr1 & USART_CR1_RXNEIE)) {
            _rxIsr(UART4, &uart4_ringbuffer_tx);
        }
        // TODO : Handling of error, blocking (aborting xfer) vs non-blocking
        return;
    }

    /* UART IDLE EVENT INTERRUPT HANDLING */
    // TODO
    if ((cr1 & USART_CR1_IDLEIE) && (isrflags & USART_ISR_IDLE)) {
        // TODO : Handling of IDLE
        ATOMIC_CLEAR_BIT(UART4->ICR, USART_ICR_IDLECF);
        return;
    }

    /* UART WAKEUP FROM STOP MODE INTERRUPT HANDLING */
    // TODO
    if ((cr3 & USART_CR3_WUFIE) && (isrflags & USART_ISR_WUF)) {
        // TODO : Handling of WAKEUP
        ATOMIC_CLEAR_BIT(UART4->ICR, USART_ICR_IDLECF);
        return;
    }

    /* UART TRANSMIT INTERRUPT HANDLING */
    // TX Data Register Empty
    if ((cr1 & USART_CR1_TXEIE) && (isrflags & USART_ISR_TXE)) {
        _txIsr(UART4, &uart4_ringbuffer_tx);
        return;
    }
    // Transmission Complete
    if ((cr1 & USART_CR1_TCIE) & (isrflags & USART_ISR_TC)) {
        // Disable the UART Transmit Complete Interrupt
        ATOMIC_CLEAR_BIT(UART4->CR1, USART_CR1_TCIE);
        // TODO : Handling of Transmit Complete
    }
}
#endif