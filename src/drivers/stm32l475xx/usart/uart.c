#include <assert.h>
#include "usart_common.h"
#include "uart.h"
#include "../clock/clock.h"

/*------------------------- Defines -------------------------------*/
#define UART_ISR_ERROR_MASK (USART_ISR_PE | USART_ISR_FE | USART_ISR_ORE | USART_ISR_NE | USART_ISR_RTOF)

/*------------------------- Static function declaration -------------------------------*/
static void _dmaTxHalfCpltCb(dma_handler_t * dma_handler);
static void _dmaRxHalfCpltCb(dma_handler_t * dma_handler);
static void _dmaTxCpltCb(dma_handler_t * dma_handler);
static void _dmaRxCpltCb(dma_handler_t * dma_handler);
static void _dmaErrorCb(dma_handler_t * dma_handler);
static void _rxIsr(uart_handler_t * uart_handler);
static void _txIsr(uart_handler_t * uart_handler);
static void _errorIsr(uart_handler_t * uart_handler);

/*------------------------- Global private variables -------------------------------*/
enum usart_periphId_t {
    USART1_ID = 0,
    USART2_ID,
    USART3_ID,
    UART4_ID,
    UART5_ID,
    UART_ID_MAX
};

struct _uart_handler_t{
    USART_TypeDef * USARTx;
    const IRQn_Type irqn;
    struct {
        ringbuffer_t * tx;
        ringbuffer_t * rx;
    } ringbuffer;
    struct {
        dma_handler_t * tx;
        dma_handler_t * rx;
    } dma;
    struct {
        void (*rxISR_cb)(uart_handler_t * uart_handler);
        void (*txISR_cb)(uart_handler_t * uart_handler);
        void (*errorISR_cb)(uart_handler_t * uart_handler);
    } callbacks;
    volatile bool rxIdleLineDetected;
};

struct _uart_handler_t _uart_handler[UART_ID_MAX] = {
    {USART1, USART1_IRQn, {NULL, NULL}, {NULL, NULL}, {NULL, NULL, NULL}, false},
    {USART2, USART2_IRQn, {NULL, NULL}, {NULL, NULL}, {NULL, NULL, NULL}, false},
    {USART3, USART3_IRQn, {NULL, NULL}, {NULL, NULL}, {NULL, NULL, NULL}, false},
    {UART4,  UART4_IRQn,  {NULL, NULL}, {NULL, NULL}, {NULL, NULL, NULL}, false},
    {UART5,  UART5_IRQn,  {NULL, NULL}, {NULL, NULL}, {NULL, NULL, NULL}, false}
};
static_assert(UART_ID_MAX == (sizeof(_uart_handler)/sizeof(_uart_handler[0])), "Wrong array size");

/*------------------------- Private functions -------------------------------*/
static inline uart_handler_t* _uart_getHandler(USART_TypeDef * usartx)
{
    for (uint8_t i = 0; i < UART_ID_MAX; i++) {
        if (_uart_handler[i].USARTx == usartx) {
            return &_uart_handler[i];
        }
    }
    return NULL;
}

static inline bool _uart_reset(USART_TypeDef * usartx)
{
    switch ((unsigned long)usartx) {
        case (unsigned long)USART1: {
            ATOMIC_SET_BIT(RCC->APB2RSTR, RCC_APB2RSTR_USART1RST);
            ATOMIC_CLEAR_BIT(RCC->APB2RSTR, RCC_APB2RSTR_USART1RST);
        } break;

        case (unsigned long)USART2: {
            ATOMIC_SET_BIT(RCC->APB1RSTR1, RCC_APB1RSTR1_USART2RST);
            ATOMIC_CLEAR_BIT(RCC->APB1RSTR1, RCC_APB1RSTR1_USART2RST);
        } break;

        case (unsigned long)USART3: {
            ATOMIC_SET_BIT(RCC->APB1RSTR1, RCC_APB1RSTR1_USART3RST);
            ATOMIC_CLEAR_BIT(RCC->APB1RSTR1, RCC_APB1RSTR1_USART3RST);
        } break;

        case (unsigned long)UART4: {
            ATOMIC_SET_BIT(RCC->APB1RSTR1, RCC_APB1RSTR1_UART4RST);
            ATOMIC_CLEAR_BIT(RCC->APB1RSTR1, RCC_APB1RSTR1_UART4RST);
        } break;

        case (unsigned long)UART5: {
            ATOMIC_SET_BIT(RCC->APB1RSTR1, RCC_APB1RSTR1_UART5RST);
            ATOMIC_CLEAR_BIT(RCC->APB1RSTR1, RCC_APB1RSTR1_UART5RST);
        } break;

        default: {
            return true;
        }
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
static inline void _uart_waitRXNE(USART_TypeDef * usartx)
{
    while (!USART_RX_DATA_REG_NOT_EMPTY(usartx));
}

// TODO : timeout
static inline void _uart_waitTXE(USART_TypeDef * usartx)
{
    while (!USART_TX_DATA_REG_EMPTY(usartx));
}

// TODO : timeout
static inline void _uart_waitTC(USART_TypeDef * usartx)
{
    while (!(USART_TX_COMPLETE(usartx)));
}

// TODO : timeout
static void _uart_waitReady(uart_handler_t * self)
{
    if ((self->USARTx->CR1 & USART_CR1_TE) == USART_CR1_TE) {
        while ((self->USARTx->ISR & USART_ISR_TEACK) != USART_ISR_TEACK);
        // if timeout occured, clear interrupts
        // ATOMIC_CLEAR_BIT(self->USARTx->CR1, (USART_CR1_TXEIE));
    }

    if ((self->USARTx->CR1 & USART_CR1_RE) == USART_CR1_RE) {
        while ((self->USARTx->ISR & USART_ISR_REACK) != USART_ISR_REACK) {
            // TODO : check Overrun and Receiver Timeout flag
        }
        // if timeout occured, clear interrupts
        // ATOMIC_CLEAR_BIT(self->USARTx->CR1, (USART_CR1_RXNEIE | USART_CR1_PEIE));
        // ATOMIC_CLEAR_BIT(self->USARTx->CR3, USART_CR3_EIE);
    }
}

/*------------------------- Public functions -------------------------------*/
uart_handler_t * uart_init(USART_TypeDef * usartx, const uart_config_t config, ringbuffer_t * rb_tx, ringbuffer_t * rb_rx, dma_handler_t * dma_tx, dma_handler_t * dma_rx)
{
    if (NULL == usartx) {
        return NULL;
    }

    uart_handler_t * self = _uart_getHandler(usartx);

    if (NULL == self) {
        return NULL;
    }

    // Disable and reset usart
    if (UART_OK != uart_deinit(self)) {
        return NULL;
    }

    /*------------- USARTx CR1 Configuration -------------*/
    /* Word length - 8 data bits (default) */
    MODIFY_REG(self->USARTx->CR1, USART_CR1_M_Msk, config.word_length);
    /* Parity control - disabled (default) */
    MODIFY_REG(self->USARTx->CR1, (USART_CR1_PS_Msk | USART_CR1_PCE_Msk), config.parity);
    /* Direction Tx and/or Rx - disabled (default) */
    MODIFY_REG(self->USARTx->CR1, (USART_CR1_TE_Msk | USART_CR1_RE_Msk), config.direction);
    /* Oversampling mode - 16 (default) */
    MODIFY_REG(self->USARTx->CR1, USART_CR1_OVER8_Msk, config.oversampling);

    /*------------- USARTx CR2 Configuration -------------*/
    /* STOP bits - 1 (default) */
    MODIFY_REG(self->USARTx->CR2, (USART_CR2_STOP_Msk), config.bit_endianness);
    /* Bit endianness - Tx/Rx bit0 first (default) */
    MODIFY_REG(self->USARTx->CR2, (USART_CR2_MSBFIRST_Msk), config.bit_endianness);

    /*------------- USARTx CR3 Configuration -------------*/
    /* Hardware Flow Controle - none (default) */
    // MODIFY_REG(self->USARTx->CR3, (USART_CR3_CTSE_Msk | USART_CR3_RTSE_Msk), config.flow_control);
    
    /*------------- USARTx BRR Configuration -------------*/
    /* UART baudrate - 0 (default) */
    // TODO : get USARTx periph clock_source freq
    self->USARTx->BRR = _uart_baudrateToBrrValue(getUsartClkFreq(self->USARTx), config.oversampling, config.baudrate);

    // In asynchronous mode, the following bits must be kept cleared:
    ATOMIC_CLEAR_BIT(self->USARTx->CR2, (USART_CR2_LINEN | USART_CR2_CLKEN));
    ATOMIC_CLEAR_BIT(self->USARTx->CR3, (USART_CR3_SCEN | USART_CR3_HDSEL | USART_CR3_IREN));

    self->ringbuffer.tx = rb_tx;
    self->ringbuffer.rx = rb_rx;
    self->dma.tx = dma_tx;
    self->dma.rx = dma_rx;
    if (self->dma.tx) {
        self->dma.tx->parent = (void *)self;
    }
    if (self->dma.rx) {
        self->dma.rx->parent = (void *)self;
    }

    uart_enable(self);

    return self;
}

uart_error_t uart_deinit(uart_handler_t * self)
{
    if (NULL == self) {
        return UART_INVALID_PARAMETER;
    }

    uart_disable(self);

    _uart_reset(self->USARTx);

    return UART_OK;
}

uart_error_t uart_enable(uart_handler_t * self)
{
    if (self == NULL) {
        return UART_INVALID_PARAMETER;
    }

    ATOMIC_SET_BIT(self->USARTx->CR1, USART_CR1_UE);
    _uart_waitReady(self);

    return UART_OK;
}

uart_error_t uart_disable(uart_handler_t * self)
{
    if (self == NULL) {
        return UART_INVALID_PARAMETER;
    }

    /* Disable Tx and Rx */
    ATOMIC_CLEAR_BIT(self->USARTx->CR1, (USART_CR1_TE | USART_CR1_RE));

    /* Wait for Transmission Complete bit */
    _uart_waitTC(self->USARTx);

    // TODO : DMA channel must be disabled before resetting the UE bit

    // Disable UART irq
    NVIC_ClearPendingIRQ(self->irqn);
    NVIC_DisableIRQ(self->irqn);

    /* Disable UART */
    ATOMIC_CLEAR_BIT(self->USARTx->CR1, USART_CR1_UE);

    return UART_OK;
}

uart_error_t uart_putchar(uart_handler_t * self, char c)
{
    if (self == NULL) {
        return UART_INVALID_PARAMETER;
    }

    _uart_waitTXE(self->USARTx);
    self->USARTx->TDR = c;

    return UART_OK;
}

uart_error_t uart_puts(uart_handler_t * self, char * s)
{
    uart_error_t error = UART_INVALID_PARAMETER;

    if ((self == NULL) || (NULL == s)) {
        return error;
    }

    NVIC_DisableIRQ(self->irqn);
    while (*s != '\0') {
        error = uart_putchar(self, *s);
        if (UART_OK != error) {
            break;
        }
        s++;
    }
    NVIC_EnableIRQ(self->irqn);

    return error;
}

char uart_getchar(uart_handler_t * self)
{
    char c;

    if (self == NULL) {
        return UINT8_MAX;
    }

    _uart_waitRXNE(self->USARTx);
    c = self->USARTx->RDR;

    return c;
}

uint16_t uart_gets(uart_handler_t * self, char * buffer, uint16_t length)
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

uart_error_t uart_send(uart_handler_t * self, char * buffer, uint16_t length)
{
    if ((self == NULL) || (buffer == NULL) || (length == 0)) {
        return UART_INVALID_PARAMETER;
    }

    if (NULL == self->ringbuffer.tx) {
        return UART_INVALID_CONFIG;
    }

    // TODO : Check for ongoing transmission (and handle it if true)

    // TODO : Is it necessary to protect buffer memcpy ?
    // memcpy
    NVIC_DisableIRQ(self->irqn);
    for (uint8_t i = 0; i < (length*sizeof(buffer[0])); i++) {
        ringbuffer_put(self->ringbuffer.tx, buffer[i]);
    }
    NVIC_EnableIRQ(self->irqn);

    // Enable TX interrupt
    ATOMIC_SET_BIT(self->USARTx->CR1, USART_CR1_TXEIE);

    return UART_OK;
}

uart_error_t uart_send_dma(uart_handler_t * self, const char * buffer, const uint16_t length)
{
    dma_error_t dma_error = DMA_OK;

    if ((self == NULL) || (buffer == NULL) || (length == 0)) {
        return UART_INVALID_PARAMETER;
    }

    if (NULL == self->dma.tx) {
        return UART_INVALID_CONFIG;
    }

    // Use default callbacks when no user callbacks have been registered 
    if (NULL == self->dma.tx->xferHalfCpltCb) {
        self->dma.tx->xferHalfCpltCb = _dmaTxHalfCpltCb;
    }
    if (NULL == self->dma.tx->xferCpltCb) {
        self->dma.tx->xferCpltCb = _dmaTxCpltCb;
    }
    if (NULL == self->dma.tx->xferErrorCb) {
        self->dma.tx->xferErrorCb = _dmaErrorCb;
    }

    dma_error = dma_start_it(self->dma.tx, (uint32_t)buffer, (uint32_t)&self->USARTx->TDR, length, (DMA_XFER_CPLT_INT | DMA_XFER_HCPLT_INT | DMA_XFER_ERROR_INT));

    if (DMA_OK != dma_error) {
        return UART_UNKNOWN_ERROR;
    }

    NVIC_EnableIRQ(self->irqn);

    ATOMIC_SET_BIT(self->USARTx->CR3, USART_CR3_DMAT);

    return UART_OK;
}

uart_error_t uart_listen(uart_handler_t * self)
{
    if (self == NULL) {
        return UART_INVALID_PARAMETER;
    }

    NVIC_EnableIRQ(self->irqn);

    // Enable the Error Interrupt (Frame error, noise error, overrun error)
    ATOMIC_SET_BIT(self->USARTx->CR3, USART_CR3_EIE);
    // Enable the Parity Error Interrupt
    if (self->USARTx->CR1 != UART_PARITY_DISABLED) {
        ATOMIC_SET_BIT(self->USARTx->CR1, USART_CR1_PEIE);
    }
    // Enable the Data Register Not Empty Interrupt
    ATOMIC_SET_BIT(self->USARTx->CR1, USART_CR1_RXNEIE);

    // TEST : NEEDS TO BE IMPLEMENTED PROPERLY
    ATOMIC_SET_BIT(self->USARTx->ICR, USART_ICR_IDLECF);
    ATOMIC_SET_BIT(self->USARTx->CR1, USART_CR1_IDLEIE);
    // ---------------------------------------

    return UART_OK;
}

uart_error_t uart_listen_dma(uart_handler_t * self, char * buffer, const uint16_t length)
{
    dma_error_t dma_error = DMA_OK;

    if ((self == NULL) || (buffer == NULL) || (length == 0)) {
        return UART_INVALID_PARAMETER;
    }

    if (NULL == self->dma.rx) {
        return UART_INVALID_CONFIG;
    }

    // TODO : Check for already ongoing rx process

    // Use default callbacks when no user callbacks have been registered 
    if (NULL == self->dma.rx->xferHalfCpltCb) {
        self->dma.rx->xferHalfCpltCb = _dmaRxHalfCpltCb;
    }
    if (NULL == self->dma.rx->xferCpltCb) {
        self->dma.rx->xferCpltCb = _dmaRxCpltCb;
    }
    if (NULL == self->dma.rx->xferErrorCb) {
        self->dma.rx->xferErrorCb = _dmaErrorCb;
    }

    dma_error = dma_start_it(self->dma.rx, (uint32_t)&self->USARTx->RDR, (uint32_t)buffer, length, (DMA_XFER_CPLT_INT | DMA_XFER_HCPLT_INT | DMA_XFER_ERROR_INT));

    if (DMA_OK != dma_error) {
        return UART_UNKNOWN_ERROR;
    }

    NVIC_EnableIRQ(self->irqn);

    // Enable the Parity Error Interrupt
    if (self->USARTx->CR1 & USART_CR1_PCE) {
        ATOMIC_SET_BIT(self->USARTx->CR1, USART_CR1_PEIE);
    }
    // Enable the Error Interrupt (Frame error, noise error, overrun error)
    // Enable the DMA transfer receiver request
    ATOMIC_SET_BIT(self->USARTx->CR3, (USART_CR3_EIE | USART_CR3_DMAR));

    return UART_OK;
}

uint16_t uart_read(uart_handler_t * self, char * buffer, const uint16_t buffer_size)
{
    if ((NULL == self) || (NULL == buffer) || buffer_size == 0) {
        return 0;
    }

    uint16_t count = 0;

    if (self->rxIdleLineDetected) {
        self->rxIdleLineDetected = false;

        while (!ringbuffer_isEmpty(self->ringbuffer.rx) && (count < buffer_size)) {
            buffer[count++] = ringbuffer_get(self->ringbuffer.rx);
        }
    }

    return count;
}


/*------------------------- DMA callbacks -------------------------------*/
__attribute__((unused)) static void _dmaTxHalfCpltCb(dma_handler_t * dma_handler)
{
    // TODO
    (void)dma_handler;
}

__attribute__((unused)) static void _dmaRxHalfCpltCb(dma_handler_t * dma_handler)
{
    // TODO
    (void)dma_handler;
}

__attribute__((unused)) static void _dmaTxCpltCb(dma_handler_t * dma_handler)
{
    uint32_t dma_ccr = dma_handler->channel->CCR;
    uart_handler_t * uart_handler = (uart_handler_t *)dma_handler->parent;

    // Not circular mode
    if (!(dma_ccr & DMA_CCR_CIRC)) {
        // Disable the DMA transfer for transmit request
        ATOMIC_CLEAR_BIT(uart_handler->USARTx->CR3, USART_CR3_DMAT);
        // Enable the UART Transmit Complete Interrupt
        ATOMIC_SET_BIT(uart_handler->USARTx->CR1, USART_CR1_TCIE);
    }
    // TODO : if circular mode
}

__attribute__((unused)) static void _dmaRxCpltCb(dma_handler_t * dma_handler)
{
    uint32_t dma_ccr = dma_handler->channel->CCR;
    uart_handler_t * uart_handler = (uart_handler_t *)dma_handler->parent;

    // Not circular mode
    if (!(dma_ccr & DMA_CCR_CIRC)) {
        // Disable Frame error, noise error, overrun error, dma receiver request and idle interrupts
        ATOMIC_CLEAR_BIT(uart_handler->USARTx->CR1, (USART_CR1_PEIE | USART_CR1_IDLEIE));
        ATOMIC_CLEAR_BIT(uart_handler->USARTx->CR3, (USART_CR3_EIE | USART_CR3_DMAR));
    }
    // TODO : Reception mode is Reception Until Idle
    // TODO : other callbacks ?
}

__attribute__((unused)) static void _dmaErrorCb(dma_handler_t * dma_handler)
{
    uart_handler_t * uart_handler = (uart_handler_t *)dma_handler->parent;
    uint32_t usart_cr3 = uart_handler->USARTx->CR3;

    // Stop UART DMA Tx request if ongoing
    if (usart_cr3 & USART_CR3_DMAT) // && (gstate == HAL_UART_STATE_BUSY_TX))
    {
        ATOMIC_CLEAR_BIT(uart_handler->USARTx->CR1, (USART_CR1_TXEIE | USART_CR1_TCIE));
    }

    // Stop UART DMA Rx request if ongoing
    if (usart_cr3 & USART_CR3_DMAR) // && (gstate == HAL_UART_STATE_BUSY_TX))
    {
        ATOMIC_CLEAR_BIT(uart_handler->USARTx->CR1, (USART_CR1_RXNEIE | USART_CR1_PEIE | USART_CR1_IDLEIE));
        ATOMIC_CLEAR_BIT(uart_handler->USARTx->CR3, USART_CR3_EIE);
    }

    // TODO : other callbacks ?
}

/*------------------------- ISR Handling -------------------------------*/
static void _rxIsr(uart_handler_t * uart_handler)
{
    uint8_t rb_error = ringbuffer_put(uart_handler->ringbuffer.rx, uart_handler->USARTx->RDR);
    if (rb_error != 0) {
        // TODO : Handling of ringbuffer full
        ATOMIC_CLEAR_BIT(uart_handler->USARTx->CR1, USART_CR1_RXNEIE);
    }
}

static void _txIsr(uart_handler_t * uart_handler)
{
    if (ringbuffer_isEmpty(uart_handler->ringbuffer.tx)) {
        // Disable the TX interrupt
        // Enable the UART Transmit Complete Interrupt
        ATOMIC_CLEAR_BIT(uart_handler->USARTx->CR1, USART_CR1_TXEIE);
        ATOMIC_SET_BIT(uart_handler->USARTx->CR1, USART_CR1_TCIE);
    }
    else {
        uart_handler->USARTx->TDR = ringbuffer_get(uart_handler->ringbuffer.tx);
    }
}

static void _errorIsr(uart_handler_t * uart_handler)
{
    uint32_t isrRegCpy = uart_handler->USARTx->ISR;
    uint32_t cr1RegCpy = uart_handler->USARTx->CR1;
    uint32_t cr3RegCpy = uart_handler->USARTx->CR3;

    /* UART parity error interrupt occurred */
    if ((isrRegCpy & USART_ISR_PE) && (cr1RegCpy & USART_CR1_PEIE))
    {
        // TODO : error handling
        ATOMIC_SET_BIT(uart_handler->USARTx->ICR, USART_ICR_PECF);
    }

    /* UART frame error interrupt occurred */
    if ((isrRegCpy & USART_ISR_FE) && (cr3RegCpy & USART_CR3_EIE))
    {
        // TODO : error handling
        ATOMIC_SET_BIT(uart_handler->USARTx->ICR, USART_ICR_FECF);
    }

    /* UART noise error interrupt occurred */
    if ((isrRegCpy & USART_ISR_NE) && (cr3RegCpy & USART_CR3_EIE))
    {
        // TODO : error handling
        ATOMIC_SET_BIT(uart_handler->USARTx->ICR, USART_ICR_NECF);
    }

    /* UART Overrun interrupt occurred */
    if ((isrRegCpy & USART_ISR_ORE) && (
        (cr1RegCpy & USART_CR1_RXNEIE) || (cr3RegCpy & USART_CR3_EIE)))
    {
        // TODO : error handling
        ATOMIC_SET_BIT(uart_handler->USARTx->ICR, USART_ICR_ORECF);
    }

    /* UART Receiver Timeout interrupt occurred */
    if ((isrRegCpy & USART_ISR_RTOF) && (cr1RegCpy & USART_CR1_RTOIE))
    {
        // TODO : error handling
        ATOMIC_SET_BIT(uart_handler->USARTx->ICR, USART_ICR_RTOCF);
    }
}

static void _usart_IRQHandler(uart_handler_t * uart_handler)
{
    uint32_t isrFlags = READ_REG(uart_handler->USARTx->ISR);
    uint32_t cr1      = READ_REG(uart_handler->USARTx->CR1);
    uint32_t cr3      = READ_REG(uart_handler->USARTx->CR3);

    /* UART RECEIVE INTERRUPT HANDLING */
    if ((0 == (isrFlags & UART_ISR_ERROR_MASK)) && (cr1 & USART_CR1_RXNEIE) && (isrFlags & USART_ISR_RXNE)) {
        _rxIsr(uart_handler);
        return;
    }

    /* UART ERROR INTERRUPT HANDLING */
    if ((0 != (isrFlags & UART_ISR_ERROR_MASK)) && ((cr3 & USART_CR3_EIE) || (cr1 & (USART_CR1_RXNEIE | USART_CR1_PEIE | USART_CR1_RTOIE)))) {
        // Handling of errors
        _errorIsr(uart_handler);
        // Handling of RX with error if RX Data Register Not Empty
        if ((isrFlags & USART_ISR_RXNE) && (cr1 & USART_CR1_RXNEIE)) {
            _rxIsr(uart_handler);
        }
        return;
    }

    /* UART IDLE EVENT INTERRUPT HANDLING */
    if ((cr1 & USART_CR1_IDLEIE) && (isrFlags & USART_ISR_IDLE)) {
        // TODO : Handling of IDLE
        ATOMIC_SET_BIT(uart_handler->USARTx->ICR, USART_ICR_IDLECF);

        // TODO : DMA mode
        // if (uart_handler->USARTx->CR3 & USART_CR3_DMAR)
        // Not DMA mode
        // else

        uart_handler->rxIdleLineDetected = true;

        return;
    }

    /* UART WAKEUP FROM STOP MODE INTERRUPT HANDLING */
    if ((cr3 & USART_CR3_WUFIE) && (isrFlags & USART_ISR_WUF)) {
        // TODO : Handling of WAKEUP
        ATOMIC_SET_BIT(uart_handler->USARTx->ICR, USART_ICR_IDLECF);
        return;
    }

    /* UART TRANSMIT INTERRUPT HANDLING */
    // TX Data Register Empty
    if ((cr1 & USART_CR1_TXEIE) && (isrFlags & USART_ISR_TXE)) {
        _txIsr(uart_handler);
        return;
    }
    // Transmission Complete
    if ((cr1 & USART_CR1_TCIE) && (isrFlags & USART_ISR_TC)) {
        // Disable the UART Transmit Complete Interrupt
        ATOMIC_CLEAR_BIT(uart_handler->USARTx->CR1, (USART_CR1_TXEIE | USART_CR1_TCIE));
        // TODO : Handling of Transmit Complete
    }
}

void USART1_IRQHandler(void)
{
    _usart_IRQHandler(&_uart_handler[USART1_ID]);
}

void USART2_IRQHandler(void)
{
    _usart_IRQHandler(&_uart_handler[USART2_ID]);
}

void USART3_IRQHandler(void)
{
    _usart_IRQHandler(&_uart_handler[USART3_ID]);
}

void UART4_IRQHandler(void)
{
    _usart_IRQHandler(&_uart_handler[UART4_ID]);
}

void UART5_IRQHandler(void)
{
    _usart_IRQHandler(&_uart_handler[UART5_ID]);
}