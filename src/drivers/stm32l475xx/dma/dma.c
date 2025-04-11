#include "dma.h"
#include "../stm32l4xx.h"
#include <stdint.h>
#include <assert.h>


/* MACROS */
#define _DMA_ENABLE(__CHANNEL__)  (SET_BIT((__CHANNEL__)->CCR, DMA_CCR_EN))
#define _DMA_DISABLE(__CHANNEL__) (CLEAR_BIT((__CHANNEL__)->CCR, DMA_CCR_EN))

/* GLOBAL PRIVATE VARIABLES */
enum dma_id_t {
    DMA1_CHANNEL1_ID = 0,
    DMA1_CHANNEL2_ID,
    DMA1_CHANNEL3_ID,
    DMA1_CHANNEL4_ID,
    DMA1_CHANNEL5_ID,
    DMA1_CHANNEL6_ID,
    DMA1_CHANNEL7_ID,
    DMA2_CHANNEL1_ID,
    DMA2_CHANNEL2_ID,
    DMA2_CHANNEL3_ID,
    DMA2_CHANNEL4_ID,
    DMA2_CHANNEL5_ID,
    DMA2_CHANNEL6_ID,
    DMA2_CHANNEL7_ID,
    DMA_ID_MAX
};

struct _dma_handler_t _dma_handler[DMA_ID_MAX] = {
    {NULL, DMA1, DMA1_Channel1, DMA1_Channel1_IRQn, 0, NULL, NULL, NULL},
    {NULL, DMA1, DMA1_Channel2, DMA1_Channel2_IRQn, 1, NULL, NULL, NULL},
    {NULL, DMA1, DMA1_Channel3, DMA1_Channel3_IRQn, 2, NULL, NULL, NULL},
    {NULL, DMA1, DMA1_Channel4, DMA1_Channel4_IRQn, 3, NULL, NULL, NULL},
    {NULL, DMA1, DMA1_Channel5, DMA1_Channel5_IRQn, 4, NULL, NULL, NULL},
    {NULL, DMA1, DMA1_Channel6, DMA1_Channel6_IRQn, 5, NULL, NULL, NULL},
    {NULL, DMA1, DMA1_Channel7, DMA1_Channel7_IRQn, 6, NULL, NULL, NULL},
    {NULL, DMA2, DMA2_Channel1, DMA2_Channel1_IRQn, 0, NULL, NULL, NULL},
    {NULL, DMA2, DMA2_Channel2, DMA2_Channel2_IRQn, 1, NULL, NULL, NULL},
    {NULL, DMA2, DMA2_Channel3, DMA2_Channel3_IRQn, 2, NULL, NULL, NULL},
    {NULL, DMA2, DMA2_Channel4, DMA2_Channel4_IRQn, 3, NULL, NULL, NULL},
    {NULL, DMA2, DMA2_Channel5, DMA2_Channel5_IRQn, 4, NULL, NULL, NULL},
    {NULL, DMA2, DMA2_Channel6, DMA2_Channel6_IRQn, 5, NULL, NULL, NULL},
    {NULL, DMA2, DMA2_Channel7, DMA2_Channel7_IRQn, 6, NULL, NULL, NULL}
};
static_assert(DMA_ID_MAX == (sizeof(_dma_handler)/sizeof(_dma_handler[0])), "Wrong array size");

/* PRIVATE FUNCTIONS */
static inline dma_handler_t* _dma_getHandler(DMA_Channel_TypeDef * dma_channel)
{
    for (uint8_t i = 0; i < DMA_ID_MAX; i++) {
        if (_dma_handler[i].channel == dma_channel) {
            return &_dma_handler[i];
        }
    }
    return NULL;
}

static void _dma_setXferConfig(dma_handler_t * dma_handler, const uint32_t srcAddr, const uint32_t destAddr, uint32_t dataLength)
{
    /* Clear all flags before configuring */
    dma_handler->dma->IFCR = DMA_IFCR_CGIF1 << (dma_handler->channelIdx << 2);

    if (dma_handler->channel->CCR & DMA_CCR_DIR) {   // DMA_DIRECTION_MEM_TO_PERIPH
        dma_handler->channel->CMAR = srcAddr;
        dma_handler->channel->CPAR = destAddr;
    }
    else {                              // DMA_DIRECTION_PERIPH_TO_MEM
        dma_handler->channel->CPAR = srcAddr;
        dma_handler->channel->CMAR = destAddr;
    }

    dma_handler->channel->CNDTR = dataLength;
}


/* PUBLIC FUNCTIONS */
/**
 * @brief Initialization/Configuration of a dma channel
 * 
 * @param channel the channel to configure
 * @param config  the desired configuration
 *
 * @note RCC AHB clock for dma must be enabled prior for the dma to work
 *
 * @note The initialization follow this following sequence :
 * 1. Set the peripheral register address in the DMA_CPARx register. 
 *    The data is moved from/to this address to/from the memory after the peripheral event, 
 *    or after the channel is enabled in memory-to-memory mode.
 * 2. Set the memory address in the DMA_CMARx register. 
 *    The data is written to/read from the memory after the peripheral event or after the 
 *    channel is enabled in memory-to-memory mode.
 * 3. Configure the total number of data to transfer in the DMA_CNDTRx register.
 *    After each data transfer, this value is decremented.
 * 4. Configure the parameters listed below in the DMA_CCRx register:
 *     - the channel priority
 *     - the data transfer direction
 *     - the circular mode
 *     - the peripheral and memory incremented mode
 *     - the peripheral and memory data size
 *     - the interrupt enable at half and/or full transfer and/or transfer error
 * 5. Activate the channel by setting the EN bit in the DMA_CCRx register.
*/
dma_handler_t * dma_init(DMA_Channel_TypeDef * channel, dma_config_t config)
{
    /* Parameters check */
    if (NULL == channel) return NULL;

    dma_handler_t * self = _dma_getHandler(channel);
    if (config.request >= DMA_REQUEST_MAX)          return NULL;
    if (config.direction & ~DMA_DIRECTION_Msk)      return NULL;
    if (config.data_size & ~DMA_DATA_SIZE_Msk)      return NULL;
    if (config.sw_priority & ~DMA_SW_PRIORITY_Msk)  return NULL;
    if (config.xfer_mode & ~DMA_XFER_MODE_Msk)      return NULL;
    if ((config.direction == DMA_DIRECTION_MEM_TO_MEM) &&
        (config.xfer_mode == DMA_XFER_MODE_CIRCULAR)) {
            return NULL;
    }

    /* Stops the DMA channel in case it was previously set */
    _DMA_DISABLE(self->channel);

    /* Copy the Channel Configuration Register */
    uint32_t dma_channel_ccr_img = self->channel->CCR;

    /* Modify the Channel Configuration Register copy */
    // Set the DMA channel priority level
    MODIFY_REG(dma_channel_ccr_img, DMA_CCR_PL_Msk, config.sw_priority);
    // Configure the DMA channel xfer mode (normal or circular)
    MODIFY_REG(dma_channel_ccr_img, DMA_CCR_CIRC_Msk, config.xfer_mode);
    // Configure the DMA channel direction
    MODIFY_REG(dma_channel_ccr_img, (DMA_CCR_DIR_Msk | DMA_CCR_MEM2MEM_Msk), config.direction);
    // Configure  Peripheral and Memory auto address increment
    MODIFY_REG(dma_channel_ccr_img, DMA_CCR_MINC_Msk, (config.memAddrAutoInc << DMA_CCR_MINC_Pos));
    MODIFY_REG(dma_channel_ccr_img, DMA_CCR_PINC_Msk, (config.periphAddrAutoInc << DMA_CCR_PINC_Pos));
    // Set the DMA channel Peripheral and Memory data size
    /* NOTE : When PSIZE[1:0] and MSIZE[1:0] are not equal, the DMA controller performs some data alignments
     *        See Table 46. Programmable data width and endian behavior (when PINC=MINC=1) in the Reference Manual */
    MODIFY_REG(dma_channel_ccr_img, (DMA_CCR_MSIZE_Msk | DMA_CCR_PSIZE_Msk), config.data_size);

    /* Write the Channel Configuration Register */
    self->channel->CCR = dma_channel_ccr_img;

    /* Set request selection */
    if (DMA_DIRECTION_MEM_TO_MEM != config.direction) {
        DMA_Request_TypeDef * dmaReq = ((self->dma == DMA1) ? DMA1_CSELR : DMA2_CSELR);
        // Set Request selection
        MODIFY_REG(dmaReq->CSELR, (DMA_CSELR_C1S_Msk << (self->channelIdx << 2)), (config.request << (self->channelIdx << 2)));
    }

    return self;
}

dma_error_t dma_start(dma_handler_t * self, const uint32_t srcAddr, const uint32_t destAddr, const size_t dataLength)
{
    return dma_start_it(self, srcAddr, destAddr, dataLength, DMA_INT_NONE);
}

dma_error_t dma_start_it(dma_handler_t * self, const uint32_t srcAddr, const uint32_t destAddr, const size_t dataLength, dma_int_t int_ena)
{
    // TODO : param assert
    if (NULL == self)    return DMA_INVALID_PARAMETER;
    if (0 == srcAddr)    return DMA_INVALID_PARAMETER;
    if (0 == destAddr)   return DMA_INVALID_PARAMETER;
    if (0 >= dataLength) return DMA_INVALID_PARAMETER;
    if (int_ena & ~DMA_INT_Msk) return DMA_INVALID_PARAMETER;

    // TODO : check if busy

    _DMA_DISABLE(self->channel);
    _dma_setXferConfig(self, srcAddr, destAddr, dataLength);
    if (int_ena & DMA_INT_Msk) {
        NVIC_EnableIRQ(self->irqn);
    }
    MODIFY_REG(self->channel->CCR, DMA_INT_Msk, int_ena);
    self->dma->IFCR = DMA_IFCR_CGIF1 << (self->channelIdx << 2);
    _DMA_ENABLE(self->channel);

    return DMA_OK;
}

dma_error_t dma_abort(dma_handler_t * self)
{
    if (NULL == self) return DMA_INVALID_PARAMETER;

    // Disable interrupts
    CLEAR_BIT(self->channel->CCR, DMA_CCR_TCIE | DMA_CCR_HTIE | DMA_CCR_TEIE);
    // Disable dma channel
    _DMA_DISABLE(self->channel);
    // Clear all interrupt flags
    self->dma->IFCR = DMA_IFCR_CGIF1 << (self->channelIdx << 2);

    return DMA_OK;
}

bool dma_isXferComplete(dma_handler_t * self, dma_error_t *status)
{
    *status = DMA_OK;

    if (NULL == self) {
        *status = DMA_INVALID_PARAMETER;
        return false;
    }

    if (self->dma->ISR & (DMA_ISR_TCIF1 << (self->channelIdx << 2))) {
        // Clear transfer complete flag
        self->dma->IFCR = DMA_IFCR_CTCIF1 << (self->channelIdx << 2);
        return true;
    }

    return false;
}

/* INTERRUPT SERVICE ROUTINES (IRQ) */
static void _dma_channel_IRQHandler(dma_handler_t * self)
{
    uint32_t isrFlags = READ_REG(self->dma->ISR);
    uint32_t ccr      = READ_REG(self->channel->CCR);

    /* HALF TRANSFER INTERRUPT */
    if ((isrFlags & (DMA_ISR_HTIF1 << (self->channelIdx << 2))) &&
        (ccr & DMA_CCR_HTIE)) {
        // Disable Half-Transfer Interrupt if not circular
        if (0 == (ccr & DMA_CCR_CIRC)) {
            ATOMIC_CLEAR_BIT(self->channel->CCR, DMA_CCR_HTIE);
        }
        // Clear the Half-Transfer flag
        self->dma->IFCR = DMA_IFCR_CHTIF1 << (self->channelIdx << 2);

        if (self->xferHalfCpltCb) {
            self->xferHalfCpltCb(self);
        }
    }
    /* TRANSFER COMPLETE INTERRUPT */
    else if ((isrFlags & (DMA_ISR_TCIF1 << (self->channelIdx << 2))) &&
             (ccr & DMA_CCR_TCIE)) {
        // Disable Transfer Complete and Error Interrupts if not circular
        if (0 == (ccr & DMA_CCR_CIRC)) {
            ATOMIC_CLEAR_BIT(self->channel->CCR, (DMA_CCR_TCIE | DMA_CCR_TEIE));
        }
        // Clear the Transfer Complete flag
        self->dma->IFCR = DMA_IFCR_CTCIF1 << (self->channelIdx << 2);

        if (self->xferCpltCb) {
            self->xferCpltCb(self);
        }
    }
    /* TRANSFER ERROR INTERRUPT */
    else if ((isrFlags & (DMA_ISR_TEIF1 << (self->channelIdx << 2))) &&
             (ccr & DMA_CCR_TEIE)) {
        // Disable all interrupts
        ATOMIC_CLEAR_BIT(self->channel->CCR, (DMA_CCR_HTIE | DMA_CCR_TCIE | DMA_CCR_TEIE));
        // Clear all flags
        self->dma->IFCR = DMA_IFCR_CGIF1 << (self->channelIdx << 2);

        if (self->xferErrorCb) {
            self->xferErrorCb(self);
        }
    }
}

void DMA2_Channel3_IRQHandler(void)
{
    _dma_channel_IRQHandler(&_dma_handler[DMA2_CHANNEL3_ID]);
}