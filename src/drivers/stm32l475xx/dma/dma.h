#ifndef DMA_H
#define DMA_H


#include "../stm32l4xx.h"   // IWYU pragma: keep
#include <stdbool.h>
#include <stddef.h>
#include "dma_error.h"


typedef enum {
    DMA_REQUEST_0 = 0,
    DMA_REQUEST_1,
    DMA_REQUEST_2,
    DMA_REQUEST_3,
    DMA_REQUEST_4,
    DMA_REQUEST_5,
    DMA_REQUEST_6,
    DMA_REQUEST_7,
    DMA_REQUEST_MAX
} dma_request_t;

typedef enum {
    DMA_SW_PRIORITY_LOW = 0,
    DMA_SW_PRIORITY_MEDIUM = DMA_CCR_PL_0,
    DMA_SW_PRIORITY_HIGH = DMA_CCR_PL_1,
    DMA_SW_PRIORITY_VERY_HIGH = DMA_CCR_PL,
    DMA_SW_PRIORITY_Msk = DMA_CCR_PL_Msk,
} dma_sw_priority_t;

typedef enum {
    DMA_DATA_SIZE_8b = 0,
    DMA_DATA_SIZE_16b = (DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0),
    DMA_DATA_SIZE_32b = (DMA_CCR_MSIZE_1 | DMA_CCR_PSIZE_1),
    DMA_DATA_SIZE_Msk = (DMA_CCR_MSIZE_Msk | DMA_CCR_PSIZE_Msk),
} dma_data_size_t;

typedef enum {
    DMA_XFER_MODE_NORMAL = 0,
    DMA_XFER_MODE_CIRCULAR = DMA_CCR_CIRC,  // For MEM_TO_PERIPH or PERIPH_TO_MEM
    DMA_XFER_MODE_Msk = DMA_CCR_CIRC_Msk
} dma_xfer_mode_t;

/**
 * @brief Assigning source/destination
 * Regardless of their usual naming, these 'peripheral' and 'memory' fields are used 
 * to define the source and destination addresses
 */
typedef enum {
    DMA_DIRECTION_PERIPH_TO_MEM = 0,
    DMA_DIRECTION_MEM_TO_PERIPH = DMA_CCR_DIR,
    DMA_DIRECTION_MEM_TO_MEM    = DMA_CCR_MEM2MEM,
    DMA_DIRECTION_Msk           = (DMA_DIRECTION_MEM_TO_PERIPH | DMA_DIRECTION_MEM_TO_MEM)
} dma_direction_t;

/**
 * @brief 
 * @note Can be ORed
 */
typedef enum {
    DMA_INT_NONE        = 0,
    DMA_XFER_CPLT_INT   = DMA_CCR_TCIE,
    DMA_XFER_HCPLT_INT  = DMA_CCR_HTIE,
    DMA_XFER_ERROR_INT  = DMA_CCR_TEIE,
    DMA_INT_Msk         = (DMA_CCR_TCIE | DMA_CCR_HTIE | DMA_CCR_TEIE)
} dma_int_t;

typedef struct {
    dma_request_t request;
    dma_sw_priority_t sw_priority;
    dma_data_size_t data_size;
    dma_xfer_mode_t xfer_mode;
    dma_direction_t direction;
    bool memAddrAutoInc;
    bool periphAddrAutoInc;
} dma_config_t;

/**
 * @brief Structure of an instance of DMAx_CHANNELx
 * 
 * @attention DO NOT MODIFY DIRECTLY
 */
typedef struct _dma_handler_t {
    void * parent;
    DMA_TypeDef * dma;
    DMA_Channel_TypeDef * channel;
    const IRQn_Type irqn;
    const uint8_t channelIdx;
    void (*xferCpltCb)(struct _dma_handler_t * dma_handler);
    void (*xferHalfCpltCb)(struct _dma_handler_t * dma_handler);
    void (*xferErrorCb)(struct _dma_handler_t * dma_handler);
} dma_handler_t;

dma_handler_t * dma_init(DMA_Channel_TypeDef * channel, dma_config_t config);
dma_error_t dma_start(dma_handler_t * self, const uint32_t srcAddr, uint32_t destAddr, const size_t dataLength);
dma_error_t dma_start_it(dma_handler_t * self, const uint32_t srcAddr, const uint32_t destAddr, const size_t dataLength, dma_int_t int_ena);
dma_error_t dma_abort(dma_handler_t * self);
bool dma_isXferComplete(dma_handler_t * self, dma_error_t *status);


#endif /* DMA_H */ 