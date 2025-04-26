#ifndef __GPIO_H
#define __GPIO_H


#include "../stm32l4xx.h"   // IWYU pragma: keep
#include "gpio_macro.h"     // IWYU pragma: keep

typedef enum {
    GPIO_MODE_INPUT = 0,
    GPIO_MODE_OUTPUT,
    GPIO_MODE_ALTFCN,
    GPIO_MODE_ANALOG,
    GPIO_MODE_Msk = GPIO_MODE_ANALOG,
    GPIO_MODE_Max
} gpioMode_t;

typedef enum {
    GPIO_OTYPE_PUSHPULL = 0,
    GPIO_OTYPE_OPENDRAIN,
    GPIO_OTYPE_Msk = GPIO_OTYPE_OPENDRAIN
} gpioOType_t;

typedef enum {
    GPIO_SPEED_LOW = 0,
    GPIO_SPEED_MEDIUM,
    GPIO_SPEED_HIGH,
    GPIO_SPEED_VERYHIGH,
    GPIO_SPEED_Msk = GPIO_SPEED_VERYHIGH,
    GPIO_SPEED_Max
} gpioSpeed_t;

typedef enum {
    GPIO_PUPDR_NONE = 0,
    GPIO_PUPDR_PULLUP,
    GPIO_PUPDR_PULLDOWN,
    GPIO_PUPDR_Max,
    GPIO_PUPDR_Msk = GPIO_PUPDR_Max
} gpioPupd_t;

/**
 * @note See the datasheet to know the alternate function mapping
 * 
 */
typedef enum {
    GPIO_ALTFCN_0 = 0,
    GPIO_ALTFCN_1,
    GPIO_ALTFCN_2,
    GPIO_ALTFCN_3,
    GPIO_ALTFCN_4,
    GPIO_ALTFCN_5,
    GPIO_ALTFCN_6,
    GPIO_ALTFCN_7,
    GPIO_ALTFCN_8,
    GPIO_ALTFCN_9,
    GPIO_ALTFCN_10,
    GPIO_ALTFCN_11,
    GPIO_ALTFCN_12,
    GPIO_ALTFCN_13,
    GPIO_ALTFCN_14,
    GPIO_ALTFCN_15,
    GPIO_ALTFCN_Msk = GPIO_ALTFCN_15,
    GPIO_ALTFCN_Max
} gpioAltFcn_t;

typedef enum {
    GPIO_PIN_IDX_0 = 0,
    GPIO_PIN_IDX_1,
    GPIO_PIN_IDX_2,
    GPIO_PIN_IDX_3,
    GPIO_PIN_IDX_4,
    GPIO_PIN_IDX_5,
    GPIO_PIN_IDX_6,
    GPIO_PIN_IDX_7,
    GPIO_PIN_IDX_8,
    GPIO_PIN_IDX_9,
    GPIO_PIN_IDX_10,
    GPIO_PIN_IDX_11,
    GPIO_PIN_IDX_12,
    GPIO_PIN_IDX_13,
    GPIO_PIN_IDX_14,
    GPIO_PIN_IDX_15,
    GPIO_PIN_IDX_Msk = GPIO_PIN_IDX_15,
    GPIO_PIN_IDX_Max
} gpioPinIdx_t;

typedef struct {
    gpioMode_t      mode;
    gpioOType_t     output_type;
    gpioSpeed_t     speed;
    gpioPupd_t      pupd;
    gpioAltFcn_t    alt_fcn;
} gpioConfig_t;

void gpio_configure(GPIO_TypeDef *port, gpioPinIdx_t pinIdx, gpioConfig_t config);


#endif // __GPIO_H
