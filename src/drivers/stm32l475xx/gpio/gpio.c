#include "gpio.h"
#include <stddef.h>

void gpio_configure(GPIO_TypeDef *port, gpioPinIdx_t pinIdx, gpioConfig_t config)
{
    if (NULL == port) return;
    if (GPIO_PIN_IDX_Max <= pinIdx) return;

    /* MODE ------------------------------------ */
    ATOMIC_MODIFY_REG(port->MODER, GPIO_MODE_Msk << (GPIO_MODER_MODE1_Pos * pinIdx), config.mode << (GPIO_MODER_MODE1_Pos * pinIdx));
    /* OUTPUT_TYPE ------------------------------------ */
    ATOMIC_MODIFY_REG(port->OTYPER, GPIO_OTYPE_Msk << (GPIO_OTYPER_OT1_Pos * pinIdx), config.output_type << (GPIO_OTYPER_OT1 * pinIdx));
    /* SPEED ------------------------------------ */
    ATOMIC_MODIFY_REG(port->OSPEEDR, GPIO_SPEED_Msk << (GPIO_OSPEEDR_OSPEED1_Pos * pinIdx), config.speed << (GPIO_OSPEEDR_OSPEED1_Pos * pinIdx));
    /* PUPD ------------------------------------ */
    ATOMIC_MODIFY_REG(port->PUPDR, GPIO_PUPDR_Msk << (GPIO_PUPDR_PUPD1_Pos * pinIdx), config.pupd << (GPIO_PUPDR_PUPD1_Pos * pinIdx));
    /* ALT_FCN ------------------------------------ */
    if (config.mode == GPIO_MODE_ALTFCN) {
        MODIFY_REG(*((uint64_t*)&port->AFR), GPIO_ALTFCN_Msk << (GPIO_AFRL_AFSEL1_Pos * pinIdx), config.alt_fcn << (GPIO_AFRL_AFSEL1_Pos * pinIdx));
    }
}