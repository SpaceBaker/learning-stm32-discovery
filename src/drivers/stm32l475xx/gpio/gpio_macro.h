#ifndef __GPIO_MACRO_H
#define __GPIO_MACRO_H


#include "../stm32l4xx.h"

#define GPIO_OUTPUT_SET(_port, _pin)     ATOMIC_SET_BIT(_port->BRR, _pin)
#define GPIO_OUTPUT_RESET(_port, _pin)   ATOMIC_SET_BIT(_port->BSRR, ((_pin) << GPIO_BSRR_BR0_Pos))
#define GPIO_OUTPUT_TOGGLE(_port, _pin)  _port->ODR ^= (_pin)

#define GPIO_INPUT_READ_BIT(_port, _pin) (_port->IDR & (_pin))


#endif // __GPIO_MACRO_H
