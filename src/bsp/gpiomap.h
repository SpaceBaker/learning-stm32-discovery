#ifndef GPIOMAP_H
#define GPIOMAP_H


#include "stm32l4xx.h"


#define _CONCAT(x,y)    x ## y
#define CONCAT(x,y)     _CONCAT(x, y)

/* Serial Comm */
#define UART_PORT (GPIOA)
#define UART_TX_PIN  (0)
#define UART_RX_PIN  (1)

/* LED */
#define ODR_PIN(pin) CONCAT(GPIO_ODR_OD, pin)
#define LED_PORT GPIOB
#define LED_PIN  14


#endif /* GPIOMAP_H */