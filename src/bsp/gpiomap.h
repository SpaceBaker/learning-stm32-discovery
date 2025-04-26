#ifndef GPIOMAP_H
#define GPIOMAP_H


#include "drivers/stm32l475xx/stm32l475xx.h"
#include "common/macro_magic.h"



/* Serial Comm */
#define UART_PORT (GPIOA)
#define UART_TX_PIN  (0)
#define UART_RX_PIN  (1)
#define LOGGER_UART (UART4)

/* LED */
#define ODR_PIN(pin) CONCAT(GPIO_ODR_OD, pin)
#define LED_PORT GPIOB
#define LED_PIN  14


#endif /* GPIOMAP_H */