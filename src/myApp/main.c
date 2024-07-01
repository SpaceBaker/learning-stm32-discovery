#include <stm32l4xx.h>
#include <system_stm32l4xx.h>
#include <stdint.h>
#include "rcc.h"

#define LED_PORT GPIOB
#define LED_PIN  (1<<14)


void led_init(void)
{
    // LED gpio configuration
    MODIFY_REG(LED_PORT->MODER, GPIO_MODER_MODER14, GPIO_MODER_MODER14_0);  // Set GPIOB Pin 14 as General Purpose Output
    CLEAR_BIT(LED_PORT->OTYPER, GPIO_OTYPER_OT_14);                         // Set to push-pull Pin 14
    CLEAR_BIT(LED_PORT->OSPEEDR, GPIO_OSPEEDR_OSPEED14);                    // Set speed to low on Pin 14
    CLEAR_BIT(LED_PORT->PUPDR, GPIO_PUPDR_PUPDR14);                         // Set to no-pull-up/down Pin 14
}

int main(void)
{
    uint32_t sysTick = 0;

    __set_PRIMASK(1);
    rcc_init();
    SystemCoreClockUpdate();
    led_init();
    __set_PRIMASK(0);

    while(1)
    {
        sysTick++;
        /* Toggle user led every second */
        if (0 == (sysTick % (SystemCoreClock))) {
            LED_PORT->ODR ^= LED_PIN;
            sysTick = 0;
        }
    }
}