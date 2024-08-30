#include <stm32l4xx.h>
#include <system_stm32l4xx.h>
#include <stdint.h>
#include <stddef.h>
#include "bsp/gpiomap.h"
#include "bsp/clock.h"
#include "stm32l4xx.h"
#include "usart/uart.h"


volatile uint32_t sysTick_ms = 0;
uart_handle_t myUart = UART4_CONFIG_DEFAULT;


/* Function prototypes */
void SysTick_Handler(void);
void delay_ms(uint32_t ms);
void clock_system_init(void);
void gpio_init(void);


int main(void)
{
    __disable_irq();
    clock_system_init();
    SystemCoreClockUpdate();
    SysTick_Config(SystemCoreClock/1000);
    gpio_init();
	myUart.config.baudrate = UART_BAUDRATE;
	uart_init(&myUart);
    __enable_irq();

	uart_enable(&myUart);

    while(1)
    {
        delay_ms(1000);
        LED_PORT->BSRR |= GPIO_BSRR_BS14;
		uart_puts(&myUart, "Hello World!\r\n");
        delay_ms(1000);
        LED_PORT->BSRR |= GPIO_BSRR_BR14;
		uart_puts(&myUart, "Hello World!\r\n");
    }
}


void SysTick_Handler(void)
{
    sysTick_ms++;
}

void delay_ms(uint32_t ms)
{
    uint32_t delay_end = sysTick_ms + ms;
    while (sysTick_ms < delay_end);
}

void clock_system_init(void)
{
	/* Clock control register (RCC_CR) - reset value 0x00000063 */
	SET_BIT(RCC->CR, RCC_CR_MSION); // Should be default after a reset
	while(!(RCC->CR & RCC_CR_MSIRDY)); // Wait until MSI is stable (ready)
	MODIFY_REG(RCC->CR, RCC_CR_MSIRANGE, RCC_CR_MSIRANGE_7); // 8MHz (default 4MHz)
	SET_BIT(RCC->CR, RCC_CR_MSIRGSEL);

	/*	Read access latency
		To correctly read data from Flash memory, the number of wait states (LATENCY) must be 
		correctly programmed in the Flash access control register (FLASH_ACR) according to the 
		frequency of the CPU clock (HCLK) and the internal voltage range of the device VCORE
	*/
	// Stays 0 WS (1 cpu cycle) for < 16MHz

	/********************************** Configuration **********************************/
	{
	/* Clock configuration register (RCC_CFGR) - reset value 0x00000000 */
	// CLEAR_BIT(RCC->CFGR, RCC_CFGR_MCOPRE);	// Microcontroller clock output prescaler (/1 is default)
	// CLEAR_BIT(RCC->CFGR, RCC_CFGR_MCOSEL);	// Microcontroller clock output source select (disabled is default)
	// CLEAR_BIT(RCC->CFGR, RCC_CFGR_STOPWUCK); // Wakeup from Stop and CSS backup clock select (MSI is default)
	// CLEAR_BIT(RCC->CFGR, RCC_CFGR_PPRE2);	// APB(2) high-speed prescaler (HCLK/1 is default) -> (PCLK2) 
	// CLEAR_BIT(RCC->CFGR, RCC_CFGR_PPRE1);	// APB(1) low-speed prescaler (HCLK/1 is default) -> (PCLK1)
	// CLEAR_BIT(RCC->CFGR, RCC_CFGR_HPRE);		// AHB prescaler (SYSCLK/1 is default) -> HCLK
	// MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, RCC_CFGR_SW_MSI); // System clock switch (MSI is default) -> SYSCLK

	/* PLL configuration register (RCC_PLLCFGR) - reset value 0x0001000 */
	// CLEAR_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLREN);	// Main PLL PLLCLK output enable (disable is default)
	// CLEAR_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLQEN);	// Main PLL PLL48M1CLK output enable (disable is default)
	// CLEAR_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLPEN);	// Main PLL PLLSAI3CLK output enable (disable is default)
	// CLEAR_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLSRC);	// Main PLL, PLLSAI1 and PLLSAI2 entry clock source (none is default, save power)

	/* PLLSAI1 configuration register (RCC_PLLSAI1CFGR) - reset value 0x0001000 */
	// CLEAR_BIT(RCC->PLLSAI1CFGR, RCC_PLLSAI1CFGR_PLLSAI1REN);	// PLLSAI1 PLLADC1CLK output enable (disable is default)
	// CLEAR_BIT(RCC->PLLSAI1CFGR, RCC_PLLSAI1CFGR_PLLSAI1QEN);	// PLLSAI1 PLL48M2CLK output enable (disable is default)
	// CLEAR_BIT(RCC->PLLSAI1CFGR, RCC_PLLSAI1CFGR_PLLSAI1PEN);	// PLLSAI1 PLLSAI1CLK output enable (disable is default)

	/* PLLSAI2 configuration register (RCC_PLLSAI2CFGR) - reset value 0x0001000 */
	// CLEAR_BIT(RCC->PLLSAI2CFGR, RCC_PLLSAI2CFGR_PLLSAI2REN);	// PLLSAI1 PLLADC2CLK output enable (disable is default)
	// CLEAR_BIT(RCC->PLLSAI2CFGR, RCC_PLLSAI2CFGR_PLLSAI2PEN);	// PLLSAI1 PLLSAI2CLK output enable (disable is default)

	/* Peripherals independent clock configuration register (RCC_CCIPR) - reset value 0x00000000 */
	// DFSDM, SWPMI, ADC, CLK48, SAI, LPTIM, I2C, LPUART, U(S)ART clock select
	
	/* Backup domain control register (RCC_BDCR) - reset value 0x00000000 */
	// RTC clock select and enable
	}

	/********************************** Interrruts **********************************/
	{
	/* Clock interrupt enable register (RCC_CIER) - reset value 0x00000000 */
	// Enables clock ready or clk security interrupts here
	}

	/********************************** Resets **********************************/
	{
	/* AHB1 peripheral reset register (RCC_AHB1RSTR) - reset value 0x00000000 */
	// Reset --DMA2D, TSC (Touch Sensing Controller), CRC, Flash, DMA2, DMA1-- here
	// SET_BIT(RCC->AHB1RSTR,	RCC_AHB1RSTR_DMA1RST | RCC_AHB1RSTR_DMA2RST | RCC_AHB1RSTR_FLASHRST |
	// 						RCC_AHB1RSTR_CRCRST  | RCC_AHB1RSTR_TSCRST);

	/* AHB2 peripheral reset register (RCC_AHB2RSTR) - reset value 0x00000000 */
	// Reset --RNG (Random Number Generator), ADC, USB-OTG, GPIO ports-- here
	// SET_BIT(RCC->AHB2RSTR,	RCC_AHB2RSTR_GPIOARST | RCC_AHB2RSTR_GPIOBRST | RCC_AHB2RSTR_GPIOCRST |
	// 						RCC_AHB2RSTR_GPIODRST | RCC_AHB2RSTR_GPIOERST | RCC_AHB2RSTR_GPIOFRST |
	// 						RCC_AHB2RSTR_GPIOGRST | RCC_AHB2RSTR_GPIOHRST);

	/* AHB3 peripheral reset register (RCC_AHB3RSTR) - reset value 0x00000000 */
	// Reset --QSPI (Quad SPI), FMC (Flexible Memory Controller)-- here
	// SET_BIT(RCC->AHB3RSTR, RCC_AHB3RSTR_FMCRST | RCC_AHB3RSTR_QSPIRST);

	/* APB1 peripheral reset register 1 (RCC_APB1RSTR1) - reset value 0x00000000 */
	// Reset --TIM, LPTIM (Low Power Timer), OPAMP, DAC, PWR, CAN, I2C, U(S)ART, SPI-- here
	// SET_BIT(RCC->APB1RSTR1, RCC_APB1RSTR1_TIM2RST | RCC_APB1RSTR1_TIM3RST | RCC_APB1RSTR1_TIM4RST |
	// 						RCC_APB1RSTR1_TIM5RST | RCC_APB1RSTR1_TIM6RST | RCC_APB1RSTR1_TIM7RST |
	// 						RCC_APB1RSTR1_SPI2RST | RCC_APB1RSTR1_SPI3RST | RCC_APB1RSTR1_USART2RST |
	// 						RCC_APB1RSTR1_USART3RST | RCC_APB1RSTR1_UART4RST | RCC_APB1RSTR1_UART5RST |
	// 						RCC_APB1RSTR1_I2C1RST | RCC_APB1RSTR1_I2C2RST | RCC_APB1RSTR1_I2C3RST |
	// 						RCC_APB1RSTR1_CAN1RST | RCC_APB1RSTR1_PWRRST | RCC_APB1RSTR1_DAC1RST |
	// 						RCC_APB1RSTR1_OPAMPRST | RCC_APB1RSTR1_LPTIM1RST);

	/* APB1 peripheral reset register 2 (RCC_APB1RSTR2) - reset value 0x00000000 */
	// Reset --LPTIM (Low Power Timer), SWP (Single Wire Protocol), I2C, LPUART-- here
	// SET_BIT(RCC->APB1RSTR2, RCC_APB1RSTR2_LPUART1RST | RCC_APB1RSTR2_SWPMI1RST | RCC_APB1RSTR2_LPTIM2RST);

	/* APB2 peripheral reset register (RCC_APB2RSTR) - reset value 0x00000000 */
	// Reset --DFSDM (Digital Filter for Sigma-Delta Modulator), SAI (Serial Audio Interface), TIM, USART, SPI, SDMMC, SYSCFG-- here
	// SET_BIT(RCC->APB2RSTR,	RCC_APB2RSTR_SYSCFGRST | RCC_APB2RSTR_SDMMC1RST | RCC_APB2RSTR_TIM1RST |
	// 						RCC_APB2RSTR_SPI1RST | RCC_APB2RSTR_TIM8RST | RCC_APB2RSTR_USART1RST |
	// 						RCC_APB2RSTR_TIM15RST | RCC_APB2RSTR_TIM16RST | RCC_APB2RSTR_TIM17RST |
	// 						RCC_APB2RSTR_SAI1RST | RCC_APB2RSTR_SAI2RST | RCC_APB2RSTR_DFSDM1RST);
	}

	/********************************** Peripherals clock enable **********************************/
	{
	/* AHB1 peripheral clock enable register (RCC_AHB1ENR) - reset value 0x00000000 */
	/* AHB1 peripheral clocks enable in Sleep and Stop modes register (RCC_AHB1SMENR) - reset value 0x00011303 */
	// Enable --DMA, TSC, CRC, Flash-- clock here

	/* AHB2 peripheral clock enable register (RCC_AHB2ENR) - reset value 0x00000000 */
	/* AHB2 peripheral clocks enable in Sleep and Stop modes register (RCC_AHB2SMENR) - reset value 0x000532FF*/
	// Enable --RNG, ADC, USB-OTG, GPIO-- clock here
	SET_BIT(RCC->AHB2ENR, RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN);

	/* AHB3 peripheral clock enable register (RCC_AHB3ENR) - reset value 0x00000000 */
	/* AHB3 peripheral clocks enable in Sleep and Stop modes register (RCC_AHB3SMENR) - reset value 0x000000101 */
	// Enable --QSPI, FMC-- clock here

	/* APB1 peripheral clock enable register 1 (RCC_APB1ENR1) - reset value 0x00000000 */
	/* APB1 peripheral clocks enable in Sleep and Stop modes register 1 (RCC_APB1SMENR1) - reset value 0xF2FECA3F */
	// Enable --TIM, LPTIM, OPAMP, DAC, PWR, CAN, I2C, U(S)ART, SPI, WWDG-- clock here

	/* APB1 peripheral clock enable register 2 (RCC_APB1ENR2) - reset value 0x00000000 */
	/* APB1 peripheral clocks enable in Sleep and Stop modes register 2 (RCC_APB1SMENR2) - reset value 0x00000025 */
	SET_BIT(RCC->APB1ENR1, RCC_APB1ENR1_UART4EN);

	/* APB2 peripheral clock enable register (RCC_APB2ENR) - reset value 0x00000000 */
	/* APB2 peripheral clocks enable in Sleep and Stop modes register (RCC_APB2SMENR) - reset value 0x01677C01 */
	// Enable --DFSDM, SAI, TIM, USART, SPI, SDMMC, SYSCFG, FW (Firewall)-- clock here
	// SET_BIT(RCC->APB2ENR, RCC_APB2ENR_SYSCFGEN);
	}
}

void gpio_init(void)
{
    /* LED gpio configuration */
	// Set GPIOB[14] as General Purpose Output
    MODIFY_REG(LED_PORT->MODER, GPIO_MODER_MODER14, GPIO_MODER_MODER14_0);
    // Set GPIOB[14] to push-pull
	CLEAR_BIT(LED_PORT->OTYPER, GPIO_OTYPER_OT_14);
    // Set GPIOB[14] speed to low on
	CLEAR_BIT(LED_PORT->OSPEEDR, GPIO_OSPEEDR_OSPEED14);
    // Set GPIOB[14] to no-pull-up/down
	CLEAR_BIT(LED_PORT->PUPDR, GPIO_PUPDR_PUPD14);

	/* UART4 gpio configuration */
	// Set GPIOA[1:0] alternate function to UART4 (tx and rx)
	MODIFY_REG(UART_PORT->AFR[0], (GPIO_AFRL_AFSEL0_Msk | GPIO_AFRL_AFSEL1_Msk), 
								  (GPIO_AFRL_AFSEL0_3 | GPIO_AFRL_AFSEL1_3));
	// Set GPIOA pin[1:0] as alternate function
	MODIFY_REG(UART_PORT->MODER, GPIO_MODER_MODER0 | GPIO_MODER_MODER1,
								 GPIO_MODER_MODER0_1 | GPIO_MODER_MODER1_1);
    // Set to no-pull-up/down GPIOA[1:0]
    CLEAR_BIT(UART_PORT->PUPDR, GPIO_PUPDR_PUPD0 | GPIO_PUPDR_PUPD1);
}