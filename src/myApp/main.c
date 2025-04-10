#include <stdint.h>
#include <stddef.h>
#include <assert.h>
#include "drivers/stm32l475xx/stm32l4xx.h"
#include "drivers/stm32l475xx/clock/clock.h"
#include "bsp/gpiomap.h"
#include "logger.h"


/* Global variables */
volatile uint32_t sysTick_ms = 0;


/* Function prototypes */
void SysTick_Handler(void);
void delay_ms(uint32_t ms);
void clock_system_init(void);
void gpio_init(void);


int main(void)
{
	const uint32_t led_toggle_interval_ms = 500;
	uint32_t led_toggle_start_ms = 0;
    __disable_irq();
    clock_system_init();
    SysTick_Config(getHClkFreq()/1000);
    gpio_init();
	logger_init();
    __enable_irq();

	while (1)
	{
		if (sysTick_ms - led_toggle_start_ms >= led_toggle_interval_ms) {
			LED_PORT->ODR ^= (1 << LED_PIN);
			led_toggle_start_ms = sysTick_ms;
			logger_write("PING\r\n", 6);
		}
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
	/* Set the Dynamic Voltage Regulation range according to the CPU clock (HCLK) --------- */
	// HCLK up to 80MHz -> Range 1 (1.2V) : High performance
	ATOMIC_MODIFY_REG(PWR->CR1, PWR_CR1_VOS_Msk, PWR_CR1_VOS_0);
	while (PWR_SR2_VOSF == (PWR->SR2 & PWR_SR2_VOSF));	// Wait for voltage regulator to be stabilized
	/* Set FLASH Latency according to the CPU clock (HCLK) ------------------------------- */
	// HCLK up to 80MHz -> 4WS / 5CPU cycles
	ATOMIC_MODIFY_REG(FLASH->ACR, FLASH_ACR_LATENCY_Msk, FLASH_ACR_LATENCY_4WS);
	while (FLASH_ACR_LATENCY_4WS != (FLASH->ACR & FLASH_ACR_LATENCY_4WS));

	/* Set system clock to pll @ 80MHz --------------------------------------------------- */
	/* Turn off PLL, turn on HSI */
	PLL_OFF();
	while (RCC->CR & RCC_CR_PLLRDY);	// Wait for not ready
	HSI_ON();
	waitOscReady(OSC_HSI);
	/* PLL configuration */
	setPllClkSrc(PLL_CLKSRC_HSI);
	setPllClkSrcDivFactor(PLL_CLKSRC_DIV_1);	// VCO = 16MHz/1
	setPllCfg(PLL_DEV_PLL, 10, PLLP_DIV_17, PLLQ_PLLR_DIV_8, PLLQ_PLLR_DIV_2);	// pll_clk = ((VCO * PLLN) / PLLQ) ==> 80MHz = ((16MHz * 40) / 8)
	/* Turn on PLL */
	PLL_SYS_CLK_ENABLE();
	PLL_ON();
	waitOscReady(OSC_PLL);
	/* Set system prescalers ------------------------------------------------------------- */
	setAhbPrescaler(AHBPRE_NONE);	// For HCLK		-> HCLK  = sysClk/ahbPre
	setApb1Prescaler(APB1PRE_NONE);	// For PCLK1	-> PCLK1 = HCLK/apb1Pre
	setApb2Prescaler(APB2PRE_NONE);	// For PCLK2	-> PCLK2 = HCLK/apb2Pre
	/* Set sysClk source to PLL, turn off MSI */
	setSysClkSrc(SYSCLKSRC_PLL);
	while ((SYSCLKSRC_PLL << RCC_CFGR_SWS_Pos) != (RCC->CFGR & (SYSCLKSRC_PLL << RCC_CFGR_SWS_Pos)));	// Wait for sysclk source changed
	MSI_OFF();

	/* Enable peripherals clocks --------------------------------------------------------- */
	GPIOA_CLK_ENABLE();
	GPIOB_CLK_ENABLE();
	UART4_CLK_ENABLE();
	DMA2_CLK_ENABLE();

	/* Update freq variables ------------------------------------------------------------- */
	updateClkFreq();
	updatePllClkFreq();
}

#if 0
void clock_system_init(void)
{
	/* Clock control register (RCC_CR) - reset value 0x00000063 */
	SET_BIT(RCC->CR, RCC_CR_MSION); // Should be default after a reset
	while (!(RCC->CR & RCC_CR_MSIRDY)); // Wait until MSI is stable (ready)
	MODIFY_REG(RCC->CR, RCC_CR_MSIRANGE, RCC_CR_MSIRANGE_8); // 16MHz (6 is default 4MHz)
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
	// ATOMIC_CLEAR_BIT(RCC->CCIPR, RCC_CCIPR_UART4SEL_Msk);	// UART4 clock select (PCLK1 is default)
	
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
	SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_DMA2EN);

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
	SET_BIT(RCC->APB1ENR1, RCC_APB1ENR1_UART4EN);

	/* APB1 peripheral clock enable register 2 (RCC_APB1ENR2) - reset value 0x00000000 */
	/* APB1 peripheral clocks enable in Sleep and Stop modes register 2 (RCC_APB1SMENR2) - reset value 0x00000025 */

	/* APB2 peripheral clock enable register (RCC_APB2ENR) - reset value 0x00000000 */
	/* APB2 peripheral clocks enable in Sleep and Stop modes register (RCC_APB2SMENR) - reset value 0x01677C01 */
	// Enable --DFSDM, SAI, TIM, USART, SPI, SDMMC, SYSCFG, FW (Firewall)-- clock here
	// SET_BIT(RCC->APB2ENR, RCC_APB2ENR_SYSCFGEN);
	}
}
#endif

void gpio_init(void)
{
    /* LED gpio configuration */
	// Set GPIOB[14] as General Purpose Output
    MODIFY_REG(LED_PORT->MODER, GPIO_MODER_MODE14, GPIO_MODER_MODE14_0);
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
	MODIFY_REG(UART_PORT->MODER, GPIO_MODER_MODE0_Msk | GPIO_MODER_MODE1_Msk,
								 GPIO_MODER_MODE0_1 | GPIO_MODER_MODE1_1);
    // Set to no-pull-up/down GPIOA[1:0]
    CLEAR_BIT(UART_PORT->PUPDR, GPIO_PUPDR_PUPD0 | GPIO_PUPDR_PUPD1);
    // Set GPIOA[1:0] to push-pull
	CLEAR_BIT(UART_PORT->OTYPER, (GPIO_OTYPER_OT0 | GPIO_OTYPER_OT1));
    // Set GPIOA[1:0] speed to very high
	SET_BIT(UART_PORT->OSPEEDR, (GPIO_OSPEEDR_OSPEED0 | GPIO_OSPEEDR_OSPEED1));
}