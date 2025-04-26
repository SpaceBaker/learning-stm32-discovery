#include <stdint.h>
#include <stddef.h>
#include <assert.h>
#include "drivers/stm32l475xx/stm32l4xx.h"
#include "drivers/stm32l475xx/gpio/gpio.h"
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
    SysTick_Config(getSysClkFreq(HCLK_ID)/1000);
    gpio_init();
	logger_init();
    __enable_irq();

	while (1)
	{
		if (sysTick_ms - led_toggle_start_ms >= led_toggle_interval_ms) {
			GPIO_OUTPUT_TOGGLE(LED_PORT, (1 << LED_PIN));
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

void gpio_init(void)
{
	/* LED gpio configuration */
	gpioConfig_t config = {
		.mode = GPIO_MODE_OUTPUT,
		.output_type = GPIO_OTYPE_PUSHPULL,
		.speed = GPIO_SPEED_LOW,
		.pupd = GPIO_PUPDR_NONE,
		.alt_fcn = GPIO_ALTFCN_Max
	};
	gpio_configure(LED_PORT, LED_PIN, config);

	/* UART4 gpio configuration */
	config.mode = GPIO_MODE_ALTFCN;
	config.output_type = GPIO_OTYPE_PUSHPULL;
	config.speed = GPIO_SPEED_VERYHIGH;
	config.pupd = GPIO_PUPDR_NONE;
	config.alt_fcn = GPIO_ALTFCN_8;
	gpio_configure(UART_PORT, UART_TX_PIN, config);
	gpio_configure(UART_PORT, UART_RX_PIN, config);
}