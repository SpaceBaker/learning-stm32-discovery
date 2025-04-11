#include "clock_periph.h"
#include "clock_pll.h"


void setRtcClkSrc(rtcClkSrc_t clkSrc) {
    ATOMIC_MODIFY_REG(RCC->BDCR, RCC_BDCR_RTCSEL_Msk, clkSrc << RCC_BDCR_RTCSEL_Pos);
}

rtcClkSrc_t getRtcClkSrc(void) {
    return (rtcClkSrc_t)((RCC->BDCR >> RCC_BDCR_RTCSEL_Pos) & RTCCLKSRC_Msk);
}

void setUsartClkSrc(USART_TypeDef * usart, usartClkSrc_t clkSrc) {
    uint32_t pos = 0;
    switch ((uint32_t)usart) {
        case (uint32_t)USART1 :
            pos = RCC_CCIPR_USART1SEL_Pos;
            break;
        case (uint32_t)USART2 :
            pos = RCC_CCIPR_USART2SEL_Pos;
            break;
        case (uint32_t)USART3 :
            pos = RCC_CCIPR_USART3SEL_Pos;
            break;
        case (uint32_t)UART4 :
            pos = RCC_CCIPR_UART4SEL_Pos;
            break;
        case (uint32_t)UART5 :
            pos = RCC_CCIPR_UART5SEL_Pos;
            break;
        case (uint32_t)LPUART1 :
            pos = RCC_CCIPR_LPUART1SEL_Pos;
            break;
        default:
            return;
    }
    ATOMIC_MODIFY_REG(RCC->CCIPR, USARTCLKSRC_Msk << pos, clkSrc << pos);
}

usartClkSrc_t getUsartClkSrc(USART_TypeDef * usart) {
    uint32_t pos = 0;
    switch ((uint32_t)usart) {
        case (uint32_t)USART1 :
            pos = RCC_CCIPR_USART1SEL_Pos;
            break;
        case (uint32_t)USART2 :
            pos = RCC_CCIPR_USART2SEL_Pos;
            break;
        case (uint32_t)USART3 :
            pos = RCC_CCIPR_USART3SEL_Pos;
            break;
        case (uint32_t)UART4 :
            pos = RCC_CCIPR_UART4SEL_Pos;
            break;
        case (uint32_t)UART5 :
            pos = RCC_CCIPR_UART5SEL_Pos;
            break;
        case (uint32_t)LPUART1 :
            pos = RCC_CCIPR_LPUART1SEL_Pos;
            break;
        default:
            return -1;
    }
    return (usartClkSrc_t)((RCC->CCIPR >> pos) & USARTCLKSRC_Msk);
}

void setI2cClkSrc(I2C_TypeDef * i2c, i2cClkSrc_t clkSrc) {
    uint32_t pos = 0;
    switch ((uint32_t)i2c) {
        case (uint32_t)I2C1 :
            pos = RCC_CCIPR_I2C1SEL_Pos;
            break;
        case (uint32_t)I2C2 :
            pos = RCC_CCIPR_I2C2SEL_Pos;
            break;
        case (uint32_t)I2C3 :
            pos = RCC_CCIPR_I2C3SEL_Pos;
            break;
        default:
            return;
    }
    MODIFY_REG(RCC->CCIPR, I2CCLKSRC_Msk << pos, clkSrc << pos);
}

i2cClkSrc_t getI2cClkSrc(I2C_TypeDef * i2c) {
    uint32_t pos = 0;
    switch ((uint32_t)i2c) {
        case (uint32_t)I2C1 :
            pos = RCC_CCIPR_I2C1SEL_Pos;
            break;
        case (uint32_t)I2C2 :
            pos = RCC_CCIPR_I2C2SEL_Pos;
            break;
        case (uint32_t)I2C3 :
            pos = RCC_CCIPR_I2C3SEL_Pos;
            break;
        default:
            return -1;
    }
    return (i2cClkSrc_t)((RCC->CCIPR >> pos) & I2CCLKSRC_Msk);
}

void setLptimClkSrc(LPTIM_TypeDef * lptim, lptimClkSrc_t clkSrc) {
    uint32_t pos = 0;
    switch ((uint32_t)lptim) {
        case (uint32_t)LPTIM1 :
            pos = RCC_CCIPR_LPTIM1SEL_Pos;
            break;
        case (uint32_t)LPTIM2 :
            pos = RCC_CCIPR_LPTIM2SEL_Pos;
            break;
        default:
            return;
    }
    MODIFY_REG(RCC->CCIPR, LPTIMCLKSRC_Msk << pos, clkSrc << pos);
}

lptimClkSrc_t getLptimClkSrc(LPTIM_TypeDef * lptim) {
    uint32_t pos = 0;
    switch ((uint32_t)lptim) {
        case (uint32_t)LPTIM1 :
            pos = RCC_CCIPR_LPTIM1SEL_Pos;
            break;
        case (uint32_t)LPTIM2 :
            pos = RCC_CCIPR_LPTIM2SEL_Pos;
            break;
        default:
            return -1;
    }
    return (lptimClkSrc_t)((RCC->CCIPR >> pos) & LPTIMCLKSRC_Msk);
}

void setSaiClkSrc(SAI_TypeDef * sai, saiClkSrc_t clkSrc) {
    uint32_t pos = 0;
    switch ((uint32_t)sai) {
        case (uint32_t)SAI1 :
            pos = RCC_CCIPR_SAI1SEL_Pos;
            break;
        case (uint32_t)SAI2 :
            pos = RCC_CCIPR_SAI2SEL_Pos;
            break;
        default:
            return;
    }
    MODIFY_REG(RCC->CCIPR, SAICLKSRC_Msk << pos, clkSrc << pos);
}

saiClkSrc_t getSaiClkSrc(SAI_TypeDef * sai) {
    uint32_t pos = 0;
    switch ((uint32_t)sai) {
        case (uint32_t)SAI1 :
            pos = RCC_CCIPR_SAI1SEL_Pos;
            break;
        case (uint32_t)SAI2 :
            pos = RCC_CCIPR_SAI2SEL_Pos;
            break;
        default:
            return -1;
    }
    return (saiClkSrc_t)((RCC->CCIPR >> pos) & SAICLKSRC_Msk);
}

void setAdcClkSrc(adcClkSrc_t clkSrc) {
    MODIFY_REG(RCC->CCIPR, RCC_CCIPR_ADCSEL_Msk, clkSrc << RCC_CCIPR_ADCSEL_Pos);
}

adcClkSrc_t getAdcClkSrc() {
    return (adcClkSrc_t)((RCC->CCIPR >> RCC_CCIPR_ADCSEL_Pos) & ADCCLKSRC_Msk);
}

void setSwpmiClkSrc(bool clkSrc) {
    MODIFY_REG(RCC->CCIPR, RCC_CCIPR_SWPMI1SEL_Msk, clkSrc << RCC_CCIPR_SWPMI1SEL_Pos);
}

bool getSwpmiClkSrc() {
    return ((RCC->CCIPR & RCC_CCIPR_SWPMI1SEL_Msk) >> RCC_CCIPR_SWPMI1SEL_Pos);
}

bool getDfsdmClkSrc() {
    return ((RCC->CCIPR & RCC_CCIPR_DFSDM1SEL_Msk) >> RCC_CCIPR_DFSDM1SEL_Pos);
}

uint32_t getRtcClkFreq(void) {
    uint32_t freq_hz;

    switch ((RCC->BDCR & RCC_BDCR_RTCSEL) >> RCC_BDCR_RTCSEL_Pos) {
        case RTCCLKSRC_LSE :
            freq_hz = LSE_VALUE;
            break;
        case RTCCLKSRC_LSI :
            freq_hz = LSI_VALUE;
            break;
        case RTCCLKSRC_HSE :
            freq_hz = HSE_VALUE / RTC_HSE_PRESCALER;
            break;
        case RTCCLKSRC_NONE :
        default:
            freq_hz = 0;
            break;
    }

    return freq_hz;
}

uint32_t getUsartClkFreq(USART_TypeDef * usart) {
    uint32_t freq_hz = 0;
    usartClkSrc_t clkSrc = getUsartClkSrc(usart);

    switch (clkSrc) {
        case USARTCLKSRC_PCLK :
            if (usart == USART1) {
                freq_hz = getSysClkFreq(PCLK2_ID);
            }
            else {
                freq_hz = getSysClkFreq(PCLK1_ID);
            }
            break;
        case USARTCLKSRC_SYSCLK :
            freq_hz = getSysClkFreq(SYSCLK_ID);
            break;
        case USARTCLKSRC_HSI :
            freq_hz = HSI_VALUE;
            break;
        case USARTCLKSRC_LSE :
            freq_hz = LSE_VALUE;
            break;
        default:
            break;
    }
    
    return freq_hz;
}

uint32_t getI2cClkFreq(I2C_TypeDef * i2c) {
    uint32_t freq_hz = 0;
    i2cClkSrc_t clkSrc = getI2cClkSrc(i2c);

    switch (clkSrc) {
        case I2CCLKSRC_PCLK :
            freq_hz = getSysClkFreq(PCLK1_ID);
            break;
        case I2CCLKSRC_SYSCLK :
            freq_hz = getSysClkFreq(SYSCLK_ID);
            break;
        case I2CCLKSRC_HSI :
            freq_hz = HSI_VALUE;
            break;
        default:
            break;
    }
    
    return freq_hz;
}

uint32_t getLptimClkFreq(LPTIM_TypeDef * lptim) {
    uint32_t freq_hz = 0;
    lptimClkSrc_t clkSrc = getLptimClkSrc(lptim);

    switch (clkSrc) {
        case LPTIMCLKSRC_PCLK :
            freq_hz = getSysClkFreq(PCLK1_ID);
            break;
        case LPTIMCLKSRC_LSI :
            freq_hz = LSI_VALUE;
            break;
        case LPTIMCLKSRC_HSI :
            freq_hz = HSI_VALUE;
            break;
        case LPTIMCLKSRC_LSE :
            freq_hz = LSE_VALUE;
            break;
        default:
            break;
    }
    
    return freq_hz;
}

uint32_t getSaiClkFreq(SAI_TypeDef * sai) {
    uint32_t freq_hz = 0;
    saiClkSrc_t clkSrc = getSaiClkSrc(sai);

    switch (clkSrc) {
        case SAICLKSRC_PLLSAI1 :
            freq_hz = getPllClkFreq(PLL_CLKOUT_SAI1);
            break;
        case SAICLKSRC_PLLSAI2 :
            freq_hz = getPllClkFreq(PLL_CLKOUT_SAI2);
            break;
        case SAICLKSRC_PLLSAI3 :
            freq_hz = getPllClkFreq(PLL_CLKOUT_SAI3);
            break;
        case SAICLKSRC_SAIEXT :
            // TODO
            // freq_hz = getPllSaiExtClkFreq();
            break;
        default:
            break;
    }
    
    return freq_hz;
}

uint32_t getAdcClkFreq(void) {
    uint32_t freq_hz = 0;
    adcClkSrc_t clkSrc = getAdcClkSrc();

    switch (clkSrc) {
        case ADCCLKSRC_PLLSAI1 :
            freq_hz = getPllClkFreq(PLL_CLKOUT_SAI1);
            break;
        case ADCCLKSRC_PLLSAI2 :
            freq_hz = getPllClkFreq(PLL_CLKOUT_SAI2);
            break;
        case ADCCLKSRC_SYSCLK :
            freq_hz = getSysClkFreq(SYSCLK_ID);
            break;
        default:
            break;
    }
    
    return freq_hz;
}

uint32_t getSwpmiClkFreq(void) {
    if (true == getSwpmiClkSrc()) {
        return HSI_VALUE;
    }

    return getSysClkFreq(PCLK1_ID);
}

uint32_t getDfsdmClkFreq(void) {
    if (true == getDfsdmClkSrc()) {
        return getSysClkFreq(SYSCLK_ID);
    }

    return getSysClkFreq(PCLK2_ID);
}
