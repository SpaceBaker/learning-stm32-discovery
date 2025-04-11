#ifndef __CLOCK_PERIPH_H
#define __CLOCK_PERIPH_H


#include "clock_common.h" // IWYU pragma: keep


#define RTC_HSE_PRESCALER (32u)

typedef enum {
    RTCCLKSRC_NONE = 0,
    RTCCLKSRC_LSE,
    RTCCLKSRC_LSI,
    RTCCLKSRC_HSE,
    RTCCLKSRC_Msk  = RTCCLKSRC_HSE
} rtcClkSrc_t;

typedef enum {
    USARTCLKSRC_PCLK = 0,
    USARTCLKSRC_SYSCLK,
    USARTCLKSRC_HSI,
    USARTCLKSRC_LSE,
    USARTCLKSRC_Msk = USARTCLKSRC_LSE
} usartClkSrc_t;

typedef enum {
    I2CCLKSRC_PCLK = 0,
    I2CCLKSRC_SYSCLK,
    I2CCLKSRC_HSI,
    I2CCLKSRC_Msk
} i2cClkSrc_t;

typedef enum {
    LPTIMCLKSRC_PCLK = 0,
    LPTIMCLKSRC_LSI,
    LPTIMCLKSRC_HSI,
    LPTIMCLKSRC_LSE,
    LPTIMCLKSRC_Msk = LPTIMCLKSRC_LSE
} lptimClkSrc_t;

typedef enum {
    SAICLKSRC_PLLSAI1 = 0,
    SAICLKSRC_PLLSAI2,
    SAICLKSRC_PLLSAI3,
    SAICLKSRC_SAIEXT,
    SAICLKSRC_Msk = SAICLKSRC_SAIEXT
} saiClkSrc_t;

typedef enum {
    ADCCLKSRC_NONE = 0,
    ADCCLKSRC_PLLSAI1,
    ADCCLKSRC_PLLSAI2,
    ADCCLKSRC_SYSCLK,
    ADCCLKSRC_Msk = ADCCLKSRC_SYSCLK
} adcClkSrc_t;

void setRtcClkSrc(rtcClkSrc_t clkSrc);
void setUsartClkSrc(USART_TypeDef * usart, usartClkSrc_t clkSrc);
void setI2cClkSrc(I2C_TypeDef * i2c, i2cClkSrc_t clkSrc);
void setLptimClkSrc(LPTIM_TypeDef * lptim, lptimClkSrc_t clkSrc);
void setSaiClkSrc(SAI_TypeDef * sai, saiClkSrc_t clkSrc);
void setAdcClkSrc(adcClkSrc_t clkSrc);
/**
 * @brief Set the Swpmi Clk Src object
 * 
 * @param clkSrc 0: PCLK, 1: HSI
 */
void setSwpmiClkSrc(bool clkSrc);
/**
 * @brief Set the Dfsdm Clk Src object
 * 
 * @param clkSrc 0: PCLK, 1: SYSCLK
 */
void setDfsdmClkSrc(bool clkSrc);


uint32_t getRtcClkFreq(void);
uint32_t getUsartClkFreq(USART_TypeDef * usart);
uint32_t getI2cClkFreq(I2C_TypeDef * i2c);
uint32_t getLptimClkFreq(LPTIM_TypeDef * lptim);
uint32_t getSaiClkFreq(SAI_TypeDef * sai);
uint32_t getAdcClkFreq(void);
uint32_t getSwpmiClkFreq(void);
uint32_t getDfsdmClkFreq(void);


#endif // __CLOCK_PERIPH_H
