#ifndef __CLOCK_H
#define __CLOCK_H


#include "clock_common.h" // IWYU pragma: keep
#include "clock_pll.h"    // IWYU pragma: keep
#include "clock_periph.h" // IWYU pragma: keep

typedef enum {
    OSC_Start   = 1,
    OSC_MSI     = 1,
    OSC_LSI     = 2,
    OSC_HSI     = 4,
    OSC_LSE     = 8,
    OSC_HSE     = 16,
    OSC_PLL     = 32,
    OSC_PLLSAI1 = 64,
    OSC_PLLSAI2 = 128,
    OSC_Msk     = 256
} osc_t;

typedef enum {
    MSI_RANGE_First = 0,
    MSI_RANGE_100000 = 0,
    MSI_RANGE_200000,
    MSI_RANGE_400000,
    MSI_RANGE_800000,
    MSI_RANGE_1000000,
    MSI_RANGE_2000000,
    MSI_RANGE_4000000,
    MSI_RANGE_8000000,
    MSI_RANGE_16000000,
    MSI_RANGE_24000000,
    MSI_RANGE_32000000,
    MSI_RANGE_48000000,
    MSI_RANGE_Last = MSI_RANGE_48000000
} msiRange_t;

typedef enum {
    MSI_RST_RANGE_First = 4,
    MSI_RST_RANGE_1000000 = 4,
    MSI_RST_RANGE_2000000,
    MSI_RST_RANGE_4000000,
    MSI_RST_RANGE_8000000,
    MSI_RST_RANGE_Last = MSI_RST_RANGE_8000000
} msiRstRange_t;

typedef enum {
    SYSCLKSRC_MSI = RCC_CFGR_SW_MSI,
    SYSCLKSRC_HSI = RCC_CFGR_SW_HSI,
    SYSCLKSRC_HSE = RCC_CFGR_SW_HSE,
    SYSCLKSRC_PLL = RCC_CFGR_SW_PLL,
    SYSCLKSRC_Msk = (SYSCLKSRC_MSI | SYSCLKSRC_HSI | SYSCLKSRC_HSE | SYSCLKSRC_PLL)
} sysClkSrc_t;

typedef enum {
    APB1PRE_NONE = 0,
    APB1PRE_2    = RCC_CFGR_PPRE1_2,
    APB1PRE_4    = RCC_CFGR_PPRE1_2 | RCC_CFGR_PPRE1_0,
    APB1PRE_8    = RCC_CFGR_PPRE1_2 | RCC_CFGR_PPRE1_1,
    APB1PRE_16   = RCC_CFGR_PPRE1_2 | RCC_CFGR_PPRE1_1 | RCC_CFGR_PPRE1_0,
    APB1PRE_Msk  = RCC_CFGR_PPRE1_Msk
} apb1Prescaler_t;

typedef enum {
    APB2PRE_NONE = 0,
    APB2PRE_2    = RCC_CFGR_PPRE2_2,
    APB2PRE_4    = RCC_CFGR_PPRE2_2 | RCC_CFGR_PPRE2_0,
    APB2PRE_8    = RCC_CFGR_PPRE2_2 | RCC_CFGR_PPRE2_1,
    APB2PRE_16   = RCC_CFGR_PPRE2_2 | RCC_CFGR_PPRE2_1 | RCC_CFGR_PPRE2_0,
    APB2PRE_Msk  = RCC_CFGR_PPRE2_Msk
} apb2Prescaler_t;

typedef enum {
    AHBPRE_NONE  = 0,
    AHBPRE_2     = RCC_CFGR_HPRE_3,
    AHBPRE_4     = RCC_CFGR_HPRE_3 | RCC_CFGR_HPRE_0,
    AHBPRE_8     = RCC_CFGR_HPRE_3 | RCC_CFGR_HPRE_1,
    AHBPRE_16    = RCC_CFGR_HPRE_3 | RCC_CFGR_HPRE_1 | RCC_CFGR_HPRE_0,
    AHBPRE_64    = RCC_CFGR_HPRE_3 | RCC_CFGR_HPRE_2,
    AHBPRE_128   = RCC_CFGR_HPRE_3 | RCC_CFGR_HPRE_2 | RCC_CFGR_HPRE_0,
    AHBPRE_256   = RCC_CFGR_HPRE_3 | RCC_CFGR_HPRE_2 | RCC_CFGR_HPRE_1,
    AHBPRE_512   = RCC_CFGR_HPRE_3 | RCC_CFGR_HPRE_2 | RCC_CFGR_HPRE_1 | RCC_CFGR_HPRE_0,
    AHBPRE_Msk   = RCC_CFGR_HPRE_Msk
} ahbPrescaler_t;

// typedef struct {
//     osc_t           oscEnable;
//     sysClkSrc_t     sysClkSrc;
//     msiRange_t      msiRange;
//     apb1Prescaler_t apb1Pre;
//     apb2Prescaler_t apb2Pre;
//     ahbPrescaler_t  ahbPre;
//     pllConfig_t     pllConfig;
// } clockConfig_t;

void waitOscReady(const osc_t osc);
void setMsiRange(const msiRange_t msiRange);
void setMsiRstRange(const msiRstRange_t msiRstRange);
void setMsiRstRange(msiRstRange_t msiRstRange);
void setApb1Prescaler(const apb1Prescaler_t pre);
void setApb2Prescaler(const apb2Prescaler_t pre);
void setAhbPrescaler(const ahbPrescaler_t pre);
void setSysClkSrc(const sysClkSrc_t clksrc);
sysClkSrc_t getSysClkSrc(void);


#endif // __CLOCK_H
