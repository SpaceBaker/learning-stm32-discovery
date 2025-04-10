/**
 * @file clock.c
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2025-04-04
 * 
 * @copyright Copyright (c) 2025
 * 
*/

#include "clock.h"


void waitOscReady(const osc_t osc) {
    for (osc_t osc_idx = OSC_Start; osc_idx < OSC_Msk; osc_idx <<= 1) {
        switch (osc & osc_idx) {
            case OSC_MSI:
                while (RCC_CR_MSION != (RCC->CR & RCC_CR_MSION));
                break;
            case OSC_LSI:
                while (RCC_CSR_LSION != (RCC->CSR & RCC_CSR_LSION));
                break;
            case OSC_HSI:
                while (RCC_CR_HSION != (RCC->CR & RCC_CR_HSION));
                break;
            case OSC_LSE:
                while (RCC_BDCR_LSEON != (RCC->BDCR & RCC_BDCR_LSEON));
                break;
            case OSC_HSE:
                while (RCC_CR_HSEON != (RCC->CR & RCC_CR_HSEON));
                break;
            case OSC_PLL:
                while (RCC_CR_PLLON != (RCC->CR & RCC_CR_PLLON));
                break;
            case OSC_PLLSAI1:
                while (RCC_CR_PLLSAI1ON != (RCC->CR & RCC_CR_PLLSAI1ON));
                break;
            case OSC_PLLSAI2:
                while (RCC_CR_PLLSAI2ON != (RCC->CR & RCC_CR_PLLSAI2ON));
                break;
            default: 
                break;
        }
    }
}

void setMsiRange(const msiRange_t msiRange) {
    if (msiRange <= MSI_RANGE_Last) {
        ATOMIC_MODIFY_REG(RCC->CR, RCC_CR_MSIRANGE_Msk, msiRange << RCC_CR_MSIRANGE_Pos);
        SET_BIT(RCC->CR, RCC_CR_MSIRGSEL);
    }
}

void setMsiRstRange(const msiRstRange_t msiRstRange) {
    if ((msiRstRange >= MSI_RST_RANGE_First) && (msiRstRange <= MSI_RST_RANGE_Last)) {
        ATOMIC_MODIFY_REG(RCC->CR, RCC_CSR_MSISRANGE_Msk, msiRstRange << RCC_CSR_MSISRANGE_Pos);
    }
}

void setApb1Prescaler(const apb1Prescaler_t pre) {
    if (0 == (pre & ~APB1PRE_Msk)) {
        ATOMIC_MODIFY_REG(RCC->CFGR, APB1PRE_Msk, pre);
    }
}

void setApb2Prescaler(const apb2Prescaler_t pre) {
    if (0 == (pre & ~APB2PRE_Msk)) {
        ATOMIC_MODIFY_REG(RCC->CFGR, APB2PRE_Msk, pre);
    }
}

void setAhbPrescaler(const ahbPrescaler_t pre) {
    if (0 == (pre & ~AHBPRE_Msk)) {
        ATOMIC_MODIFY_REG(RCC->CFGR, AHBPRE_Msk, pre);
    }
}

void setSysClkSrc(const sysClkSrc_t clksrc) {
    if (0 == (clksrc & ~SYSCLKSRC_Msk)) {
        ATOMIC_MODIFY_REG(RCC->CFGR, SYSCLKSRC_Msk, clksrc);
    }
}

sysClkSrc_t getSysClkSrc(void) {
    return (sysClkSrc_t)((RCC->CFGR & RCC_CFGR_SWS_Msk) >> RCC_CFGR_SWS_Pos);
}



#if 0

void calibrateMsi(/*TBD*/) {

}

void calibrateHsi16(/*TBD*/) {

}

void calibrateLsi(/*TBD*/) {
    
}

#endif