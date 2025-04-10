#include "clock_common.h"

/*--------------------- CLOCK CURRENT FREQUENCY VARIABLES ---------------------*/
static uint32_t sysClk = MSI_VALUE;
static uint32_t hClk   = MSI_VALUE;
static uint32_t pClk1  = MSI_VALUE;
static uint32_t pClk2  = MSI_VALUE;

static const uint8_t  AHBPrescTable[16] = {0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 1U, 2U, 3U, 4U, 6U, 7U, 8U, 9U};
static const uint8_t  APBPrescTable[8] =  {0U, 0U, 0U, 0U, 1U, 2U, 3U, 4U};
static const uint32_t MSIRangeTable[12] = {100000U,   200000U,   400000U,   800000U,  1000000U,  2000000U, \
                                           4000000U, 8000000U, 16000000U, 24000000U, 32000000U, 48000000U};

/**
  * @brief  Update sysClk variable according to Clock Register Values.
  *         The sysClk variable contains the core clock (HCLK), it can
  *         be used by the user application to setup the SysTick timer or configure
  *         other parameters.
  *
  * @note   Each time the core clock (HCLK) changes, this function must be called
  *         to update sysClk variable value. Otherwise, any configuration
  *         based on this variable will be incorrect.
  *
  * @note   - The system frequency computed by this function is not the real
  *           frequency in the chip. It is calculated based on the predefined
  *           constant and the selected clock source:
  *
  *           - If SYSCLK source is MSI, sysClk will contain the MSI_VALUE(*)
  *
  *           - If SYSCLK source is HSI, sysClk will contain the HSI_VALUE(**)
  *
  *           - If SYSCLK source is HSE, sysClk will contain the HSE_VALUE(***)
  *
  *           - If SYSCLK source is PLL, sysClk will contain the HSE_VALUE(***)
  *             or HSI_VALUE(*) or MSI_VALUE(*) multiplied/divided by the PLL factors.
  *
  *         (*) MSI_VALUE is a constant defined in stm32l4xx_hal.h file (default value
  *             4 MHz) but the real value may vary depending on the variations
  *             in voltage and temperature.
  *
  *         (**) HSI_VALUE is a constant defined in stm32l4xx_hal.h file (default value
  *              16 MHz) but the real value may vary depending on the variations
  *              in voltage and temperature.
  *
  *         (***) HSE_VALUE is a constant defined in stm32l4xx_hal.h file (default value
  *              8 MHz), user has to ensure that HSE_VALUE is same as the real
  *              frequency of the crystal used. Otherwise, this function may
  *              have wrong result.
  *
  *         - The result of this function could be not correct when using fractional
  *           value for HSE crystal.
  *
  * @retval None
  */
void updateClkFreq(void) {
    uint32_t tmp, msiFreq, pllvco, pllm, pllr;

    /* Get MSI Range frequency--------------------------------------------------*/
    msiFreq = getMsiFreq();

    /* Update SYSCLK source -------------------------------------------------------*/
    switch (RCC->CFGR & RCC_CFGR_SWS) {
        case RCC_CFGR_SWS_MSI:
            sysClk = msiFreq;
            break;
        case RCC_CFGR_SWS_HSI:
            sysClk = HSI_VALUE;
            break;
        case RCC_CFGR_SWS_HSE:
            sysClk = HSE_VALUE;
            break;
        case RCC_CFGR_SWS_PLL: {
            pllm = ((RCC->PLLCFGR & RCC_PLLCFGR_PLLM) >> RCC_PLLCFGR_PLLM_Pos) + 1U;    // Division factor
            switch (RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) {
                case RCC_PLLCFGR_PLLSRC_HSI:
                    pllvco = (HSI_VALUE / pllm);
                    break;
                case RCC_PLLCFGR_PLLSRC_HSE:
                    pllvco = (HSE_VALUE / pllm);
                    break;
                default: /* MSI used as PLL clock source */
                    pllvco = (msiFreq / pllm);
                    break;
            }
            pllvco = pllvco * ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> RCC_PLLCFGR_PLLN_Pos);
            pllr = (((RCC->PLLCFGR & RCC_PLLCFGR_PLLR) >> 25U) + 1U) * 2U;
            sysClk = pllvco / pllr;
        } break;
        default:
            sysClk = msiFreq;
            break;
    }

    /* Update HCLK clock frequency --------------------------------------------*/
    /* Get HCLK prescaler */
    tmp = AHBPrescTable[((RCC->CFGR & RCC_CFGR_HPRE) >> RCC_CFGR_HPRE_Pos)];
    /* HCLK clock frequency */
    hClk = sysClk >> tmp;

    /* Update PCLK1 clock frequency --------------------------------------------*/
    /* Get PCLK1 prescaler */
    tmp = APBPrescTable[((RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos)];
    /* PCLK1 clock frequency */
    pClk1 = hClk >> tmp;

    /* Update PCLK2 clock frequency --------------------------------------------*/
    /* Get PCLK2 prescaler */
    tmp = APBPrescTable[((RCC->CFGR & RCC_CFGR_PPRE2) >> RCC_CFGR_PPRE2_Pos)];
    /* PCLK2 clock frequency */
    pClk2 = hClk >> tmp;
}

uint32_t getSysClkFreq(void) {
    return sysClk;
}

uint32_t getHClkFreq(void) {
    return hClk;
}

uint32_t getPClk1Freq(void) {
    return pClk1;
}

uint32_t getPClk2Freq(void) {
    return pClk2;
}

uint32_t getMsiFreq(void) {
    uint32_t msiRangeSel;

    if ((RCC->CR & RCC_CR_MSIRGSEL) == 0U) {
        msiRangeSel = (RCC->CSR & RCC_CSR_MSISRANGE) >> RCC_CSR_MSISRANGE_Pos;
    }
    else {
        msiRangeSel = (RCC->CR & RCC_CR_MSIRANGE) >> RCC_CR_MSIRANGE_Pos;
    }
    return MSIRangeTable[msiRangeSel];
}