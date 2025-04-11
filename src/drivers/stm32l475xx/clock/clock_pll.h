#ifndef __CLOCK_PLL_H
#define __CLOCK_PLL_H


#include "clock_common.h" // IWYU pragma: keep

typedef enum {
    PLL_DEV_PLL = 0,
    PLL_DEV_SAI1,
    PLL_DEV_SAI2,
    PLL_DEV_Max
} pllDev_t;

typedef enum {
    PLL_CLKSRC_NONE = 0,
    PLL_CLKSRC_MSI  = RCC_PLLCFGR_PLLSRC_MSI,
    PLL_CLKSRC_HSI  = RCC_PLLCFGR_PLLSRC_HSI,
    PLL_CLKSRC_HSE  = RCC_PLLCFGR_PLLSRC_HSE,
    PLL_CLKSRC_Msk  = PLL_CLKSRC_HSE
} pllClkSrc_t;

typedef enum {
    PLL_CLKOUT_SYSTEM = 1,
    PLL_CLKOUT_SAI1   = 2,
    PLL_CLKOUT_SAI2   = 4,
    PLL_CLKOUT_SAI3   = 8,
    PLL_CLKOUT_48M1   = 16,
    PLL_CLKOUT_48M2   = 32,
    PLL_CLKOUT_ADC1   = 64,
    PLL_CLKOUT_ADC2   = 128,
    PLL_CLKOUT_Msk    = (PLL_CLKOUT_ADC2 << 1),
    PLL_CLKOUT_N      = 8
} pllClkOut_t;

typedef enum {
    PLL_CLKSRC_DIV_1 = 0,
    PLL_CLKSRC_DIV_2,
    PLL_CLKSRC_DIV_3,
    PLL_CLKSRC_DIV_4,
    PLL_CLKSRC_DIV_5,
    PLL_CLKSRC_DIV_6,
    PLL_CLKSRC_DIV_7,
    PLL_CLKSRC_DIV_8
} pllClkSrcDiv_t;

typedef enum {
    PLLQ_PLLR_DIV_2 = 0,
    PLLQ_PLLR_DIV_4,
    PLLQ_PLLR_DIV_6,
    PLLQ_PLLR_DIV_8
} pllQDiv_t, pllRDiv_t;

typedef enum {
    PLLP_DIV_7 = 0,
    PLLP_DIV_17
} pllPDiv_t;

typedef struct {
    uint32_t vcoMul_plln;
    uint8_t  outClkDiv_pllq;
    uint8_t  outClkDiv_pllr;
    bool     outClkDiv_pllp;
} pllFactors_t;

typedef struct {
    pllClkSrc_t  clkSrc;
    uint8_t      clkSrcDiv_pllm;
    pllClkOut_t  outClkEnable;
    pllFactors_t pllFactors;
    pllFactors_t pllSAI1Factors;
    pllFactors_t pllSAI2Factors;
} pllConfig_t;

void setPllClkSrc(const pllClkSrc_t clksrc);
pllClkSrc_t getPllClkSrc(void);
/**
 * @brief Set the PLLs Clock Source Div Factor (PLLM)
 * Can only be written when all PLLs are disabled.
 * VCO input frequency = (PLL input clock frequency) / (PLLM with 1 <= PLLM <= 8)
 * 
 * @param pllm Allowed values are :
 * 000: PLLM = 1,
 * 001: PLLM = 2,
 * 010: PLLM = 3,
 * 011: PLLM = 4,
 * 100: PLLM = 5,
 * 101: PLLM = 6,
 * 110: PLLM = 7,
 * 111: PLLM = 8
 */
void setPllClkSrcDivFactor(const pllClkSrcDiv_t pllm);
void setPllCfg(pllDev_t pllDev, const uint8_t plln, const pllPDiv_t pllp, const pllQDiv_t pllq, const pllRDiv_t pllr);
void updatePllClkFreq(void);
uint32_t getPllClkFreq(pllClkOut_t clk);


#endif // __CLOCK_PLL_H
