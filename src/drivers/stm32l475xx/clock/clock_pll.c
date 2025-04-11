#include "clock_pll.h"
#include <assert.h>


enum pllClkOut_idx_t {
    PLL_CLKOUT_SYSTEM_IDX= 0,
    PLL_CLKOUT_SAI1_IDX,
    PLL_CLKOUT_SAI2_IDX,
    PLL_CLKOUT_SAI3_IDX,
    PLL_CLKOUT_48M1_IDX,
    PLL_CLKOUT_48M2_IDX,
    PLL_CLKOUT_ADC1_IDX,
    PLL_CLKOUT_ADC2_IDX,
    PLL_CLKOUT_IDX_Max
};
static_assert((size_t)PLL_CLKOUT_IDX_Max == (size_t)PLL_CLKOUT_N, "enum not in synch!");

/*--------------------- STATIC GLOBAL VARIABLES ---------------------*/
static __IO uint32_t * pll_cfgr_reg[PLL_DEV_Max] = {&RCC->PLLCFGR, &RCC->PLLSAI1CFGR, &RCC->PLLSAI2CFGR};
static uint32_t pllOutClkFreq[PLL_CLKOUT_N] = {0};

void setPllClkSrc(const pllClkSrc_t clksrc) {
    if (0 == (clksrc & ~PLL_CLKSRC_Msk)) {
        ATOMIC_MODIFY_REG(RCC->PLLCFGR, PLL_CLKSRC_Msk, clksrc);
    }
}

pllClkSrc_t getPllClkSrc(void) {
    return (pllClkSrc_t)(RCC->PLLCFGR & PLL_CLKSRC_Msk);
}

void setPllClkSrcDivFactor(const pllClkSrcDiv_t pllm) {
    if (0 == (pllm & ~(RCC_PLLCFGR_PLLM_Msk >> RCC_PLLCFGR_PLLM_Pos))) {
        ATOMIC_MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLLM_Msk, pllm << RCC_PLLCFGR_PLLM_Pos);
    }
}

/**
 * @brief Configure a PLL (main, SAI1 or SAI2)
 *
 * See Referance Manual to provide allowed values 
 * 
 * @param pllcfgr_reg 
 * @param plln 8 < plln < 86
 * @param pllp 0: pllp = 7, 1: pllp = 17
 * @param pllq Values can be :
 * 00: pllq = 2,
 * 01: pllq = 4,
 * 10: pllq = 6,
 * 11: pllq = 8,
 * Always 0 for PLLSAI2
 * @param pllr Values can be :
 * 00: pllr = 2,
 * 01: pllr = 4,
 * 10: pllr = 6,
 * 11: pllr = 8
*/
void setPllCfg(pllDev_t pllDev, const uint8_t plln, const pllPDiv_t pllp, const pllQDiv_t pllq, const pllRDiv_t pllr) {
    if (pllDev >= PLL_DEV_Max) return;
    if ((plln < 8) || (plln > 86)) return;
    if (pllq > (RCC_PLLCFGR_PLLQ_Msk >> RCC_PLLCFGR_PLLQ_Pos)) return;
    if (pllr > (RCC_PLLCFGR_PLLR_Msk >> RCC_PLLCFGR_PLLR_Pos)) return;

    uint32_t clear_mask = RCC_PLLCFGR_PLLR_Msk | RCC_PLLCFGR_PLLQ_Msk | RCC_PLLCFGR_PLLP_Msk | RCC_PLLCFGR_PLLN_Msk; 
    uint32_t set_mask = (pllr << RCC_PLLCFGR_PLLR_Pos) | (pllq << RCC_PLLCFGR_PLLQ_Pos) | (pllp << RCC_PLLCFGR_PLLP_Pos) | (plln << RCC_PLLCFGR_PLLN_Pos); 

    ATOMIC_MODIFY_REG(*pll_cfgr_reg[pllDev], clear_mask, set_mask);
}

pllConfig_t getPllConfig(void) {
    uint32_t cfg_reg_cpy;
    pllConfig_t config = {0};

    /* PLL */
    cfg_reg_cpy = RCC->PLLCFGR;
    config.clkSrc         = (cfg_reg_cpy  & RCC_PLLCFGR_PLLSRC_Msk) >> RCC_PLLCFGR_PLLSRC_Pos;
    config.clkSrcDiv_pllm = ((cfg_reg_cpy & RCC_PLLCFGR_PLLM_Msk) >> RCC_PLLCFGR_PLLM_Pos);
    if (cfg_reg_cpy & RCC_PLLCFGR_PLLREN) config.outClkEnable |= PLL_CLKOUT_SYSTEM;
    if (cfg_reg_cpy & RCC_PLLCFGR_PLLPEN) config.outClkEnable |= PLL_CLKOUT_SAI3;
    if (cfg_reg_cpy & RCC_PLLCFGR_PLLQEN) config.outClkEnable |= PLL_CLKOUT_48M1;
    config.pllFactors.vcoMul_plln    = (cfg_reg_cpy & RCC_PLLCFGR_PLLN_Msk) >> RCC_PLLCFGR_PLLN_Pos;
    config.pllFactors.outClkDiv_pllp = (cfg_reg_cpy & RCC_PLLCFGR_PLLP_Msk) >> RCC_PLLCFGR_PLLP_Pos;
    config.pllFactors.outClkDiv_pllq = (cfg_reg_cpy & RCC_PLLCFGR_PLLQ_Msk) >> RCC_PLLCFGR_PLLQ_Pos;
    config.pllFactors.outClkDiv_pllr = (cfg_reg_cpy & RCC_PLLCFGR_PLLR_Msk) >> RCC_PLLCFGR_PLLR_Pos;

    /* PLLSAI1 */
    cfg_reg_cpy = RCC->PLLSAI1CFGR;
    if (cfg_reg_cpy & RCC_PLLSAI1CFGR_PLLSAI1PEN) config.outClkEnable |= PLL_CLKOUT_SAI1;
    if (cfg_reg_cpy & RCC_PLLSAI1CFGR_PLLSAI1QEN) config.outClkEnable |= PLL_CLKOUT_48M2;
    if (cfg_reg_cpy & RCC_PLLSAI1CFGR_PLLSAI1REN) config.outClkEnable |= PLL_CLKOUT_ADC1;
    config.pllSAI1Factors.vcoMul_plln    = (cfg_reg_cpy & RCC_PLLSAI1CFGR_PLLSAI1N_Msk) >> RCC_PLLSAI1CFGR_PLLSAI1N_Pos;
    config.pllSAI1Factors.outClkDiv_pllp = (cfg_reg_cpy & RCC_PLLSAI1CFGR_PLLSAI1P_Msk) >> RCC_PLLSAI1CFGR_PLLSAI1P_Pos;
    config.pllSAI1Factors.outClkDiv_pllq = (cfg_reg_cpy & RCC_PLLSAI1CFGR_PLLSAI1Q_Msk) >> RCC_PLLSAI1CFGR_PLLSAI1Q_Pos;
    config.pllSAI1Factors.outClkDiv_pllr = (cfg_reg_cpy & RCC_PLLSAI1CFGR_PLLSAI1R_Msk) >> RCC_PLLSAI1CFGR_PLLSAI1R_Pos;

    /* PLLSAI2 */
    cfg_reg_cpy = RCC->PLLSAI2CFGR;
    if (cfg_reg_cpy & RCC_PLLSAI2CFGR_PLLSAI2PEN) config.outClkEnable |= PLL_CLKOUT_SAI2;
    if (cfg_reg_cpy & RCC_PLLSAI2CFGR_PLLSAI2REN) config.outClkEnable |= PLL_CLKOUT_ADC2;
    config.pllSAI2Factors.vcoMul_plln    = (cfg_reg_cpy & RCC_PLLSAI2CFGR_PLLSAI2N_Msk) >> RCC_PLLSAI2CFGR_PLLSAI2N_Pos;
    config.pllSAI2Factors.outClkDiv_pllp = (cfg_reg_cpy & RCC_PLLSAI2CFGR_PLLSAI2P_Msk) >> RCC_PLLSAI2CFGR_PLLSAI2P_Pos;
    config.pllSAI2Factors.outClkDiv_pllq = 0;
    config.pllSAI2Factors.outClkDiv_pllr = (cfg_reg_cpy & RCC_PLLSAI2CFGR_PLLSAI2R_Msk) >> RCC_PLLSAI2CFGR_PLLSAI2R_Pos;

    return config;
}

void updatePllClkFreq(void) {
    uint32_t freq, pllvco;
    pllConfig_t config = getPllConfig();

    switch (config.clkSrc) {
        case PLL_CLKSRC_MSI:
            freq = (getMsiFreq() / (config.clkSrcDiv_pllm + 1));
            break;
        case PLL_CLKSRC_HSI:
            freq = (HSI_VALUE / (config.clkSrcDiv_pllm + 1));
            break;
        case PLL_CLKSRC_HSE:
            freq = (HSE_VALUE / (config.clkSrcDiv_pllm + 1));
            break;
        case PLL_CLKSRC_NONE :
        default:
            for (enum pllClkOut_idx_t i = 0; i < PLL_CLKOUT_IDX_Max; i++) {
                pllOutClkFreq[i] = 0;
            }
            return;
    }

    /* PLL */
    if (RCC->CR & RCC_CR_PLLON) {
        pllvco = freq * config.pllFactors.vcoMul_plln;
        if (config.outClkEnable & PLL_CLKOUT_SYSTEM) {
            pllOutClkFreq[PLL_CLKOUT_SYSTEM_IDX] = pllvco / ((config.pllFactors.outClkDiv_pllr + 1) * 2);  // 0b00:2, 0b01:4, 0b10:6, 0b11:8
        }
        else {
            pllOutClkFreq[PLL_CLKOUT_SYSTEM_IDX] = 0;
        }
        if (config.outClkEnable & PLL_CLKOUT_SAI3) {
            pllOutClkFreq[PLL_CLKOUT_SAI3_IDX] = pllvco / ((config.pllFactors.outClkDiv_pllp * 10) + 7); // 0b0:7, 0b1:17
        }
        else {
            pllOutClkFreq[PLL_CLKOUT_SAI3_IDX] = 0;
        }
        if (config.outClkEnable & PLL_CLKOUT_48M1) {
            pllOutClkFreq[PLL_CLKOUT_48M1_IDX] = pllvco / ((config.pllFactors.outClkDiv_pllq + 1) * 2);  // 0b00:2, 0b01:4, 0b10:6, 0b11:8
        }
        else {
            pllOutClkFreq[PLL_CLKOUT_48M1_IDX] = 0;
        }
    }
    else {
        pllOutClkFreq[PLL_CLKOUT_SYSTEM_IDX] = 0;
        pllOutClkFreq[PLL_CLKOUT_SAI3_IDX]   = 0;
        pllOutClkFreq[PLL_CLKOUT_48M1_IDX]   = 0;
    }

    /* PLLSAI1 */
    if (RCC->CR & RCC_CR_PLLON) {
        pllvco = freq * config.pllSAI1Factors.vcoMul_plln;
        if (config.outClkEnable & PLL_CLKOUT_ADC1) {
            pllOutClkFreq[PLL_CLKOUT_ADC1_IDX] = pllvco / ((config.pllSAI1Factors.outClkDiv_pllr + 1) * 2);  // 0b00:2, 0b01:4, 0b10:6, 0b11:8
        }
        else {
            pllOutClkFreq[PLL_CLKOUT_ADC1_IDX] = 0;
        }
        if (config.outClkEnable & PLL_CLKOUT_SAI1) {
            pllOutClkFreq[PLL_CLKOUT_SAI1_IDX] = pllvco / ((config.pllSAI1Factors.outClkDiv_pllp * 10) + 7); // 0b0:7, 0b1:17
        }
        else {
            pllOutClkFreq[PLL_CLKOUT_SAI1_IDX] = 0;
        }
        if (config.outClkEnable & PLL_CLKOUT_48M2) {
            pllOutClkFreq[PLL_CLKOUT_48M2_IDX] = pllvco / ((config.pllSAI1Factors.outClkDiv_pllq + 1) * 2);  // 0b00:2, 0b01:4, 0b10:6, 0b11:8
        }
        else {
            pllOutClkFreq[PLL_CLKOUT_48M2_IDX] = 0;
        }
    }
    else {
        pllOutClkFreq[PLL_CLKOUT_ADC1_IDX] = 0;
        pllOutClkFreq[PLL_CLKOUT_SAI1_IDX] = 0;
        pllOutClkFreq[PLL_CLKOUT_48M2_IDX] = 0;
    }
    
    /* PLLSAI2 */
    if (RCC->CR & RCC_CR_PLLON) {
        pllvco = freq * config.pllSAI2Factors.vcoMul_plln;
        if (config.outClkEnable & PLL_CLKOUT_ADC2) {
            pllOutClkFreq[PLL_CLKOUT_ADC2_IDX] = pllvco / ((config.pllSAI2Factors.outClkDiv_pllr + 1) * 2);  // 0b00:2, 0b01:4, 0b10:6, 0b11:8
        }
        else {
            pllOutClkFreq[PLL_CLKOUT_ADC2_IDX] = 0;
        }
        if (config.outClkEnable & PLL_CLKOUT_SAI2) {
            pllOutClkFreq[PLL_CLKOUT_SAI2_IDX] = pllvco / ((config.pllSAI2Factors.outClkDiv_pllp * 10) + 7); // 0b0:7, 0b1:17
        }
        else {
            pllOutClkFreq[PLL_CLKOUT_SAI2_IDX] = 0;
        }
    }
    else {
        pllOutClkFreq[PLL_CLKOUT_ADC2_IDX] = 0;
        pllOutClkFreq[PLL_CLKOUT_SAI2_IDX] = 0;
    }
}

uint32_t getPllClkFreq(pllClkOut_t clk) {
    if (clk & ~PLL_CLKOUT_Msk) {
        return 0;
    }

    for (enum pllClkOut_idx_t idx = 0; idx < PLL_CLKOUT_IDX_Max; idx++) {
        if (clk & (1 << idx)) {
            return pllOutClkFreq[idx];
        }
    }

    // INVALID pllClkOut_t
    return 0;
}