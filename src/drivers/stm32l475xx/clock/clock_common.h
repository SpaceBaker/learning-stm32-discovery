#ifndef __CLOCK_COMMON_H
#define __CLOCK_COMMON_H

#include "../stm32l4xx.h" // IWYU pragma: keep
#include "clock_macro.h"  // IWYU pragma: keep
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#if !defined  (LSE_VALUE)
  #define LSE_VALUE    32768U  /*!< Value of the Low Speed External oscillator in Hz */
#endif

#if !defined  (HSE_VALUE)
  #define HSE_VALUE    8000000U  /*!< Value of the High Speed External oscillator in Hz */
#endif

#if !defined  (MSI_VALUE)
  #define MSI_VALUE    4000000U  /*!< Value of the Multi Speed Internal oscillator in Hz*/
#endif /* MSI_VALUE */

#if !defined  (LSI_VALUE)
  #define LSI_VALUE    32000U  /*!< Value of the Low Speed Internal oscillator in Hz */
#endif

#if !defined  (HSI_VALUE)
  #define HSI_VALUE    16000000U /*!< Value of the High Speed Internal oscillator in Hz*/
#endif

#if !defined  (SAI1_EXTCLK_VALUE)
  #define SAI1_EXTCLK_VALUE    0U /*!< Value of the External SAI1 Clock in Hz*/
#endif

#if !defined  (SAI2_EXTCLK_VALUE)
  #define SAI2_EXTCLK_VALUE    0U /*!< Value of the External SAI2 Clock in Hz*/
#endif

typedef enum {
  SYSCLK_ID = 0,
  HCLK_ID,
  PCLK1_ID,
  PCLK2_ID,
  SYSCLKID_Max
} sysClkId_t;

void updateClkFreq(void);
uint32_t getMsiFreq(void);
uint32_t getSysClkFreq(sysClkId_t sysClkId);


#endif // __CLOCK_COMMON_H
