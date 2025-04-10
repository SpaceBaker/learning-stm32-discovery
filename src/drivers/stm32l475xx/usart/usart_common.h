#include "../stm32l4xx.h"

/**
 * @note These macros force a read from the register,
 *       do not use if you intend to read a copy of the register.
 * 
 */
#define USART_IS_BUSY(__HANDLE__)               ((__HANDLE__)->ISR & USART_ISR_BUSY)
#define USART_IS_IDLE(__HANDLE__)               ((__HANDLE__)->ISR & USART_ISR_IDLE)
#define USART_TX_DATA_REG_EMPTY(__HANDLE__)     ((__HANDLE__)->ISR & USART_ISR_TXE)
#define USART_TX_COMPLETE(__HANDLE__)           ((__HANDLE__)->ISR & USART_ISR_TC)
#define USART_RX_DATA_REG_NOT_EMPTY(__HANDLE__) ((__HANDLE__)->ISR & USART_ISR_RXNE)
#define USART_OVERRUN(__HANDLE__)               ((__HANDLE__)->ISR & USART_ISR_ORE)
#define USART_NOISE_ERROR(__HANDLE__)           ((__HANDLE__)->ISR & USART_ISR_NE)
#define USART_FRAMING_ERROR(__HANDLE__)         ((__HANDLE__)->ISR & USART_ISR_FE)
#define USART_PARITY_ERROR(__HANDLE__)          ((__HANDLE__)->ISR & USART_ISR_PE)

/**
 * @brief These macros are intended to check against a copy of a register value,
 *       do not use if you intend to force a read of the register.
 * 
 */
#define USART_CPY_IS_BUSY(__REG_CPY__)               READ_BIT((__REG_CPY__), USART_ISR_BUSY)
#define USART_CPY_IS_IDLE(__REG_CPY__)               READ_BIT((__REG_CPY__), USART_ISR_IDLE)
#define USART_CPY_TX_DATA_REG_EMPTY(__REG_CPY__)     READ_BIT((__REG_CPY__), USART_ISR_TXE)
#define USART_CPY_TX_COMPLETE(__REG_CPY__)           READ_BIT((__REG_CPY__), USART_ISR_TC)
#define USART_CPY_RX_DATA_REG_NOT_EMPTY(__REG_CPY__) READ_BIT((__REG_CPY__), USART_ISR_RXNE)
#define USART_CPY_OVERRUN(__REG_CPY__)               READ_BIT((__REG_CPY__), USART_ISR_ORE)
#define USART_CPY_NOISE_ERROR(__REG_CPY__)           READ_BIT((__REG_CPY__), USART_ISR_NE)
#define USART_CPY_FRAMING_ERROR(__REG_CPY__)         READ_BIT((__REG_CPY__), USART_ISR_FE)
#define USART_CPY_PARITY_ERROR(__REG_CPY__)          READ_BIT((__REG_CPY__), USART_ISR_PE)
