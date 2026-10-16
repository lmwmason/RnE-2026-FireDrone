#ifndef SHIM_STM32F4XX_LL_BUS_H
#define SHIM_STM32F4XX_LL_BUS_H

#include "stm32f4xx_hal.h"

#define LL_AHB1_GRP1_PERIPH_GPIOA  0x01u
#define LL_AHB1_GRP1_PERIPH_GPIOB  0x02u
#define LL_AHB1_GRP1_PERIPH_GPIOC  0x04u
#define LL_AHB1_GRP1_PERIPH_GPIOD  0x08u
#define LL_APB1_GRP1_PERIPH_TIM6   0x10u
#define LL_APB2_GRP1_PERIPH_TIM1   0x01u
#define LL_APB2_GRP1_PERIPH_TIM8   0x02u

static inline void LL_AHB1_GRP1_EnableClock(uint32_t p) { (void)p; }
static inline void LL_APB1_GRP1_EnableClock(uint32_t p) { (void)p; }
static inline void LL_APB2_GRP1_EnableClock(uint32_t p) { (void)p; }

#endif
