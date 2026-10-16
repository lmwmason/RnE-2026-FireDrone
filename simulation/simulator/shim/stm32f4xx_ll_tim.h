#ifndef SHIM_STM32F4XX_LL_TIM_H
#define SHIM_STM32F4XX_LL_TIM_H

#include "stm32f4xx_hal.h"

#define LL_TIM_COUNTERMODE_UP    0u
#define LL_TIM_CLOCKDIVISION_DIV1 0u
#define LL_TIM_OCMODE_PWM1       6u
#define LL_TIM_OCPOLARITY_HIGH   0u
#define LL_TIM_OCIDLESTATE_LOW   0u
#define LL_TIM_OCSTATE_DISABLE   0u
#define LL_TIM_OCSTATE_ENABLE    1u
#define LL_TIM_CHANNEL_CH1       1u
#define LL_TIM_CHANNEL_CH2       2u
#define LL_TIM_CHANNEL_CH3       3u
#define LL_TIM_CHANNEL_CH4       4u

typedef struct {
    uint16_t Prescaler;
    uint32_t CounterMode;
    uint32_t Autoreload;
    uint32_t ClockDivision;
    uint8_t  RepetitionCounter;
} LL_TIM_InitTypeDef;

typedef struct {
    uint32_t OCMode, OCState, OCNState, CompareValue;
    uint32_t OCPolarity, OCNPolarity, OCIdleState, OCNIdleState;
} LL_TIM_OC_InitTypeDef;

static inline int  LL_TIM_Init(TIM_TypeDef *t, const LL_TIM_InitTypeDef *i) {
    t->PSC = i->Prescaler;
    t->ARR = i->Autoreload;
    return 0;
}
static inline int  LL_TIM_OC_Init(TIM_TypeDef *t, uint32_t ch, const LL_TIM_OC_InitTypeDef *i) {
    switch (ch) {
    case LL_TIM_CHANNEL_CH1: t->CCR1 = i->CompareValue; break;
    case LL_TIM_CHANNEL_CH2: t->CCR2 = i->CompareValue; break;
    case LL_TIM_CHANNEL_CH3: t->CCR3 = i->CompareValue; break;
    case LL_TIM_CHANNEL_CH4: t->CCR4 = i->CompareValue; break;
    default: break;
    }
    return 0;
}
static inline void LL_TIM_OC_DisableFast(TIM_TypeDef *t, uint32_t ch) { (void)t; (void)ch; }
static inline void LL_TIM_EnableARRPreload(TIM_TypeDef *t)            { (void)t; }
static inline void LL_TIM_EnableCounter(TIM_TypeDef *t)               { (void)t; }
static inline void LL_TIM_EnableIT_UPDATE(TIM_TypeDef *t)             { (void)t; }
static inline uint32_t LL_TIM_IsActiveFlag_UPDATE(TIM_TypeDef *t)     { (void)t; return 1u; }
static inline void LL_TIM_ClearFlag_UPDATE(TIM_TypeDef *t)            { (void)t; }

static inline void LL_TIM_OC_SetCompareCH1(TIM_TypeDef *t, uint32_t v) { t->CCR1 = v; }
static inline void LL_TIM_OC_SetCompareCH2(TIM_TypeDef *t, uint32_t v) { t->CCR2 = v; }
static inline void LL_TIM_OC_SetCompareCH3(TIM_TypeDef *t, uint32_t v) { t->CCR3 = v; }
static inline void LL_TIM_OC_SetCompareCH4(TIM_TypeDef *t, uint32_t v) { t->CCR4 = v; }

#endif
