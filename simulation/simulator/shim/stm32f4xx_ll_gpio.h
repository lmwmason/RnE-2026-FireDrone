#ifndef SHIM_STM32F4XX_LL_GPIO_H
#define SHIM_STM32F4XX_LL_GPIO_H

#include "stm32f4xx_hal.h"

#define LL_GPIO_PIN_2   GPIO_PIN_2
#define LL_GPIO_PIN_6   GPIO_PIN_6
#define LL_GPIO_PIN_7   GPIO_PIN_7
#define LL_GPIO_PIN_8   GPIO_PIN_8
#define LL_GPIO_PIN_9   GPIO_PIN_9
#define LL_GPIO_PIN_12  GPIO_PIN_12
#define LL_GPIO_PIN_13  GPIO_PIN_13
#define LL_GPIO_PIN_14  GPIO_PIN_14
#define LL_GPIO_PIN_15  GPIO_PIN_15

#define LL_GPIO_MODE_OUTPUT         1u
#define LL_GPIO_MODE_ALTERNATE      2u
#define LL_GPIO_SPEED_FREQ_LOW      0u
#define LL_GPIO_SPEED_FREQ_HIGH     2u
#define LL_GPIO_SPEED_FREQ_VERY_HIGH 3u
#define LL_GPIO_OUTPUT_PUSHPULL     0u
#define LL_GPIO_PULL_NO             0u
#define LL_GPIO_PULL_UP             1u
#define LL_GPIO_AF_1                1u
#define LL_GPIO_AF_3                3u

typedef struct {
    uint32_t Pin, Mode, Speed, OutputType, Pull, Alternate;
} LL_GPIO_InitTypeDef;

static inline int  LL_GPIO_Init(GPIO_TypeDef *p, const LL_GPIO_InitTypeDef *i) { (void)p; (void)i; return 0; }
static inline void LL_GPIO_SetOutputPin(GPIO_TypeDef *p, uint32_t pin)   { p->ODR |=  pin; }
static inline void LL_GPIO_ResetOutputPin(GPIO_TypeDef *p, uint32_t pin) { p->ODR &= ~pin; }

#endif
