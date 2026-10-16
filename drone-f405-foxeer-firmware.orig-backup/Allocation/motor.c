#include "motor.h"
#include "stm32f4xx_ll_tim.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_bus.h"
#include <stdint.h>

/* M1: TIM1_CH1   PA8  AF1   rear-right CCW
 * M2: TIM8_CH4   PC9  AF3   front-right CW
 * M3: TIM8_CH3   PC8  AF3   rear-left  CW
 * M4: TIM1_CH3N  PB15 AF1   front-left CCW (complementary) */

static uint8_t s_armed;

static void gpio_init(void)
{
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA
                            | LL_AHB1_GRP1_PERIPH_GPIOB
                            | LL_AHB1_GRP1_PERIPH_GPIOC);

    LL_GPIO_InitTypeDef g = {
        .Mode       = LL_GPIO_MODE_ALTERNATE,
        .Speed      = LL_GPIO_SPEED_FREQ_VERY_HIGH,
        .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
        .Pull       = LL_GPIO_PULL_NO,
    };

    g.Pin = LL_GPIO_PIN_8;
    g.Alternate = LL_GPIO_AF_1;
    LL_GPIO_Init(GPIOA, &g);

    g.Pin = LL_GPIO_PIN_15;
    g.Alternate = LL_GPIO_AF_1;
    LL_GPIO_Init(GPIOB, &g);

    g.Pin = LL_GPIO_PIN_8 | LL_GPIO_PIN_9;
    g.Alternate = LL_GPIO_AF_3;
    LL_GPIO_Init(GPIOC, &g);
}

static void tim1_init(void)
{
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);

    LL_TIM_InitTypeDef ti = {
        .Prescaler         = 167,
        .CounterMode       = LL_TIM_COUNTERMODE_UP,
        .Autoreload        = 1999,
        .ClockDivision     = LL_TIM_CLOCKDIVISION_DIV1,
        .RepetitionCounter = 0,
    };
    LL_TIM_Init(TIM1, &ti);
    LL_TIM_EnableARRPreload(TIM1);

    LL_TIM_OC_InitTypeDef oc = {
        .OCMode       = LL_TIM_OCMODE_PWM1,
        .OCPolarity   = LL_TIM_OCPOLARITY_HIGH,
        .OCNPolarity  = LL_TIM_OCPOLARITY_HIGH,
        .OCIdleState  = LL_TIM_OCIDLESTATE_LOW,
        .OCNIdleState = LL_TIM_OCIDLESTATE_LOW,
        .CompareValue = MOTOR_PWM_DISARM_US,
    };

    oc.OCState  = LL_TIM_OCSTATE_ENABLE;
    oc.OCNState = LL_TIM_OCSTATE_DISABLE;
    LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH1, &oc);
    LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH1);

    oc.OCState  = LL_TIM_OCSTATE_DISABLE;
    oc.OCNState = LL_TIM_OCSTATE_ENABLE;
    LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH3, &oc);
    LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH3);

    TIM1->BDTR |= TIM_BDTR_MOE;
    LL_TIM_EnableCounter(TIM1);
}

static void tim8_init(void)
{
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM8);

    LL_TIM_InitTypeDef ti = {
        .Prescaler         = 167,
        .CounterMode       = LL_TIM_COUNTERMODE_UP,
        .Autoreload        = 1999,
        .ClockDivision     = LL_TIM_CLOCKDIVISION_DIV1,
        .RepetitionCounter = 0,
    };
    LL_TIM_Init(TIM8, &ti);
    LL_TIM_EnableARRPreload(TIM8);

    LL_TIM_OC_InitTypeDef oc = {
        .OCMode       = LL_TIM_OCMODE_PWM1,
        .OCState      = LL_TIM_OCSTATE_ENABLE,
        .OCNState     = LL_TIM_OCSTATE_DISABLE,
        .OCPolarity   = LL_TIM_OCPOLARITY_HIGH,
        .OCNPolarity  = LL_TIM_OCPOLARITY_HIGH,
        .OCIdleState  = LL_TIM_OCIDLESTATE_LOW,
        .OCNIdleState = LL_TIM_OCIDLESTATE_LOW,
        .CompareValue = MOTOR_PWM_DISARM_US,
    };

    LL_TIM_OC_Init(TIM8, LL_TIM_CHANNEL_CH3, &oc);
    LL_TIM_OC_DisableFast(TIM8, LL_TIM_CHANNEL_CH3);

    LL_TIM_OC_Init(TIM8, LL_TIM_CHANNEL_CH4, &oc);
    LL_TIM_OC_DisableFast(TIM8, LL_TIM_CHANNEL_CH4);

    TIM8->BDTR |= TIM_BDTR_MOE;
    LL_TIM_EnableCounter(TIM8);
}

void motor_init(void)
{
    s_armed = 0;
    gpio_init();
    tim1_init();
    tim8_init();
}

void motor_arm(void)  { s_armed = 1; }
void motor_disarm(void)
{
    s_armed = 0;
    motor_set_us(0, MOTOR_PWM_DISARM_US);
    motor_set_us(1, MOTOR_PWM_DISARM_US);
    motor_set_us(2, MOTOR_PWM_DISARM_US);
    motor_set_us(3, MOTOR_PWM_DISARM_US);
}

void motor_set_us(uint8_t idx, uint16_t pulse_us)
{
    if (pulse_us < MOTOR_PWM_MIN_US) pulse_us = MOTOR_PWM_MIN_US;
    if (pulse_us > MOTOR_PWM_MAX_US) pulse_us = MOTOR_PWM_MAX_US;

    switch (idx) {
    case 0: LL_TIM_OC_SetCompareCH1(TIM1, pulse_us); break;
    case 1: LL_TIM_OC_SetCompareCH4(TIM8, pulse_us); break;
    case 2: LL_TIM_OC_SetCompareCH3(TIM8, pulse_us); break;
    case 3: LL_TIM_OC_SetCompareCH3(TIM1, pulse_us); break;
    default: break;
    }
}

void motor_set(uint8_t idx, float normalized)
{
    if (!s_armed) { motor_set_us(idx, MOTOR_PWM_DISARM_US); return; }
    if (normalized < 0.0f) normalized = 0.0f;
    if (normalized > 1.0f) normalized = 1.0f;
    motor_set_us(idx, (uint16_t)(MOTOR_PWM_MIN_US + normalized * 1000.0f));
}

void motor_set_all(const float u[4])
{
    for (uint8_t i = 0; i < 4; i++) motor_set(i, u[i]);
}
