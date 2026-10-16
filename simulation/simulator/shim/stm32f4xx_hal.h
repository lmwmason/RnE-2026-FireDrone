#ifndef SHIM_STM32F4XX_HAL_H
#define SHIM_STM32F4XX_HAL_H

/* Host stand-in for the STM32F4 HAL / CMSIS device header.
 * Only what the flight firmware touches is modelled; timer compare registers
 * are real memory so the simulator can read the PWM the firmware produced. */

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    volatile uint32_t CCR1, CCR2, CCR3, CCR4;
    volatile uint32_t BDTR;
    volatile uint32_t ARR, PSC;
} TIM_TypeDef;

typedef struct { volatile uint32_t ODR; } GPIO_TypeDef;
typedef struct { int unused; } USART_TypeDef;

extern TIM_TypeDef  sim_tim1, sim_tim6, sim_tim8;
extern GPIO_TypeDef sim_gpio[8];
extern USART_TypeDef sim_usart2;

#define TIM1   (&sim_tim1)
#define TIM6   (&sim_tim6)
#define TIM8   (&sim_tim8)
#define GPIOA  (&sim_gpio[0])
#define GPIOB  (&sim_gpio[1])
#define GPIOC  (&sim_gpio[2])
#define GPIOD  (&sim_gpio[3])
#define GPIOH  (&sim_gpio[7])
#define USART2 (&sim_usart2)

#define TIM_BDTR_MOE (1u << 15)

#define GPIO_PIN_0   ((uint16_t)0x0001)
#define GPIO_PIN_1   ((uint16_t)0x0002)
#define GPIO_PIN_2   ((uint16_t)0x0004)
#define GPIO_PIN_3   ((uint16_t)0x0008)
#define GPIO_PIN_4   ((uint16_t)0x0010)
#define GPIO_PIN_5   ((uint16_t)0x0020)
#define GPIO_PIN_6   ((uint16_t)0x0040)
#define GPIO_PIN_7   ((uint16_t)0x0080)
#define GPIO_PIN_8   ((uint16_t)0x0100)
#define GPIO_PIN_9   ((uint16_t)0x0200)
#define GPIO_PIN_10  ((uint16_t)0x0400)
#define GPIO_PIN_11  ((uint16_t)0x0800)
#define GPIO_PIN_12  ((uint16_t)0x1000)
#define GPIO_PIN_13  ((uint16_t)0x2000)
#define GPIO_PIN_14  ((uint16_t)0x4000)
#define GPIO_PIN_15  ((uint16_t)0x8000)

typedef enum {
    TIM6_DAC_IRQn = 54,
    EXTI4_IRQn    = 10,
    USART2_IRQn   = 38,
} IRQn_Type;

static inline void NVIC_SetPriority(IRQn_Type irq, uint32_t prio) { (void)irq; (void)prio; }
static inline void NVIC_EnableIRQ(IRQn_Type irq)                  { (void)irq; }

typedef struct {
    USART_TypeDef *Instance;
    struct {
        uint32_t BaudRate, WordLength, StopBits, Parity, Mode, HwFlowCtl, OverSampling;
    } Init;
} UART_HandleTypeDef;

typedef struct { TIM_TypeDef *Instance; } TIM_HandleTypeDef;

#define UART_WORDLENGTH_8B     0u
#define UART_STOPBITS_1        0u
#define UART_PARITY_NONE       0u
#define UART_MODE_TX_RX        0x0Cu
#define UART_HWCONTROL_NONE    0u
#define UART_OVERSAMPLING_16   0u

typedef struct {
    uint32_t OscillatorType, HSEState;
    struct { uint32_t PLLState, PLLSource, PLLM, PLLN, PLLP, PLLQ; } PLL;
} RCC_OscInitTypeDef;

typedef struct {
    uint32_t ClockType, SYSCLKSource, AHBCLKDivider, APB1CLKDivider, APB2CLKDivider;
} RCC_ClkInitTypeDef;

#define RCC_OSCILLATORTYPE_HSE  1u
#define RCC_HSE_ON              1u
#define RCC_PLL_ON              2u
#define RCC_PLLSOURCE_HSE       1u
#define RCC_PLLP_DIV2           2u
#define RCC_CLOCKTYPE_HCLK      1u
#define RCC_CLOCKTYPE_SYSCLK    2u
#define RCC_CLOCKTYPE_PCLK1     4u
#define RCC_CLOCKTYPE_PCLK2     8u
#define RCC_SYSCLKSOURCE_PLLCLK 2u
#define RCC_SYSCLK_DIV1         0u
#define RCC_HCLK_DIV2           0x1000u
#define RCC_HCLK_DIV4           0x1400u
#define FLASH_LATENCY_5         5u

int      HAL_Init(void);
uint32_t HAL_GetTick(void);
int      HAL_RCC_OscConfig(RCC_OscInitTypeDef *osc);
int      HAL_RCC_ClockConfig(RCC_ClkInitTypeDef *clk, uint32_t latency);
int      HAL_UART_Init(UART_HandleTypeDef *huart);
void     HAL_UART_IRQHandler(UART_HandleTypeDef *huart);
int      HAL_UARTEx_ReceiveToIdle_IT(UART_HandleTypeDef *huart, uint8_t *buf, uint16_t size);
void     HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size);

#endif
