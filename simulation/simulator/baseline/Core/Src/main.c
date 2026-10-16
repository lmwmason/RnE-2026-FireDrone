/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Extreme-environment drone — UKF + INDI + ADRC + MRAC + CAM
  ******************************************************************************
  */
#include "main.h"
#include "stm32f4xx_ll_tim.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_bus.h"

#include "imu.h"
#include "baro.h"
#include "gps.h"
#include "motor.h"
#include "cam.h"
#include "ukf.h"
#include "sensor_gate.h"
#include "indi.h"
#include "adrc.h"
#include "mrac.h"
#include "fault_manager.h"
#include "scheduler.h"
#include "crsf.h"

#include <math.h>
#include <string.h>

/* -------------------------------------------------------------------------- */
/*  HAL handle (USART2 retained for CRSF — HAL UART driver)                  */
/* -------------------------------------------------------------------------- */
UART_HandleTypeDef huart2;

/* -------------------------------------------------------------------------- */
/*  Physical / tuning constants  (tune per airframe)                          */
/* -------------------------------------------------------------------------- */
#define ARM_M           0.080f
#define K_THRUST        5.0f
#define K_DRAG          0.020f
#define IXX             0.003f
#define IYY             0.003f
#define IZZ             0.005f

#define INDI_CUTOFF_HZ  60.0f
#define DT_CTRL         0.001f

#define KP_RATE         50.0f
#define KP_RATE_YAW     30.0f

#define ADRC_WO_ROLL    20.0f
#define ADRC_WO_PITCH   20.0f
#define ADRC_WO_YAW     15.0f
#define ADRC_B0_RP      (K_THRUST * ARM_M * 0.70710678f / IXX)
#define ADRC_B0_YAW     (K_DRAG / IZZ)
#define ADRC_KP         10.0f
#define ADRC_KD         2.0f

#define MRAC_AM         10.0f
#define MRAC_BM         10.0f
#define MRAC_GAMMA1     0.001f
#define MRAC_GAMMA2     0.001f
#define MRAC_THETA1_0   1.0f
#define MRAC_THETA2_0   0.1f

#define MAX_ROLL_RATE_RPS   (600.0f * 0.01745329f)
#define MAX_PITCH_RATE_RPS  (600.0f * 0.01745329f)
#define MAX_YAW_RATE_RPS    (400.0f * 0.01745329f)

#define ARM_AUX_US          1700U
#define ARM_THRO_MAX_US     1100U
#define MOTOR_IDLE          0.05f
#define COMM_TIMEOUT_MS     300U
#define MOTOR_SAT_MS        2000U

/* CW270 sensor alignment indices */
#define GYRO_ROLL_IDX  1
#define GYRO_PITCH_IDX 0
#define GYRO_YAW_IDX   2

/* -------------------------------------------------------------------------- */
/*  Module state                                                               */
/* -------------------------------------------------------------------------- */
static ukf_t       g_ukf;
static indi_t      g_indi;
static cam_t       g_cam;
static adrc_t      g_adrc;
static mrac_t      g_mrac;
static fault_ctx_t g_fault;
static scheduler_t g_sched;

static imu_data_t  g_imu;
static baro_data_t g_baro;
static gps_data_t  g_gps;

static float       g_gyro_bias[3];
static float       g_omega_meas[3];

static volatile uint8_t  g_tick_flag;
static volatile uint32_t g_tick_count;

typedef enum { STATE_DISARMED = 0, STATE_ARMED } arm_state_t;
static arm_state_t g_arm;

/* -------------------------------------------------------------------------- */
/*  Forward declarations                                                       */
/* -------------------------------------------------------------------------- */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void tim6_ll_init(void);
static void led_set(uint8_t led, uint8_t on);

/* -------------------------------------------------------------------------- */
/*  Scheduler tasks                                                            */
/* -------------------------------------------------------------------------- */
static void task_1khz_ctrl(void)
{
    imu_read(&g_imu);

    /* CW270 alignment + bias correction */
    g_omega_meas[0] = -(g_imu.gyro_dps[GYRO_ROLL_IDX]  - g_gyro_bias[GYRO_ROLL_IDX])
                      * 0.01745329f;
    g_omega_meas[1] =  (g_imu.gyro_dps[GYRO_PITCH_IDX] - g_gyro_bias[GYRO_PITCH_IDX])
                      * 0.01745329f;
    g_omega_meas[2] =  (g_imu.gyro_dps[GYRO_YAW_IDX]   - g_gyro_bias[GYRO_YAW_IDX])
                      * 0.01745329f;

    /* UKF predict every tick, update every other tick */
    float gyro_rps[3] = { g_omega_meas[0], g_omega_meas[1], g_omega_meas[2] };
    float accel_g[3]  = { g_imu.accel_g[0], g_imu.accel_g[1], g_imu.accel_g[2] };
    ukf_predict(&g_ukf, gyro_rps, DT_CTRL);
    if (g_tick_count & 1u)
        ukf_update(&g_ukf, accel_g, gyro_rps, CHI2_95_6DOF);

    /* Fault: update comm status */
    fault_state_t fst = fault_update(&g_fault, (uint32_t)HAL_GetTick());

    /* RC input */
    uint16_t thro_us = CRSF_GetChannelUs(CRSF_CH_THROTTLE);
    uint16_t roll_us = CRSF_GetChannelUs(CRSF_CH_ROLL);
    uint16_t pitch_us= CRSF_GetChannelUs(CRSF_CH_PITCH);
    uint16_t yaw_us  = CRSF_GetChannelUs(CRSF_CH_YAW);
    uint16_t aux_us  = CRSF_GetChannelUs(CRSF_CH_AUX1);

    bool link_up = CRSF_IsLinkUp((uint32_t)HAL_GetTick());
    if (!link_up) fault_report_comm_rx(&g_fault, 0);
    else          fault_report_comm_rx(&g_fault, (uint32_t)HAL_GetTick());

    /* Arming FSM */
    if (!link_up || fst >= FAULT_COMM_LOST) {
        g_arm = STATE_DISARMED;
    } else if (g_arm == STATE_DISARMED
               && aux_us > ARM_AUX_US
               && thro_us < ARM_THRO_MAX_US) {
        g_arm = STATE_ARMED;
        motor_arm();
        indi_reset(&g_indi, MOTOR_IDLE);
    } else if (g_arm == STATE_ARMED && aux_us <= ARM_AUX_US) {
        g_arm = STATE_DISARMED;
    }

    led_set(1, g_arm == STATE_ARMED);

    if (g_arm == STATE_DISARMED || fst >= FAULT_EMERGENCY_LAND) {
        motor_disarm();
        return;
    }

    /* Rate setpoints from sticks (rad/s) */
    float omega_ref[3];
    omega_ref[0] = ((float)roll_us  - 1500.0f) / 500.0f * MAX_ROLL_RATE_RPS;
    omega_ref[1] = ((float)pitch_us - 1500.0f) / 500.0f * MAX_PITCH_RATE_RPS;
    omega_ref[2] = ((float)yaw_us   - 1500.0f) / 500.0f * MAX_YAW_RATE_RPS;

    /* MRAC adaptive control signal */
    float u_mrac[3];
    mrac_update(&g_mrac, g_omega_meas, omega_ref, u_mrac);

    /* ADRC disturbance estimate */
    float f_hat[3];
    adrc_get_disturbance(&g_adrc, f_hat);
    {
        float omega_ukf[3];
        ukf_get_rates(&g_ukf, omega_ukf);
        float u_adrc[3];
        float r_dot[3] = {0};
        float u_prev_adrc[3] = { g_omega_meas[0], g_omega_meas[1], g_omega_meas[2] };
        adrc_update(&g_adrc, g_omega_meas, omega_ref, r_dot, u_prev_adrc, u_adrc);
    }
    adrc_get_disturbance(&g_adrc, f_hat);

    /* Angular acceleration reference for INDI */
    float alpha_ref[3];
    alpha_ref[0] = KP_RATE     * (omega_ref[0] - g_omega_meas[0]) - f_hat[0] + u_mrac[0];
    alpha_ref[1] = KP_RATE     * (omega_ref[1] - g_omega_meas[1]) - f_hat[1] + u_mrac[1];
    alpha_ref[2] = KP_RATE_YAW * (omega_ref[2] - g_omega_meas[2]) - f_hat[2] + u_mrac[2];

    /* Throttle */
    float throttle = ((float)thro_us - 1000.0f) / 1000.0f;
    if (throttle < 0.0f) throttle = 0.0f;
    if (throttle > 1.0f) throttle = 1.0f;
    throttle = MOTOR_IDLE + throttle * (1.0f - MOTOR_IDLE);

    /* INDI: compute motor commands */
    float u_motor[4];
    if (fst == FAULT_MOTOR_FAULT) {
        /* CAM fault-tolerant allocation */
        float tau[4];
        float I[3] = { IXX, IYY, IZZ };
        tau[0] = throttle * K_THRUST * 4.0f;
        tau[1] = alpha_ref[0] * I[0];
        tau[2] = alpha_ref[1] * I[1];
        tau[3] = alpha_ref[2] * I[2];
        cam_solve(&g_cam, tau, u_motor);
    } else {
        indi_update(&g_indi, g_omega_meas, alpha_ref, throttle, u_motor);
    }

    motor_set_all(u_motor);
}

static void task_50hz_baro(void)
{
    baro_read(&g_baro);
}

static void task_10hz_gps(void)
{
    gps_get_latest(&g_gps);
}

/* -------------------------------------------------------------------------- */
/*  Main entry                                                                 */
/* -------------------------------------------------------------------------- */
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART2_UART_Init();

    led_set(0, 1);
    led_set(1, 0);

    motor_init();
    imu_init();
    baro_init();
    gps_init();

    imu_calibrate_gyro(g_gyro_bias, 500);

    ukf_init(&g_ukf, 1e-3f, 2.0f, 0.0f);
    indi_init(&g_indi, ARM_M, K_THRUST, K_DRAG, IXX, IYY, IZZ, INDI_CUTOFF_HZ, DT_CTRL);
    cam_init(&g_cam, K_THRUST, K_DRAG, ARM_M);
    adrc_init(&g_adrc,
              ADRC_WO_ROLL, ADRC_WO_PITCH, ADRC_WO_YAW,
              ADRC_B0_RP, ADRC_B0_RP, ADRC_B0_YAW,
              ADRC_KP, ADRC_KD, DT_CTRL);
    mrac_init(&g_mrac, MRAC_AM, MRAC_BM, MRAC_GAMMA1, MRAC_GAMMA2,
              MRAC_THETA1_0, MRAC_THETA2_0, DT_CTRL);
    fault_init(&g_fault, COMM_TIMEOUT_MS, MOTOR_SAT_MS);

    CRSF_Init();

    scheduler_init(&g_sched);
    scheduler_add_task(&g_sched, task_1khz_ctrl, 1);
    scheduler_add_task(&g_sched, task_50hz_baro, 20);
    scheduler_add_task(&g_sched, task_10hz_gps,  100);

    tim6_ll_init();

    led_set(0, 0);

    while (1) {
        if (g_tick_flag) {
            g_tick_flag = 0;
            g_tick_count++;
            scheduler_run(&g_sched, g_tick_count);
        }
    }
}

/* -------------------------------------------------------------------------- */
/*  TIM6 1kHz LL init                                                         */
/* -------------------------------------------------------------------------- */
static void tim6_ll_init(void)
{
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM6);

    LL_TIM_InitTypeDef ti = {
        .Prescaler   = 83,
        .CounterMode = LL_TIM_COUNTERMODE_UP,
        .Autoreload  = 999,
    };
    LL_TIM_Init(TIM6, &ti);
    LL_TIM_EnableIT_UPDATE(TIM6);
    LL_TIM_EnableCounter(TIM6);
    NVIC_SetPriority(TIM6_DAC_IRQn, 0);
    NVIC_EnableIRQ(TIM6_DAC_IRQn);
}

/* TIM6 ISR — sets scheduler tick flag */
void TIM6_DAC_IRQHandler(void)
{
    if (LL_TIM_IsActiveFlag_UPDATE(TIM6)) {
        LL_TIM_ClearFlag_UPDATE(TIM6);
        g_tick_flag = 1;
    }
}

/* -------------------------------------------------------------------------- */
/*  LL GPIO helpers                                                            */
/* -------------------------------------------------------------------------- */
static void led_set(uint8_t led, uint8_t on)
{
    GPIO_TypeDef *port = (led == 0) ? GPIOA : GPIOA;
    uint32_t pin  = (led == 0) ? LL_GPIO_PIN_13 : LL_GPIO_PIN_14;
    if (on) LL_GPIO_SetOutputPin(port, pin);
    else    LL_GPIO_ResetOutputPin(port, pin);
}

/* -------------------------------------------------------------------------- */
/*  HAL GPIO Init (LEDs, BEEPER, GYRO_INT)                                    */
/* -------------------------------------------------------------------------- */
static void MX_GPIO_Init(void)
{
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA
                            | LL_AHB1_GRP1_PERIPH_GPIOB
                            | LL_AHB1_GRP1_PERIPH_GPIOC
                            | LL_AHB1_GRP1_PERIPH_GPIOD);

    LL_GPIO_InitTypeDef g = {
        .Pin        = LL_GPIO_PIN_13 | LL_GPIO_PIN_14,
        .Mode       = LL_GPIO_MODE_OUTPUT,
        .Speed      = LL_GPIO_SPEED_FREQ_LOW,
        .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
        .Pull       = LL_GPIO_PULL_NO,
    };
    LL_GPIO_Init(GPIOA, &g);

    g.Pin = LL_GPIO_PIN_14;
    LL_GPIO_Init(GPIOB, &g);
}

/* -------------------------------------------------------------------------- */
/*  USART2 HAL Init (CRSF)                                                    */
/* -------------------------------------------------------------------------- */
static void MX_USART2_UART_Init(void)
{
    huart2.Instance          = USART2;
    huart2.Init.BaudRate     = 420000;
    huart2.Init.WordLength   = UART_WORDLENGTH_8B;
    huart2.Init.StopBits     = UART_STOPBITS_1;
    huart2.Init.Parity       = UART_PARITY_NONE;
    huart2.Init.Mode         = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart2);
}

/* -------------------------------------------------------------------------- */
/*  SystemClock_Config  168MHz from 8MHz HSE                                  */
/* -------------------------------------------------------------------------- */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {0};
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState       = RCC_HSE_ON;
    osc.PLL.PLLState   = RCC_PLL_ON;
    osc.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLM       = 8;
    osc.PLL.PLLN       = 336;
    osc.PLL.PLLP       = RCC_PLLP_DIV2;
    osc.PLL.PLLQ       = 7;
    HAL_RCC_OscConfig(&osc);

    RCC_ClkInitTypeDef clk = {0};
    clk.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                       | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV4;
    clk.APB2CLKDivider = RCC_HCLK_DIV2;
    HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_5);
}

void Error_Handler(void) { while (1); }

void USART2_IRQHandler(void) { HAL_UART_IRQHandler(&huart2); }
