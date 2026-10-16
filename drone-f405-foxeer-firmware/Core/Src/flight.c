#include "flight.h"
#include "main.h"
#include "icm42688.h"
#include "crsf.h"
#include "pid.h"
#include "mixer.h"
#include <stdbool.h>

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim8;

/* Starting/conservative gains: output is motor-fraction (0..1) per dps of
 * rate error. MUST be tuned in the field - validate with props off first,
 * then a cautious low, tethered hover before any real flight. */
#define PID_ROLL_KP   0.0035f
#define PID_ROLL_KI   0.00002f
#define PID_ROLL_KD   0.00010f

#define PID_PITCH_KP  0.0035f
#define PID_PITCH_KI  0.00002f
#define PID_PITCH_KD  0.00010f

#define PID_YAW_KP    0.0050f
#define PID_YAW_KI    0.00003f
#define PID_YAW_KD    0.00000f

#define PID_I_LIMIT   0.20f

#define MAX_ROLLPITCH_RATE_DPS  600.0f
#define MAX_YAW_RATE_DPS        400.0f
#define MOTOR_IDLE_FRACTION     0.05f
#define ARM_AUX_THRESHOLD_US    1700
#define ARM_THROTTLE_MAX_US     1100
#define LOOP_DT_S               0.001f /* TIM6 @ 1kHz */

typedef enum { FLIGHT_DISARMED = 0, FLIGHT_ARMED } flight_state_t;

static pid_t s_pidRoll, s_pidPitch, s_pidYaw;
static float s_gyroBias[3];
static flight_state_t s_state = FLIGHT_DISARMED;

static uint16_t UsToPulse(float us)
{
    if (us < 1000.0f) us = 1000.0f;
    if (us > 2000.0f) us = 2000.0f;
    return (uint16_t)us;
}

static void SetMotors(float m1, float m2, float m3, float m4)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, UsToPulse(1000.0f + m1 * 1000.0f)); /* M1 */
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, UsToPulse(1000.0f + m2 * 1000.0f)); /* M2 */
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, UsToPulse(1000.0f + m3 * 1000.0f)); /* M3 */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, UsToPulse(1000.0f + m4 * 1000.0f)); /* M4 (CH3N) */
}

static void FatalBlink(void)
{
    while (1) {
        HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
        HAL_GPIO_WritePin(BEEPER_GPIO_Port, BEEPER_Pin, GPIO_PIN_SET);
        HAL_Delay(100);
        HAL_GPIO_WritePin(BEEPER_GPIO_Port, BEEPER_Pin, GPIO_PIN_RESET);
        HAL_Delay(100);
    }
}

void Flight_Init(void)
{
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);

    if (!ICM42688_Init()) {
        FatalBlink(); /* never spin motors without a working gyro */
    }

    /* board must be still during this calibration window */
    ICM42688_CalibrateGyroBias(s_gyroBias, 500);

    CRSF_Init();

    PID_Init(&s_pidRoll,  PID_ROLL_KP,  PID_ROLL_KI,  PID_ROLL_KD,  PID_I_LIMIT);
    PID_Init(&s_pidPitch, PID_PITCH_KP, PID_PITCH_KI, PID_PITCH_KD, PID_I_LIMIT);
    PID_Init(&s_pidYaw,   PID_YAW_KP,   PID_YAW_KI,   PID_YAW_KD,   PID_I_LIMIT);

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
    SetMotors(0.0f, 0.0f, 0.0f, 0.0f);

    HAL_TIM_Base_Start_IT(&htim6);

    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
}

void Flight_Loop(void)
{
    icm42688_data_t imu;
    ICM42688_Read(&imu);

    /* CW270 sensor alignment: body roll = -rawY, body pitch = rawX, body yaw = rawZ */
    float rollRate  = -(imu.gyro_dps[1] - s_gyroBias[1]);
    float pitchRate =  (imu.gyro_dps[0] - s_gyroBias[0]);
    float yawRate   =  (imu.gyro_dps[2] - s_gyroBias[2]);

    uint32_t now = HAL_GetTick();
    bool linkUp = CRSF_IsLinkUp(now);

    uint16_t rollUs  = CRSF_GetChannelUs(CRSF_CH_ROLL);
    uint16_t pitchUs = CRSF_GetChannelUs(CRSF_CH_PITCH);
    uint16_t throUs  = CRSF_GetChannelUs(CRSF_CH_THROTTLE);
    uint16_t yawUs   = CRSF_GetChannelUs(CRSF_CH_YAW);
    uint16_t auxUs   = CRSF_GetChannelUs(CRSF_CH_AUX1);

    if (!linkUp) {
        s_state = FLIGHT_DISARMED;
    } else if (s_state == FLIGHT_DISARMED && auxUs > ARM_AUX_THRESHOLD_US && throUs < ARM_THROTTLE_MAX_US) {
        s_state = FLIGHT_ARMED;
        PID_Reset(&s_pidRoll);
        PID_Reset(&s_pidPitch);
        PID_Reset(&s_pidYaw);
    } else if (s_state == FLIGHT_ARMED && auxUs <= ARM_AUX_THRESHOLD_US) {
        s_state = FLIGHT_DISARMED;
    }

    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, (s_state == FLIGHT_ARMED) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    if (s_state != FLIGHT_ARMED) {
        SetMotors(0.0f, 0.0f, 0.0f, 0.0f);
        return;
    }

    float throttle = ((float)throUs - 1000.0f) / 1000.0f;
    if (throttle < 0.0f) throttle = 0.0f;
    if (throttle > 1.0f) throttle = 1.0f;
    throttle = MOTOR_IDLE_FRACTION + throttle * (1.0f - MOTOR_IDLE_FRACTION);

    float rollSetpoint  = (((float)rollUs  - 1500.0f) / 500.0f) * MAX_ROLLPITCH_RATE_DPS;
    float pitchSetpoint = (((float)pitchUs - 1500.0f) / 500.0f) * MAX_ROLLPITCH_RATE_DPS;
    float yawSetpoint   = (((float)yawUs   - 1500.0f) / 500.0f) * MAX_YAW_RATE_DPS;

    float rollOut  = PID_Update(&s_pidRoll,  rollSetpoint,  rollRate,  LOOP_DT_S);
    float pitchOut = PID_Update(&s_pidPitch, pitchSetpoint, pitchRate, LOOP_DT_S);
    float yawOut   = PID_Update(&s_pidYaw,   yawSetpoint,   yawRate,   LOOP_DT_S);

    mixer_out_t mix;
    Mixer_QuadX(throttle, rollOut, pitchOut, yawOut, &mix);
    SetMotors(mix.m1, mix.m2, mix.m3, mix.m4);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) {
        Flight_Loop();
    }
}
