#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <ucontext.h>

#include "stm32_shim.h"
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

#include "fw_host.h"

static ucontext_t host_ctx, fw_ctx;

void sim_fw_yield(void) {
    swapcontext(&fw_ctx, &host_ctx);
}

/* Pull the real firmware translation unit in so its file-scope state
 * (g_ukf, g_fault, ...) is visible.  The only edits are at the preprocessor
 * level: main() is renamed and the super-loop condition yields to the host
 * once per iteration, which turns the free-running loop into a lock-step one. */
#define main fw_main
#define while(cond) while (sim_fw_yield(), (cond))
#include "main.c"
#undef while
#undef main

#define FW_STACK_BYTES (1u << 20)

static void fw_entry(void) {
    fw_main();
}

void fw_host_init(const float gyro_cal_bias_dps[3]) {
    memcpy(sim_gyro_cal_bias, gyro_cal_bias_dps, 3 * sizeof(float));
    sim_tick_ms = 0;

    getcontext(&fw_ctx);
    fw_ctx.uc_stack.ss_sp   = malloc(FW_STACK_BYTES);
    fw_ctx.uc_stack.ss_size = FW_STACK_BYTES;
    fw_ctx.uc_link          = &host_ctx;
    makecontext(&fw_ctx, fw_entry, 0);

    swapcontext(&host_ctx, &fw_ctx);   /* runs init up to the first loop check */
}

/* ---- CRSF RC_CHANNELS_PACKED encoder (what an ELRS receiver would send) ---- */
static uint8_t crsf_crc8(const uint8_t *d, uint8_t n) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < n; i++) {
        crc ^= d[i];
        for (uint8_t b = 0; b < 8; b++)
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (uint8_t)(crc << 1);
    }
    return crc;
}

static uint16_t us_to_crsf(uint16_t us) {
    double v = 172.0 + ((double)us - 988.0) / 0.62477;
    if (v < 0.0)    v = 0.0;
    if (v > 2047.0) v = 2047.0;
    return (uint16_t)lround(v);
}

void fw_host_set_time(uint32_t now_ms) {
    sim_tick_ms = now_ms;
}

void fw_host_rc_frame(const uint16_t rc_us[5]) {
    uint16_t ch[16];
    for (int i = 0; i < 16; i++) ch[i] = 992;
    for (int i = 0; i < 5; i++)  ch[i] = us_to_crsf(rc_us[i]);

    uint8_t frame[26] = { 0xC8, 24, 0x16 };
    uint32_t bits = 0;
    int nbits = 0, pos = 3;
    for (int i = 0; i < 16; i++) {
        bits |= (uint32_t)ch[i] << nbits;
        nbits += 11;
        while (nbits >= 8) {
            frame[pos++] = (uint8_t)(bits & 0xFF);
            bits >>= 8;
            nbits -= 8;
        }
    }
    frame[25] = crsf_crc8(&frame[2], 23);

    if (!sim_uart_rx_buf) return;
    memcpy(sim_uart_rx_buf, frame, sizeof(frame));
    HAL_UARTEx_RxEventCallback(&huart2, sizeof(frame));
}

static uint16_t pwm_of(const volatile uint32_t *ccr) {
    return (uint16_t)*ccr;
}

void fw_host_step(const FwImuIn *imu, FwOut *out) {
    memcpy(sim_imu_in.gyro_dps, imu->gyro_dps, sizeof(sim_imu_in.gyro_dps));
    memcpy(sim_imu_in.accel_g,  imu->accel_g,  sizeof(sim_imu_in.accel_g));
    sim_baro_alt_m = imu->baro_alt_m;

    g_tick_flag = 1;                       /* TIM6 update interrupt */
    swapcontext(&host_ctx, &fw_ctx);

    out->pwm_us[0] = pwm_of(&TIM1->CCR1);  /* M1  PA8  */
    out->pwm_us[1] = pwm_of(&TIM8->CCR4);  /* M2  PC9  */
    out->pwm_us[2] = pwm_of(&TIM8->CCR3);  /* M3  PC8  */
    out->pwm_us[3] = pwm_of(&TIM1->CCR3);  /* M4  PB15 */
    out->arm         = (g_arm == STATE_ARMED);
    out->fault_state = (int)g_fault.state;

    ukf_get_attitude(&g_ukf, out->ukf_q);
    ukf_get_rates(&g_ukf, out->ukf_w);
    adrc_get_disturbance(&g_adrc, out->adrc_fhat);
    mrac_get_params(&g_mrac, out->mrac_theta1, out->mrac_theta2);

    uint32_t ov[SCHED_MAX_TASKS];
    scheduler_get_overruns(&g_sched, ov);
    out->sched_overruns = 0;
    for (uint32_t i = 0; i < SCHED_MAX_TASKS; i++) out->sched_overruns += ov[i];
    out->tick_count = g_tick_count;
}
