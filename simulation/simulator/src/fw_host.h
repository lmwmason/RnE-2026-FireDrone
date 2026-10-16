#ifndef FW_HOST_H
#define FW_HOST_H

#include <stdbool.h>
#include <stdint.h>

/* Runs the unmodified firmware main.c as a coroutine.  One fw_host_step()
 * is exactly one pass of the firmware main loop == one 1 kHz TIM6 tick. */

typedef struct {
    float gyro_dps[3];
    float accel_g[3];
    float baro_alt_m;           /* barometric height above the take-off point */
} FwImuIn;

typedef struct {
    uint16_t pwm_us[8];         /* TIM compare values: M1..M4 */
    int      arm;               /* 0 disarmed, 1 armed */
    int      fault_state;       /* fault_state_t */
    float    ukf_q[4];
    float    ukf_w[3];
    float    adrc_fhat[3];
    float    mrac_theta1[3];
    float    mrac_theta2[3];
    uint32_t sched_overruns;
    uint32_t tick_count;
} FwOut;

void fw_host_init(const float gyro_cal_bias_dps[3]);
void fw_host_set_time(uint32_t now_ms);
void fw_host_rc_frame(const uint16_t rc_us[5]);     /* UART idle-line interrupt */
void fw_host_step(const FwImuIn *imu, FwOut *out);  /* one main-loop pass */

#endif
