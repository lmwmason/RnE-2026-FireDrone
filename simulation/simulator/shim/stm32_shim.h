#ifndef STM32_SHIM_H
#define STM32_SHIM_H

#include "stm32f4xx_hal.h"

typedef struct {
    float gyro_dps[3];
    float accel_g[3];
} ShimImu;

extern uint32_t sim_tick_ms;
extern ShimImu  sim_imu_in;
extern float    sim_gyro_cal_bias[3];
extern float    sim_baro_alt_m;
extern uint8_t *sim_uart_rx_buf;
extern uint16_t sim_uart_rx_size;

#endif
