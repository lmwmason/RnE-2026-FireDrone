#ifndef DRIVERS_IMU_H
#define DRIVERS_IMU_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    float gyro_dps[3];
    float accel_g[3];
} imu_data_t;

bool imu_init(void);
bool imu_read(imu_data_t *out);
void imu_calibrate_gyro(float bias_dps[3], uint16_t samples);

#endif
