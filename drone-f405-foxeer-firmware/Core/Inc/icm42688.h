#ifndef ICM42688_H
#define ICM42688_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float gyro_dps[3];   /* roll, pitch, yaw rate, deg/s */
    float accel_g[3];    /* X, Y, Z, g */
} icm42688_data_t;

bool ICM42688_Init(void);
bool ICM42688_Read(icm42688_data_t *out);
void ICM42688_CalibrateGyroBias(float bias_dps[3], uint16_t samples);

#endif /* ICM42688_H */
