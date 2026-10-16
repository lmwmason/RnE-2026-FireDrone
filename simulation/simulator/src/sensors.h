#ifndef SENSORS_H
#define SENSORS_H

#include "physics.h"
#include "rng.h"

/* ICM-42688 style IMU in the *sensor* frame.  The firmware applies a CW270
 * alignment to the gyro:  rate = (-raw[1], raw[0], raw[2]), so the model
 * produces raw[0] = w.y, raw[1] = -w.x, raw[2] = w.z. */
typedef struct {
    double gyro_sigma_dps;
    double accel_sigma_g;
    double gyro_walk_dps;
    double gyro_bias_dps[3];
    double accel_bias_g[3];

    /* injected faults */
    double gyro_fault_dps[3];
    int    accel_frozen;
    double accel_hold[3];
    double spike_prob;
    double spike_g;

    Rng   *rng;
} ImuModel;

typedef struct {
    float gyro_dps[3];
    float accel_g[3];
} ImuSample;

void imu_model_init(ImuModel *m, Rng *rng);
void imu_model_sample(ImuModel *m, const DroneState *s, Vec3 f_spec_body,
                      double mean_cmd, double dt, ImuSample *out);

/* position / velocity source for the outer loop (GPS / VIO class) */
typedef struct {
    double pos_sigma, vel_sigma;
    double period;
    double next_t;
    Vec3   pos, vel;
    Rng   *rng;
} NavModel;

void nav_model_init(NavModel *n, Rng *rng);
void nav_model_update(NavModel *n, const DroneState *s, double t);

#endif
