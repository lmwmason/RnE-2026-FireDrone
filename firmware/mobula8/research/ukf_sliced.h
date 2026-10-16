/* SPDX-License-Identifier: GPL-3.0-or-later */
#pragma once
#include <stdbool.h>
#include "original/ukf.h"
#include "original/quaternion.h"
typedef struct {
    bool active, integrated, forceGyroOnly, accelRejected;
    quat_t rotation;
    unsigned phase, cursor;
    float dt, gate, gyro[3], accel[3];
    float xpred[STATE_DIM], Ppred[STATE_DIM*STATE_DIM];
    float h[SIGMA_COUNT*MEAS_DIM], zpred[MEAS_DIM], S[MEAS_DIM*MEAS_DIM];
    float Pxy[STATE_DIM*MEAS_DIM], v[MEAS_DIM], Sinv[MEAS_DIM*MEAS_DIM];
    float K[STATE_DIM*MEAS_DIM], KS[STATE_DIM*MEAS_DIM], KSKt[STATE_DIM*STATE_DIM];
} fdUkfWork;
bool fdUkfStart(fdUkfWork *w, const float gyro[3], const float accel[3], float dt, float gate);
/* One bounded slice. 0=in progress, 1=complete, -1=invalid state. */
int fdUkfSlice(ukf_t *u, fdUkfWork *w);

/* Integrate each PID sample; finite/interval failures must stop flight observation. */
bool fdGyroAdvance(quat_t *q, const float gyro[3], float dt);
bool fdUkfStartIntegrated(fdUkfWork *w, const float gyro[3], const float accel[3], float dt,
    float gate, const quat_t *rotation, bool forceGyroOnly);
