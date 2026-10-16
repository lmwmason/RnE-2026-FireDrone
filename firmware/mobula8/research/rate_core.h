/* SPDX-License-Identifier: GPL-3.0-or-later */
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "original/adrc.h"
#include "original/mrac.h"

typedef struct {
    uint8_t mode; /* 0 OFF, 1 SHADOW, 2 bounded INDI/ADRC/MRAC blend */
    float effectiveness[3]; /* rad/s^2 per Betaflight PID-sum unit; identified, not inertia */
    float bandwidth, rate_gain, motor_tau, derivative_hz;
    float motor_tau_yaw; /* 0 inherits common lag */
    float blend, correction_limit, adaptation;
} fdConfig;
typedef struct {
    float rate[3], setpoint[3], baseline[3], dt;
    bool eligible; /* armed, non-low throttle, healthy RX, no special BF recovery */
    bool saturated;
} fdInput;
typedef struct {
    adrc_t adrc;
    mrac_t mrac;
    float filtered[3], alpha[3], lag[3], applied[3];
    float candidate[3], disturbance[3], correction[3];
    fdConfig config;
    float inverseEffectiveness[3], filteredLag[3];
    float cachedDt, inverseDt, derivativeK, lagK[3];
    bool initialized, valid;
    float warmup;
} fdState;
void fdReset(fdState *s);
bool fdFinite(float x);
bool fdConfigValid(const fdConfig *c);
/* Returns BF baseline unchanged unless every guard is satisfied. */
void fdStep(fdState *s, const fdConfig *c, const fdInput *in, float out[3]);
