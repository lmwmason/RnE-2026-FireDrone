/* SPDX-License-Identifier: GPL-3.0-or-later */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "pg/pg.h"

typedef struct {
    uint8_t mode, blend, ukf, axis, auxChannel;
    uint16_t effectiveness[3]; /* 0 = unknown; rad/s^2 per PID unit * 100 */
    uint16_t bandwidth, rateGain, motorTauMs, derivativeHz, correctionLimit;
    uint16_t adaptation; /* / 1e6 */
    uint16_t motorTauYawMs; /* appended PG tail: 0 follows motorTauMs */
} firedroneConfig_t;
PG_DECLARE(firedroneConfig_t, firedroneConfig);
void firedroneUpdate(const float setpointDps[3], float dt, uint32_t nowUs, bool permitted);

bool firedroneLevelAngles(const float baseline[2], float out[2], uint32_t nowUs, float dt, bool permitted);

void firedroneRateLog(int32_t correction[3], int32_t candidate[3]);

void firedroneAttitudeLog(int32_t out[6]);

bool firedroneArmingReady(void);
