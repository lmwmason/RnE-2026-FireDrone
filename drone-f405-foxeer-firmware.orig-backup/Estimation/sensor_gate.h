#ifndef ESTIMATION_SENSOR_GATE_H
#define ESTIMATION_SENSOR_GATE_H

#include <stdint.h>
#include <stdbool.h>

#define CHI2_95_6DOF  12.592f
#define CHI2_99_6DOF  16.812f
#define CHI2_95_3DOF   7.815f
#define CHI2_99_3DOF  11.345f

bool sensor_gate_check(const float *v, const float *S_inv, uint8_t n, float threshold);

#endif
