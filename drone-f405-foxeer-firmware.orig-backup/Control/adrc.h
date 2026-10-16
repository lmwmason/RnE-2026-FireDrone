#ifndef CONTROL_ADRC_H
#define CONTROL_ADRC_H

#include <stdint.h>

typedef struct {
    float z1;
    float z2;
    float z3;
    float beta1;
    float beta2;
    float beta3;
    float b0;
    float kp;
    float kd;
    float dt;
} adrc_axis_t;

typedef struct {
    adrc_axis_t axis[3];
} adrc_t;

void adrc_axis_init(adrc_axis_t *ax, float omega_o, float b0, float kp, float kd, float dt);
float adrc_axis_update(adrc_axis_t *ax, float y_meas, float r, float r_dot, float u_prev);
void adrc_init(adrc_t *adrc, float omega_o_roll, float omega_o_pitch, float omega_o_yaw,
               float b0_roll, float b0_pitch, float b0_yaw,
               float kp, float kd, float dt);
void adrc_update(adrc_t *adrc, const float theta_meas[3], const float r[3],
                 const float r_dot[3], const float u_prev[3], float u_out[3]);
void adrc_get_disturbance(const adrc_t *adrc, float f_hat[3]);

#endif
