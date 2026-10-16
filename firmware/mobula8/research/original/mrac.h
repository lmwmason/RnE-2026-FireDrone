#ifndef CONTROL_MRAC_H
#define CONTROL_MRAC_H

#include <stdint.h>

/* Per-axis MRAC for angular rate control.
 * Plant:   ω_dot = θ1*u - θ2*ω + d  (unknown θ1=b, θ2=a)
 * Ref model: ωm_dot = -am*ωm + bm*r
 * Control: u = θ1_hat*r + θ2_hat*ω
 * Adaptation (gradient descent, Lyapunov-stable):
 *   dθ1_hat/dt = -γ1 * e * r
 *   dθ2_hat/dt = -γ2 * e * ω  */

typedef struct {
    float am;
    float bm;
    float omega_m;
    float theta1_hat;
    float theta2_hat;
    float gamma1;
    float gamma2;
    float theta1_min;
    float theta2_min;
    float dt;
} mrac_axis_t;

typedef struct {
    mrac_axis_t axis[3];
} mrac_t;

void  mrac_axis_init(mrac_axis_t *ax, float am, float bm,
                     float gamma1, float gamma2,
                     float theta1_init, float theta2_init,
                     float theta1_min, float theta2_min,
                     float dt);
float mrac_axis_update(mrac_axis_t *ax, float omega_meas, float r);
void  mrac_init(mrac_t *mrac, float am, float bm,
                float gamma1, float gamma2,
                float theta1_init, float theta2_init,
                float dt);
void  mrac_update(mrac_t *mrac, const float omega_meas[3], const float r[3],
                  float u_out[3]);
void  mrac_get_params(const mrac_t *mrac, float theta1[3], float theta2[3]);

#endif
