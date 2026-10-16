#include "adrc.h"
#include <string.h>

/* 3rd-order ESO for 2nd-order plant:
 *   θ_dot  = ω          (z1 ≈ θ)
 *   ω_dot  = f + b0*u   (z2 ≈ ω, z3 ≈ f = total disturbance)
 *
 * ESO (Euler discrete):
 *   e      = y - z1
 *   z1    += dt * (z2 + β1*e)
 *   z2    += dt * (z3 + β2*e + b0*u)
 *   z3    += dt * (β3*e)
 *
 * Pole placement at -ω_o (all three):
 *   β1 = 3*ω_o,  β2 = 3*ω_o²,  β3 = ω_o³
 *
 * Control law:
 *   u0  = kp*(r - z1) + kd*(r_dot - z2)
 *   u   = (u0 - z3) / b0                */

void adrc_axis_init(adrc_axis_t *ax, float omega_o, float b0, float kp, float kd, float dt)
{
    memset(ax, 0, sizeof(adrc_axis_t));
    ax->beta1 = 3.0f * omega_o;
    ax->beta2 = 3.0f * omega_o * omega_o;
    ax->beta3 = omega_o * omega_o * omega_o;
    ax->b0    = b0;
    ax->kp    = kp;
    ax->kd    = kd;
    ax->dt    = dt;
}

float adrc_axis_update(adrc_axis_t *ax, float y_meas, float r, float r_dot, float u_prev)
{
    float e   = y_meas - ax->z1;

    ax->z1 += ax->dt * (ax->z2 + ax->beta1 * e);
    ax->z2 += ax->dt * (ax->z3 + ax->beta2 * e + ax->b0 * u_prev);
    ax->z3 += ax->dt * (ax->beta3 * e);

    float u0 = ax->kp * (r - ax->z1) + ax->kd * (r_dot - ax->z2);
    float u  = (u0 - ax->z3) / ax->b0;
    return u;
}

void adrc_init(adrc_t *adrc,
               float omega_o_roll, float omega_o_pitch, float omega_o_yaw,
               float b0_roll, float b0_pitch, float b0_yaw,
               float kp, float kd, float dt)
{
    adrc_axis_init(&adrc->axis[0], omega_o_roll,  b0_roll,  kp, kd, dt);
    adrc_axis_init(&adrc->axis[1], omega_o_pitch, b0_pitch, kp, kd, dt);
    adrc_axis_init(&adrc->axis[2], omega_o_yaw,   b0_yaw,   kp, kd, dt);
}

void adrc_update(adrc_t *adrc, const float theta_meas[3], const float r[3],
                 const float r_dot[3], const float u_prev[3], float u_out[3])
{
    for (uint8_t i = 0; i < 3; i++)
        u_out[i] = adrc_axis_update(&adrc->axis[i],
                                    theta_meas[i], r[i], r_dot[i], u_prev[i]);
}

void adrc_get_disturbance(const adrc_t *adrc, float f_hat[3])
{
    for (uint8_t i = 0; i < 3; i++) f_hat[i] = adrc->axis[i].z3;
}
