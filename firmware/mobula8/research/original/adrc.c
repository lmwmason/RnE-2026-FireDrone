#include "adrc.h"
#include <string.h>

/* Rate-loop ESO (2nd order).  The measured signal is the body rate, so the
 * plant is  omega_dot = f + b0*u  with  b0*u = angular acceleration that was
 * commanded to the inner (INDI) loop one tick ago.
 *   e   = y - z1
 *   z1 += dt * (z2 + b0*u + 2*w_o*e)        (z1 ~ omega)
 *   z2 += dt * (w_o^2 * e)                  (z2 ~ f, total disturbance)
 * Both poles at -w_o.  z3 mirrors z2 so adrc_get_disturbance() is unchanged. */

void adrc_axis_init(adrc_axis_t *ax, float omega_o, float b0, float kp, float kd, float dt)
{
    memset(ax, 0, sizeof(adrc_axis_t));
    ax->beta1 = 2.0f * omega_o;
    ax->beta2 = omega_o * omega_o;
    ax->beta3 = 0.0f;
    ax->b0    = b0;
    ax->kp    = kp;
    ax->kd    = kd;
    ax->dt    = dt;
}

float adrc_axis_update(adrc_axis_t *ax, float y_meas, float r, float r_dot, float u_prev)
{
    float e = y_meas - ax->z1;

    ax->z1 += ax->dt * (ax->z2 + ax->b0 * u_prev + ax->beta1 * e);
    ax->z2 += ax->dt * (ax->beta2 * e);
    ax->z3  = ax->z2;

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
