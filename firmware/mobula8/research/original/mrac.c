#include "mrac.h"
#include <string.h>

void mrac_axis_init(mrac_axis_t *ax, float am, float bm,
                    float gamma1, float gamma2,
                    float theta1_init, float theta2_init,
                    float theta1_min, float theta2_min,
                    float dt)
{
    memset(ax, 0, sizeof(mrac_axis_t));
    ax->am          = am;
    ax->bm          = bm;
    ax->gamma1      = gamma1;
    ax->gamma2      = gamma2;
    ax->theta1_hat  = theta1_init;
    ax->theta2_hat  = theta2_init;
    ax->theta1_min  = theta1_min;
    ax->theta2_min  = theta2_min;
    ax->dt          = dt;
    ax->omega_m     = 0.0f;
}

float mrac_axis_update(mrac_axis_t *ax, float omega_meas, float r)
{
    ax->omega_m += ax->dt * (-ax->am * ax->omega_m + ax->bm * r);

    float e = omega_meas - ax->omega_m;

    ax->theta1_hat -= ax->dt * ax->gamma1 * e * r;
    ax->theta2_hat -= ax->dt * ax->gamma2 * e * omega_meas;

    if (ax->theta1_hat < ax->theta1_min) ax->theta1_hat = ax->theta1_min;
    if (ax->theta2_hat < ax->theta2_min) ax->theta2_hat = ax->theta2_min;

    return ax->theta1_hat * r + ax->theta2_hat * omega_meas;
}

void mrac_init(mrac_t *mrac, float am, float bm,
               float gamma1, float gamma2,
               float theta1_init, float theta2_init,
               float dt)
{
    for (uint8_t i = 0; i < 3; i++) {
        mrac_axis_init(&mrac->axis[i], am, bm,
                       gamma1, gamma2,
                       theta1_init, theta2_init,
                       0.01f, 0.0f, dt);
    }
}

void mrac_update(mrac_t *mrac, const float omega_meas[3], const float r[3],
                 float u_out[3])
{
    for (uint8_t i = 0; i < 3; i++)
        u_out[i] = mrac_axis_update(&mrac->axis[i], omega_meas[i], r[i]);
}

void mrac_get_params(const mrac_t *mrac, float theta1[3], float theta2[3])
{
    for (uint8_t i = 0; i < 3; i++) {
        theta1[i] = mrac->axis[i].theta1_hat;
        theta2[i] = mrac->axis[i].theta2_hat;
    }
}
