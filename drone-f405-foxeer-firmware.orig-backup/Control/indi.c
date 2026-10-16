#include "indi.h"
#include "../Math/matrix.h"
#include <math.h>
#include <string.h>

/* Betaflight/KISS motor order, X-config:
 * M1=rear-right(CCW), M2=front-right(CW),
 * M3=rear-left(CW),   M4=front-left(CCW)
 *
 * G rows = [roll, pitch, yaw], cols = [M1..M4]
 * roll:  [-1, -1, +1, +1] × g_r
 * pitch: [+1, -1, +1, -1] × g_p
 * yaw:   [-1, +1, +1, -1] × g_y */

void indi_recompute_G(indi_t *indi, float arm_m, float k_thrust, float k_drag,
                      float ixx, float iyy, float izz)
{
    float lsq2   = arm_m * 0.70710678f;
    float g_r    = k_thrust * lsq2 / ixx;
    float g_p    = k_thrust * lsq2 / iyy;
    float g_y    = k_drag / izz;

    float *G = indi->G;
    G[0]  = -g_r;  G[1]  = -g_r;  G[2]  =  g_r;  G[3]  =  g_r;
    G[4]  =  g_p;  G[5]  = -g_p;  G[6]  =  g_p;  G[7]  = -g_p;
    G[8]  = -g_y;  G[9]  =  g_y;  G[10] =  g_y;  G[11] = -g_y;

    float GGt[INDI_AXES * INDI_AXES];
    float GGt_inv[INDI_AXES * INDI_AXES];

    mat_mul_bT(G, G, GGt, INDI_AXES, INDI_MOTORS, INDI_AXES);
    mat_inv(GGt, GGt_inv, INDI_AXES);
    mat_mul_aT(G, GGt_inv, indi->G_inv, INDI_AXES, INDI_MOTORS, INDI_AXES);
}

void indi_init(indi_t *indi, float arm_m, float k_thrust, float k_drag,
               float ixx, float iyy, float izz,
               float cutoff_hz, float dt)
{
    memset(indi, 0, sizeof(indi_t));
    indi->dt = dt;

    float wc = 2.0f * 3.14159265f * cutoff_hz;
    indi->alpha_filt = wc * dt / (1.0f + wc * dt);

    indi_recompute_G(indi, arm_m, k_thrust, k_drag, ixx, iyy, izz);
}

void indi_reset(indi_t *indi, float throttle_idle)
{
    for (uint8_t i = 0; i < INDI_MOTORS; i++) indi->u_prev[i] = throttle_idle;
    for (uint8_t i = 0; i < INDI_AXES; i++)   indi->omega_filt[i] = 0.0f;
}

void indi_update(indi_t *indi, const float omega_meas[INDI_AXES],
                 const float alpha_ref[INDI_AXES], float throttle,
                 float u_out[INDI_MOTORS])
{
    float alpha_meas[INDI_AXES];
    float oma = indi->alpha_filt;
    float omb = 1.0f - oma;

    for (uint8_t i = 0; i < INDI_AXES; i++) {
        float filt_new = oma * omega_meas[i] + omb * indi->omega_filt[i];
        alpha_meas[i]  = (filt_new - indi->omega_filt[i]) / indi->dt;
        indi->omega_filt[i] = filt_new;
    }

    float da[INDI_AXES];
    for (uint8_t i = 0; i < INDI_AXES; i++) da[i] = alpha_ref[i] - alpha_meas[i];

    float du[INDI_MOTORS];
    mat_mul(indi->G_inv, da, du, INDI_MOTORS, INDI_AXES, 1);

    float c_prev = 0.25f * (indi->u_prev[0] + indi->u_prev[1]
                           + indi->u_prev[2] + indi->u_prev[3]);

    for (uint8_t i = 0; i < INDI_MOTORS; i++) {
        float d = indi->u_prev[i] - c_prev + du[i];
        float u = throttle + d;
        u_out[i] = u < 0.0f ? 0.0f : (u > 1.0f ? 1.0f : u);
        indi->u_prev[i] = u_out[i];
    }
}
