#ifndef CONTROL_INDI_H
#define CONTROL_INDI_H

#include <stdint.h>

#define INDI_AXES   3U
#define INDI_MOTORS 4U

typedef struct {
    float G[INDI_AXES * INDI_MOTORS];
    float G_inv[INDI_MOTORS * INDI_AXES];
    float omega_filt[INDI_AXES];
    float alpha_filt;
    float u_prev[INDI_MOTORS];
    float dt;
} indi_t;

void indi_init(indi_t *indi, float arm_m, float k_thrust, float k_drag,
               float ixx, float iyy, float izz,
               float cutoff_hz, float dt);
void indi_recompute_G(indi_t *indi, float arm_m, float k_thrust, float k_drag,
                      float ixx, float iyy, float izz);
void indi_update(indi_t *indi, const float omega_meas[INDI_AXES],
                 const float alpha_ref[INDI_AXES], float throttle,
                 float u_out[INDI_MOTORS]);
void indi_reset(indi_t *indi, float throttle_idle);

#endif
