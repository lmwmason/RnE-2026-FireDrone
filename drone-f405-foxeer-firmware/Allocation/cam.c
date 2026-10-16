#include "cam.h"
#include "../Math/matrix.h"
#include <string.h>

/* B [CAM_IN × CAM_OUT] maps motor commands → [Fz, τx, τy, τz].
 * Motor order: M1=rear-right(CCW), M2=front-right(CW),
 *              M3=rear-left(CW),   M4=front-left(CCW).
 *
 * Row 0 (Fz):  [k_t,    k_t,    k_t,    k_t  ]
 * Row 1 (τx):  [-k_t*l, -k_t*l, +k_t*l, +k_t*l]   l = arm/√2
 * Row 2 (τy):  [+k_t*l, -k_t*l, +k_t*l, -k_t*l]
 * Row 3 (τz):  [-k_m,   +k_m,   +k_m,   -k_m  ] */

static void rebuild_B(cam_t *cam, float k_t, float k_m, float arm_m)
{
    float l = arm_m * 0.70710678f;
    float *B = cam->B;

    B[0]  =  k_t;   B[1]  =  k_t;   B[2]  =  k_t;   B[3]  =  k_t;
    B[4]  = -k_t*l; B[5]  = -k_t*l; B[6]  =  k_t*l; B[7]  =  k_t*l;
    B[8]  =  k_t*l; B[9]  = -k_t*l; B[10] =  k_t*l; B[11] = -k_t*l;
    B[12] = -k_m;   B[13] =  k_m;   B[14] =  k_m;   B[15] = -k_m;
}

void cam_init(cam_t *cam, float k_t, float k_m, float arm_m)
{
    memset(cam, 0, sizeof(cam_t));
    cam->yaw_ctrl_en = 1;

    rebuild_B(cam, k_t, k_m, arm_m);

    /* equal weight for a 1 N thrust error and the same force at the arm; yaw by its own scale */
    float l = arm_m * 0.70710678f;
    cam->W1_diag[0] = 1.0f;
    cam->W1_diag[1] = 1.0f / (l * l);
    cam->W1_diag[2] = 1.0f / (l * l);
    cam->W1_diag[3] = 1.0f / (k_m * k_m);
    for (uint8_t i = 0; i < CAM_OUT; i++) cam->W2_diag[i] = 1e-4f;
}

void cam_set_fault(cam_t *cam, uint8_t motor_idx)
{
    if (motor_idx >= CAM_OUT) return;
    cam->fault_mask |= (uint8_t)(1u << motor_idx);
    cam->W2_diag[motor_idx] = 1e9f;

    uint8_t n_faulted = 0;
    for (uint8_t i = 0; i < CAM_OUT; i++)
        if (cam->fault_mask & (1u << i)) n_faulted++;
    if (n_faulted >= 1) { cam->yaw_ctrl_en = 0; cam->W1_diag[3] = 0.0f; }
}

void cam_clear_faults(cam_t *cam)
{
    cam->fault_mask = 0;
    cam->yaw_ctrl_en = 1;
    cam->W1_diag[3] = 1.0f / (cam->B[13] * cam->B[13]);
    for (uint8_t i = 0; i < CAM_OUT; i++) cam->W2_diag[i] = 1e-4f;
}

int cam_solve(cam_t *cam, const float tau_des[CAM_IN], float u_out[CAM_OUT])
{
    float tau[CAM_IN];
    for (uint8_t i = 0; i < CAM_IN; i++) tau[i] = tau_des[i];
    if (!cam->yaw_ctrl_en) tau[3] = 0.0f;

    float A[CAM_OUT * CAM_OUT];
    float b[CAM_OUT];

    for (uint8_t i = 0; i < CAM_OUT; i++) {
        b[i] = 0.0f;
        for (uint8_t k = 0; k < CAM_IN; k++)
            b[i] += cam->B[(uint16_t)k * CAM_OUT + i] * cam->W1_diag[k] * tau[k];

        for (uint8_t j = 0; j < CAM_OUT; j++) {
            float s = (i == j) ? cam->W2_diag[i] : 0.0f;
            for (uint8_t k = 0; k < CAM_IN; k++)
                s += cam->B[(uint16_t)k * CAM_OUT + i]
                   * cam->W1_diag[k]
                   * cam->B[(uint16_t)k * CAM_OUT + j];
            A[(uint16_t)i * CAM_OUT + j] = s;
        }
    }

    float A_inv[CAM_OUT * CAM_OUT];
    if (mat_inv(A, A_inv, CAM_OUT) != 0) {
        for (uint8_t i = 0; i < CAM_OUT; i++) u_out[i] = 0.0f;
        return -1;
    }

    mat_mul(A_inv, b, u_out, CAM_OUT, CAM_OUT, 1);

    for (uint8_t i = 0; i < CAM_OUT; i++) {
        if (u_out[i] < 0.0f) u_out[i] = 0.0f;
        if (u_out[i] > 1.0f) u_out[i] = 1.0f;
    }

    return 0;
}
