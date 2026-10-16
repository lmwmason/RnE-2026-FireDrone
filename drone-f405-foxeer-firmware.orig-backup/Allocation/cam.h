#ifndef ALLOCATION_CAM_H
#define ALLOCATION_CAM_H

#include <stdint.h>

#define CAM_OUT   4U
#define CAM_IN    4U

typedef struct {
    float B[CAM_IN * CAM_OUT];
    float W1_diag[CAM_IN];
    float W2_diag[CAM_OUT];
    uint8_t fault_mask;
    uint8_t yaw_ctrl_en;
} cam_t;

void cam_init(cam_t *cam, float k_t, float k_m, float arm_m);
void cam_set_fault(cam_t *cam, uint8_t motor_idx);
void cam_clear_faults(cam_t *cam);
int  cam_solve(cam_t *cam, const float tau_des[CAM_IN], float u_out[CAM_OUT]);

#endif
