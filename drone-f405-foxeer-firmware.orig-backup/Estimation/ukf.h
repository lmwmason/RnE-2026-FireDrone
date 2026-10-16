#ifndef ESTIMATION_UKF_H
#define ESTIMATION_UKF_H

#include <stdint.h>

#define STATE_DIM   10U
#define MEAS_DIM    6U
#define SIGMA_COUNT (2U * STATE_DIM + 1U)

#define IDX_Q0  0U
#define IDX_Q1  1U
#define IDX_Q2  2U
#define IDX_Q3  3U
#define IDX_WX  4U
#define IDX_WY  5U
#define IDX_WZ  6U
#define IDX_BAX 7U
#define IDX_BAY 8U
#define IDX_BAZ 9U

typedef struct {
    float x[STATE_DIM];
    float P[STATE_DIM * STATE_DIM];
    float Q[STATE_DIM * STATE_DIM];
    float R[MEAS_DIM  * MEAS_DIM];
    float sigma[SIGMA_COUNT * STATE_DIM];
    float w_m[SIGMA_COUNT];
    float w_c[SIGMA_COUNT];
    float lambda;
} ukf_t;

void ukf_init(ukf_t *ukf, float alpha, float beta, float kappa);
void ukf_predict(ukf_t *ukf, const float gyro[3], float dt);
void ukf_update(ukf_t *ukf, const float accel[3], const float gyro[3], float chi2_gate);
void ukf_get_attitude(const ukf_t *ukf, float q[4]);
void ukf_get_rates(const ukf_t *ukf, float omega[3]);

#endif
