#include "ukf.h"
#include "../Math/matrix.h"
#include "../Math/quaternion.h"
#include <math.h>
#include <string.h>

#define REG_EPS      1e-6f

#define UKF_QUIET_RATE_RPS  0.35f
#define UKF_QUIET_ACC_G     0.12f
#define UKF_R_ACCEL_QUIET   3.0f

static void sigma_to_quat(const float *sp, quat_t *q)
{
    q->w = sp[IDX_Q0];
    q->x = sp[IDX_Q1];
    q->y = sp[IDX_Q2];
    q->z = sp[IDX_Q3];
}

static void quat_to_sigma(const quat_t *q, float *sp)
{
    sp[IDX_Q0] = q->w;
    sp[IDX_Q1] = q->x;
    sp[IDX_Q2] = q->y;
    sp[IDX_Q3] = q->z;
}

static void process_sigma(float *sp, float dt)
{
    float omega[3] = { sp[IDX_WX], sp[IDX_WY], sp[IDX_WZ] };
    quat_t q, q_new;
    sigma_to_quat(sp, &q);
    quat_integrate(&q, omega, dt, &q_new);
    quat_to_sigma(&q_new, sp);
}

static void meas_sigma(const float *sp, float *h)
{
    quat_t q;
    sigma_to_quat(sp, &q);

    float R[9];
    quat_to_dcm(&q, R);

    h[0] = sp[IDX_WX];
    h[1] = sp[IDX_WY];
    h[2] = sp[IDX_WZ];

    /* accelerometer in g: world "up" expressed in the body frame = third ROW of R (body -> world) */
    h[3] = R[6] + sp[IDX_BAX];
    h[4] = R[7] + sp[IDX_BAY];
    h[5] = R[8] + sp[IDX_BAZ];
}

static void compute_sigma_points(ukf_t *ukf)
{
    float L[STATE_DIM * STATE_DIM];
    float Ps[STATE_DIM * STATE_DIM];

    float scale = (float)(STATE_DIM) + ukf->lambda;
    mat_scale(ukf->P, scale, Ps, STATE_DIM, STATE_DIM);

    if (mat_chol(Ps, L, STATE_DIM) != 0) {
        for (uint8_t i = 0; i < STATE_DIM; i++)
            Ps[(uint16_t)i * STATE_DIM + i] += REG_EPS;
        mat_chol(Ps, L, STATE_DIM);
    }

    mat_copy(ukf->x, &ukf->sigma[0], 1, STATE_DIM);

    for (uint8_t i = 0; i < STATE_DIM; i++) {
        for (uint8_t j = 0; j < STATE_DIM; j++) {
            float col_val = L[(uint16_t)j * STATE_DIM + i];
            ukf->sigma[(uint16_t)(i + 1)         * STATE_DIM + j] = ukf->x[j] + col_val;
            ukf->sigma[(uint16_t)(i + 1 + STATE_DIM) * STATE_DIM + j] = ukf->x[j] - col_val;
        }
        quat_t q;
        sigma_to_quat(&ukf->sigma[(uint16_t)(i + 1) * STATE_DIM], &q);
        quat_norm(&q);
        quat_to_sigma(&q, &ukf->sigma[(uint16_t)(i + 1) * STATE_DIM]);

        sigma_to_quat(&ukf->sigma[(uint16_t)(i + 1 + STATE_DIM) * STATE_DIM], &q);
        quat_norm(&q);
        quat_to_sigma(&q, &ukf->sigma[(uint16_t)(i + 1 + STATE_DIM) * STATE_DIM]);
    }
}

static void weighted_mean_state(ukf_t *ukf, float *x_mean)
{
    mat_zero(x_mean, 1, STATE_DIM);
    for (uint8_t i = 0; i < SIGMA_COUNT; i++) {
        float w = ukf->w_m[i];
        for (uint8_t j = 0; j < STATE_DIM; j++)
            x_mean[j] += w * ukf->sigma[(uint16_t)i * STATE_DIM + j];
    }
    quat_t q = { x_mean[IDX_Q0], x_mean[IDX_Q1], x_mean[IDX_Q2], x_mean[IDX_Q3] };
    quat_norm(&q);
    x_mean[IDX_Q0] = q.w;
    x_mean[IDX_Q1] = q.x;
    x_mean[IDX_Q2] = q.y;
    x_mean[IDX_Q3] = q.z;
}

void ukf_init(ukf_t *ukf, float alpha, float beta, float kappa)
{
    memset(ukf, 0, sizeof(ukf_t));

    float n     = (float)STATE_DIM;
    float alpha2 = alpha * alpha;
    ukf->lambda = alpha2 * (n + kappa) - n;

    float inv_2nl = 1.0f / (2.0f * (n + ukf->lambda));
    ukf->w_m[0] = ukf->lambda / (n + ukf->lambda);
    ukf->w_c[0] = ukf->w_m[0] + (1.0f - alpha2 + beta);
    for (uint8_t i = 1; i < SIGMA_COUNT; i++) {
        ukf->w_m[i] = inv_2nl;
        ukf->w_c[i] = inv_2nl;
    }

    ukf->x[IDX_Q0] = 1.0f;

    float p_q   = 0.01f;
    float p_w   = 0.1f;
    float p_ba  = 0.1f;
    for (uint8_t i = 0; i < 4; i++)   ukf->P[(uint16_t)i * STATE_DIM + i] = p_q;
    for (uint8_t i = 4; i < 7; i++)   ukf->P[(uint16_t)i * STATE_DIM + i] = p_w;
    for (uint8_t i = 7; i < 10; i++)  ukf->P[(uint16_t)i * STATE_DIM + i] = p_ba;

    float q_q   = 1e-5f;
    float q_w   = 1e-4f;
    float q_ba  = 1e-6f;
    for (uint8_t i = 0; i < 4; i++)   ukf->Q[(uint16_t)i * STATE_DIM + i] = q_q;
    for (uint8_t i = 4; i < 7; i++)   ukf->Q[(uint16_t)i * STATE_DIM + i] = q_w;
    for (uint8_t i = 7; i < 10; i++)  ukf->Q[(uint16_t)i * STATE_DIM + i] = q_ba;

    float r_gyro  = 1e-4f;
    float r_accel = 1e2f;
    for (uint8_t i = 0; i < 3; i++)   ukf->R[(uint16_t)i * MEAS_DIM + i] = r_gyro;
    for (uint8_t i = 3; i < 6; i++)   ukf->R[(uint16_t)i * MEAS_DIM + i] = r_accel;
}

void ukf_predict(ukf_t *ukf, const float gyro[3], float dt)
{
    (void)gyro;

    compute_sigma_points(ukf);

    for (uint8_t i = 0; i < SIGMA_COUNT; i++)
        process_sigma(&ukf->sigma[(uint16_t)i * STATE_DIM], dt);

    float x_pred[STATE_DIM];
    weighted_mean_state(ukf, x_pred);

    float P_pred[STATE_DIM * STATE_DIM];
    mat_copy(ukf->Q, P_pred, STATE_DIM, STATE_DIM);

    for (uint8_t i = 0; i < SIGMA_COUNT; i++) {
        float dx[STATE_DIM];
        for (uint8_t j = 0; j < STATE_DIM; j++)
            dx[j] = ukf->sigma[(uint16_t)i * STATE_DIM + j] - x_pred[j];

        float w = ukf->w_c[i];
        for (uint8_t r = 0; r < STATE_DIM; r++)
            for (uint8_t c = 0; c < STATE_DIM; c++)
                P_pred[(uint16_t)r * STATE_DIM + c] += w * dx[r] * dx[c];
    }

    mat_sym(P_pred, STATE_DIM);
    mat_copy(x_pred, ukf->x, 1, STATE_DIM);
    mat_copy(P_pred, ukf->P, STATE_DIM, STATE_DIM);
}

void ukf_update(ukf_t *ukf, const float accel[3], const float gyro[3], float chi2_gate)
{
    compute_sigma_points(ukf);

    float h_sigma[SIGMA_COUNT * MEAS_DIM];
    for (uint8_t i = 0; i < SIGMA_COUNT; i++)
        meas_sigma(&ukf->sigma[(uint16_t)i * STATE_DIM], &h_sigma[(uint16_t)i * MEAS_DIM]);

    float z_pred[MEAS_DIM];
    mat_zero(z_pred, 1, MEAS_DIM);
    for (uint8_t i = 0; i < SIGMA_COUNT; i++) {
        float w = ukf->w_m[i];
        for (uint8_t j = 0; j < MEAS_DIM; j++)
            z_pred[j] += w * h_sigma[(uint16_t)i * MEAS_DIM + j];
    }

    float S[MEAS_DIM * MEAS_DIM];
    mat_copy(ukf->R, S, MEAS_DIM, MEAS_DIM);

    /* The accelerometer of a multirotor reads thrust, so it only says "level" when the airframe is
     * quiet and carrying 1 g.  Trust it then (this is what removes gyro bias), ignore it otherwise. */
    float g_norm = sqrtf(gyro[0] * gyro[0] + gyro[1] * gyro[1] + gyro[2] * gyro[2]);
    float a_norm = sqrtf(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);
    if (g_norm < UKF_QUIET_RATE_RPS && fabsf(a_norm - 1.0f) < UKF_QUIET_ACC_G)
        for (uint8_t i = 3; i < 6; i++)
            S[(uint16_t)i * MEAS_DIM + i] += UKF_R_ACCEL_QUIET - ukf->R[(uint16_t)i * MEAS_DIM + i];

    float Pxy[STATE_DIM * MEAS_DIM];
    mat_zero(Pxy, STATE_DIM, MEAS_DIM);

    for (uint8_t i = 0; i < SIGMA_COUNT; i++) {
        float dx[STATE_DIM], dh[MEAS_DIM];
        float w = ukf->w_c[i];

        for (uint8_t j = 0; j < STATE_DIM; j++)
            dx[j] = ukf->sigma[(uint16_t)i * STATE_DIM + j] - ukf->x[j];
        for (uint8_t j = 0; j < MEAS_DIM; j++)
            dh[j] = h_sigma[(uint16_t)i * MEAS_DIM + j] - z_pred[j];

        for (uint8_t r = 0; r < MEAS_DIM; r++)
            for (uint8_t c = 0; c < MEAS_DIM; c++)
                S[(uint16_t)r * MEAS_DIM + c] += w * dh[r] * dh[c];

        for (uint8_t r = 0; r < STATE_DIM; r++)
            for (uint8_t c = 0; c < MEAS_DIM; c++)
                Pxy[(uint16_t)r * MEAS_DIM + c] += w * dx[r] * dh[c];
    }

    float z[MEAS_DIM];
    z[0] = gyro[0];  z[1] = gyro[1];  z[2] = gyro[2];
    z[3] = accel[0]; z[4] = accel[1]; z[5] = accel[2];

    float v[MEAS_DIM];
    for (uint8_t j = 0; j < MEAS_DIM; j++) v[j] = z[j] - z_pred[j];

    float S_inv[MEAS_DIM * MEAS_DIM];
    if (mat_inv(S, S_inv, MEAS_DIM) != 0) return;

    /* A manoeuvre makes the accelerometer disagree with the model (it reads thrust, not gravity).
     * Reject only the accel block; the gyro block must still update the rate states, otherwise the
     * attitude integrates stale rates for as long as the manoeuvre lasts. */
    if (chi2_gate > 0.0f) {
        float Sa[9], Sa_inv[9], va[3] = { v[3], v[4], v[5] };
        for (uint8_t r = 0; r < 3; r++)
            for (uint8_t c = 0; c < 3; c++) Sa[r * 3 + c] = S[(uint16_t)(r + 3) * MEAS_DIM + (c + 3)];
        float mahal_sq = 1e30f;
        if (mat_inv(Sa, Sa_inv, 3) == 0) {
            float t[3];
            mat_mul(Sa_inv, va, t, 3, 3, 1);
            mahal_sq = va[0] * t[0] + va[1] * t[1] + va[2] * t[2];
        }
        if (mahal_sq > chi2_gate) {
            float Sg[9], Sg_inv[9];
            for (uint8_t r = 0; r < 3; r++)
                for (uint8_t c = 0; c < 3; c++) Sg[r * 3 + c] = S[(uint16_t)r * MEAS_DIM + c];
            if (mat_inv(Sg, Sg_inv, 3) != 0) return;
            float Kg[STATE_DIM * 3], Pg[STATE_DIM * 3];
            for (uint8_t r = 0; r < STATE_DIM; r++)
                for (uint8_t c = 0; c < 3; c++) Pg[r * 3 + c] = Pxy[(uint16_t)r * MEAS_DIM + c];
            mat_mul(Pg, Sg_inv, Kg, STATE_DIM, 3, 3);
            float vg[3] = { v[0], v[1], v[2] }, Kvg[STATE_DIM];
            mat_mul(Kg, vg, Kvg, STATE_DIM, 3, 1);
            for (uint8_t j = 0; j < STATE_DIM; j++) ukf->x[j] += Kvg[j];
            quat_t qg = { ukf->x[IDX_Q0], ukf->x[IDX_Q1], ukf->x[IDX_Q2], ukf->x[IDX_Q3] };
            quat_norm(&qg);
            ukf->x[IDX_Q0] = qg.w; ukf->x[IDX_Q1] = qg.x; ukf->x[IDX_Q2] = qg.y; ukf->x[IDX_Q3] = qg.z;
            float KS[STATE_DIM * 3], KSKt[STATE_DIM * STATE_DIM];
            mat_mul(Kg, Sg, KS, STATE_DIM, 3, 3);
            mat_mul_bT(KS, Kg, KSKt, STATE_DIM, 3, STATE_DIM);
            mat_sub(ukf->P, KSKt, ukf->P, STATE_DIM, STATE_DIM);
            mat_sym(ukf->P, STATE_DIM);
            return;
        }
    }

    float K[STATE_DIM * MEAS_DIM];
    mat_mul(Pxy, S_inv, K, STATE_DIM, MEAS_DIM, MEAS_DIM);

    float Kv[STATE_DIM];
    mat_mul(K, v, Kv, STATE_DIM, MEAS_DIM, 1);
    for (uint8_t j = 0; j < STATE_DIM; j++) ukf->x[j] += Kv[j];

    quat_t q = { ukf->x[IDX_Q0], ukf->x[IDX_Q1], ukf->x[IDX_Q2], ukf->x[IDX_Q3] };
    quat_norm(&q);
    ukf->x[IDX_Q0] = q.w;
    ukf->x[IDX_Q1] = q.x;
    ukf->x[IDX_Q2] = q.y;
    ukf->x[IDX_Q3] = q.z;

    float KS[STATE_DIM * MEAS_DIM];
    mat_mul(K, S, KS, STATE_DIM, MEAS_DIM, MEAS_DIM);

    float KSKt[STATE_DIM * STATE_DIM];
    mat_mul_bT(KS, K, KSKt, STATE_DIM, MEAS_DIM, STATE_DIM);

    mat_sub(ukf->P, KSKt, ukf->P, STATE_DIM, STATE_DIM);
    mat_sym(ukf->P, STATE_DIM);
}

void ukf_get_attitude(const ukf_t *ukf, float q[4])
{
    q[0] = ukf->x[IDX_Q0];
    q[1] = ukf->x[IDX_Q1];
    q[2] = ukf->x[IDX_Q2];
    q[3] = ukf->x[IDX_Q3];
}

void ukf_get_rates(const ukf_t *ukf, float omega[3])
{
    omega[0] = ukf->x[IDX_WX];
    omega[1] = ukf->x[IDX_WY];
    omega[2] = ukf->x[IDX_WZ];
}
