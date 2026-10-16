/* SPDX-License-Identifier: GPL-3.0-or-later
 * Cooperative slices of original/ukf.c equations. Original source is preserved.
 * Input snapshot stays immutable until a full predict/update cycle commits.
 */
#include "ukf_sliced.h"
#include "original/matrix.h"
#include "original/quaternion.h"
#include "rate_core.h"
#include <math.h>
#include <string.h>
#define REG_EPS 1e-6f
#define UKF_QUIET_RATE_RPS .35f
#define UKF_QUIET_ACC_G .12f
#define UKF_R_ACCEL_QUIET 3.0f
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

static void process_sigma(float *sp, float dt, const fdUkfWork *work)
{
    float omega[3] = { sp[IDX_WX], sp[IDX_WY], sp[IDX_WZ] };
    quat_t q, q_new;
    sigma_to_quat(sp, &q);
    if (work->integrated) {
        /* Measured rotation is a process input; Q retains attitude uncertainty.
         * Do not add the old rate-state prediction a second time. */
        quat_mul(&q,&work->rotation,&q_new);quat_norm(&q_new);
    } else quat_integrate(&q, omega, dt, &q_new);
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

static bool compute_sigma_points(ukf_t *ukf)
{
    float L[STATE_DIM * STATE_DIM];
    float Ps[STATE_DIM * STATE_DIM];

    float scale = (float)(STATE_DIM) + ukf->lambda;
    mat_scale(ukf->P, scale, Ps, STATE_DIM, STATE_DIM);

    if (mat_chol(Ps, L, STATE_DIM) != 0) {
        for (uint8_t i = 0; i < STATE_DIM; i++)
            Ps[(uint16_t)i * STATE_DIM + i] += REG_EPS;
        if (mat_chol(Ps, L, STATE_DIM) != 0) return false;
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
    return true;
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


static bool finiteState(const ukf_t *u)
{
    for (unsigned i=0;i<STATE_DIM;i++) if (!fdFinite(u->x[i])) return false;
    for (unsigned i=0;i<STATE_DIM*STATE_DIM;i++) if (!fdFinite(u->P[i])) return false;
    return true;
}
static int finish(ukf_t *u, fdUkfWork *w)
{
    w->active=false;
    return finiteState(u) ? 1 : -1;
}
bool fdUkfStart(fdUkfWork *w, const float gyro[3], const float accel[3], float dt, float gate)
{
    if (w->active || !fdFinite(dt) || dt<=0 || dt>.05f) return false;
    for (unsigned i=0;i<3;i++) if (!fdFinite(gyro[i]) || !fdFinite(accel[i])) return false;
    memcpy(w->gyro,gyro,sizeof(w->gyro));memcpy(w->accel,accel,sizeof(w->accel));
    w->integrated=false;w->forceGyroOnly=false;w->accelRejected=false;w->dt=dt;w->gate=gate;w->phase=0;w->cursor=0;w->active=true;return true;
}
bool fdGyroAdvance(quat_t *q, const float gyro[3], float dt)
{
    if(!fdFinite(dt) || dt<=0 || dt>.002f)return false;
    for(unsigned j=0;j<3;j++)if(!fdFinite(gyro[j]) || fabsf(gyro[j])>40)return false;
    if(!fdFinite(q->w) || !fdFinite(q->x) || !fdFinite(q->y) || !fdFinite(q->z))return false;
    float n=quat_norm_sq(q);if(n<.5f || n>1.5f)return false;
    float x=gyro[0]*dt*.5f,y=gyro[1]*dt*.5f,z=gyro[2]*dt*.5f;
    float a=x*x+y*y+z*z;
    /* Bounded |half rotation| <=0.07 rad: fourth-order exponential map. */
    float scale=1-a/6+a*a/120;
    quat_t dq={1-a*.5f+a*a/24,x*scale,y*scale,z*scale},out;
    quat_mul(q,&dq,&out);quat_norm(&out);*q=out;return true;
}
bool fdUkfStartIntegrated(fdUkfWork *w,const float gyro[3],const float accel[3],float dt,
    float gate,const quat_t *rotation,bool forceGyroOnly)
{
    if(!fdFinite(rotation->w)||!fdFinite(rotation->x)||!fdFinite(rotation->y)||!fdFinite(rotation->z))return false;
    float n=quat_norm_sq(rotation);if(n<.99f||n>1.01f)return false;
    if(!fdUkfStart(w,gyro,accel,dt,gate))return false;
    w->integrated=true;w->rotation=*rotation;w->forceGyroOnly=forceGyroOnly;return true;
}
int fdUkfSlice(ukf_t *u, fdUkfWork *w)
{
    if (!w->active) return -1;
    switch(w->phase) {
    case 0:
        if (!compute_sigma_points(u)) {w->active=false;return -1;}
        w->phase++;break;
    case 1:
        for (unsigned i=0;i<SIGMA_COUNT;i++) process_sigma(&u->sigma[i*STATE_DIM],w->dt,w);
        weighted_mean_state(u,w->xpred);
        mat_copy(u->Q,w->Ppred,STATE_DIM,STATE_DIM);w->cursor=0;w->phase++;break;
    case 2:
        for(unsigned n=0;n<7 && w->cursor<SIGMA_COUNT;n++,w->cursor++) {
            unsigned i=w->cursor;float dx[STATE_DIM];
            for(unsigned j=0;j<STATE_DIM;j++) dx[j]=u->sigma[i*STATE_DIM+j]-w->xpred[j];
            for(unsigned r=0;r<STATE_DIM;r++) for(unsigned c=0;c<STATE_DIM;c++)
                w->Ppred[r*STATE_DIM+c]+=u->w_c[i]*dx[r]*dx[c];
        }
        if (w->cursor==SIGMA_COUNT) w->phase++;
        break;
    case 3:
        mat_sym(w->Ppred,STATE_DIM);mat_copy(w->xpred,u->x,1,STATE_DIM);mat_copy(w->Ppred,u->P,STATE_DIM,STATE_DIM);
        if (!compute_sigma_points(u)) {w->active=false;return -1;}
        w->phase++;break;
    case 4: {
        for(unsigned i=0;i<SIGMA_COUNT;i++) meas_sigma(&u->sigma[i*STATE_DIM],&w->h[i*MEAS_DIM]);
        mat_zero(w->zpred,1,MEAS_DIM);
        for(unsigned i=0;i<SIGMA_COUNT;i++) for(unsigned j=0;j<MEAS_DIM;j++) w->zpred[j]+=u->w_m[i]*w->h[i*MEAS_DIM+j];
        mat_copy(u->R,w->S,MEAS_DIM,MEAS_DIM);
        float gn=sqrtf(w->gyro[0]*w->gyro[0]+w->gyro[1]*w->gyro[1]+w->gyro[2]*w->gyro[2]);
        float an=sqrtf(w->accel[0]*w->accel[0]+w->accel[1]*w->accel[1]+w->accel[2]*w->accel[2]);
        if (gn<UKF_QUIET_RATE_RPS && fabsf(an-1)<UKF_QUIET_ACC_G)
            for(unsigned i=3;i<6;i++) w->S[i*MEAS_DIM+i]+=UKF_R_ACCEL_QUIET-u->R[i*MEAS_DIM+i];
        mat_zero(w->Pxy,STATE_DIM,MEAS_DIM);w->cursor=0;w->phase++;break;
    }
    case 5:
        for(unsigned n=0;n<7 && w->cursor<SIGMA_COUNT;n++,w->cursor++) {
            unsigned i=w->cursor;float dx[STATE_DIM],dh[MEAS_DIM];
            for(unsigned j=0;j<STATE_DIM;j++) dx[j]=u->sigma[i*STATE_DIM+j]-u->x[j];
            for(unsigned j=0;j<MEAS_DIM;j++) dh[j]=w->h[i*MEAS_DIM+j]-w->zpred[j];
            for(unsigned r=0;r<MEAS_DIM;r++) for(unsigned c=0;c<MEAS_DIM;c++) w->S[r*MEAS_DIM+c]+=u->w_c[i]*dh[r]*dh[c];
            for(unsigned r=0;r<STATE_DIM;r++) for(unsigned c=0;c<MEAS_DIM;c++) w->Pxy[r*MEAS_DIM+c]+=u->w_c[i]*dx[r]*dh[c];
        }
        if(w->cursor==SIGMA_COUNT) w->phase++;
        break;
    case 6:
        for(unsigned j=0;j<3;j++) {w->v[j]=w->gyro[j]-w->zpred[j];w->v[j+3]=w->accel[j]-w->zpred[j+3];}
        if(mat_inv(w->S,w->Sinv,MEAS_DIM)!=0) return finish(u,w);
        w->phase++;break;
    case 7: {
        if(w->gate>0 || w->forceGyroOnly) {
            float Sa[9],Sai[9],va[3]={w->v[3],w->v[4],w->v[5]},vtemp[3];float mahal=1e30f;
            for(unsigned r=0;r<3;r++) for(unsigned c=0;c<3;c++) Sa[r*3+c]=w->S[(r+3)*MEAS_DIM+c+3];
            if(mat_inv(Sa,Sai,3)==0) {mat_mul(Sai,va,vtemp,3,3,1);mahal=va[0]*vtemp[0]+va[1]*vtemp[1]+va[2]*vtemp[2];}
            if(w->forceGyroOnly || (w->gate>0 && mahal>w->gate)) {
                w->accelRejected=true;
                float Sg[9],Sgi[9],Pg[STATE_DIM*3],Kg[STATE_DIM*3],Kv[STATE_DIM],KS[STATE_DIM*3],KSKt[STATE_DIM*STATE_DIM];
                for(unsigned r=0;r<3;r++) for(unsigned c=0;c<3;c++) Sg[r*3+c]=w->S[r*MEAS_DIM+c];
                if(mat_inv(Sg,Sgi,3)!=0) return finish(u,w);
                for(unsigned r=0;r<STATE_DIM;r++) for(unsigned c=0;c<3;c++) Pg[r*3+c]=w->Pxy[r*MEAS_DIM+c];
                mat_mul(Pg,Sgi,Kg,STATE_DIM,3,3);mat_mul(Kg,w->v,Kv,STATE_DIM,3,1);
                for(unsigned j=0;j<STATE_DIM;j++) u->x[j]+=Kv[j];
                quat_t q={u->x[0],u->x[1],u->x[2],u->x[3]};quat_norm(&q);quat_to_sigma(&q,u->x);
                mat_mul(Kg,Sg,KS,STATE_DIM,3,3);mat_mul_bT(KS,Kg,KSKt,STATE_DIM,3,STATE_DIM);
                mat_sub(u->P,KSKt,u->P,STATE_DIM,STATE_DIM);mat_sym(u->P,STATE_DIM);return finish(u,w);
            }
        }
        float Kv[STATE_DIM];mat_mul(w->Pxy,w->Sinv,w->K,STATE_DIM,MEAS_DIM,MEAS_DIM);
        mat_mul(w->K,w->v,Kv,STATE_DIM,MEAS_DIM,1);
        for(unsigned j=0;j<STATE_DIM;j++) u->x[j]+=Kv[j];
        quat_t q={u->x[0],u->x[1],u->x[2],u->x[3]};quat_norm(&q);quat_to_sigma(&q,u->x);
        w->phase++;break;
    }
    case 8:
        mat_mul(w->K,w->S,w->KS,STATE_DIM,MEAS_DIM,MEAS_DIM);
        mat_mul_bT(w->KS,w->K,w->KSKt,STATE_DIM,MEAS_DIM,STATE_DIM);
        mat_sub(u->P,w->KSKt,u->P,STATE_DIM,STATE_DIM);mat_sym(u->P,STATE_DIM);return finish(u,w);
    default:w->active=false;return -1;
    }
    return 0;
}
