#include "quaternion.h"
#include <math.h>

float quat_norm_sq(const quat_t *q)
{
    return q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z;
}

void quat_norm(quat_t *q)
{
    float inv_n = 1.0f / sqrtf(quat_norm_sq(q));
    q->w *= inv_n;
    q->x *= inv_n;
    q->y *= inv_n;
    q->z *= inv_n;
}

void quat_mul(const quat_t *a, const quat_t *b, quat_t *out)
{
    out->w = a->w * b->w - a->x * b->x - a->y * b->y - a->z * b->z;
    out->x = a->w * b->x + a->x * b->w + a->y * b->z - a->z * b->y;
    out->y = a->w * b->y - a->x * b->z + a->y * b->w + a->z * b->x;
    out->z = a->w * b->z + a->x * b->y - a->y * b->x + a->z * b->w;
}

void quat_conj(const quat_t *q, quat_t *out)
{
    out->w =  q->w;
    out->x = -q->x;
    out->y = -q->y;
    out->z = -q->z;
}

void quat_to_dcm(const quat_t *q, float R[9])
{
    float w = q->w, x = q->x, y = q->y, z = q->z;
    float xx = x * x, yy = y * y, zz = z * z;
    float xy = x * y, xz = x * z, yz = y * z;
    float wx = w * x, wy = w * y, wz = w * z;

    R[0] = 1.0f - 2.0f * (yy + zz);
    R[1] = 2.0f * (xy - wz);
    R[2] = 2.0f * (xz + wy);

    R[3] = 2.0f * (xy + wz);
    R[4] = 1.0f - 2.0f * (xx + zz);
    R[5] = 2.0f * (yz - wx);

    R[6] = 2.0f * (xz - wy);
    R[7] = 2.0f * (yz + wx);
    R[8] = 1.0f - 2.0f * (xx + yy);
}

void quat_integrate(const quat_t *q, const float omega[3], float dt, quat_t *out)
{
    float half_dt = 0.5f * dt;
    float ox = omega[0] * half_dt;
    float oy = omega[1] * half_dt;
    float oz = omega[2] * half_dt;

    float angle_sq = ox * ox + oy * oy + oz * oz;
    float dw, ds;

    if (angle_sq > 1e-8f) {
        float angle = sqrtf(angle_sq);
        float sinc  = sinf(angle) / angle;
        dw = cosf(angle);
        ds = sinc;
    } else {
        dw = 1.0f - 0.5f * angle_sq;
        ds = 1.0f - angle_sq / 6.0f;
    }

    quat_t dq = { dw, ox * ds, oy * ds, oz * ds };
    quat_mul(q, &dq, out);
    quat_norm(out);
}
