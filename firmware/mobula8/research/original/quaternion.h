#ifndef MATH_QUATERNION_H
#define MATH_QUATERNION_H

typedef struct { float w, x, y, z; } quat_t;

void  quat_norm(quat_t *q);
void  quat_mul(const quat_t *a, const quat_t *b, quat_t *out);
void  quat_conj(const quat_t *q, quat_t *out);
void  quat_to_dcm(const quat_t *q, float R[9]);
void  quat_integrate(const quat_t *q, const float omega[3], float dt, quat_t *out);
float quat_norm_sq(const quat_t *q);

#endif
