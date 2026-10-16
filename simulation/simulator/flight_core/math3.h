#ifndef MATH3_H
#define MATH3_H

#include <math.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

typedef struct { double x, y, z; } Vec3;
typedef struct { double w, x, y, z; } Quat;

static inline Vec3 v3(double x, double y, double z) {
    Vec3 r = { x, y, z };
    return r;
}

static inline Vec3 v3_add(Vec3 a, Vec3 b)     { return v3(a.x + b.x, a.y + b.y, a.z + b.z); }
static inline Vec3 v3_sub(Vec3 a, Vec3 b)     { return v3(a.x - b.x, a.y - b.y, a.z - b.z); }
static inline Vec3 v3_scale(Vec3 a, double s) { return v3(a.x * s, a.y * s, a.z * s); }
static inline double v3_dot(Vec3 a, Vec3 b)   { return a.x * b.x + a.y * b.y + a.z * b.z; }
static inline double v3_norm(Vec3 a)          { return sqrt(v3_dot(a, a)); }

static inline Vec3 v3_cross(Vec3 a, Vec3 b) {
    return v3(a.y * b.z - a.z * b.y,
              a.z * b.x - a.x * b.z,
              a.x * b.y - a.y * b.x);
}

static inline Quat q_mul(Quat a, Quat b) {
    Quat r = {
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
    };
    return r;
}

static inline Quat q_normalize(Quat q) {
    double n = sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    Quat r = { q.w / n, q.x / n, q.y / n, q.z / n };
    return r;
}

/* body -> world */
static inline Vec3 q_rotate(Quat q, Vec3 v) {
    Vec3 u = v3(q.x, q.y, q.z);
    Vec3 t = v3_scale(v3_cross(u, v), 2.0);
    return v3_add(v, v3_add(v3_scale(t, q.w), v3_cross(u, t)));
}

/* world -> body */
static inline Vec3 q_rotate_inv(Quat q, Vec3 v) {
    Quat c = { q.w, -q.x, -q.y, -q.z };
    return q_rotate(c, v);
}

static inline double q_yaw(Quat q) {
    return atan2(2.0 * (q.w * q.z + q.x * q.y),
                 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

static inline Quat q_from_yaw(double yaw) {
    Quat r = { cos(0.5 * yaw), 0.0, 0.0, sin(0.5 * yaw) };
    return r;
}

static inline double wrap_pi(double a) {
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

static inline double clamp(double v, double lo, double hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

#endif
