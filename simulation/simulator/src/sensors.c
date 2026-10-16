#include "sensors.h"
#include <string.h>

#define GYRO_LSB_PER_DPS 16.4
#define ACCEL_LSB_PER_G  2048.0

void imu_model_init(ImuModel *m, Rng *rng) {
    memset(m, 0, sizeof(*m));
    m->rng            = rng;
    m->gyro_sigma_dps = 0.12;
    m->accel_sigma_g  = 0.008;
    m->gyro_walk_dps  = 0.003;
    for (int i = 0; i < 3; i++) {
        m->gyro_bias_dps[i] = 0.5 * rng_normal(rng);
        m->accel_bias_g[i]  = 0.01 * rng_normal(rng);
    }
}

static double quantize(double v, double lsb) {
    return round(v * lsb) / lsb;
}

void imu_model_sample(ImuModel *m, const DroneState *s, Vec3 f_spec_body,
                      double mean_cmd, double dt, ImuSample *out) {
    for (int i = 0; i < 3; i++)
        m->gyro_bias_dps[i] += m->gyro_walk_dps * sqrt(dt) * rng_normal(m->rng);

    double w[3] = { s->w.y, -s->w.x, s->w.z };
    double f[3] = { f_spec_body.y, -f_spec_body.x, f_spec_body.z };

    double vib = 0.002 + 0.04 * mean_cmd;           /* motor vibration [g] */
    for (int i = 0; i < 3; i++) {
        double g_dps = w[i] * 180.0 / M_PI + m->gyro_bias_dps[i] + m->gyro_fault_dps[i]
                     + m->gyro_sigma_dps * rng_normal(m->rng);
        double a_g   = f[i] / DRONE_PARAMS.g + m->accel_bias_g[i]
                     + sqrt(m->accel_sigma_g * m->accel_sigma_g + vib * vib) * rng_normal(m->rng);

        if (m->accel_frozen)
            a_g = m->accel_hold[i];
        if (m->spike_prob > 0.0 && rng_uniform(m->rng) < m->spike_prob)
            a_g += m->spike_g * (rng_uniform(m->rng) < 0.5 ? -1.0 : 1.0);

        out->gyro_dps[i] = (float)clamp(quantize(g_dps, GYRO_LSB_PER_DPS), -2000.0, 2000.0);
        out->accel_g[i]  = (float)clamp(quantize(a_g, ACCEL_LSB_PER_G), -16.0, 16.0);
        if (!m->accel_frozen) m->accel_hold[i] = out->accel_g[i];
    }
}

void nav_model_init(NavModel *n, Rng *rng) {
    memset(n, 0, sizeof(*n));
    n->rng       = rng;
    n->pos_sigma = 0.03;
    n->vel_sigma = 0.03;
    n->period    = 0.02;
}

void nav_model_update(NavModel *n, const DroneState *s, double t) {
    if (t < n->next_t) return;
    n->next_t = t + n->period;
    n->pos = v3(s->pos.x + n->pos_sigma * rng_normal(n->rng),
                s->pos.y + n->pos_sigma * rng_normal(n->rng),
                s->pos.z + n->pos_sigma * rng_normal(n->rng));
    n->vel = v3(s->vel.x + n->vel_sigma * rng_normal(n->rng),
                s->vel.y + n->vel_sigma * rng_normal(n->rng),
                s->vel.z + n->vel_sigma * rng_normal(n->rng));
}
