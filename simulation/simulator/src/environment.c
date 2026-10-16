#include "environment.h"
#include <string.h>

void wind_init(Wind *w) {
    memset(w, 0, sizeof(*w));
    w->tau      = 1.5;
    w->gust_dur = 1.0;
    w->gust_t0  = -1e9;
}

void wind_trigger_gust(Wind *w, double t0, double dur, Vec3 amp) {
    w->gust_t0  = t0;
    w->gust_dur = dur;
    w->gust_amp = amp;
}

void wind_step(Wind *w, Rng *rng, double dt) {
    double a = exp(-dt / w->tau);
    double b = w->sigma * sqrt(1.0 - a * a);
    w->turb.x = a * w->turb.x + b * rng_normal(rng);
    w->turb.y = a * w->turb.y + b * rng_normal(rng);
    w->turb.z = a * w->turb.z + 0.5 * b * rng_normal(rng);
}

Vec3 wind_at(const Wind *w, double t) {
    Vec3 out = v3_add(w->mean, w->turb);
    double u = (t - w->gust_t0) / w->gust_dur;
    if (u > 0.0 && u < 1.0)
        out = v3_add(out, v3_scale(w->gust_amp, 0.5 * (1.0 - cos(2.0 * M_PI * u))));
    return out;
}
