#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

#include "math3.h"
#include "rng.h"

/* Mean wind + Ornstein-Uhlenbeck turbulence + scripted (1-cos) gusts. */
typedef struct {
    Vec3   mean;
    double sigma;       /* turbulence intensity [m/s] */
    double tau;         /* turbulence correlation time [s] */
    Vec3   turb;
    Vec3   gust_amp;
    double gust_t0, gust_dur;
} Wind;

void wind_init(Wind *w);
void wind_trigger_gust(Wind *w, double t0, double dur, Vec3 amp);
void wind_step(Wind *w, Rng *rng, double dt);
Vec3 wind_at(const Wind *w, double t);

#endif
