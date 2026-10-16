#include "rng.h"
#include <math.h>

void rng_seed(Rng *r, uint64_t seed) {
    r->s = seed ? seed : 0x9E3779B97F4A7C15ULL;
}

static uint64_t next_u64(Rng *r) {
    uint64_t x = r->s;
    x ^= x << 13;
    x ^= x >> 7;
    x ^= x << 17;
    r->s = x;
    return x;
}

double rng_uniform(Rng *r) {
    return (double)(next_u64(r) >> 11) * (1.0 / 9007199254740992.0);
}

double rng_normal(Rng *r) {
    double u1 = rng_uniform(r);
    double u2 = rng_uniform(r);
    if (u1 < 1e-12) u1 = 1e-12;
    return sqrt(-2.0 * log(u1)) * cos(2.0 * M_PI * u2);
}
