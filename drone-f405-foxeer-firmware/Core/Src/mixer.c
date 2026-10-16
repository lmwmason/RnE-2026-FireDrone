#include "mixer.h"

static float Clamp01(float v)
{
    if (v < 0.0f) return 0.0f;
    if (v > 1.0f) return 1.0f;
    return v;
}

void Mixer_QuadX(float throttle, float roll, float pitch, float yaw, mixer_out_t *out)
{
    out->m1 = Clamp01(throttle - roll + pitch - yaw);
    out->m2 = Clamp01(throttle - roll - pitch + yaw);
    out->m3 = Clamp01(throttle + roll + pitch + yaw);
    out->m4 = Clamp01(throttle + roll - pitch - yaw);
}
