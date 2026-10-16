#ifndef MIXER_H
#define MIXER_H

/*
 * Standard Betaflight/KISS Quad-X motor order:
 *   M1 = rear-right (CCW), M2 = front-right (CW),
 *   M3 = rear-left (CW),   M4 = front-left (CCW)
 * matching the M1-M4 silkscreen pads on the Reaper 55A stack.
 */
typedef struct {
    float m1, m2, m3, m4; /* 0.0 .. 1.0 */
} mixer_out_t;

void Mixer_QuadX(float throttle, float roll, float pitch, float yaw, mixer_out_t *out);

#endif /* MIXER_H */
