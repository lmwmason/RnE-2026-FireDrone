#ifndef LIVE_SIM_H
#define LIVE_SIM_H
#include <stdint.h>
typedef struct LiveSim LiveSim;
typedef struct {
    double t, pos[3], q[4], vel[3], cmd[8], eff[8], wind[3];
    double arm_angle, arm_target, arm_torque, impact;
    int armed, crashed, failed_mask, events, hazard, cycles, control_lost;
} LiveView;
LiveSim *live_create(const char *config, unsigned seed);
void live_destroy(LiveSim *sim);
/* USB channels already calibrated into 1000..2000 us, roll/pitch/yaw centered.
 * Advance an exact number of 1-ms ticks; the UI never changes integration dt. */
void live_step(LiveSim *sim, const uint16_t rc[5], int connected, int hazards, unsigned ticks);
void live_view(const LiveSim *sim, LiveView *out);
#endif
