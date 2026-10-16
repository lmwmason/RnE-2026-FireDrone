#ifndef PILOT_H
#define PILOT_H

#include <stdint.h>
#include "math3.h"

/* Outer-loop autopilot.  The firmware only closes the *rate* loop, so this
 * plays the role of the companion computer: it turns a waypoint mission into
 * stick positions (roll/pitch/yaw rate, throttle, arm switch) that are sent
 * over CRSF exactly like an RC link. */

typedef enum { PH_PREARM, PH_ARM, PH_TAKEOFF, PH_MISSION, PH_LAND, PH_DONE } PilotPhase;
typedef enum { ATT_TRUTH, ATT_UKF } AttSource;

typedef struct {
    PilotPhase phase;
    AttSource  att_source;
    double     t_phase;
    int        wp;
    int        lap, laps;   /* circuits flown / requested */
    double     mission_not_before; /* do not land before the final scheduled event */
    double     mass, total_thrust;
    double     hold;
    Vec3       ref_pos, ref_vel;
    double     ref_yaw;
    Vec3       integ;
    uint16_t   rc[5];       /* roll, pitch, throttle, yaw, aux1 [us] */
} Pilot;

extern const Vec3 MISSION[];
extern const int  MISSION_LEN;
extern const char *PHASE_NAMES[];

void pilot_init(Pilot *p, AttSource src);
void pilot_update(Pilot *p, double t, double dt, Vec3 nav_pos, Vec3 nav_vel, Quat att);

#endif
