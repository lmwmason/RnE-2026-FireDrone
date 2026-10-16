#ifndef SCENARIO_H
#define SCENARIO_H

#include "physics.h"
#include "environment.h"
#include "sensors.h"

#define SCENARIO_MAX_EVENTS 128

typedef enum {
    EV_WIND_MEAN,       /* a0..a2 mean wind [m/s] */
    EV_GUST,            /* a0..a2 peak gust [m/s], a3 duration [s] */
    EV_ARM_TORQUE,      /* a0 axis(0 x,1 y), a1 amp [Nm], a2 freq [Hz] (0 = step), a3 duration */
    EV_MOTOR_EFF,       /* a0 motor index, a1 efficiency (0 = dead) */
    EV_GYRO_BIAS,       /* a0 raw gyro axis, a1 offset [dps], a2 duration (0 = permanent) */
    EV_ACCEL_FREEZE,    /* a0 duration [s] */
    EV_ACCEL_SPIKES,    /* a0 duration [s], a1 probability per sample, a2 magnitude [g] */
    EV_GPS_LOSS,        /* a0 duration; navigation data unavailable */
    EV_IMU_LOSS,        /* a0 duration; complete IMU transport outage */
    EV_RC_LOSS,         /* a0 duration [s] */
} EventType;

typedef struct {
    double    t;
    EventType type;
    double    a[4];
} Event;

typedef struct {
    const char  *name;
    const char  *desc;
    double       turbulence;    /* background OU sigma [m/s] */
    const Event *events;
    int          n_events;
    int          laps;          /* circuits of the mission loop (0 = one pass) */
} Scenario;

typedef struct {
    PhysInputs phys;
    Wind       wind;
    ImuModel   imu;
    NavModel   nav;
    int        rc_link_up;
    int        nav_valid, imu_valid;
    double     nav_end, imu_end;

    int        fired[SCENARIO_MAX_EVENTS];
    struct { int on; int axis; double amp, freq, t0, t1; } arm;
    double     gyro_end[3];
    double     freeze_end, spike_end, rc_end;
} World;

int             scenario_count(void);
const Scenario *scenario_at(int i);
const Scenario *scenario_find(const char *name);

void world_init(World *w, Rng *rng, const Scenario *sc);
void world_update(World *w, const Scenario *sc, double t, double dt, Rng *rng);

#endif
