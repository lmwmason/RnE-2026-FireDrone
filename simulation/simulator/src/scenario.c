#include "scenario.h"
#include <string.h>

#define N(a) ((int)(sizeof(a) / sizeof((a)[0])))

static const Event EV_NOMINAL[] = { { 0.0, EV_WIND_MEAN, { 0, 0, 0, 0 } } };

static const Event EV_GUSTY[] = {
    {  5.0, EV_WIND_MEAN,  { 3.0,  1.5, 0.0, 0.0 } },
    {  9.0, EV_GUST,       { 7.0, -3.0, 1.5, 1.6 } },
    { 15.0, EV_GUST,       {-8.0,  4.0,-2.0, 1.2 } },
    { 21.0, EV_GUST,       { 5.0,  6.0, 1.0, 2.0 } },
};

static const Event EV_ARM[] = {
    {  8.0, EV_ARM_TORQUE, { 1, 0.06, 1.5, 12.0 } },
    { 21.0, EV_ARM_TORQUE, { 0, 0.15, 0.0,  1.0 } },
    { 24.0, EV_ARM_TORQUE, { 1,-0.20, 0.0,  0.8 } },
};

static const Event EV_PROP[] = {
    { 12.0, EV_MOTOR_EFF,  { 1, 0.65, 0, 0 } },
};

static const Event EV_MOTORDEAD[] = {
    { 12.0, EV_MOTOR_EFF,  { 1, 0.0, 0, 0 } },
};

static const Event EV_SENSOR[] = {
    {  8.0, EV_ACCEL_FREEZE, { 3.0, 0, 0, 0 } },
    { 13.0, EV_GYRO_BIAS,    { 1, 6.0, 4.0, 0 } },
    { 19.0, EV_ACCEL_SPIKES, { 3.0, 0.15, 8.0, 0 } },
};

static const Event EV_RCLOSS[] = {
    { 10.0, EV_RC_LOSS,    { 0.2, 0, 0, 0 } },
    { 17.0, EV_RC_LOSS,    { 1.0, 0, 0, 0 } },
};

static const Event EV_RCLOST[] = {
    { 12.0, EV_RC_LOSS,    { 1e18, 0, 0, 0 } },
};

static const Event EV_EXTREME[] = {
    {  5.0, EV_WIND_MEAN,  { 3.0, -1.0, 0.0, 0.0 } },
    {  8.0, EV_ARM_TORQUE, { 1, 0.06, 1.5, 14.0 } },
    { 12.0, EV_GUST,       { 7.0,  3.0, 1.0, 1.5 } },
    { 14.0, EV_MOTOR_EFF,  { 2, 0.70, 0, 0 } },
    { 17.0, EV_ACCEL_SPIKES, { 3.0, 0.10, 6.0, 0 } },
    { 22.0, EV_GUST,       {-7.0, -4.0,-1.5, 1.3 } },
};

/* Transient faults first; permanent prop damage is last so it cannot
 * contaminate the isolated sensor/link trials. No in-flight repair/reset. */
static const Event EV_GAUNTLET[] = {
    {  7.0, EV_WIND_MEAN,    { 3.0, 1.5, 0, 0 } },
    {  9.0, EV_GUST,         { 7.0,-3.0, 1.5, 1.6 } },
    { 13.0, EV_WIND_MEAN,    { 0, 0, 0, 0 } },
    { 16.0, EV_ARM_TORQUE,   { 1, 0.06, 1.5, 6.0 } },
    { 24.0, EV_ARM_TORQUE,   { 0, 0.15, 0, 1.0 } },
    { 28.0, EV_ACCEL_FREEZE, { 3.0, 0, 0, 0 } },
    { 34.0, EV_GYRO_BIAS,    { 1, 6.0, 4.0, 0 } },
    { 41.0, EV_ACCEL_SPIKES, { 3.0, 0.15, 8.0, 0 } },
    { 47.0, EV_RC_LOSS,      { 0.2, 0, 0, 0 } },
    { 50.0, EV_RC_LOSS,      { 1.0, 0, 0, 0 } },
    { 54.0, EV_MOTOR_EFF,    { 1, 0.65, 0, 0 } },
    /* All nonterminal fault types start on the same control tick. */
    { 60.0, EV_WIND_MEAN,    { 4.0,-2.0, 0, 0 } },
    { 60.0, EV_GUST,         { 8.0, 4.0, 1.5, 1.5 } },
    { 60.0, EV_ARM_TORQUE,   { 1, 0.08, 1.5, 12.0 } },
    { 60.0, EV_MOTOR_EFF,    { 3, 0.75, 0, 0 } },
    { 60.0, EV_ACCEL_FREEZE, { 2.0, 0, 0, 0 } },
    { 60.0, EV_GYRO_BIAS,    { 0, 5.0, 6.0, 0 } },
    { 60.0, EV_ACCEL_SPIKES, { 6.0, 0.10, 6.0, 0 } },
    { 60.0, EV_RC_LOSS,      { 0.5, 0, 0, 0 } },
    { 72.0, EV_WIND_MEAN,    { 0, 0, 0, 0 } },
};

/* Destructive finale: distinct motors preserve both partial and complete loss.
 * Persistent link loss is longer than the simulation, with no forced recovery. */
static const Event EV_ALL_FAILURES[] = {
    { 12.0, EV_GPS_LOSS,      { 6.0, 0, 0, 0 } },
    { 12.0, EV_IMU_LOSS,      { 3.0, 0, 0, 0 } },
    { 12.0, EV_WIND_MEAN,    { 4.0,-2.0, 0, 0 } },
    { 12.0, EV_GUST,         { 8.0, 4.0, 1.5, 1.5 } },
    { 12.0, EV_ARM_TORQUE,   { 1, 0.08, 1.5, 12.0 } },
    { 12.0, EV_MOTOR_EFF,    { 1, 0.0, 0, 0 } },
    { 12.0, EV_MOTOR_EFF,    { 3, 0.65, 0, 0 } },
    { 12.0, EV_ACCEL_FREEZE, { 2.0, 0, 0, 0 } },
    { 12.0, EV_GYRO_BIAS,    { 0, 6.0, 6.0, 0 } },
    { 12.0, EV_ACCEL_SPIKES, { 6.0, 0.15, 8.0, 0 } },
    { 12.0, EV_RC_LOSS,      { 1e18, 0, 0, 0 } },
};
static const Event EV_FREEZE[] = {{ 12.0, EV_ACCEL_FREEZE, {3.0, 0, 0, 0} }};
static const Event EV_BIAS[] = {{ 12.0, EV_GYRO_BIAS, {1, 6.0, 4.0, 0} }};
static const Event EV_SPIKES[] = {{ 12.0, EV_ACCEL_SPIKES, {3.0, 0.15, 8.0, 0} }};

static const Event EV_GPS[] = {{12.0, EV_GPS_LOSS, {3.0,0,0,0}}};
static const Event EV_IMU[] = {{12.0, EV_IMU_LOSS, {3.0,0,0,0}}};
static const Scenario SCENARIOS[] = {
    { "nominal",      "calm air, healthy airframe",                              0.2, EV_NOMINAL,   N(EV_NOMINAL), 1 },
    { "gust",         "3 m/s mean wind + 8 m/s gusts + turbulence",              0.9, EV_GUSTY,     N(EV_GUSTY), 1 },
    { "arm_swing",    "robot-arm reaction torque (sinusoid + steps)",            0.2, EV_ARM,       N(EV_ARM), 1 },
    { "prop_damage",  "propeller on M2 loses 35 % thrust mid-flight",            0.2, EV_PROP,      N(EV_PROP), 1 },
    { "motor_fail",   "M2 stops completely mid-flight",                          0.2, EV_MOTORDEAD, N(EV_MOTORDEAD), 1 },
    { "sensor_fault", "accel freeze, gyro bias jump, accel shock spikes",        0.2, EV_SENSOR,    N(EV_SENSOR), 1 },
    { "rc_loss",      "RC link drops for 0.2 s and again for 1.0 s",             0.2, EV_RCLOSS,    N(EV_RCLOSS), 1 },
    { "rc_lost_land", "RC link lost for good at 12 s: firmware must land alone",  0.2, EV_RCLOST,    N(EV_RCLOST), 1 },
    { "extreme",      "wind + arm swing + damaged prop + sensor shocks",         0.9, EV_EXTREME,   N(EV_EXTREME), 1 },
    { "accel_freeze", "isolated accelerometer sample hold", 0.2, EV_FREEZE, N(EV_FREEZE), 1 },
    { "gyro_bias", "isolated gyro bias jump", 0.2, EV_BIAS, N(EV_BIAS), 1 },
    { "accel_spikes", "isolated accelerometer shocks", 0.2, EV_SPIKES, N(EV_SPIKES), 1 },
    { "gps_loss", "complete navigation source outage", 0.2, EV_GPS, N(EV_GPS), 1 },
    { "imu_loss", "complete IMU transport outage (no redundant IMU assumed)", 0.2, EV_IMU, N(EV_IMU), 1 },
    { "all_failures", "ALL faults simultaneously, including dead motor and persistent RC loss", 0.9, EV_ALL_FAILURES, N(EV_ALL_FAILURES), 8 },
    { "gauntlet",     "hazards one by one, then all together over 8 laps",       0.2, EV_GAUNTLET,  N(EV_GAUNTLET), 8 },
};

int scenario_count(void) { return N(SCENARIOS); }
const Scenario *scenario_at(int i) { return &SCENARIOS[i]; }

const Scenario *scenario_find(const char *name) {
    for (int i = 0; i < N(SCENARIOS); i++)
        if (strcmp(SCENARIOS[i].name, name) == 0) return &SCENARIOS[i];
    return NULL;
}

void world_init(World *w, Rng *rng, const Scenario *sc) {
    memset(w, 0, sizeof(*w));
    for (int i = 0; i < 8; i++) w->phys.eff[i] = 1.0;
    wind_init(&w->wind);
    w->wind.sigma = sc->turbulence;
    imu_model_init(&w->imu, rng);
    nav_model_init(&w->nav, rng);
    w->rc_link_up = 1;
    w->nav_valid = w->imu_valid = 1;
}

static void fire(World *w, const Event *e, double t) {
    switch (e->type) {
    case EV_WIND_MEAN:
        w->wind.mean = v3(e->a[0], e->a[1], e->a[2]);
        break;
    case EV_GUST:
        wind_trigger_gust(&w->wind, t, e->a[3], v3(e->a[0], e->a[1], e->a[2]));
        break;
    case EV_ARM_TORQUE:
        w->arm.on   = 1;
        w->arm.axis = (int)e->a[0];
        w->arm.amp  = e->a[1];
        w->arm.freq = e->a[2];
        w->arm.t0   = t;
        w->arm.t1   = t + e->a[3];
        break;
    case EV_MOTOR_EFF:
        w->phys.eff[(int)e->a[0] & 7] = e->a[1];
        break;
    case EV_GYRO_BIAS: {
        int ax = (int)e->a[0] % 3;
        w->imu.gyro_fault_dps[ax] = e->a[1];
        w->gyro_end[ax] = e->a[2] > 0.0 ? t + e->a[2] : 1e18;
        break;
    }
    case EV_ACCEL_FREEZE:
        w->imu.accel_frozen = 1;
        w->freeze_end = t + e->a[0];
        break;
    case EV_ACCEL_SPIKES:
        w->imu.spike_prob = e->a[1];
        w->imu.spike_g    = e->a[2];
        w->spike_end      = t + e->a[0];
        break;
    case EV_GPS_LOSS:
        w->nav_valid = 0; w->nav_end = t + e->a[0]; break;
    case EV_IMU_LOSS:
        w->imu_valid = 0; w->imu_end = t + e->a[0]; break;
    case EV_RC_LOSS:
        w->rc_link_up = 0;
        w->rc_end     = t + e->a[0];
        break;
    }
}

void world_update(World *w, const Scenario *sc, double t, double dt, Rng *rng) {
    for (int i = 0; i < sc->n_events; i++) {
        if (!w->fired[i] && t >= sc->events[i].t) {
            w->fired[i] = 1;
            fire(w, &sc->events[i], t);
        }
    }

    if (w->imu.accel_frozen && t >= w->freeze_end) w->imu.accel_frozen = 0;
    if (w->imu.spike_prob > 0.0 && t >= w->spike_end) w->imu.spike_prob = 0.0;
    if (!w->rc_link_up && t >= w->rc_end) w->rc_link_up = 1;
    for (int i = 0; i < 3; i++)
        if (w->imu.gyro_fault_dps[i] != 0.0 && t >= w->gyro_end[i])
            w->imu.gyro_fault_dps[i] = 0.0;

    if (!w->nav_valid && t >= w->nav_end) w->nav_valid = 1;
    if (!w->imu_valid && t >= w->imu_end) w->imu_valid = 1;
    wind_step(&w->wind, rng, dt);
    w->phys.wind = wind_at(&w->wind, t);

    w->phys.ext_torque = v3(0.0, 0.0, 0.0);
    if (w->arm.on) {
        if (t >= w->arm.t1) {
            w->arm.on = 0;
        } else {
            double s = (w->arm.freq > 0.0) ? sin(2.0 * M_PI * w->arm.freq * (t - w->arm.t0)) : 1.0;
            double tq = w->arm.amp * s;
            if (w->arm.axis == 0) w->phys.ext_torque.x = tq;
            else                  w->phys.ext_torque.y = tq;
        }
    }
}
