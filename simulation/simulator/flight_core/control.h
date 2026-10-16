#ifndef FC_CONTROL_H
#define FC_CONTROL_H
#include "allocator.h"
#include "mission.h"
#include "ukf.h"
#include "adrc.h"
#include "mrac.h"

typedef struct {
    float inertia[3], motor_tau, mass;
    FcMotor motor[8];
} FcConfig;
/* The board adapter supplies timestamped measurements in body FLU/SI units.
 * This interface contains no true pose, true disturbance, or injected faults. */
typedef struct {
    enum { FC_MANUAL = 0, FC_MISSION = 1 } mode;
    uint16_t rc[5]; /* roll, pitch, throttle, yaw, arm: microseconds */
    uint32_t rc_ms;
    float gyro[3], accel[3]; /* rad/s, g */
    bool imu_valid;
    Vec3 nav_pos, nav_vel;
    bool nav_valid;
    uint32_t nav_ms;
    float rpm[8];
    bool rpm_valid[8];
    float arm_torque[3];    /* model/telemetry estimate, Nm, NOT actual plant torque */
    uint32_t arm_ms;
    bool arm_valid;
    bool link_up;
    bool mission_authorized; /* explicit start authorization, latched only in healthy prearm */
    float baro_alt;
    uint32_t baro_ms;
    bool baro_valid;
} FcInput;
typedef struct {
    FcConfig config;
    bool initialized;
    FcAllocator allocator;
    FcAllocation allocation;
    ukf_t ukf;
    adrc_t adrc;
    mrac_t mrac;
    Pilot mission;
    float omega_prev[3], omega_f[3], alpha[3], u_lag[8];
    float achieved_alpha[3], ff_previous[3];
    unsigned dead_ticks[8], imu_missing;
    uint8_t failed_mask;
    bool nav_degraded, imu_degraded;
    bool mission_authorized;
    bool manual_armed, arm_requested, arm_ready, failsafe;
    float altitude, vertical_speed;
    uint32_t last_baro_ms, ground_since;
    unsigned ticks;
} FcControl;
bool fc_init(FcControl *c, const FcConfig *config, unsigned laps);
/* Called at 1 kHz on BOTH the simulator and MCU. Default mode: MANUAL ANGLE.
 * The explicit MISSION mode is reserved for offline regression experiments.
 * A false return requires the caller to latch control-unavailable status. */
bool fc_step(FcControl *c, const FcInput *in, uint32_t now_ms);
#endif
