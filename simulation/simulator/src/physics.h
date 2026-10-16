#ifndef PHYSICS_H
#define PHYSICS_H

#include "math3.h"

/* World: ENU (z up).  Body: FLU (x forward, y left, z up).
 * Motor order follows the firmware: M1 rear-right CCW, M2 front-right CW,
 * M3 rear-left CW, M4 front-left CCW. */

typedef struct {
    Vec3   pos, vel;
    Quat   q;           /* body -> world */
    Vec3   w;           /* body rates [rad/s] */
    double s[8];        /* motor thrust state, 0..1 */
} DroneState;

typedef struct {
    int motors;         /* 4 legacy, 8 coaxial X8 */
    double rotor_x[8], rotor_y[8];
    int spin[8];
    double rotor_gain[8]; /* calibrated effective thrust/torque factors */
    double arm_mass, arm_length; /* reference moving link, included in total mass */
    double mass;
    double Ixx, Iyy, Izz;
    double g;
    double arm;         /* centre -> motor [m] */
    double k_thrust;    /* thrust per unit command [N] */
    double k_drag;      /* reaction torque per unit command [Nm] */
    double motor_tau;   /* first-order motor lag [s] */
    double drag_lin;
    double drag_quad;
    double cp_height;   /* centre of pressure above CoG [m] */
} DroneParams;

typedef struct {
    double eff[8];      /* 1.0 healthy, <1 damaged prop, 0 dead motor */
    Vec3   wind;        /* world wind velocity [m/s] */
    Vec3   ext_torque;  /* body disturbance torque [Nm] (arm reaction) */
} PhysInputs;

extern const DroneParams DRONE_PARAMS;
extern const int MOTOR_SPIN[4];

void physics_init(DroneState *s);
void physics_step(DroneState *s, const double cmd[8], const DroneParams *dp,
                  const PhysInputs *in, double dt, Vec3 *f_spec_body);
double physics_total_thrust(const DroneState *s, const DroneParams *dp,
                            const PhysInputs *in);

#endif
