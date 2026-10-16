#include "physics.h"
#include <string.h>

/* "True" airframe.  Deliberately a few percent away from the constants the
 * firmware assumes (K_THRUST 5.0, K_DRAG 0.02, I = 0.003/0.003/0.005). */
const DroneParams DRONE_PARAMS = {
    .motors = 4,
    .rotor_x = {-0.0565685424949238, 0.0565685424949238, -0.0565685424949238, 0.0565685424949238},
    .rotor_y = {-0.0565685424949238, -0.0565685424949238, 0.0565685424949238, 0.0565685424949238},
    .spin = {1, -1, -1, 1},
    .rotor_gain = {1, 1, 1, 1},
    .mass      = 0.45,
    .Ixx       = 0.0033,
    .Iyy       = 0.0033,
    .Izz       = 0.0052,
    .g         = 9.80665,
    .arm       = 0.080,
    .k_thrust  = 4.6,
    .k_drag    = 0.022,
    .motor_tau = 0.025,
    .drag_lin  = 0.10,
    .drag_quad = 0.03,
    .cp_height = 0.04,
};

const int MOTOR_SPIN[4] = { +1, -1, -1, +1 };   /* +1 = CCW */


void physics_init(DroneState *s) {
    memset(s, 0, sizeof(*s));
    s->q.w = 1.0;
}

double physics_total_thrust(const DroneState *s, const DroneParams *dp,
                            const PhysInputs *in) {
    double T = 0.0;
    for (int i = 0; i < dp->motors; i++) T += dp->k_thrust * dp->rotor_gain[i] * in->eff[i] * s->s[i];
    return T;
}

static void deriv(const DroneState *s, const double cmd[8], const DroneParams *dp,
                  const PhysInputs *in, DroneState *d, Vec3 *f_spec) {
    double T[8], Tsum = 0.0, Qz = 0.0;
    Vec3   tau = in->ext_torque;

    for (int i = 0; i < dp->motors; i++) {
        T[i]  = dp->k_thrust * dp->rotor_gain[i] * in->eff[i] * s->s[i];
        Tsum += T[i];
        tau.x += dp->rotor_y[i] * T[i];
        tau.y += -dp->rotor_x[i] * T[i];
        Qz    += -dp->spin[i] * dp->k_drag * dp->rotor_gain[i] * in->eff[i] * s->s[i];
        d->s[i] = (cmd[i] - s->s[i]) / dp->motor_tau;
    }
    tau.z += Qz;

    Vec3 v_rel  = v3_sub(s->vel, in->wind);
    double vmag = v3_norm(v_rel);
    Vec3 F_drag = v3_scale(v_rel, -(dp->drag_lin + dp->drag_quad * vmag));
    Vec3 F_drag_b = q_rotate_inv(s->q, F_drag);

    /* drag acts at the centre of pressure above the CoG -> pitching/rolling moment */
    tau = v3_add(tau, v3_cross(v3(0.0, 0.0, dp->cp_height), F_drag_b));

    Vec3 F_thrust_w = q_rotate(s->q, v3(0.0, 0.0, Tsum));
    Vec3 F = v3_add(F_thrust_w, F_drag);
    F.z -= dp->mass * dp->g;

    d->pos = s->vel;
    d->vel = v3_scale(F, 1.0 / dp->mass);

    Quat wq = { 0.0, s->w.x, s->w.y, s->w.z };
    Quat qd = q_mul(s->q, wq);
    d->q = (Quat){ 0.5 * qd.w, 0.5 * qd.x, 0.5 * qd.y, 0.5 * qd.z };

    Vec3 Iw = v3(dp->Ixx * s->w.x, dp->Iyy * s->w.y, dp->Izz * s->w.z);
    Vec3 gyro = v3_cross(s->w, Iw);
    d->w = v3((tau.x - gyro.x) / dp->Ixx,
              (tau.y - gyro.y) / dp->Iyy,
              (tau.z - gyro.z) / dp->Izz);

    if (f_spec) *f_spec = v3_scale(v3_add(v3(0.0, 0.0, Tsum), F_drag_b), 1.0 / dp->mass);
}

static DroneState axpy(const DroneState *s, const DroneState *k, double h) {
    DroneState r = *s;
    r.pos = v3_add(s->pos, v3_scale(k->pos, h));
    r.vel = v3_add(s->vel, v3_scale(k->vel, h));
    r.w   = v3_add(s->w,   v3_scale(k->w,   h));
    r.q.w = s->q.w + h * k->q.w;
    r.q.x = s->q.x + h * k->q.x;
    r.q.y = s->q.y + h * k->q.y;
    r.q.z = s->q.z + h * k->q.z;
    for (int i = 0; i < 8; i++) r.s[i] = s->s[i] + h * k->s[i];
    return r;
}

void physics_step(DroneState *s, const double cmd[8], const DroneParams *dp,
                  const PhysInputs *in, double dt, Vec3 *f_spec_body) {
    DroneState k1 = {0}, k2 = {0}, k3 = {0}, k4 = {0}, t;
    Vec3 f1;

    deriv(s, cmd, dp, in, &k1, &f1);
    t = axpy(s, &k1, 0.5 * dt); deriv(&t, cmd, dp, in, &k2, NULL);
    t = axpy(s, &k2, 0.5 * dt); deriv(&t, cmd, dp, in, &k3, NULL);
    t = axpy(s, &k3, dt);       deriv(&t, cmd, dp, in, &k4, NULL);

    double h = dt / 6.0;
    s->pos = v3_add(s->pos, v3_scale(v3_add(v3_add(k1.pos, k4.pos),
                                            v3_scale(v3_add(k2.pos, k3.pos), 2.0)), h));
    s->vel = v3_add(s->vel, v3_scale(v3_add(v3_add(k1.vel, k4.vel),
                                            v3_scale(v3_add(k2.vel, k3.vel), 2.0)), h));
    s->w   = v3_add(s->w,   v3_scale(v3_add(v3_add(k1.w, k4.w),
                                            v3_scale(v3_add(k2.w, k3.w), 2.0)), h));
    s->q.w += h * (k1.q.w + 2.0 * k2.q.w + 2.0 * k3.q.w + k4.q.w);
    s->q.x += h * (k1.q.x + 2.0 * k2.q.x + 2.0 * k3.q.x + k4.q.x);
    s->q.y += h * (k1.q.y + 2.0 * k2.q.y + 2.0 * k3.q.y + k4.q.y);
    s->q.z += h * (k1.q.z + 2.0 * k2.q.z + 2.0 * k3.q.z + k4.q.z);
    s->q = q_normalize(s->q);
    for (int i = 0; i < dp->motors; i++)
        s->s[i] += h * (k1.s[i] + 2.0 * k2.s[i] + 2.0 * k3.s[i] + k4.s[i]);

    /* ground contact */
    if (s->pos.z <= 0.0) {
        s->pos.z = 0.0;
        if (s->vel.z < 0.0) s->vel.z = 0.0;
        if (physics_total_thrust(s, dp, in) < dp->mass * dp->g) {
            s->vel = v3(0.0, 0.0, 0.0);
            s->w   = v3(0.0, 0.0, 0.0);
            f1 = q_rotate_inv(s->q, v3(0.0, 0.0, dp->g));
        } else {
            s->vel.x *= 0.98;
            s->vel.y *= 0.98;
        }
    }

    if (f_spec_body) *f_spec_body = f1;
}
