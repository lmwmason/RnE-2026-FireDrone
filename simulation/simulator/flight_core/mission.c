#include "mission.h"

/* What the pilot believes about the airframe: the same numbers the firmware
 * was tuned with, not the plant's real ones. */
#define NOM_MASS      0.45
#define NOM_KTHRUST   5.0
#define G0            9.80665
#define MOTOR_IDLE    0.05

#define MAX_ROLL_RATE   (600.0 * 0.01745329)
#define MAX_YAW_RATE    (400.0 * 0.01745329)

#define KP_XY   2.4
#define KD_XY   3.0
#define KI_XY   0.35
#define KP_Z    6.0
#define KD_Z    5.0
#define KI_Z    2.0
#define KP_ATT  9.0
#define KP_YAW  3.0
#define MAX_TILT   (30.0 * M_PI / 180.0)

#define CRUISE_SPEED   2.0
#define CLIMB_SPEED    0.9
#define WP_RADIUS      0.35
#define WP_HOLD        1.0
#define FLYBY_RADIUS  1.5
#define ARM_DELAY      1.0
#define ARM_SETTLE     0.6

const Vec3 MISSION[] = {
    { 0.0, 0.0, 3.0 },
    { 6.0, 0.0, 3.0 },
    { 6.0, 6.0, 4.0 },
    { 0.0, 6.0, 3.0 },
    { 0.0, 0.0, 3.0 },
};
const int MISSION_LEN = (int)(sizeof(MISSION) / sizeof(MISSION[0]));

const char *PHASE_NAMES[] = { "PREARM", "ARM", "TAKEOFF", "MISSION", "LAND", "DONE" };

void pilot_init(Pilot *p, AttSource src) {
    *p = (Pilot){ 0 };
    p->mass = NOM_MASS;
    p->total_thrust = 4.0 * NOM_KTHRUST;
    p->att_source = src;
    p->phase      = PH_PREARM;
    p->rc[0] = p->rc[1] = p->rc[3] = 1500;
    p->rc[2] = 1000;
    p->rc[4] = 1000;
}

static void set_phase(Pilot *p, PilotPhase ph) {
    p->phase   = ph;
    p->t_phase = 0.0;
}

static uint16_t rate_to_us(double rate, double max_rate) {
    return (uint16_t)(1500.0 + clamp(rate / max_rate, -1.0, 1.0) * 500.0);
}

static void advance_reference(Pilot *p, Vec3 target, double speed, double dt) {
    Vec3 d = v3_sub(target, p->ref_pos);
    double dist = v3_norm(d);
    double step = speed * dt;
    if (dist <= step) {
        p->ref_pos = target;
        p->ref_vel = v3(0.0, 0.0, 0.0);
    } else {
        p->ref_vel = v3_scale(d, speed / dist);
        p->ref_pos = v3_add(p->ref_pos, v3_scale(d, step / dist));
    }
}

void pilot_update(Pilot *p, double t, double dt, Vec3 nav_pos, Vec3 nav_vel, Quat att) {
    p->t_phase += dt;

    int    throttle_min = 0;
    double land_speed   = 0.0;

    switch (p->phase) {
    case PH_PREARM:
        throttle_min = 1;
        if (p->t_phase > ARM_DELAY) set_phase(p, PH_ARM);
        break;
    case PH_ARM:
        throttle_min = 1;
        p->rc[4] = 2000;
        if (p->t_phase > ARM_SETTLE) {
            p->ref_pos = nav_pos;
            set_phase(p, PH_TAKEOFF);
        }
        break;
    case PH_TAKEOFF: {
        Vec3 tgt = v3(p->ref_pos.x, p->ref_pos.y, MISSION[0].z);
        advance_reference(p, tgt, CLIMB_SPEED, dt);
        if (fabs(nav_pos.z - MISSION[0].z) < 0.25 && p->t_phase > 1.0) {
            p->wp = 1;
            p->hold = 0.0;
            set_phase(p, PH_MISSION);
        }
        break;
    }
    case PH_MISSION: {
        advance_reference(p, MISSION[p->wp], CRUISE_SPEED, dt);
        if (v3_norm(v3_sub(MISSION[p->wp], nav_pos)) < WP_RADIUS) p->hold += dt;
        else                                                       p->hold = 0.0;
        /* with several laps the corners are flown through, only the last one is held */
        int last   = (p->wp + 1 >= MISSION_LEN) && (p->lap + 1 >= p->laps);
        int flyby  = p->laps > 1 && !last;
        int reached = flyby ? (v3_norm(v3_sub(MISSION[p->wp], p->ref_pos)) < 1e-6
                               && v3_norm(v3_sub(MISSION[p->wp], nav_pos)) < FLYBY_RADIUS)
                            : p->hold > WP_HOLD;
        if (reached) {
            p->hold = 0.0;
            if (p->wp + 1 < MISSION_LEN)    p->wp++;
            else if (++p->lap < p->laps)    p->wp = 1;
            else if (t >= p->mission_not_before) set_phase(p, PH_LAND);
            else p->wp = 1;
        }
        break;
    }
    case PH_LAND: {
        land_speed = clamp(0.4 * nav_pos.z, 0.25, 0.6);
        Vec3 tgt = v3(p->ref_pos.x, p->ref_pos.y, -0.3);
        advance_reference(p, tgt, land_speed, dt);
        if (nav_pos.z < 0.08) {
            if (p->t_phase > 0.3) {
                p->rc[4] = 1000;
                set_phase(p, PH_DONE);
            }
        } else {
            p->t_phase = 0.0;
        }
        break;
    }
    case PH_DONE:
        p->rc[4] = 1000;
        throttle_min = 1;
        break;
    }

    /* ---- position / velocity loop -> desired specific force [m/s^2] ---- */
    Vec3 e_p = v3_sub(p->ref_pos, nav_pos);
    Vec3 e_v = v3_sub(p->ref_vel, nav_vel);

    if (nav_pos.z > 0.15 && p->phase != PH_DONE) {
        p->integ = v3_add(p->integ, v3_scale(e_p, dt));
        p->integ.x = clamp(p->integ.x, -8.0, 8.0);
        p->integ.y = clamp(p->integ.y, -8.0, 8.0);
        p->integ.z = clamp(p->integ.z, -3.0, 3.0);
    }

    Vec3 a = v3(KP_XY * e_p.x + KD_XY * e_v.x + KI_XY * p->integ.x,
                KP_XY * e_p.y + KD_XY * e_v.y + KI_XY * p->integ.y,
                KP_Z  * e_p.z + KD_Z  * e_v.z + KI_Z  * p->integ.z);

    double az = clamp(a.z, -0.6 * G0, 0.8 * G0);
    double axy_max = tan(MAX_TILT) * (G0 + az);
    double axy = sqrt(a.x * a.x + a.y * a.y);
    if (axy > axy_max) {
        a.x *= axy_max / axy;
        a.y *= axy_max / axy;
    }

    Vec3 F  = v3(a.x, a.y, G0 + az);
    double Fn = v3_norm(F);
    Vec3 zd = v3_scale(F, 1.0 / Fn);

    /* ---- attitude loop -> body rate commands ---- */
    Vec3 zb  = q_rotate(att, v3(0.0, 0.0, 1.0));
    Vec3 eb  = q_rotate_inv(att, v3_cross(zb, zd));
    double w_x = KP_ATT * eb.x;
    double w_y = KP_ATT * eb.y;
    double w_z = KP_YAW * wrap_pi(p->ref_yaw - q_yaw(att));

    /* ---- collective -> throttle stick ---- */
    double u_mean = p->mass * Fn / p->total_thrust;
    double stick  = clamp((u_mean - MOTOR_IDLE) / (1.0 - MOTOR_IDLE), 0.0, 1.0);

    p->rc[0] = rate_to_us(w_x, MAX_ROLL_RATE);
    p->rc[1] = rate_to_us(w_y, MAX_ROLL_RATE);
    p->rc[3] = rate_to_us(w_z, MAX_YAW_RATE);
    p->rc[2] = (uint16_t)(1000.0 + 1000.0 * stick);

    if (throttle_min) {
        p->rc[0] = p->rc[1] = p->rc[3] = 1500;
        p->rc[2] = 1000;
    }
}
