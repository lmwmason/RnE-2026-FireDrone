#include "physics.h"
#include "scenario.h"
#include "pilot.h"
#include "logger.h"
#include "fw_host.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define DT_CTRL      0.001       /* firmware 1 kHz tick */
#define PHYS_SUBSTEP 2
#define RC_PERIOD    4           /* 250 Hz CRSF frames */
#define LOG_PERIOD   5           /* 200 Hz log */
#define SIM_MAX      200.0

static const char *FAULT_NAMES[] = { "NORMAL", "SENSOR_DEGRADED", "MOTOR_FAULT",
                                     "COMM_LOST", "EMERGENCY_LAND" };

static void usage(const char *prog) {
    printf("usage: %s [--scenario NAME] [--out FILE] [--seed N] [--att truth|ukf]\n"
           "          [--duration SEC] [--motor-tau SEC] [--noise SCALE] [--cost-us EVEN,ODD]\n"
           "          [--laps N] [--list] [--mission]\n", prog);
}

static void list_scenarios(void) {
    for (int i = 0; i < scenario_count(); i++) {
        const Scenario *sc = scenario_at(i);
        int supported = 1;
        for (int e=0;e<sc->n_events;e++)
            if(sc->events[e].type==EV_GPS_LOSS || sc->events[e].type==EV_IMU_LOSS) supported=0;
        if(supported) printf("%-14s %s\n", sc->name, sc->desc);
    }
}

int main(int argc, char **argv) {
    const char *scn_name = "nominal";
    const char *out_path = "result.csv";
    unsigned long seed   = 42;
    double duration      = SIM_MAX;
    AttSource att_src    = ATT_UKF;
    int requested_laps = 0;
    double noise_scale   = 1.0;
    double cost_even_us  = 0.0, cost_odd_us = 0.0;   /* 0 = infinitely fast MCU */
    DroneParams dp       = DRONE_PARAMS;

    for (int i = 1; i < argc; i++) {
        if      (!strcmp(argv[i], "--scenario") && i + 1 < argc) scn_name = argv[++i];
        else if (!strcmp(argv[i], "--out")      && i + 1 < argc) out_path = argv[++i];
        else if (!strcmp(argv[i], "--seed")     && i + 1 < argc) seed     = strtoul(argv[++i], NULL, 10);
        else if (!strcmp(argv[i], "--duration") && i + 1 < argc) duration = atof(argv[++i]);
        else if (!strcmp(argv[i], "--motor-tau") && i + 1 < argc) dp.motor_tau = atof(argv[++i]);
        else if (!strcmp(argv[i], "--noise")    && i + 1 < argc) noise_scale = atof(argv[++i]);
        else if (!strcmp(argv[i], "--cost-us")  && i + 1 < argc) {
            if (sscanf(argv[++i], "%lf,%lf", &cost_even_us, &cost_odd_us) != 2) { usage(argv[0]); return 1; }
        }
        else if (!strcmp(argv[i], "--laps") && i + 1 < argc) {
            char *end;
            long n = strtol(argv[++i], &end, 10);
            if (*end || n < 1 || n > 1000) { usage(argv[0]); return 1; }
            requested_laps = (int)n;
        }
        else if (!strcmp(argv[i], "--att")      && i + 1 < argc)
        {
            const char *value = argv[++i];
            if (!strcmp(value, "ukf")) att_src = ATT_UKF;
            else if (!strcmp(value, "truth")) att_src = ATT_TRUTH;
            else { usage(argv[0]); return 1; }
        }
        else if (!strcmp(argv[i], "--list")) { list_scenarios(); return 0; }
        else if (!strcmp(argv[i], "--mission")) {
            for (int k = 0; k < MISSION_LEN; k++)
                printf("%.2f %.2f %.2f\n", MISSION[k].x, MISSION[k].y, MISSION[k].z);
            return 0;
        }
        else { usage(argv[0]); return 1; }
    }

    if (!isfinite(duration) || duration <= 0 || duration > 86400 ||
        !isfinite(dp.motor_tau) || dp.motor_tau <= 0 ||
        !isfinite(noise_scale) || noise_scale < 0 ||
        !isfinite(cost_even_us) || cost_even_us < 0 ||
        !isfinite(cost_odd_us) || cost_odd_us < 0) {
        fprintf(stderr, "invalid simulation parameters\n"); return 1;
    }
    const Scenario *sc = scenario_find(scn_name);
    if (!sc) {
        fprintf(stderr, "unknown scenario '%s'\n", scn_name);
        list_scenarios();
        return 1;
    }

    for (int e=0;e<sc->n_events;e++) {
        if(sc->events[e].type==EV_GPS_LOSS || sc->events[e].type==EV_IMU_LOSS) {
            fprintf(stderr,"this scenario requires firedrone_x8_sim sensor interfaces\n"); return 1;
        }
    }
    Rng rng;
    double baro_alt = 0.0;
    rng_seed(&rng, seed);

    World w;
    world_init(&w, &rng, sc);
    w.imu.gyro_sigma_dps *= noise_scale;
    w.imu.accel_sigma_g  *= noise_scale;

    DroneState s;
    physics_init(&s);

    Pilot pilot;
    pilot_init(&pilot, att_src);
    pilot.laps = requested_laps ? requested_laps : sc->laps;
    pilot.mission_not_before = sc->events[sc->n_events - 1].t;

    /* gyro calibration at rest: the firmware averages 500 samples */
    float cal[3];
    for (int i = 0; i < 3; i++)
        cal[i] = (float)(w.imu.gyro_bias_dps[i] + w.imu.gyro_sigma_dps * rng_normal(&rng) / sqrt(500.0));
    fw_host_init(cal);

    Logger lg;
    if (!logger_open(&lg, out_path)) {
        fprintf(stderr, "cannot open %s\n", out_path);
        return 1;
    }
    logger_write_header(&lg);

    printf("scenario : %s  (%s)\n", sc->name, sc->desc);

    printf("attitude : %s | requested laps %d\n", att_src == ATT_UKF ? "firmware UKF" : "TRUTH (diagnostic)", pilot.laps);

    long   steps      = (long)(duration / DT_CTRL);
    double dt_phys    = DT_CTRL / PHYS_SUBSTEP;
    Vec3   f_spec     = v3(0.0, 0.0, DRONE_PARAMS.g);
    FwOut  fo         = { 0 };
    double cmd[8]     = { 0 };
    int    crashed    = 0;
    double t_end      = -1.0;
    double impact_vz  = 0.0;
    double max_alt    = 0.0;
    int    last_fault = 0;
    int    last_arm   = 0;
    int    last_phase = -1;
    double busy_until = 0.0;        /* MCU is still computing the previous tick [s] */
    long   dropped    = 0;

    for (long i = 0; i < steps; i++) {
        double t = i * DT_CTRL;

        int fired_before[SCENARIO_MAX_EVENTS];
        memcpy(fired_before, w.fired, sizeof(fired_before));
        world_update(&w, sc, t, DT_CTRL, &rng);
        for (int e = 0; e < sc->n_events; e++)
            if (w.fired[e] && !fired_before[e])
                printf("  event=%d t=%.3f type=%d\n", e, t, sc->events[e].type);
        nav_model_update(&w.nav, &s, t);

        Quat att_est = s.q;
        if (att_src == ATT_UKF) {
            att_est = (Quat){ fo.ukf_q[0], fo.ukf_q[1], fo.ukf_q[2], fo.ukf_q[3] };
            if (i == 0) att_est = (Quat){1, 0, 0, 0};
        }

        int send_rc = (i % RC_PERIOD == 0);
        if (send_rc) pilot_update(&pilot, t, DT_CTRL * RC_PERIOD, w.nav.pos, w.nav.vel, att_est);

        double mean_cmd = 0.25 * (cmd[0] + cmd[1] + cmd[2] + cmd[3]);
        ImuSample imu;
        imu_model_sample(&w.imu, &s, f_spec, mean_cmd, DT_CTRL, &imu);

        FwImuIn in;
        if (i % 20 == 0) baro_alt = s.pos.z + 0.10 * rng_normal(&rng);     /* 50 Hz, 10 cm noise */
        in.baro_alt_m = (float)baro_alt;
        memcpy(in.gyro_dps, imu.gyro_dps, sizeof(in.gyro_dps));
        memcpy(in.accel_g,  imu.accel_g,  sizeof(in.accel_g));

        fw_host_set_time((uint32_t)(i + 1));
        if (send_rc && w.rc_link_up) fw_host_rc_frame(pilot.rc);       /* UART ISR preempts */

        /* TIM6 latches one pending tick; if the previous pass is still running
         * the new tick is merged into it and the sample is lost. */
        double t_now = (i + 1) * DT_CTRL;
        if (t_now >= busy_until) {
            fw_host_step(&in, &fo);
            double cost = ((fo.tick_count + 1) & 1u) ? cost_odd_us : cost_even_us;
            busy_until = t_now + cost * 1e-6;
        } else {
            dropped++;
        }

        for (int m = 0; m < 4; m++)
            cmd[m] = clamp(((double)fo.pwm_us[m] - 1000.0) / 1000.0, 0.0, 1.0);

        double vz_before = s.vel.z;
        for (int k = 0; k < PHYS_SUBSTEP; k++)
            physics_step(&s, cmd, &dp, &w.phys, dt_phys, &f_spec);

        if (vz_before < impact_vz && s.pos.z <= 0.0) impact_vz = vz_before;
        if (s.pos.z > max_alt) max_alt = s.pos.z;

        Vec3 zb = q_rotate(s.q, v3(0.0, 0.0, 1.0));
        if (!crashed && t > 3.0 && s.pos.z < 0.2 && (zb.z < 0.0 || vz_before < -3.0)) {
            crashed = 1;
            t_end   = t + 1.5;
        }

        if (fo.fault_state != last_fault) {
            printf("  t=%6.2f  fault state -> %s\n", t, FAULT_NAMES[fo.fault_state]);
            last_fault = fo.fault_state;
        }
        if (fo.arm != last_arm) {
            printf("  t=%6.2f  %s\n", t, fo.arm ? "ARMED" : "DISARMED");
            last_arm = fo.arm;
        }
        if ((int)pilot.phase != last_phase) {
            printf("  t=%6.2f  phase %s\n", t, PHASE_NAMES[pilot.phase]);
            last_phase = (int)pilot.phase;
        }

        if (i % LOG_PERIOD == 0) {
            LogRow r = { 0 };
            r.t = t;
            r.s = s;
            r.ref_pos = pilot.ref_pos;
            r.wind = w.phys.wind;
            r.ext_torque = w.phys.ext_torque;
            for (int m = 0; m < 4; m++) {
                r.eff[m]    = w.phys.eff[m];
                r.thrust[m] = dp.k_thrust * w.phys.eff[m] * s.s[m];
            }
            r.motor_count = 4;
            r.nav_valid = w.nav_valid; r.imu_valid = w.imu_valid;
            r.lap = pilot.lap;
            for (int e = 0; e < sc->n_events; e++) r.events_fired += w.fired[e];
            r.crashed = crashed;
            r.impact_vz = impact_vz;
            r.phase = (int)pilot.phase;
            r.wp = pilot.wp;
            r.link_up = w.rc_link_up;
            r.imu_fault = w.imu.accel_frozen || w.imu.spike_prob > 0.0
                        || w.imu.gyro_fault_dps[0] != 0.0 || w.imu.gyro_fault_dps[1] != 0.0
                        || w.imu.gyro_fault_dps[2] != 0.0;
            r.fw = fo;
            memcpy(r.rc, pilot.rc, sizeof(r.rc));
            logger_write(&lg, &r);
        }

        /* firmware stopped itself on the ground after losing the RC link or a motor: the mission is over */
        if (pilot.phase >= PH_TAKEOFF && pilot.phase < PH_DONE
            && (!w.rc_link_up || fo.fault_state == 2) && !fo.arm && s.pos.z < 0.3 && t > 3.0)
            pilot.phase = PH_DONE;
        if (pilot.phase == PH_DONE && t_end < 0.0) t_end = t + 2.0;
        if (t_end > 0.0 && t >= t_end) break;
    }
    logger_close(&lg);

    printf("result   : %s | max altitude %.2f m | touchdown vz %.2f m/s | overruns %u | dropped ticks %ld\n",
           crashed ? "CRASHED" : (pilot.phase == PH_DONE ? "LANDED" : "INCOMPLETE"),
           max_alt, impact_vz, fo.sched_overruns, dropped);
    int events_fired = 0;
    for (int e = 0; e < sc->n_events; e++) events_fired += w.fired[e];
    printf("coverage : %d/%d events | laps %d/%d\n", events_fired, sc->n_events, pilot.lap, pilot.laps);
    printf("log      : %s\n", out_path);
    return 0;
}
