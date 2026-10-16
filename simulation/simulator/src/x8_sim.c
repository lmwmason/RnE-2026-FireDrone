#include "x8_config.h"
#include "scenario.h"
#include "logger.h"
#include <stdlib.h>
#include <string.h>

int main(int argc,char **argv)
{
    const char *name="gauntlet", *path="out/x8_gauntlet.csv";
    const char *config="configs/x8_design_study.conf";
    double duration=240;
    unsigned seed=42, laps=8;
    int override_laps=0;
    for(int i=1;i<argc;i++) {
        if(!strcmp(argv[i],"--list")) {
            for(int j=0;j<scenario_count();j++) printf("%-14s %s\n",scenario_at(j)->name,scenario_at(j)->desc);
            return 0;
        }
        if(!strcmp(argv[i],"--mission")) {
            for(int j=0;j<MISSION_LEN;j++) printf("%.2f %.2f %.2f\n",MISSION[j].x,MISSION[j].y,MISSION[j].z);
            return 0;
        }
        if(i+1>=argc) { fprintf(stderr,"missing argument\n"); return 1; }
        const char *key=argv[i], *value=argv[++i];
        if(!strcmp(key,"--scenario")) name=value;
        else if(!strcmp(key,"--out")) path=value;
        else if(!strcmp(key,"--config")) config=value;
        else if(!strcmp(key,"--att") && !strcmp(value,"ukf")) {} /* no truth mode */
        else if(!strcmp(key,"--seed")) seed=(unsigned)strtoul(value,NULL,10);
        else if(!strcmp(key,"--duration")) duration=strtod(value,NULL);
        else if(!strcmp(key,"--laps")) { laps=(unsigned)strtoul(value,NULL,10); override_laps=1; }
        else { fprintf(stderr,"unknown option %s (X8 uses onboard UKF only)\n",key); return 1; }
    }
    const Scenario *sc=scenario_find(name);
    if(!sc || !isfinite(duration) || duration<=0 || duration>86400 || laps<1 || laps>1000) return 1;
    if(!override_laps) laps=(unsigned)sc->laps;
    DroneParams dp; FcConfig cc;
    if(!x8_load_config(config,&dp,&cc)) return 1;
    FcControl control;
    if(!fc_init(&control,&cc,laps)) return 1;
    control.mission.mission_not_before=sc->events[sc->n_events-1].t;
    Rng rng; rng_seed(&rng,seed);
    World w; world_init(&w,&rng,sc);
    DroneState state; physics_init(&state);
    Logger log; if(!logger_open(&log,path)) { perror(path); return 1; }
    logger_write_header(&log);
    printf("airframe : coaxial X8 / UNCALIBRATED DESIGN STUDY / %s\n",config);
    printf("scenario : %s | onboard C99 UKF/INDI/ADRC/MRAC/CAM | laps %u\n",name,laps);
    double cmd[8]={0}, max_alt=0, impact=0, end=-1;
    Vec3 specific=v3(0,0,dp.g);
    int crashed=0, control_failed=0;
    FcInput input={0}; input.mode=FC_MISSION;
    input.mission_authorized=true; /* this executable explicitly starts a trial */
    /* Rest calibration is available on real hardware; fault offsets are not. */
    double bias[3];
    for(int j=0;j<3;j++) bias[j]=w.imu.gyro_bias_dps[j]+w.imu.gyro_sigma_dps*rng_normal(&rng)/sqrt(500.0);
    for(long step=0;step<(long)(duration/0.001);step++) {
        double t=step*0.001;
        int before[SCENARIO_MAX_EVENTS]; memcpy(before,w.fired,sizeof(before));
        world_update(&w,sc,t,0.001,&rng);
        for(int e=0;e<sc->n_events;e++) if(w.fired[e]&&!before[e])
            printf("  event=%d t=%.3f type=%d\n",e,t,sc->events[e].type);
        if(w.nav_valid) {
            nav_model_update(&w.nav,&state,t);
            input.nav_pos=w.nav.pos; input.nav_vel=w.nav.vel;
            input.nav_ms=(uint32_t)((w.nav.next_t-w.nav.period)*1000);
        }
        input.nav_valid=w.nav_valid;
        input.imu_valid=w.imu_valid;
        input.link_up=w.rc_link_up;
        if(step%20==0) {
            input.baro_alt=(float)(state.pos.z+0.10*rng_normal(&rng));
            input.baro_ms=(uint32_t)(step+1); input.baro_valid=true;
        }
        double mean=0; for(int j=0;j<8;j++) mean+=cmd[j]/8;
        ImuSample sample;
        imu_model_sample(&w.imu,&state,specific,mean,0.001,&sample);
        input.gyro[0]=-(sample.gyro_dps[1]-bias[1])*M_PI/180;
        input.gyro[1]=(sample.gyro_dps[0]-bias[0])*M_PI/180;
        input.gyro[2]=(sample.gyro_dps[2]-bias[2])*M_PI/180;
        input.accel[0]=-sample.accel_g[1]; input.accel[1]=sample.accel_g[0]; input.accel[2]=sample.accel_g[2];
        for(int j=0;j<8;j++) {
            /* Ideal ESC RPM transport model: a stopped rotor reports zero.
             * Prop damage does not falsely report a dead ESC. No fault mask
             * or plant efficiency is passed across the control interface. */
            input.rpm[j]=w.phys.eff[j]==0?0:(float)(8000*sqrt(fmax(0,state.s[j])));
            input.rpm_valid[j]=true;
        }
        /* No arm kinematics/telemetry exists yet: never feed actual external
         * plant torque to the controller as a perfect feedforward estimate. */
        input.arm_valid=false;
        bool healthy=fc_step(&control,&input,(uint32_t)(step+1));
        if(!healthy && !control_failed) {
            printf("  t=%.3f CONTROL_UNAVAILABLE (sensor/solver)\n",t);
            control_failed=1;
        }
        for(int j=0;j<8;j++) cmd[j]=control_failed?0:control.allocation.u[j];
        double vz=state.vel.z;
        for(int k=0;k<2;k++) physics_step(&state,cmd,&dp,&w.phys,0.0005,&specific);
        if(state.pos.z<=0 && vz<impact) impact=vz;
        max_alt=fmax(max_alt,state.pos.z);
        Vec3 up=q_rotate(state.q,v3(0,0,1));
        if(!isfinite(state.pos.z) || !isfinite(state.q.w)) { crashed=1; break; }
        if(!crashed && t>3 && state.pos.z<0.2 && (up.z<0 || vz<-3)) { crashed=1; end=t+1.5; }
        if(control.mission.phase==PH_DONE && end<0) end=t+2;
        if(step%5==0) {
            LogRow r={0}; r.t=t; r.s=state; r.ref_pos=control.mission.ref_pos;
            r.wind=w.phys.wind; r.ext_torque=w.phys.ext_torque;
            r.phase=control.mission.phase; r.wp=control.mission.wp; r.lap=control.mission.lap;
            r.link_up=w.rc_link_up; r.crashed=crashed; r.impact_vz=impact;
            r.motor_count=8; r.motor_mask=control.failed_mask;
            r.nav_valid=input.nav_valid; r.imu_valid=input.imu_valid;
            r.imu_fault=!w.imu_valid || w.imu.accel_frozen || w.imu.spike_prob>0;
            r.allocator_converged=control.allocation.converged;
            for(int j=0;j<4;j++) r.allocation_residual+=control.allocation.residual[j]*control.allocation.residual[j];
            r.allocation_residual=sqrt(r.allocation_residual);
            r.fw.arm=control.mission.rc[4]>=1700 && !control_failed;
            r.fw.fault_state=control_failed?4:control.failed_mask?2:control.nav_degraded?1:!w.rc_link_up?3:0;
            ukf_get_attitude(&control.ukf,r.fw.ukf_q);
            adrc_get_disturbance(&control.adrc,r.fw.adrc_fhat);
            mrac_get_params(&control.mrac,r.fw.mrac_theta1,r.fw.mrac_theta2);
            for(int j=0;j<8;j++) {
                r.fw.pwm_us[j]=(uint16_t)(1000+1000*cmd[j]); r.eff[j]=w.phys.eff[j];
                r.thrust[j]=dp.k_thrust*dp.rotor_gain[j]*w.phys.eff[j]*state.s[j];
            }
            for(int e=0;e<sc->n_events;e++) r.events_fired+=w.fired[e];
            memcpy(r.rc,control.mission.rc,sizeof(r.rc)); logger_write(&log,&r);
        }
        if(end>0 && t>=end) break;
    }
    logger_close(&log);
    int fired=0; for(int e=0;e<sc->n_events;e++) fired+=w.fired[e];
    printf("result   : %s | max altitude %.2f m | touchdown vz %.2f m/s\n",
        crashed?"CRASHED":control_failed?"CONTROL_UNAVAILABLE":control.mission.phase==PH_DONE?"LANDED":"INCOMPLETE",max_alt,impact);
    printf("coverage : %d/%d events | laps %d/%u\n",fired,sc->n_events,control.mission.lap,laps);
    printf("log      : %s\n",path);
    return 0;
}
