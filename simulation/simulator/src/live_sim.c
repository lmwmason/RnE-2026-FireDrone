#include "live_sim.h"
#include "x8_config.h"
#include "scenario.h"
#include <stdlib.h>
#include <string.h>

struct LiveSim {
    DroneParams params;
    FcControl control;
    FcInput input;
    DroneState state;
    World world;
    Rng rng, fault_rng, arm_rng;
    Vec3 specific;
    double calibration[3], cmd[8];
    unsigned tick, events, cycles, hazard_used;
    int crashed, control_lost, hazard;
    double impact, next_fault, next_arm;
    double arm_angle, arm_speed, arm_target, arm_torque;
};

LiveSim *live_create(const char *config,unsigned seed)
{
    LiveSim *s=calloc(1,sizeof(*s)); if(!s) return NULL;
    FcConfig controller;
    if(!x8_load_config(config,&s->params,&controller) || !fc_init(&s->control,&controller,1)) {
        free(s); return NULL;
    }
    rng_seed(&s->rng,seed); rng_seed(&s->fault_rng,seed^0x51a79e3u);
    rng_seed(&s->arm_rng,seed^0x79ac03bu);
    world_init(&s->world,&s->rng,scenario_find("nominal"));
    physics_init(&s->state); s->specific=v3(0,0,s->params.g);
    s->input.mode=FC_MANUAL;
    s->input.rc[0]=s->input.rc[1]=s->input.rc[3]=1500;
    s->input.rc[2]=s->input.rc[4]=1000;
    for(int i=0;i<3;i++) s->calibration[i]=s->world.imu.gyro_bias_dps[i];
    s->next_fault=-1; s->hazard=-1;
    return s;
}
void live_destroy(LiveSim *s) { free(s); }

static void trigger(LiveSim *s,int type,double t)
{
    World *w=&s->world; Rng *r=&s->fault_rng;
    switch(type) {
    case 0:
        w->wind.mean=v3(6*rng_uniform(r)-3,6*rng_uniform(r)-3,0);
        wind_trigger_gust(&w->wind,t,1+2*rng_uniform(r),
            v3(16*rng_uniform(r)-8,16*rng_uniform(r)-8,4*rng_uniform(r)-2)); break;
    case 1: {
        int start=(int)(8*rng_uniform(r));
        for(int n=0;n<8;n++) {
            int m=(start+n)%8;
            if(w->phys.eff[m]>0) { w->phys.eff[m]*=0.45+0.35*rng_uniform(r); break; }
        }
        break;
    }
    case 2: {
        /* Choose a still-operating motor when possible; irreversible until R. */
        int start=(int)(8*rng_uniform(r));
        for(int n=0;n<8;n++) { int m=(start+n)%8; if(w->phys.eff[m]>0) {w->phys.eff[m]=0; break;} }
        break;
    }
    case 3: w->imu.accel_frozen=1; w->freeze_end=t+1+2*rng_uniform(r); break;
    case 4: {
        int ax=(int)(3*rng_uniform(r)); if(ax>2) ax=2;
        w->imu.gyro_fault_dps[ax]=(rng_uniform(r)<0.5?-1:1)*(3+5*rng_uniform(r));
        w->gyro_end[ax]=t+3+3*rng_uniform(r); break;
    }
    case 5: w->imu.spike_prob=0.15; w->imu.spike_g=8; w->spike_end=t+3; break;
    case 6: w->rc_link_up=0; w->rc_end=t+0.3+1.2*rng_uniform(r); break;
    case 7: w->nav_valid=0; w->nav_end=t+2+3*rng_uniform(r); break;
    case 8: w->imu_valid=0; w->imu_end=t+3; break;
    }
}

static void random_faults(LiveSim *s,double t,int enabled)
{
    if(!enabled) { s->next_fault=-1; return; }
    if(!s->control.manual_armed) return;
    if(s->next_fault<0) s->next_fault=t+5+7*rng_uniform(&s->fault_rng);
    if(t<s->next_fault) return;
    /* Random times, shuffled individual hazards. Every tenth event is ALL,
     * including a sustained complete IMU outage. No in-flight resurrection. */
    if(s->events%10==9) {
        for(int type=0;type<=8;type++) trigger(s,type,t);
        s->hazard=9; s->cycles++;
        s->hazard_used=0;
    } else {
        int type;
        do { type=(int)(9*rng_uniform(&s->fault_rng)); if(type>8) type=8; }
        while(s->hazard_used&(1u<<type));
        s->hazard_used|=1u<<type;
        trigger(s,type,t); s->hazard=type;
        /* Short transport outage in the isolated trial, sustained loss in ALL.
         * A single-IMU airframe cannot continue with no gyro indefinitely. */
        if(type==8) s->world.imu_end=t+0.010;
    }
    s->events++;
    s->next_fault=t+5+10*rng_uniform(&s->fault_rng);
}

static void random_arm(LiveSim *s,double t)
{
    /* Reduced single-link inverse-dynamics disturbance model, not a coupled
     * multibody robot. Reference link values are configurable and uncalibrated.
     * The total airframe mass includes the arm. Random targets are smoothly
     * tracked with bounded velocity/acceleration; torque is not drawn randomly. */
    if(t>=s->next_arm) {
        s->arm_target=1.4*rng_uniform(&s->arm_rng)-0.7;
        s->next_arm=t+0.8+2.2*rng_uniform(&s->arm_rng);
    }
    double acceleration=clamp(12*(s->arm_target-s->arm_angle)-5*s->arm_speed,-4,4);
    double previous_speed=s->arm_speed;
    s->arm_speed=clamp(s->arm_speed+0.001*acceleration,-1.5,1.5);
    acceleration=(s->arm_speed-previous_speed)/0.001;
    s->arm_angle+=0.001*s->arm_speed;
    double mass=s->params.arm_mass, length=s->params.arm_length;
    double inertia=mass*length*length/3;
    s->arm_torque=-(inertia*acceleration+mass*s->params.g*length*0.5*sin(s->arm_angle));
    s->world.phys.ext_torque.y+=s->arm_torque;
    /* No actual plant torque is provided as FF. An eventual arm encoder/model
     * driver can populate FcInput.arm_torque; leave it unavailable for now. */
    s->input.arm_valid=false;
}

void live_step(LiveSim *s,const uint16_t rc[5],int connected,int hazards,unsigned ticks)
{
    if(!s || !rc || s->crashed || ticks>1000) return;
    const Scenario empty={.turbulence=0.2};
    for(unsigned k=0;k<ticks;k++) {
        double t=s->tick*0.001; World *w=&s->world;
        world_update(w,&empty,t,0.001,&s->rng);
        random_faults(s,t,hazards);
        /* refresh wind after the random trigger without advancing the OU process twice */
        w->phys.wind=wind_at(&w->wind,t);
        random_arm(s,t);
        FcInput *in=&s->input;
        if(w->nav_valid) {
            nav_model_update(&w->nav,&s->state,t);
            in->nav_pos=w->nav.pos; in->nav_vel=w->nav.vel;
            in->nav_ms=(uint32_t)llround((w->nav.next_t-w->nav.period)*1000)+1;
        }
        in->nav_valid=w->nav_valid; in->imu_valid=w->imu_valid;
        in->link_up=connected && w->rc_link_up;
        if(in->link_up) { memcpy(in->rc,rc,sizeof(in->rc)); in->rc_ms=s->tick+1; }
        if(s->tick%20==0) {
            in->baro_alt=(float)(s->state.pos.z+0.1*rng_normal(&s->rng));
            in->baro_valid=true; in->baro_ms=s->tick+1;
        }
        double mean=0; for(int j=0;j<8;j++) mean+=s->cmd[j]/8;
        ImuSample sample; imu_model_sample(&w->imu,&s->state,s->specific,mean,0.001,&sample);
        in->gyro[0]=-(sample.gyro_dps[1]-s->calibration[1])*M_PI/180;
        in->gyro[1]=(sample.gyro_dps[0]-s->calibration[0])*M_PI/180;
        in->gyro[2]=(sample.gyro_dps[2]-s->calibration[2])*M_PI/180;
        in->accel[0]=-sample.accel_g[1]; in->accel[1]=sample.accel_g[0]; in->accel[2]=sample.accel_g[2];
        for(int j=0;j<8;j++) {
            in->rpm[j]=w->phys.eff[j]==0?0:(float)(8000*sqrt(fmax(0,s->state.s[j])));
            in->rpm_valid[j]=true;
        }
        if(!fc_step(&s->control,in,s->tick+1)) s->control_lost=1;
        for(int j=0;j<8;j++) s->cmd[j]=s->control_lost?0:s->control.allocation.u[j];
        double vz=s->state.vel.z;
        for(int j=0;j<2;j++) physics_step(&s->state,s->cmd,&s->params,&w->phys,0.0005,&s->specific);
        if(s->state.pos.z<=0 && vz<s->impact) s->impact=vz;
        Vec3 up=q_rotate(s->state.q,v3(0,0,1));
        if((s->state.pos.z<0.1 && (vz<-3 || up.z<0)) || !isfinite(s->state.pos.z)) s->crashed=1;
        s->tick++;
        if(s->crashed) break;
    }
}
void live_view(const LiveSim *s,LiveView *out)
{
    memset(out,0,sizeof(*out)); if(!s) return;
    out->t=s->tick*0.001;
    out->pos[0]=s->state.pos.x; out->pos[1]=s->state.pos.y; out->pos[2]=s->state.pos.z;
    out->vel[0]=s->state.vel.x; out->vel[1]=s->state.vel.y; out->vel[2]=s->state.vel.z;
    out->q[0]=s->state.q.w; out->q[1]=s->state.q.x; out->q[2]=s->state.q.y; out->q[3]=s->state.q.z;
    memcpy(out->cmd,s->cmd,sizeof(out->cmd)); memcpy(out->eff,s->world.phys.eff,sizeof(out->eff));
    out->wind[0]=s->world.phys.wind.x; out->wind[1]=s->world.phys.wind.y; out->wind[2]=s->world.phys.wind.z;
    out->arm_angle=s->arm_angle; out->arm_target=s->arm_target; out->arm_torque=s->arm_torque;
    out->impact=s->impact; out->armed=s->control.manual_armed && !s->control_lost && !s->crashed;
    out->crashed=s->crashed; out->failed_mask=s->control.failed_mask;
    out->events=(int)s->events; out->hazard=s->hazard; out->cycles=(int)s->cycles;
    out->control_lost=s->control_lost;
}
