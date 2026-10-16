#include "control.h"
#include "x8_config.h"
#include "scenario.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static void allocation_tests(void)
{
    DroneParams p; FcConfig cfg;
    assert(x8_load_config("configs/x8_design_study.conf",&p,&cfg));
    for(int failed=-1;failed<8;failed++) {
        FcAllocator a; FcAllocation result;
        float weights[4]={1,40,40,25};
        assert(fc_allocator_init(&a,cfg.motor,weights));
        float target[4]={cfg.mass*9.80665f,0,0,0};
        assert(fc_allocate(&a,target,failed<0?0:(1u<<failed),&result));
        assert(result.converged);
        for(int j=0;j<8;j++) assert(result.u[j]>=0 && result.u[j]<=1);
        if(failed>=0) assert(result.u[failed]==0);
        for(int k=0;k<4;k++) assert(fabsf(result.residual[k])<0.002f);
        target[0]=1e4f;
        assert(fc_allocate(&a,target,0,&result));
        assert(result.residual[0]<-9000); /* infeasibility is visible */
        assert(fc_allocate(&a,target,255,&result));
        for(int j=0;j<8;j++) assert(result.u[j]==0);
        target[0]=NAN;
        assert(!fc_allocate(&a,target,0,&result));
        for(int j=0;j<8;j++) assert(result.u[j]==0);
    }
}

static void physics_tests(void)
{
    DroneParams p; FcConfig cfg;
    assert(x8_load_config("configs/x8_design_study.conf",&p,&cfg));
    p.drag_lin=p.drag_quad=0;
    PhysInputs in={0}; for(int j=0;j<8;j++) in.eff[j]=1;
    double cmd[8]={0}; Vec3 specific;
    DroneState state; physics_init(&state); state.pos.z=100;
    for(int i=0;i<1000;i++) physics_step(&state,cmd,&p,&in,0.001,&specific);
    assert(fabs(state.pos.z-(100-0.5*p.g))<1e-9);
    assert(fabs(state.vel.z+p.g)<1e-9);
    assert(v3_norm(state.w)<1e-12);
    physics_init(&state); state.pos.z=100;
    for(int j=0;j<8;j++) state.s[j]=cmd[j]=0.5;
    double acc=4*p.k_thrust/p.mass-p.g;
    for(int i=0;i<1000;i++) physics_step(&state,cmd,&p,&in,0.001,&specific);
    assert(fabs(state.pos.z-(100+0.5*acc))<1e-8);
    assert(v3_norm(state.w)<1e-10);
    /* Matching upper/lower pair: yaw cancels, roll/pitch moments add. */
    physics_init(&state); state.pos.z=100;
    memset(cmd,0,sizeof(cmd)); cmd[0]=cmd[4]=state.s[0]=state.s[4]=0.5;
    physics_step(&state,cmd,&p,&in,1e-6,&specific);
    assert(state.w.x<0 && state.w.y>0 && fabs(state.w.z)<1e-10);
}

static void sensor_and_scenario_tests(void)
{
    Rng rng; rng_seed(&rng,42);
    ImuModel imu; imu_model_init(&imu,&rng);
    DroneState s; physics_init(&s);
    ImuSample first, frozen;
    imu_model_sample(&imu,&s,v3(1,2,9),0.2,0.001,&first);
    imu.accel_frozen=1;
    imu_model_sample(&imu,&s,v3(8,9,10),0.8,0.001,&frozen);
    for(int i=0;i<3;i++) assert(frozen.accel_g[i]==first.accel_g[i]);
    imu.spike_prob=1; imu.spike_g=2;
    imu_model_sample(&imu,&s,v3(8,9,10),0.8,0.001,&frozen);
    for(int i=0;i<3;i++) assert(fabsf(frozen.accel_g[i]-first.accel_g[i])>1.9);
    for(int i=0;i<scenario_count();i++) {
        const Scenario *sc=scenario_at(i);
        assert(sc->n_events<=SCENARIO_MAX_EVENTS);
        for(int e=1;e<sc->n_events;e++) assert(sc->events[e].t>=sc->events[e-1].t);
    }
    World w; const Scenario *sc=scenario_find("all_failures");
    world_init(&w,&rng,sc); world_update(&w,sc,12,0.001,&rng);
    for(int e=0;e<sc->n_events;e++) assert(w.fired[e]);
    assert(!w.imu_valid && !w.nav_valid && !w.rc_link_up);
    assert(w.phys.eff[1]==0 && w.phys.eff[3]<1);
    assert(w.arm.on && w.imu.accel_frozen && w.imu.spike_prob>0);
    world_update(&w,sc,1000,0.001,&rng); assert(!w.rc_link_up);
}

static void control_tests(void)
{
    DroneParams p; FcConfig cfg; FcControl a,b;
    assert(x8_load_config("configs/x8_design_study.conf",&p,&cfg));
    assert(fc_init(&a,&cfg,8)); assert(fc_init(&b,&cfg,8));
    FcInput input={0}; input.mode=FC_MISSION; input.imu_valid=input.nav_valid=input.baro_valid=true;
    input.accel[2]=1; input.nav_pos.z=3; input.baro_alt=3;
    for(int j=0;j<8;j++) { input.rpm[j]=4000; input.rpm_valid[j]=true; }
    input.nav_ms=input.baro_ms=1;
    assert(fc_step(&a,&input,1));
    for(int j=0;j<8;j++) assert(a.allocation.u[j]==0); /* no implicit arming */
    assert(fc_init(&a,&cfg,8));
    input.mission_authorized=true;
    for(unsigned t=1;t<=2000;t++) {
        input.nav_ms=input.baro_ms=t;
        input.link_up=true; assert(fc_step(&a,&input,t));
        input.link_up=t<1000; assert(fc_step(&b,&input,t));
        for(int j=0;j<8;j++) assert(a.allocation.u[j]==b.allocation.u[j]);
    }
    assert(fc_init(&a,&cfg,8));
    input.mode=FC_MANUAL; input.link_up=true;
    input.rc[0]=input.rc[1]=input.rc[3]=1500;
    input.rc[2]=input.rc[4]=1000;
    input.rc_ms=1998; assert(fc_step(&a,&input,1998));
    input.rc[4]=2000; input.rc_ms=1999; assert(fc_step(&a,&input,1999));
    input.rc[2]=1300; input.rc_ms=2000; assert(fc_step(&a,&input,2000));
    assert(a.manual_armed);
    FcControl stale=a, enabled=a;
    input.nav_ms=input.baro_ms=input.rc_ms=2001;
    input.arm_valid=false; assert(fc_step(&a,&input,2001));
    input.arm_valid=true; input.arm_ms=1; input.arm_torque[0]=0.1;
    assert(fc_step(&stale,&input,2001));
    for(int j=0;j<8;j++) assert(a.allocation.u[j]==stale.allocation.u[j]);
    input.arm_ms=2001; assert(fc_step(&enabled,&input,2001));
    float diff=0; for(int j=0;j<8;j++) diff+=fabsf(a.allocation.u[j]-enabled.allocation.u[j]);
    assert(diff>1e-4); /* usable FF is independent telemetry, not plant torque */
    FcControl lost_link=enabled;
    input.link_up=false;
    input.arm_valid=false;
    assert(fc_step(&lost_link,&input,2002));
    assert(lost_link.failsafe && lost_link.manual_armed);
    input.link_up=true;
    input.imu_valid=false;
    for(unsigned t=2002;t<=2022;t++) {
        bool ok=fc_step(&a,&input,t);
        if(t==2022) assert(!ok);
    }
    for(int j=0;j<8;j++) assert(a.allocation.u[j]==0);
}

static void invalid_and_rearm_tests(void)
{
    FcAllocator a={0}; FcAllocation output;
    float request[4]={20,0,0,0};
    assert(!fc_allocate(&a,request,0,&output));
    assert(!fc_allocator_init(NULL,NULL,NULL));
    DroneParams p; FcConfig cfg; FcControl c;
    assert(x8_load_config("configs/x8_design_study.conf",&p,&cfg));
    assert(fc_init(&c,&cfg,1));
    FcInput in={0}; in.imu_valid=true; in.accel[2]=1; in.link_up=true;
    in.rc[0]=in.rc[1]=in.rc[3]=1500; in.rc[2]=in.rc[4]=1000;
    in.rc_ms=1; assert(fc_step(&c,&in,1));
    in.rc[2]=0; in.rc[4]=2000; in.rc_ms=2;
    assert(fc_step(&c,&in,2)); assert(!c.manual_armed);
    in.rc[2]=1000; in.rc[4]=1000; in.rc_ms=3; assert(fc_step(&c,&in,3));
    in.rc[4]=2000; in.rc_ms=4; assert(fc_step(&c,&in,4)); assert(c.manual_armed);
    in.rc[2]=1300; in.rc_ms=5; assert(fc_step(&c,&in,5));
    in.rc[4]=1000; in.rc_ms=6; assert(fc_step(&c,&in,6));
    for(int j=0;j<8;j++) assert(c.allocator.u[j]==0 && c.u_lag[j]==0);
    /* Bad accel alone must not turn a healthy gyro into a total IMU outage. */
    in.accel[0]=NAN;
    for(unsigned ms=7;ms<50;ms++) { in.rc_ms=ms; assert(fc_step(&c,&in,ms)); }
    assert(c.imu_degraded && c.imu_missing==0);
    in.accel[0]=0; in.rc[2]=1000; in.rc[4]=2000; in.rc_ms=50; assert(fc_step(&c,&in,50));
    assert(c.manual_armed);
    in.link_up=false; in.baro_valid=true; in.baro_ms=51; in.baro_alt=NAN;
    assert(!fc_step(&c,&in,51));
    for(int j=0;j<8;j++) assert(c.allocation.u[j]==0);
}

int main(void)
{
    invalid_and_rearm_tests(); allocation_tests(); physics_tests(); sensor_and_scenario_tests(); control_tests();
    puts("X8 allocator, rigid-body invariants, fault schedule, sensor hold, control boundary: PASS");
    return 0;
}
