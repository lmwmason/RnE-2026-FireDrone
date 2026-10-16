#include "control.h"
#include <math.h>
#include <string.h>

static void stop_outputs(FcControl *c)
{
    memset(&c->allocation,0,sizeof(c->allocation));
    memset(c->allocator.u,0,sizeof(c->allocator.u));
    memset(c->u_lag,0,sizeof(c->u_lag));
    memset(c->achieved_alpha,0,sizeof(c->achieved_alpha));
    memset(c->ff_previous,0,sizeof(c->ff_previous));
    memset(c->dead_ticks,0,sizeof(c->dead_ticks));
    adrc_init(&c->adrc,20,20,15,1,1,1,10,2,0.001f);
    mrac_init(&c->mrac,10,10,0.001f,0.001f,1,0.1f,0.001f);
}

bool fc_init(FcControl *c, const FcConfig *config, unsigned laps)
{
    if(!c) return false;
    memset(c,0,sizeof(*c));
    if(!config || laps<1 || !isfinite(config->mass) || config->mass<=0 ||
       !isfinite(config->motor_tau) || config->motor_tau<0.001f) return false;
    for(int i=0;i<3;i++) if(!isfinite(config->inertia[i]) || config->inertia[i]<=0) return false;
    c->config=*config;
    /* Weight torque errors by rotor leverage, not by an X4 geometry constant. */
    float lever=0, yaw=0, thrust=0;
    for(int j=0;j<8;j++) {
        lever=fmaxf(lever,hypotf(config->motor[j].x,config->motor[j].y));
        yaw=fmaxf(yaw,fabsf(config->motor[j].yaw_torque));
        thrust+=config->motor[j].thrust;
    }
    if(lever<=0 || yaw<=0 || thrust<=0) return false;
    float weight[4]={1,4/(lever*lever),4/(lever*lever),1};
    if(!fc_allocator_init(&c->allocator,config->motor,weight)) return false;
    ukf_init(&c->ukf,0.5f,2,0);
    adrc_init(&c->adrc,20,20,15,1,1,1,10,2,0.001f);
    mrac_init(&c->mrac,10,10,0.001f,0.001f,1,0.1f,0.001f);
    pilot_init(&c->mission,ATT_UKF);
    c->mission.mass=config->mass;
    c->mission.total_thrust=thrust;
    c->mission.laps=(int)laps;
    c->initialized=true;
    return true;
}

bool fc_step(FcControl *c, const FcInput *in, uint32_t now_ms)
{
    if(!c) return false;
    if(!in || !c->initialized) { memset(c->allocation.u,0,sizeof(c->allocation.u)); return false; }
    if(in->mode!=FC_MANUAL && in->mode!=FC_MISSION) {
        memset(c->allocation.u,0,sizeof(c->allocation.u)); return false;
    }
    bool valid=in->imu_valid, accel_valid=in->imu_valid;
    for(int i=0;i<3;i++) {
        valid=valid && isfinite(in->gyro[i]);
        accel_valid=accel_valid && isfinite(in->accel[i]);
    }
    c->imu_degraded=!valid || !accel_valid;
    c->imu_missing=valid?0:c->imu_missing+1;
    /* A single missing IMU cannot be reconstructed from simulator truth.
     * A board with redundant IMUs should select the healthy unit before here.
     * A prolonged total outage is an explicit unsuccessful control step. */
    if(c->imu_missing>20) { memset(c->allocation.u,0,sizeof(c->allocation.u)); return false; }
    float gyro[3], accel[3];
    for(int i=0;i<3;i++) { gyro[i]=valid?in->gyro[i]:c->omega_prev[i]; accel[i]=accel_valid?in->accel[i]:0; }
    ukf_predict(&c->ukf,gyro,0.001f);
    if(valid && (c->ticks&1u)) ukf_update(&c->ukf,accel,gyro,12.592f);
    float q[4]; ukf_get_attitude(&c->ukf,q);
    for(int i=0;i<4;i++) if(!isfinite(q[i])) {
        memset(c->allocation.u,0,sizeof(c->allocation.u)); return false;
    }
    c->nav_degraded=!in->nav_valid || (uint32_t)(now_ms-in->nav_ms)>200;
    if(!isfinite(in->nav_pos.x) || !isfinite(in->nav_pos.y) || !isfinite(in->nav_pos.z) ||
       !isfinite(in->nav_vel.x) || !isfinite(in->nav_vel.y) || !isfinite(in->nav_vel.z))
        c->nav_degraded=true;
    bool baro_fresh=in->baro_valid && isfinite(in->baro_alt) && (uint32_t)(now_ms-in->baro_ms)<=200;
    if(baro_fresh && in->baro_ms!=c->last_baro_ms) {
        float dt=(uint32_t)(in->baro_ms-c->last_baro_ms)*0.001f;
        float previous=c->altitude;
        c->altitude+=0.15f*(in->baro_alt-c->altitude);
        if(dt>0 && dt<1) c->vertical_speed+=0.08f*((c->altitude-previous)/dt-c->vertical_speed);
        c->last_baro_ms=in->baro_ms;
    }
    bool manual=in->mode==FC_MANUAL;
    /* Keep derivative history current while disarmed, too. */
    for(int i=0;i<3;i++) {
        float previous=c->omega_f[i];
        c->omega_f[i]+=0.2738f*(gyro[i]-c->omega_f[i]);
        c->alpha[i]=(c->omega_f[i]-previous)/0.001f;
        c->omega_prev[i]=gyro[i];
    }
    bool fresh=in->link_up && (uint32_t)(now_ms-in->rc_ms)<=100;
    for(int j=0;j<5;j++) if(in->rc[j]<1000 || in->rc[j]>2000) fresh=false;
    if(manual) {
        if(fresh) {
            bool request=in->rc[4]>1700;
            if(!request) { c->manual_armed=false; c->arm_ready=true; }
            if(request && !c->arm_requested && c->arm_ready && in->rc[2]<1100 && valid) {
                c->manual_armed=true;
                c->arm_ready=false;
            }
            c->arm_requested=request;
            memcpy(c->mission.rc,in->rc,sizeof(c->mission.rc));
            for(int j=0;j<5;j++) {
                if(c->mission.rc[j]<1000) c->mission.rc[j]=1000;
                if(c->mission.rc[j]>2000) c->mission.rc[j]=2000;
            }
        }
        c->failsafe=c->manual_armed && !fresh;
        c->ticks++;
        if(!c->manual_armed) { stop_outputs(c); return true; }
    } else {
        if(in->mission_authorized && valid && !c->nav_degraded) c->mission_authorized=true;
        if(!c->mission_authorized) { memset(c->allocation.u,0,sizeof(c->allocation.u)); return true; }
        if((c->ticks%4)==0 && !c->nav_degraded)
            pilot_update(&c->mission,now_ms*0.001,0.004,in->nav_pos,in->nav_vel,
                         (Quat){q[0],q[1],q[2],q[3]});
        c->ticks++;
        if(c->mission.rc[4]<1700) { stop_outputs(c); return true; }
        c->failsafe=c->nav_degraded;
    }
    float rate[3];
    const int channel[3]={0,1,3};
    for(int i=0;i<3;i++) {
        rate[i]=((float)c->mission.rc[channel[i]]-1500)/500*(i==2?6.981317f:10.471975f);
        if(manual && i<2) {
            const double max_angle=35.0*M_PI/180.0;
            double roll=clamp(((double)c->mission.rc[0]-1500)/500,-1,1)*max_angle;
            double pitch=clamp(((double)c->mission.rc[1]-1500)/500,-1,1)*max_angle;
            double magnitude=hypot(roll,pitch);
            if(magnitude>max_angle) { roll*=max_angle/magnitude; pitch*=max_angle/magnitude; }
            Quat attitude={q[0],q[1],q[2],q[3]};
            double yaw=q_yaw(attitude);
            Quat qr={cos(roll/2),sin(roll/2),0,0};
            Quat qp={cos(pitch/2),0,sin(pitch/2),0};
            Quat qy={cos(yaw/2),0,0,sin(yaw/2)};
            Vec3 desired=q_rotate(q_mul(q_mul(qy,qp),qr),v3(0,0,1));
            Vec3 up=q_rotate(attitude,v3(0,0,1));
            Vec3 error=q_rotate_inv(attitude,v3_cross(up,desired));
            rate[i]=(float)(8*(i==0?error.x:error.y));
        }
        if(manual && i==2) rate[i]=((float)c->mission.rc[3]-1500)/500*3.14159265f;
        if(c->failsafe) {
            Quat attitude={q[0],q[1],q[2],q[3]};
            Vec3 up=q_rotate(attitude,v3(0,0,1));
            Vec3 error=q_rotate_inv(attitude,v3_cross(up,v3(0,0,1)));
            rate[i]=i==0?(float)(9*error.x):i==1?(float)(9*error.y):0;
        }
    }
    for(int j=0;j<8;j++) {
        bool stopped=in->rpm_valid[j] && isfinite(in->rpm[j]) && in->rpm[j]<50 &&
                     c->u_lag[j]>0.15f && baro_fresh && c->altitude>0.8;
        c->dead_ticks[j]=stopped?c->dead_ticks[j]+1:0;
        if(c->dead_ticks[j]>=200) c->failed_mask|=(uint8_t)(1u<<j);
        c->u_lag[j]+=0.001f/c->config.motor_tau*(c->allocation.u[j]-c->u_lag[j]);
        if(c->failed_mask&(1u<<j)) c->u_lag[j]=0;
    }
    float adaptation[3], adrc_u[3], derivative[3]={0}, disturbance[3];
    mrac_update(&c->mrac,gyro,rate,adaptation);
    adrc_update(&c->adrc,gyro,rate,derivative,c->achieved_alpha,adrc_u);
    adrc_get_disturbance(&c->adrc,disturbance);
    float request[4]={0};
    float throttle=((float)c->mission.rc[2]-1000)/1000;
    throttle=0.05f+0.95f*throttle;
    if(c->failsafe) {
        /* Navigation cannot be invented. Level and descend using the independent
         * barometer; horizontal position is unobservable with this sensor set. */
        if(!baro_fresh) {
            memset(c->allocation.u,0,sizeof(c->allocation.u)); return false;
        }
        float total=0; for(int j=0;j<8;j++) total+=c->config.motor[j].thrust;
        throttle=c->config.mass*9.80665f/total+0.06f*(-0.4f-c->vertical_speed);
        throttle=fminf(0.9f,fmaxf(0.05f,throttle));
        if(c->altitude<0.15f && fabsf(c->vertical_speed)<0.3f) {
            if(!c->ground_since) c->ground_since=now_ms;
            if((uint32_t)(now_ms-c->ground_since)>1000) {
                c->mission.phase=PH_DONE; c->mission.rc[4]=1000; c->manual_armed=false;
                memset(c->allocation.u,0,sizeof(c->allocation.u)); return true;
            }
        } else c->ground_since=0;
    } else c->ground_since=0;
    for(int j=0;j<8;j++) request[0]+=throttle*c->config.motor[j].thrust;
    bool ff=in->arm_valid && (uint32_t)(now_ms-in->arm_ms)<=100;
    for(int i=0;i<3;i++) {
        float nominal=0;
        for(int j=0;j<8;j++) nominal+=c->allocator.B[i+1][j]*c->u_lag[j];
        float alpha_ref=(i==2?30:50)*(rate[i]-gyro[i])-disturbance[i]+adaptation[i];
        float torque_ff=ff && isfinite(in->arm_torque[i])?in->arm_torque[i]:0;
        /* Incremental inversion: feed forward CHANGES in predicted arm torque.
         * The acceleration measurement already contains the previous torque. */
        request[i+1]=nominal+c->config.inertia[i]*(alpha_ref-c->alpha[i])
                     -(torque_ff-c->ff_previous[i]);
        c->ff_previous[i]=torque_ff;
    }
    if(!fc_allocate(&c->allocator,request,c->failed_mask,&c->allocation)) return false;
    for(int i=0;i<3;i++)
        c->achieved_alpha[i]=c->allocation.achieved[i+1]/c->config.inertia[i];
    return true;
}
