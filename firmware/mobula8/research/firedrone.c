/* SPDX-License-Identifier: GPL-3.0-or-later */
#include "platform.h"
#include <math.h>
#include <string.h>
#include "build/debug.h"
#include "drivers/time.h"
#include "fc/runtime_config.h"
#include "flight/pid.h"
#include "flight/mixer.h"
#include "flight/imu.h"
#include "flight/failsafe.h"
#include "rx/rx.h"
#include "sensors/acceleration.h"
#include "sensors/gyro.h"
#include "research/firedrone.h"
#include "research/rate_core.h"
#include "research/original/ukf.h"
#include "research/ukf_sliced.h"
#include "research/ukf_level.h"

static fdState state;
static ukf_t estimator;
static fdUkfWork ukfWork;
static quat_t gyroWindow={1,0,0,0}, propagated={1,0,0,0};
static float impactHold;
static bool accelRejected, observerTurn;
static uint32_t ukfQuietSince;
static bool ukfReady, ukfPublished, ukfStopped, overBudget;
static uint32_t ukfCompleted;
static int16_t ukfDifference[2];
static float ukfAngles[2];
static fdLevelState levelState;
static uint32_t levelCost;
static uint32_t lastUkf, maximumUs, lastStep;
static int16_t debugValue(float x)
{
    if (!fdFinite(x)) return 0;
    if (x>32767) return 32767;
    if (x<-32768) return -32768;
    return (int16_t)lrintf(x);
}
/* First integrated flight stage: all extra outputs bounded to 0.25 PID units.
 * A configured AUX, full UKF ANGLE entry, and original flight guards are required. */
static bool integratedAllowed(const firedroneConfig_t *p)
{
    return p->ukf==3 && p->mode==2 && p->blend>0 && p->blend<=5 &&
        p->correctionLimit>0 && p->correctionLimit<=5 && p->auxChannel>=1 && p->auxChannel<=14;
}
void firedroneRateLog(int32_t correction[3], int32_t candidate[3])
{
    for(int a=0;a<3;a++){
        correction[a]=debugValue(state.correction[a]*1000);
        candidate[a]=debugValue(state.candidate[a]*10);
    }
}
void firedroneAttitudeLog(int32_t out[6])
{
    bool valid=ukfPublished && ukfReady && !ukfStopped && !overBudget;
    out[0]=valid ? debugValue(ukfAngles[0]*100) : -32768;
    out[1]=valid ? debugValue(ukfAngles[1]*100) : -32768;
    out[2]=attitude.values.roll*10;out[3]=attitude.values.pitch*10;
    uint32_t age=micros()-ukfCompleted;out[4]=valid && age<65535 ? age : 65535;
    out[5]=accelRejected | ((impactHold>0)<<1);
}
bool firedroneLevelAngles(const float baseline[2], float out[2], uint32_t nowUs, float dt, bool permitted)
{
    const uint32_t start=micros();
    const firedroneConfig_t *p=firedroneConfig();
    const bool enabled=p->ukf==3 && ((p->mode<=1 && p->blend==0) || integratedAllowed(p));
    const bool valid=ukfPublished && ukfReady && !ukfStopped && !overBudget;
    fdLevelSelect(&levelState,baseline,ukfAngles,enabled,permitted,valid,nowUs-ukfCompleted,dt,out);
    levelCost=micros()-start;
    return levelState.active;
}
void firedroneUpdate(const float setpointDps[3], float dt, uint32_t nowUs, bool permitted)
{
    const firedroneConfig_t *p=firedroneConfig();
    if (!p->mode && !p->ukf) { fdReset(&state); ukfReady=false; ukfPublished=false; ukfWork.active=false; ukfQuietSince=0; lastUkf=nowUs; return; }
    const uint32_t start=micros();
    // Detect delayed scheduler ticks, including micros() wraparound.
    const uint32_t elapsed=nowUs-lastStep; lastStep=nowUs;
    const bool cadenceOk=elapsed<=dt*1e6f*2.0f;
    // A dedicated AUX switch can immediately return experimental output to baseline.
    const bool auxEnabled=p->auxChannel>=1 && p->auxChannel<=14 && rcData[3+p->auxChannel]>1700;
    if (p->mode==2 && !auxEnabled) permitted=false;
    if (p->ukf==3 && p->mode==2) permitted=permitted && integratedAllowed(p) && levelState.active && levelState.blend>=1;
    fdConfig c={.mode=p->mode,.bandwidth=p->bandwidth,.rate_gain=p->rateGain,
        .motor_tau=p->motorTauMs*.001f,.motor_tau_yaw=p->motorTauYawMs*.001f,.derivative_hz=p->derivativeHz,
        .blend=p->blend*.01f,.correction_limit=p->correctionLimit,.adaptation=p->adaptation*1e-6f};
    fdInput in={.dt=dt,.eligible=permitted && cadenceOk && !overBudget && getMixerMode()==MIXER_QUADX,
        .saturated=getMotorMixRange()>=1.0f};
    for (int a=0;a<3;++a) {
        c.effectiveness[a]=p->effectiveness[a]*.01f;
        in.rate[a]=gyro.gyroADCf[a]*0.01745329252f;
        in.setpoint[a]=setpointDps[a]*0.01745329252f;
        in.baseline[a]=pidData[a].Sum;
    }
    float out[3]; fdStep(&state,&c,&in,out);
    /* All original arming, failsafe, mixer saturation, and gyro-overflow guards remain upstream/downstream. */
    for (int a=0;a<3;++a) pidData[a].Sum=out[a];
    const int a=p->axis<3 ? p->axis : 0;
    DEBUG_SET(DEBUG_FIREDRONE,0,debugValue(setpointDps[a]*10));
    DEBUG_SET(DEBUG_FIREDRONE,1,debugValue(gyro.gyroADCf[a]*10));
    DEBUG_SET(DEBUG_FIREDRONE,2,debugValue(in.baseline[a]*10));
    DEBUG_SET(DEBUG_FIREDRONE,3,debugValue(state.candidate[a]*10));
    DEBUG_SET(DEBUG_FIREDRONE_OBSERVER,0,debugValue(state.disturbance[a]*10));
    DEBUG_SET(DEBUG_FIREDRONE_OBSERVER,1,debugValue(state.correction[a]*10));
    DEBUG_SET(DEBUG_FIREDRONE_OBSERVER,2,state.valid);
    DEBUG_SET(DEBUG_FIREDRONE_OBSERVER,3,overBudget);

    /* ukf=1: bench; ukf=2: flight observation; ukf=3: guarded ANGLE input.
     * Initialize only on the ground; an armed fault stops observation until reboot. */
#ifdef USE_ACC
    if (p->ukf>=2 && ARMING_FLAG(ARMED) &&
        (!cadenceOk || !ukfReady || (ukfPublished && (uint32_t)(nowUs-ukfCompleted)>30000))) ukfStopped=true;
    if (!p->ukf || p->ukf>3 || (p->ukf==1 && ARMING_FLAG(ARMED)) ||
        (p->ukf>=2 && p->mode>1 && !integratedAllowed(p)) || ukfStopped || overBudget) {
        ukfWork.active=false;ukfReady=false;ukfQuietSince=0;
    } else if (acc.isAccelUpdatedAtLeastOnce && acc.dev.acc_1G_rec>0) {
        float accel[3];bool valid=true;
        for(int j=0;j<3;j++) {accel[j]=acc.accADC[j]*acc.dev.acc_1G_rec;valid=valid && fdFinite(accel[j]) && fdFinite(in.rate[j]);}
        if (!valid) {ukfReady=false;ukfWork.active=false;ukfQuietSince=0;}
        else if (!ukfReady && !ARMING_FLAG(ARMED)) {
            // Do not seed from BF's startup quaternion before its attitude has settled.
            const float an=sqrtf(accel[0]*accel[0]+accel[1]*accel[1]+accel[2]*accel[2]);
            const float gn=sqrtf(in.rate[0]*in.rate[0]+in.rate[1]*in.rate[1]+in.rate[2]*in.rate[2]);
            const float ar=atan2f(accel[1],accel[2])*572.957795f;
            const float ap=atan2f(-accel[0],sqrtf(accel[1]*accel[1]+accel[2]*accel[2]))*572.957795f;
            float rollError=ar-attitude.values.roll;
            if (rollError>1800) rollError-=3600;
            if (rollError<-1800) rollError+=3600;
            const bool settled=gn<.1f && fabsf(an-1)<.12f && fabsf(rollError)<30 && fabsf(ap-attitude.values.pitch)<30;
            if (!settled) ukfQuietSince=0;
            else if (!ukfQuietSince) ukfQuietSince=nowUs;
            else if ((uint32_t)(nowUs-ukfQuietSince)>=1000000) {
                ukf_init(&estimator,.5f,2,0);
                quaternion q;getQuaternion(&q);
                estimator.x[0]=q.w;estimator.x[1]=q.x;estimator.x[2]=q.y;estimator.x[3]=q.z;
                for(int j=0;j<3;j++) estimator.x[4+j]=in.rate[j];
                // Existing per-aircraft accelerometer calibration is the bias reference.
                // Avoid absorbing initial gravity tilt into unobservable accel bias states.
                for(int j=7;j<10;j++) {estimator.P[j*STATE_DIM+j]=1e-6f;estimator.Q[j*STATE_DIM+j]=0;}
                ukfReady=true;ukfWork.active=false;lastUkf=nowUs;
                gyroWindow=(quat_t){1,0,0,0};propagated=(quat_t){q.w,q.x,q.y,q.z};impactHold=0;accelRejected=false;observerTurn=false;
            }
        } else if (ukfReady) {
            /* Preserve every gyro increment between 100 Hz observer snapshots. */
            if(!fdGyroAdvance(&gyroWindow,in.rate,elapsed*1e-6f) || !fdGyroAdvance(&propagated,in.rate,elapsed*1e-6f)) {
                ukfReady=false;ukfWork.active=false;ukfQuietSince=0;
            } else {
                float an2=0,gn2=0;
                for(int j=0;j<3;j++){an2+=accel[j]*accel[j];gn2+=in.rate[j]*in.rate[j];}
                if(an2<.64f || an2>1.44f || gn2>64)impactHold=.15f;
                else impactHold=fmaxf(0,impactHold-dt);
                if (!ukfWork.active && (uint32_t)(nowUs-lastUkf)>=10000) {
                    const float ukfDt=(uint32_t)(nowUs-lastUkf)*1e-6f;
                    lastUkf=nowUs;
                    if (!fdUkfStartIntegrated(&ukfWork,in.rate,accel,ukfDt,11.345f,&gyroWindow,impactHold>0)) {ukfReady=false;ukfQuietSince=0;}
                    gyroWindow=(quat_t){1,0,0,0};
                }
                /* At 3.2kHz separate heavy UKF slices from Euler conversion.
                 * 13 slices finish in about8.1ms, while output angles refresh
                 * every0.624ms from the continuously propagated quaternion. */
                const bool angleTick=dt>=.0005f || (observerTurn=!observerTurn);
                if (ukfWork.active && (dt>=.0005f || !angleTick)) {
                    const int result=fdUkfSlice(&estimator,&ukfWork);
                    if(result<0) {ukfReady=false;ukfQuietSince=0;}
                    else if(result==1) {
                        ukfPublished=true;ukfCompleted=nowUs;accelRejected=ukfWork.accelRejected;
                        quat_t committed={estimator.x[0],estimator.x[1],estimator.x[2],estimator.x[3]};
                        /* The committed estimate is timestamped at snapshot start. */
                        quat_mul(&committed,&gyroWindow,&propagated);quat_norm(&propagated);
                    }
                }
                if(ukfPublished && ukfReady && angleTick){
                    const quat_t *q=&propagated;
                    float pitch=2*(q->w*q->y-q->z*q->x);pitch=fmaxf(-1,fminf(1,pitch));
                    const int16_t roll=debugValue(atan2f(2*(q->w*q->x+q->y*q->z),1-2*(q->x*q->x+q->y*q->y))*572.957795f);
                    const int16_t pitchAngle=debugValue(asinf(pitch)*572.957795f);
                    ukfAngles[0]=roll*.1f;ukfAngles[1]=pitchAngle*.1f;
                    ukfDifference[0]=roll-attitude.values.roll;
                    if(ukfDifference[0]>1800) ukfDifference[0]-=3600;
                    if(ukfDifference[0]<-1800) ukfDifference[0]+=3600;
                    ukfDifference[1]=pitchAngle-attitude.values.pitch;
                    DEBUG_SET(DEBUG_FIREDRONE_UKF,0,roll);DEBUG_SET(DEBUG_FIREDRONE_UKF,1,pitchAngle);
                    DEBUG_SET(DEBUG_FIREDRONE_UKF,2,attitude.values.roll);DEBUG_SET(DEBUG_FIREDRONE_UKF,3,attitude.values.pitch);
                }
            }
        }
    } else {ukfReady=false;ukfWork.active=false;ukfQuietSince=0;}
#endif

    const uint32_t cost=micros()-start+levelCost;levelCost=0;
    if(cost>maximumUs) maximumUs=cost;
    /* Latch research off on a rate-loop budget violation, preserving baseline PID. */
    if (ARMING_FLAG(ARMED) && cost>dt*1e6f*.35f) {
        overBudget=true; fdReset(&state);
        for(int j=0;j<3;++j) pidData[j].Sum=in.baseline[j];
        DEBUG_SET(DEBUG_FIREDRONE_OBSERVER,1,0);
        DEBUG_SET(DEBUG_FIREDRONE_OBSERVER,2,0);
        DEBUG_SET(DEBUG_FIREDRONE_OBSERVER,3,1);
    }
    if (!ukfReady || overBudget || ukfStopped) {
        ukfPublished=false;
        if (p->ukf>=2 && ARMING_FLAG(ARMED)) ukfStopped=true;
        DEBUG_SET(DEBUG_FIREDRONE_UKF,0,-32768);
        DEBUG_SET(DEBUG_FIREDRONE_UKF,1,-32768);
    }
    const bool ukfValid=ukfPublished && ukfReady && !ukfStopped && !overBudget && p->ukf &&
        (p->ukf>=2 || !ARMING_FLAG(ARMED));
    const unsigned ukfFlags=overBudget | (ukfValid<<1) | (ukfStopped<<2) | (levelState.active<<3) | (levelState.rejected<<4);
    DEBUG_SET(DEBUG_FIREDRONE_UKF_CONTROL,0,debugValue(levelState.applied[0]*10));
    DEBUG_SET(DEBUG_FIREDRONE_UKF_CONTROL,1,debugValue(levelState.applied[1]*10));
    DEBUG_SET(DEBUG_FIREDRONE_UKF_CONTROL,2,debugValue(maximumUs));
    bool correcting=false,clipped=false;
    for(int j=0;j<3;j++){
        correcting=correcting || state.correction[j]!=0;
        clipped=clipped || (state.valid && fabsf(state.candidate[j]-in.baseline[j])>c.correction_limit);
    }
    DEBUG_SET(DEBUG_FIREDRONE_UKF_CONTROL,3,ukfFlags | ((levelState.blend>=1)<<5) | (levelState.hardFault<<6) | (levelState.reason<<7) | (state.valid<<10) | (correcting<<11) | (clipped<<12));
    DEBUG_SET(DEBUG_FIREDRONE_UKF_FLIGHT,0,ukfValid ? ukfDifference[0] : -32768);
    DEBUG_SET(DEBUG_FIREDRONE_UKF_FLIGHT,1,ukfValid ? ukfDifference[1] : -32768);
    DEBUG_SET(DEBUG_FIREDRONE_UKF_FLIGHT,2,debugValue(maximumUs));
    DEBUG_SET(DEBUG_FIREDRONE_UKF_FLIGHT,3,ukfFlags);
    DEBUG_SET(DEBUG_FIREDRONE_TIMING,0,debugValue(cost));
    DEBUG_SET(DEBUG_FIREDRONE_TIMING,1,debugValue(maximumUs));
    DEBUG_SET(DEBUG_FIREDRONE_TIMING,2,debugValue(dt*1e6f));
    DEBUG_SET(DEBUG_FIREDRONE_TIMING,3,ukfFlags);
}
