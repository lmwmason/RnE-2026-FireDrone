#include "rate_core.h"
#include "original/ukf.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static fdConfig config(void) {
    return (fdConfig){.mode=2,.effectiveness={1,1,1},.bandwidth=20,.rate_gain=15,
        .motor_tau=.025f,.derivative_hz=30,.blend=.25f,.correction_limit=25,.adaptation=.001f};
}
int main(void) {
    fdState s={0}; fdConfig c=config();
    fdInput in={.dt=.001f,.eligible=true,.baseline={10,-20,30},.setpoint={1,-1,.5f}};
    float out[3];
    c.mode=0; fdStep(&s,&c,&in,out); assert(!memcmp(out,in.baseline,sizeof out));
    c.mode=1;
    for(int i=0;i<2000;++i) { fdStep(&s,&c,&in,out); assert(!memcmp(out,in.baseline,sizeof out)); }
    assert(s.valid); c.mode=2;
    for(int i=0;i<1000;++i) {
        fdStep(&s,&c,&in,out);
        for(int a=0;a<3;++a) assert(fabsf(out[a]-in.baseline[a])<=6.2501f);
    }
    in.saturated=true; fdStep(&s,&c,&in,out); assert(!memcmp(out,in.baseline,sizeof out));
    in.saturated=false; in.eligible=false; fdStep(&s,&c,&in,out);
    assert(!s.initialized && !memcmp(out,in.baseline,sizeof out));
    in.eligible=true; c.effectiveness[2]=0; fdStep(&s,&c,&in,out);
    assert(!s.initialized && !memcmp(out,in.baseline,sizeof out));
    c=config(); in.dt=.1f; fdStep(&s,&c,&in,out); assert(!s.initialized);
    in.dt=.001f; uint32_t nan=0x7fc00000; memcpy(&in.rate[1],&nan,4);
    assert(!fdFinite(in.rate[1])); fdStep(&s,&c,&in,out); assert(!s.initialized);
    in.rate[1]=0; fdStep(&s,&c,&in,out); assert(s.initialized);
    memcpy(&s.adrc.axis[2].z2,&nan,4); fdStep(&s,&c,&in,out);
    assert(!s.initialized && !memcmp(out,in.baseline,sizeof out));
    /* Closed-loop rate response on a separate linear plant with lag and disturbance.
       This verifies equations/signs only, NOT a calibrated Pavo flight model. */
    fdReset(&s); memset(&in,0,sizeof in); in.dt=.001f; in.eligible=true; c=config();
    float motor[3]={0};
    for(int i=0;i<6000;++i) {
        for(int a=0;a<3;++a) {
            in.setpoint[a]=i>500 ? (a==1 ? -1 : 1) : 0;
            in.baseline[a]=20*(in.setpoint[a]-in.rate[a]);
        }
        fdStep(&s,&c,&in,out);
        for(int a=0;a<3;++a) {
            motor[a]+=.001f/.026f*(out[a]-motor[a]);
            in.rate[a]+=.001f*(motor[a]+(i>2000 ? .5f : 0));
            assert(fdFinite(in.rate[a]) && fabsf(in.rate[a])<2);
        }
    }
    for(int a=0;a<3;++a) assert(fabsf(in.rate[a]-in.setpoint[a])<.1f);
    /* Controller retuning must reset observer coefficients and warmup, not retain stale gains. */
    c=config();fdReset(&s);in=(fdInput){.dt=.001f,.eligible=true,.baseline={1,1,1}};
    for(int i=0;i<600;i++)fdStep(&s,&c,&in,out);
    c.bandwidth=30;fdStep(&s,&c,&in,out);
    assert(s.adrc.axis[0].beta1==60 && s.warmup<.01f);
    assert(!memcmp(out,in.baseline,sizeof out));
    in.dt=.000312f;fdStep(&s,&c,&in,out);assert(s.cachedDt==in.dt);
    // Matching actuator low pass and normalized adaptation remain bounded across plant variants.
    const float gains[]={.2f,1,5},taus[]={.01f,.025f,.06f};
    for(int g=0;g<3;g++)for(int lag=0;lag<3;lag++){
        c=config();c.motor_tau=taus[lag];
        for(int a=0;a<3;a++)c.effectiveness[a]=gains[g];
        fdReset(&s);in=(fdInput){.dt=.001f,.eligible=true};float plant[3]={0};
        for(int i=0;i<5000;i++){
            for(int a=0;a<3;a++){
                in.setpoint[a]=i>500?(a==1?-.5f:.5f):0;
                in.baseline[a]=20/gains[g]*(in.setpoint[a]-in.rate[a]);
            }
            fdStep(&s,&c,&in,out);
            for(int a=0;a<3;a++){
                assert(fabsf(out[a]-in.baseline[a])<=6.251f);
                plant[a]+=.001f/(.001f+taus[lag])*(out[a]-plant[a]);
                in.rate[a]+=.001f*(gains[g]*plant[a]+(i>2000?.2f:0));
                assert(fdFinite(in.rate[a]) && fabsf(in.rate[a])<3);
            }
        }
        in.saturated=true;
        for(int i=0;i<100;i++)fdStep(&s,&c,&in,out);
        assert(!s.initialized && !s.valid && !memcmp(out,in.baseline,sizeof out));
        in.saturated=false;
        for(int i=0;i<400;i++){fdStep(&s,&c,&in,out);assert(!memcmp(out,in.baseline,sizeof out));}
        assert(s.warmup<.5f);
    }
    // Per-axis lag override inherits common value when zero, resets on retuning.
    c=config();c.motor_tau=.04f;c.motor_tau_yaw=.005f;fdReset(&s);
    in=(fdInput){.dt=.000312f,.eligible=true};fdStep(&s,&c,&in,out);
    assert(s.lagK[2]>s.lagK[0]*5 && s.lagK[0]==s.lagK[1]);
    c.motor_tau_yaw=0;fdStep(&s,&c,&in,out);assert(s.lagK[2]==s.lagK[0]);
    c.motor_tau_yaw=.0005f;assert(!fdConfigValid(&c));
    // Limited integration on uncertain local plants: this checks bounds, not flight qualification.
    for(int ratio=0;ratio<3;ratio++)for(int delay=0;delay<3;delay++)for(int dyn=0;dyn<3;dyn++){
        const float nominal[3]={2.54f,2.19f,.72f};
        const float scale[3]={.5f,1,1.5f},lag[3]={.01f,.04f,.06f},stateA[3]={-3,0,6};
        c=config();c.blend=.05f;c.correction_limit=5;c.motor_tau=.04f;c.motor_tau_yaw=.005f;
        for(int a=0;a<3;a++)c.effectiveness[a]=nominal[a];
        fdReset(&s);in=(fdInput){.dt=.001f,.eligible=true};float actuator[3]={0};
        for(int i=0;i<8000;i++){
            for(int a=0;a<3;a++){
                in.setpoint[a]=i>1000 ? .3f : 0;
                in.baseline[a]=20/nominal[a]*(in.setpoint[a]-in.rate[a]);
            }
            fdStep(&s,&c,&in,out);
            for(int a=0;a<3;a++){
                assert(fabsf(out[a]-in.baseline[a])<=.25001f);
                actuator[a]+=.001f/(.001f+lag[delay])*(out[a]-actuator[a]);
                in.rate[a]+=.001f*(nominal[a]*scale[ratio]*actuator[a]+stateA[dyn]*in.rate[a]);
                assert(fdFinite(in.rate[a]) && fabsf(in.rate[a])<3);
            }
        }
    }
    /* Original UKF uses g and rad/s. Check rest + known positive roll. */
    ukf_t u; ukf_init(&u,.5f,2,0); float g[3]={0},acc[3]={0,0,1};
    for(int i=0;i<100;++i) { ukf_predict(&u,g,.01f); ukf_update(&u,acc,g,11.345f); }
    assert(fabsf(u.x[0]-1)<.01f);
    g[0]=.3f;
    for(int i=0;i<100;++i) {
        const float angle=.3f*(i+1)*.01f; acc[1]=sinf(angle); acc[2]=cosf(angle);
        ukf_predict(&u,g,.01f); ukf_update(&u,acc,g,11.345f);
    }
    assert(u.x[1]>.12f && u.x[1]<.18f);
    puts("PASS: baseline isolation, output bounds, guards, saturation, finite recovery, rate plant, UKF units/sign");
}
