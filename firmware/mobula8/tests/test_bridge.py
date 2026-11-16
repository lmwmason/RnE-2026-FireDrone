"""Compile the actual BF adapter against narrow hardware stubs; exercise output guards."""
import pathlib,subprocess,tempfile,unittest
ROOT=pathlib.Path(__file__).resolve().parents[1]
STUB=r"""
#pragma once
#include <stdint.h>
#include <stdbool.h>
#define USE_ACC
#define PG_DECLARE(t,n) const t *n(void)
#define ARMED 1
extern bool testArmed;
#define ARMING_FLAG(x) (testArmed)
#define MIXER_QUADX 3
#define DEBUG_FIREDRONE 1
#define DEBUG_FIREDRONE_UKF 2
#define DEBUG_FIREDRONE_TIMING 3
#define DEBUG_FIREDRONE_OBSERVER 4
#define DEBUG_FIREDRONE_UKF_FLIGHT 5
#define DEBUG_FIREDRONE_UKF_CONTROL 6
extern int16_t logs[7][4];
#define DEBUG_SET(m,i,v) (logs[m][i]=(v))
typedef struct {float Sum;} pidTest;
extern pidTest pidData[3];
extern struct gyroTest {float gyroADCf[3];} gyro;
extern struct accTest {float accADC[3]; bool isAccelUpdatedAtLeastOnce; struct {float acc_1G_rec;} dev;} acc;
extern struct attTest {struct {int16_t roll,pitch;} values;} attitude;
typedef struct {float w,x,y,z;} quaternion;
extern float rcData[18];
uint32_t micros(void);
int getMixerMode(void);
float getMotorMixRange(void);
void getQuaternion(quaternion *q);
"""
MAIN=r"""
#include "stub.h"
#include "research/firedrone.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
bool testArmed;
int16_t logs[7][4];
pidTest pidData[3];
struct gyroTest gyro;
struct accTest acc;
struct attTest attitude;
float rcData[18];
static uint32_t clockUs,cost=1;
static float mixRange;
static firedroneConfig_t cfg={.mode=2,.blend=100,.effectiveness={100,100,100},.bandwidth=20,
    .rateGain=15,.motorTauMs=25,.derivativeHz=30,.correctionLimit=25,.auxChannel=1};
const firedroneConfig_t *firedroneConfig(void){return &cfg;}
uint32_t micros(void){clockUs+=cost;return clockUs;}
int getMixerMode(void){return MIXER_QUADX;}
float getMotorMixRange(void){return mixRange;}
void getQuaternion(quaternion*q){*q=(quaternion){.965925826f,.258819045f,0,0};}
static uint32_t now,stepUs=1000;
static void step(bool permitted){
    float sp[3]={30,0,0};now+=stepUs;
    for(int a=0;a<3;++a)pidData[a].Sum=10;
    firedroneUpdate(sp,stepUs*1e-6f,now,permitted);
}
int main(int argc,char **argv){
    if(argc>1 && (!strcmp(argv[1],"level") || !strcmp(argv[1],"integrated") || !strcmp(argv[1],"shock") || !strcmp(argv[1],"fast") || !strcmp(argv[1],"rearm"))){
        cfg.mode=0;cfg.blend=0;cfg.ukf=3;
        acc.isAccelUpdatedAtLeastOnce=true;acc.dev.acc_1G_rec=1;
        acc.accADC[1]=.5f;acc.accADC[2]=.8660254f;attitude.values.roll=300;
        assert(!firedroneArmingReady());
        for(int i=0;i<1200;i++)step(false);
        clockUs=now;assert(firedroneArmingReady());
        testArmed=true;float base[2]={28,0},selected[2];bool active=false;
        for(int i=0;i<1500;i++){
            active=firedroneLevelAngles(base,selected,now,.001f,true);step(true);
            for(int a=0;a<3;a++)assert(pidData[a].Sum==10);
        }
        assert(active && selected[0]>29 && selected[0]<31);
        if(!strcmp(argv[1],"rearm")){
            clockUs=now;assert(!firedroneArmingReady());
            // Large disagreement remains selected, including after repeated impacts.
            base[0]=-50;
            assert(firedroneLevelAngles(base,selected,now,.001f,true));
            assert(selected[0]>29 && selected[0]<31);
            testArmed=false;step(false);clockUs=now;
            assert(!firedroneArmingReady());
            for(int i=0;i<900;i++)step(false);
            clockUs=now;assert(!firedroneArmingReady());
            for(int i=0;i<300;i++)step(false);
            clockUs=now;assert(firedroneArmingReady());
            testArmed=true;
            for(int i=0;i<1500;i++){
                firedroneLevelAngles(base,selected,now,.001f,true);step(true);
            }
            assert(selected[0]>29 && selected[0]<31);
            assert(!(logs[DEBUG_FIREDRONE_UKF_CONTROL][3]&(16|64|4|1)));
            puts("PASS: normal disarm requires quiet UKF initialization before next arming");return 0;
        }
        if(!strcmp(argv[1],"fast")){
            stepUs=312;
            for(int i=0;i<12000;i++){
                assert(firedroneLevelAngles(base,selected,now,.000312f,true));step(true);
                assert(!(logs[DEBUG_FIREDRONE_UKF_CONTROL][3]&(16|64|4|1)));
                clockUs=now;int32_t obs[6];firedroneAttitudeLog(obs);
                assert(obs[4]<20000 && abs(obs[0]-3000)<100);
            }
            puts("PASS: interleaved3.2kHz observer stays fresh and active");return 0;
        }
        if(!strcmp(argv[1],"shock")){
            float truth=30;
            for(int i=0;i<400;i++){
                gyro.gyroADCf[0]=(i>=2 && i<=7)?2000:0;
                truth+=gyro.gyroADCf[0]*.001f;
                attitude.values.roll=(int16_t)lrintf(truth*10);base[0]=truth;
                float g=(i>=2 && i<=7)?10:1;
                acc.accADC[1]=g*sinf(truth*.01745329252f);acc.accADC[2]=g*cosf(truth*.01745329252f);
                assert(firedroneLevelAngles(base,selected,now,.001f,true));step(true);
                assert(fabsf(selected[0]-truth)<3);
                assert(!(logs[DEBUG_FIREDRONE_UKF_CONTROL][3]&(16|64|4|1)));
            }
            int32_t obs[6];clockUs=now;firedroneAttitudeLog(obs);
            assert(abs(obs[0]-4200)<100 && obs[2]==4200 && obs[4]<20000);
            puts("PASS: real adapter retains short gyro impulse through sliced update and propagation");return 0;
        }
        cfg.mode=1; // All three rate algorithms in shadow must coexist with UKF ANGLE.
        for(int i=0;i<1000;i++){
            active=firedroneLevelAngles(base,selected,now,.001f,true);step(true);
            for(int a=0;a<3;a++)assert(pidData[a].Sum==10);
        }
        assert(active && selected[0]>29 && selected[0]<31);
        assert((logs[DEBUG_FIREDRONE_UKF_CONTROL][3]&42)==42);
        if(!strcmp(argv[1],"integrated")){
            cfg.mode=2;cfg.blend=5;cfg.correctionLimit=5;cfg.auxChannel=1;cfg.motorTauYawMs=5;rcData[4]=1900;
            bool corrected=false;
            for(int i=0;i<1800;i++){
                assert(firedroneLevelAngles(base,selected,now,.001f,true));step(true);
                for(int a=0;a<3;a++){
                    assert(fabsf(pidData[a].Sum-10)<=.25001f);
                    corrected=corrected || fabsf(pidData[a].Sum-10)>.01f;
                }
            }
            assert(corrected && (logs[DEBUG_FIREDRONE_UKF_CONTROL][3]&42)==42);
            int32_t corr[3],candidate[3];firedroneRateLog(corr,candidate);
            for(int a=0;a<3;a++)assert(abs(corr[a])<=250);
            rcData[4]=1000;firedroneLevelAngles(base,selected,now,.001f,true);step(true);
            for(int a=0;a<3;a++)assert(pidData[a].Sum==10);
            rcData[4]=1900;
            assert(!firedroneLevelAngles(base,selected,now,.001f,false));step(true);
            for(int a=0;a<3;a++)assert(pidData[a].Sum==10);
            // An accidentally excessive blend must not enable unreviewed combined output.
            cfg.blend=6;firedroneLevelAngles(base,selected,now,.001f,true);step(true);
            for(int a=0;a<3;a++)assert(pidData[a].Sum==10);
            puts("PASS: actual UKF plus bounded rate control, AUX exit, UKF exit, invalid blend, correction logging");return 0;
        }
        assert(!firedroneLevelAngles(base,selected,now,.001f,false));
        assert(selected[0]==28);
        assert(!firedroneLevelAngles(base,selected,now+40000,.001f,true));
        assert(selected[0]==28);
        puts("PASS: actual adapter uses published UKF for level input; mode exit/stale fallback");return 0;
    }
    if(argc>1){
        cfg.mode=0;cfg.ukf=2;
        acc.isAccelUpdatedAtLeastOnce=true;acc.dev.acc_1G_rec=1;
        acc.accADC[1]=.5f;acc.accADC[2]=.8660254f;attitude.values.roll=300;
        if(strcmp(argv[1],"early")!=0){
            for(int i=0;i<1200;i++)step(false);
            assert(logs[DEBUG_FIREDRONE_TIMING][3]&2);
        }
        testArmed=true;
        if(!strcmp(argv[1],"cadence"))now+=5000;
        if(!strcmp(argv[1],"budget"))cost=400;
        step(true);cost=1;
        for(int i=0;i<100;i++)step(true);
        assert(!(logs[DEBUG_FIREDRONE_UKF_FLIGHT][3]&2));
        assert(logs[DEBUG_FIREDRONE_UKF_FLIGHT][3]&4);
        assert(logs[DEBUG_FIREDRONE_UKF_FLIGHT][0]==-32768);
        for(int a=0;a<3;a++)assert(pidData[a].Sum==10);
        puts("PASS: armed observer startup/cadence/budget rejection preserves PID");return 0;
    }
    testArmed=true;
    // no experiment AUX => unchanged
    for(int i=0;i<600;++i)step(true);
    assert(pidData[0].Sum==10);
    rcData[4]=1900;
    for(int i=0;i<600;++i)step(true);
    assert(fabsf(pidData[0].Sum-10)>1 && fabsf(pidData[0].Sum-10)<=25);
    mixRange=1.1f;step(true);assert(pidData[0].Sum==10);
    mixRange=0;step(false);assert(pidData[0].Sum==10);
    for(int i=0;i<600;++i)step(true);
    rcData[4]=1000;step(true);assert(pidData[0].Sum==10);
    // bench UKF is paused whenever armed
    cfg.ukf=1;cfg.mode=0;testArmed=false;
    acc.isAccelUpdatedAtLeastOnce=true;acc.dev.acc_1G_rec=1;acc.accADC[1]=.5f;acc.accADC[2]=.8660254f;attitude.values.roll=300;
    for(int i=0;i<500;++i)step(false);
    assert((logs[DEBUG_FIREDRONE_TIMING][3]&2)==0);
    for(int i=0;i<700;++i)step(false);
    assert(abs(logs[DEBUG_FIREDRONE_UKF][0]-300)<10);
    assert(pidData[0].Sum==10);
    assert((logs[DEBUG_FIREDRONE_TIMING][3]&2)==2);
    testArmed=true;step(false);assert((logs[DEBUG_FIREDRONE_TIMING][3]&2)==0);
    // Flight observation must preserve every PID axis while continuing the UKF.
    cfg.ukf=2;testArmed=false;
    for(int i=0;i<1200;++i)step(false);
    assert((logs[DEBUG_FIREDRONE_TIMING][3]&2)==2);
    testArmed=true;
    for(int i=0;i<500;++i){step(true);for(int a=0;a<3;a++)assert(pidData[a].Sum==10);}
    assert((logs[DEBUG_FIREDRONE_UKF_FLIGHT][3]&2)==2);
    assert(abs(logs[DEBUG_FIREDRONE_UKF_FLIGHT][0])<10);
    // Invalid sensor input must stop flight observation and mark stale data invalid.
    acc.accADC[0]=NAN;step(true);acc.accADC[0]=0;
    assert(logs[DEBUG_FIREDRONE_UKF_FLIGHT][0]==-32768);
    assert(logs[DEBUG_FIREDRONE_UKF_FLIGHT][3]&4);
    for(int i=0;i<1200;++i)step(true);
    assert(!(logs[DEBUG_FIREDRONE_UKF_FLIGHT][3]&2));
    testArmed=false;for(int i=0;i<1200;++i)step(false);
    assert(!(logs[DEBUG_FIREDRONE_UKF_FLIGHT][3]&2));
    testArmed=true;
    cfg.mode=2;cfg.ukf=0;rcData[4]=1900;
    for(int i=0;i<600;++i)step(true);
    cost=400;step(true);assert(pidData[0].Sum==10);
    assert(logs[DEBUG_FIREDRONE_OBSERVER][1]==0 && logs[DEBUG_FIREDRONE_OBSERVER][3]==1);
    cost=1;for(int i=0;i<600;++i)step(true);assert(pidData[0].Sum==10);
    puts("PASS: actual adapter AUX/fallback/saturation/UKF armed exclusion/budget latch");
}
"""
class BridgeTests(unittest.TestCase):
    def test_actual_adapter(self):
        with tempfile.TemporaryDirectory() as folder:
            d=pathlib.Path(folder);(d/"stub.h").write_text(STUB)
            includes=["platform.h","pg/pg.h","build/debug.h","drivers/time.h","fc/runtime_config.h",
                "flight/pid.h","flight/mixer.h","flight/imu.h","flight/failsafe.h","rx/rx.h",
                "sensors/acceleration.h","sensors/gyro.h"]
            for h in includes:
                p=d/h;p.parent.mkdir(parents=True,exist_ok=True);p.write_text("#include <stub.h>\n")
            (d/"main.c").write_text(MAIN)
            sources=[ROOT/"research/firedrone.c",ROOT/"research/rate_core.c",ROOT/"research/ukf_sliced.c",ROOT/"research/ukf_level.c"]+list((ROOT/"research/original").glob("*.c"))
            subprocess.run(["gcc","-std=c99","-Ofast","-Wall","-Wextra","-Werror",
                "-I"+str(d),"-I"+str(ROOT),str(d/"main.c"),*map(str,sources),"-lm","-o",str(d/"test")],check=True)
            for case in [[],["early"],["cadence"],["budget"],["level"],["integrated"],["shock"],["fast"],["rearm"]]:
                subprocess.run([str(d/"test"),*case],check=True)
