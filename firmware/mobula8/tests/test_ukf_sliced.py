"""Regression: cooperative UKF must preserve the original numerical updates."""
import pathlib, subprocess, tempfile, unittest
ROOT=pathlib.Path(__file__).resolve().parents[1]
MAIN=r"""
#include "ukf_sliced.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
static void compare(float *a,float *b,unsigned n) {
    for(unsigned i=0;i<n;i++) assert(isfinite(a[i]) && isfinite(b[i]) && fabsf(a[i]-b[i])<2e-5f);
}
int main(void) {
    for(int gated=0;gated<2;gated++) {
        ukf_t a,b;fdUkfWork work={0};ukf_init(&a,.5f,2,0);
        a.x[0]=cosf(.2617994f);a.x[1]=sinf(.2617994f);
        for(int j=7;j<10;j++){a.P[j*STATE_DIM+j]=1e-6f;a.Q[j*STATE_DIM+j]=0;}
        b=a;
        for(int k=0;k<1000;k++) {
            float gyro[3]={.03f*sinf(k*.1f),.02f*cosf(k*.13f),.01f};
            float accel[3]={.01f*sinf(k*.04f),.5f,.8660254f};
            if(gated && k%7==0) accel[0]=50;
            float gate=gated?11.345f:0;
            ukf_predict(&a,gyro,.01f);ukf_update(&a,accel,gyro,gate);
            assert(fdUkfStart(&work,gyro,accel,.01f,gate));
            assert(!fdUkfStart(&work,gyro,accel,.01f,gate));
            int result=0,steps=0;
            while(result==0){result=fdUkfSlice(&b,&work);assert(++steps<=13);}
            assert(result==1 && !work.active);
            compare(a.x,b.x,STATE_DIM);compare(a.P,b.P,STATE_DIM*STATE_DIM);
        }
    }
    ukf_t u;ukf_init(&u,.5f,2,0);fdUkfWork w={0};
    float gyro[3]={0},accel[3]={0,0,1};
    assert(!fdUkfStart(&w,gyro,accel,NAN,0));
    assert(!fdUkfStart(&w,gyro,accel,.1f,0));
    gyro[0]=NAN;assert(!fdUkfStart(&w,gyro,accel,.01f,0));gyro[0]=0;
    u.P[0]=-10;assert(fdUkfStart(&w,gyro,accel,.01f,0));
    assert(fdUkfSlice(&u,&w)==-1 && !w.active);
    // A 6ms impulse entirely between 100Hz sample points must not disappear.
    ukf_t old,integrated;ukf_init(&old,.5f,2,0);integrated=old;
    fdUkfWork wo={0},wi={0};quat_t truth={1,0,0,0},window={1,0,0,0};
    for(int k=1;k<=100;k++){
        float g[3]={0,(k>=2 && k<=7)?34.906585f:0,0};
        quat_t exact;quat_integrate(&truth,g,.001f,&exact);truth=exact;
        assert(fdGyroAdvance(&window,g,.001f));
        if(k%10==0){
            float a[3]={0,0,10}; // Dynamic acceleration must never become gravity.
            assert(fdUkfStart(&wo,g,a,.01f,11.345f));
            assert(fdUkfStartIntegrated(&wi,g,a,.01f,11.345f,&window,true));
            while(wo.active)assert(fdUkfSlice(&old,&wo)>=0);
            while(wi.active)assert(fdUkfSlice(&integrated,&wi)>=0);
            assert(wi.accelRejected);window=(quat_t){1,0,0,0};
        }
    }
    float dot=fabsf(truth.w*integrated.x[0]+truth.y*integrated.x[2]);
    assert(dot>.99999f);assert(fabsf(old.x[2]-truth.y)>.09f);
    quat_t bad={1,0,0,0};float nanRate[3]={NAN,0,0};
    assert(!fdGyroAdvance(&bad,nanRate,.001f));
    float fast[3]={41,0,0};assert(!fdGyroAdvance(&bad,fast,.001f));
    assert(!fdGyroAdvance(&bad,gyro,.003f));
    puts("PASS: integrated gyro captures between-snapshot impulse and rejects shock acceleration");
    puts("PASS: 2000 sliced/reference UKF updates, acceleration rejection, invalid inputs, failed covariance");
}
"""
class SlicedTests(unittest.TestCase):
    def test_equations(self):
        with tempfile.TemporaryDirectory() as folder:
            d=pathlib.Path(folder);(d/"main.c").write_text(MAIN)
            sources=[ROOT/"research/ukf_sliced.c",ROOT/"research/rate_core.c"]+list((ROOT/"research/original").glob("*.c"))
            for flags in (["-O1","-fsanitize=undefined","-fno-sanitize-recover=all"],["-Ofast"]):
                subprocess.run(["gcc","-std=c99",*flags,"-Wall","-Wextra","-Werror","-I"+str(ROOT/"research"),str(d/"main.c"),*map(str,sources),"-lm","-o",str(d/"test")],check=True)
                subprocess.run([str(d/"test")],check=True)
