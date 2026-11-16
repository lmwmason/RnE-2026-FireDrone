import pathlib,subprocess,tempfile,unittest
ROOT=pathlib.Path(__file__).resolve().parents[1]
MAIN=r"""
#include "ukf_level.h"
#include <math.h>
#include <assert.h>
#include <stdio.h>
int main(void){
    float bf[2]={10,-5},ukf[2]={12,-6},out[2];fdLevelState s={0};
    fdLevelSelect(&s,bf,ukf,false,true,true,0,.000312f,out);
    assert(out[0]==bf[0] && out[1]==bf[1] && !s.active);
    for(int i=0;i<4000;i++)fdLevelSelect(&s,bf,ukf,true,true,true,10000,.000312f,out);
    assert(s.active && s.blend==1 && out[0]==ukf[0] && out[1]==ukf[1]);
    // Mode switch/failsafe restores both axes immediately without a fault latch.
    fdLevelSelect(&s,bf,ukf,true,false,true,0,.000312f,out);
    assert(out[0]==bf[0] && out[1]==bf[1] && !s.active && !s.rejected && s.blend==0);
    for(int fault=0;fault<6;fault++){
        s=(fdLevelState){0};ukf[0]=12;bf[0]=10;
        unsigned age=0;bool valid=true;float dt=.000312f;
        if(fault==0)age=20001;
        if(fault==1)dt=0;
        if(fault==2)ukf[0]=NAN;
        if(fault==3)valid=false;
        if(fault==4)dt=.003f;
        if(fault==5)dt=NAN;
        fdLevelSelect(&s,bf,ukf,true,true,valid,age,dt,out);
        assert(s.rejected && !s.active && out[0]==bf[0] && out[1]==bf[1]);
        bf[0]=10;ukf[0]=12;
        for(int i=0;i<4000;i++)fdLevelSelect(&s,bf,ukf,true,true,true,0,.000312f,out);
        assert(!s.active && out[0]==bf[0]);
    }
    // Large disagreement, near vertical attitude and Euler wrap cannot deselect UKF.
    s=(fdLevelState){0};bf[0]=0;ukf[0]=35;
    for(int i=0;i<4000;i++)fdLevelSelect(&s,bf,ukf,true,true,true,10000,.000312f,out);
    assert(s.active && !s.rejected && s.blend==1 && out[0]==35);
    const float cases[][4]={{52,0,61,0},{10,0,62,0},{-179,78,179,81},{0,0,35,-35},{170,85,-170,88}};
    for(unsigned i=0;i<sizeof(cases)/sizeof(cases[0]);i++){
        bf[0]=cases[i][0];bf[1]=cases[i][1];ukf[0]=cases[i][2];ukf[1]=cases[i][3];
        fdLevelSelect(&s,bf,ukf,true,true,true,10000,.000312f,out);
        assert(s.active && !s.rejected && !s.hardFault && s.reason==0);
        assert(fabsf(out[0]-ukf[0])<.001f && fabsf(out[1]-ukf[1])<.001f);
    }
    fdLevelSelect(&s,bf,ukf,true,true,false,0,.000312f,out);
    assert(s.hardFault && s.rejected && s.reason==1);
    puts("PASS: UKF ANGLE transition, exact baseline fallback, stale/nonfinite/interval protection, uninterrupted valid UKF across large attitudes");
}
"""
class LevelTests(unittest.TestCase):
 def test_level_selection(self):
  with tempfile.TemporaryDirectory() as tmp:
   d=pathlib.Path(tmp);(d/"test.c").write_text(MAIN)
   sources=[ROOT/"research/ukf_level.c",ROOT/"research/rate_core.c"]+list((ROOT/"research/original").glob("*.c"))
   for flags in (["-O1","-fsanitize=undefined","-fno-sanitize-recover=all"],["-Ofast"]):
    subprocess.run(["gcc","-std=c99",*flags,"-Wall","-Wextra","-Werror","-I"+str(ROOT/"research"),str(d/"test.c"),*map(str,sources),"-lm","-o",str(d/"test")],check=True)
    subprocess.run([str(d/"test")],check=True)
