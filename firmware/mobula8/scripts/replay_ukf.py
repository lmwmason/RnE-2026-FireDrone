#!/usr/bin/env python3
"""Approximate synchronous observer replay at recorded half-rate; not flight truth."""
import csv,ctypes,json,subprocess,tempfile,argparse
from pathlib import Path
import numpy as np
R=Path(__file__).resolve().parents[1]
C=r"""
#include "ukf_sliced.h"
#include <math.h>
#include <string.h>
int replay(int n,const double *data,double *out){
 ukf_t old,u;ukf_init(&u,.5f,2,0);
 float roll=atan2f(data[5],data[6]),pitch=atan2f(-data[4],sqrtf(data[5]*data[5]+data[6]*data[6]));
 u.x[0]=cosf(roll/2)*cosf(pitch/2);u.x[1]=sinf(roll/2)*cosf(pitch/2);u.x[2]=cosf(roll/2)*sinf(pitch/2);u.x[3]=-sinf(roll/2)*sinf(pitch/2);
 for(int j=0;j<3;j++)u.x[4+j]=data[1+j]*.01745329252f;
 for(int j=7;j<10;j++){u.P[j*STATE_DIM+j]=1e-6f;u.Q[j*STATE_DIM+j]=0;}
 old=u;fdUkfWork wo={0},w={0};quat_t win={1,0,0,0},q={u.x[0],u.x[1],u.x[2],u.x[3]};
 double last=data[0];float hold=0;
 for(int i=0;i<n;i++){
  const double *row=data+7*i;float dt=i?row[0]-data[7*(i-1)]:0;
  float g[3],a[3],an=0,gn=0;for(int j=0;j<3;j++){g[j]=row[1+j]*.01745329252f;a[j]=row[4+j];an+=a[j]*a[j];gn+=g[j]*g[j];}
  if(i && (!fdGyroAdvance(&win,g,dt)||!fdGyroAdvance(&q,g,dt)))return -(i+1);
  if(an<.64f||an>1.44f||gn>64)hold=.15f;else hold=fmaxf(0,hold-dt);
  if(row[0]-last>=.01){
   float span=row[0]-last;last=row[0];
   if(!fdUkfStart(&wo,g,a,span,11.345f)||!fdUkfStartIntegrated(&w,g,a,span,11.345f,&win,hold>0))return -1;
   while(wo.active)if(fdUkfSlice(&old,&wo)<0)return -2;
   while(w.active)if(fdUkfSlice(&u,&w)<0)return -3;
   q=(quat_t){u.x[0],u.x[1],u.x[2],u.x[3]};win=(quat_t){1,0,0,0};
  }
  float x=old.x[0],y=old.x[1],z=old.x[2],v=old.x[3];
  out[5*i]=atan2f(2*(x*y+z*v),1-2*(y*y+z*z))*57.2957795f;
  out[5*i+1]=asinf(fmaxf(-1,fminf(1,2*(x*z-v*y))))*57.2957795f;
  out[5*i+2]=atan2f(2*(q.w*q.x+q.y*q.z),1-2*(q.x*q.x+q.y*q.y))*57.2957795f;
  out[5*i+3]=asinf(fmaxf(-1,fminf(1,2*(q.w*q.y-q.z*q.x))))*57.2957795f;
  out[5*i+4]=w.accelRejected;
 }
 return n;
}
"""
def main():
 ap=argparse.ArgumentParser();ap.add_argument("folder",type=Path);a=ap.parse_args();reports=[]
 with tempfile.TemporaryDirectory() as tmp:
  p=Path(tmp);(p/"replay.c").write_text(C)
  src=[R/"research/ukf_sliced.c",R/"research/rate_core.c",*sorted((R/"research/original").glob("*.c"))]
  subprocess.run(["gcc","-shared","-fPIC","-Ofast","-I"+str(R/"research"),str(p/"replay.c"),*map(str,src),"-lm","-o",str(p/"lib.so")],check=True)
  lib=ctypes.CDLL(str(p/"lib.so"));arr=np.ctypeslib.ndpointer(dtype=np.float64,flags="C_CONTIGUOUS");lib.replay.argtypes=[ctypes.c_int,arr,arr];lib.replay.restype=ctypes.c_int
  for path in sorted(a.folder.glob("flight.0[12].csv")):
   with path.open() as f:names=[v.strip() for v in next(csv.reader(f))];values=np.loadtxt(f,delimiter=",")
   d=dict(zip(names,values.T));t=(d["time (us)"]-d["time (us)"][0])*1e-6
   gaps=np.flatnonzero(np.diff(t)>.002);end=int(gaps[0]+1) if len(gaps) else len(t)
   with path.with_name(path.stem+".headers.csv").open() as f:h=dict(csv.reader(f))
   inputs=np.ascontiguousarray(np.column_stack([t,*[d[f"fdGyro[{j}]"]*.01 for j in range(3)],*[d[f"accSmooth[{j}]"]/float(h["acc_1G"]) for j in range(3)]])[:end]);out=np.zeros((end,5))
   result=lib.replay(end,inputs,out);assert result==end,result
   diff=(out[:,2:4]-out[:,:2]+180)%360-180
   report=dict(file=path.name,samples=end,duration_s=float(t[end-1]),finite=bool(np.all(np.isfinite(out))),max_old_vs_new_deg=np.max(abs(diff),axis=0).tolist(),accel_rejected_fraction=float(np.mean(out[:,4])),warning="Replay starts from measured gravity, not original pre-arm UKF state. Half-rate recorded gyro, synchronous updates, no BF reference or external truth. Differences are not accuracy improvements.")
   np.savez_compressed(path.with_suffix(".replay.npz"),time=t[:end],old_angles=out[:,:2],new_angles=out[:,2:4],rejected=out[:,4]);reports.append(report)
 (a.folder/"integrated-gyro-replay.json").write_text(json.dumps(reports,indent=2)+"\n");print(json.dumps(reports,indent=2))
if __name__=="__main__":main()
