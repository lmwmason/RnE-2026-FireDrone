#!/usr/bin/env python3
"""Exploratory RPM-to-angular-acceleration model. Training-only delay selection.
Does not produce flash commands or assert flight qualification.
"""
import argparse,csv,json,re
from pathlib import Path
import numpy as np
from scipy.signal import lfilter
from review_identification import r2

MIX=np.array([[-1,1,-1],[-1,-1,1],[1,1,1],[1,-1,-1]],dtype=float)
def low(v,dt,hz):
    k=dt/(dt+1/(2*np.pi*hz));v=np.asarray(v)
    if v.ndim==1:return lfilter([k],[1,k-1],v,zi=[(1-k)*v[0]])[0]
    return np.column_stack([low(v[:,i],dt,hz) for i in range(v.shape[1])])

def read(path):
    with path.open() as f:
        n=[v.strip() for v in next(csv.reader(f))];a=np.loadtxt(f,delimiter=",",ndmin=2)
    with path.with_name(path.stem+".headers.csv").open() as f:header=dict(csv.reader(f))
    return dict(zip(n,a.T)),header

def review(path):
    d,h=read(path);t=(d["time (us)"]-d["time (us)"][0])*1e-6
    if "fdErpm[0]" not in d:raise ValueError("Extended RPM recording required")
    rpm=np.column_stack([d[f"fdErpm[{i}]"] for i in range(4)])*200/int(h["motor_poles"])
    mask=(t>=2)&(t<=t[-1]-2)&((d["flightModeFlags"].astype(int)&1)!=0)&(d["failsafePhase"]==0)&(d["rxFlightChannelsValid"]!=0)&(d["fdTelemetryMask"]==15)&np.all(rpm>0,axis=1)
    for j in range(4):mask &= (d[f"fdErpmAge[{j}]"]<=100)&(d[f"motor[{j}]"]<2047)
    ii=np.flatnonzero(mask);groups=np.split(ii,np.flatnonzero((np.diff(ii)>1)|(np.diff(t[ii])>.002))+1)
    groups=[g for g in groups if len(g)>1 and t[g[-1]]-t[g[0]]>=6]
    result={"file":path.name,"duration_s":float(t[-1]),"eligible_fraction":float(mask.mean()),"intervals":[]}
    for g in groups:
        dt=.002;grid=np.arange(t[g[0]],t[g[-1]],dt)
        def interp(v):return np.interp(grid,t[g],v[g])
        R=np.column_stack([interp(rpm[:,j]) for j in range(4)])/1000
        W=np.column_stack([interp(d[f"fdGyro[{j}]"])*np.pi/18000 for j in range(3)])
        U=np.column_stack([interp(d[f"motor[{j}]"]) for j in range(4)])
        split=int(len(grid)*.65);segment={"start_s":float(grid[0]),"end_s":float(grid[-1]),"motor_command_rpm_correlations":np.corrcoef(U.T,R.T)[:4,4:].tolist(),"axes":{}}
        torque=(R*R)@MIX
        for j,name in enumerate(["roll","pitch","yaw"]):
            candidates=[]
            for hz in [10,30]:
                y=np.gradient(low(W[:,j],dt,hz),dt)
                coupling=W[:,(j+1)%3]*W[:,(j+2)%3]
                for delay_ms in [0,2,4,6,8,10,15,20]:
                    delayed=np.interp(grid-delay_ms*.001,grid,torque[:,j])
                    x=np.column_stack([low(delayed,dt,hz),low(W[:,j],dt,hz),low(coupling,dt,hz),np.ones(len(grid))])
                    lo=int(.5/dt);coef,_,rank,_=np.linalg.lstsq(x[lo:split],y[lo:split],rcond=None)
                    candidates.append({"filter_hz":hz,"delay_ms":delay_ms,"coefficients":coef.tolist(),"rank":int(rank),"train_r2":r2(y[lo:split],x[lo:split]@coef),"validation_r2":r2(y[split:],x[split:]@coef),"validation_rmse":float(np.sqrt(np.mean((y[split:]-x[split:]@coef)**2)))})
            best=max(candidates,key=lambda c:c["train_r2"])
            segment["axes"][name]={"rate_std_deg_s":float(np.std(W[:,j])*180/np.pi),"best_training_candidate":best,"candidates":candidates,"note":"Coefficient multiplies signed squared kRPM, not BF PID. Conversion to controller b requires separate command-to-RPM model."}
        result["intervals"].append(segment)
    return result

if __name__=="__main__":
    ap=argparse.ArgumentParser();ap.add_argument("folder",type=Path);a=ap.parse_args()
    results=[review(p) for p in sorted(a.folder.glob("flight.*.csv")) if re.fullmatch(r"flight\.\d+\.csv",p.name)]
    (a.folder/"rpm-model-review.json").write_text(json.dumps(results,indent=2,allow_nan=False)+"\n")
    for flight in results:
        print(flight["file"],flight["duration_s"],"eligible",flight["eligible_fraction"])
        for s in flight["intervals"]:
            print("interval",s["start_s"],s["end_s"])
            print("motor correlations",s["motor_command_rpm_correlations"])
            for name,axis in s["axes"].items():print(name,axis["best_training_candidate"])
