#!/usr/bin/env python3
"""Training-only lag selection with chronologically held-out validation. Never writes FC settings."""
import argparse,json,re,csv
from pathlib import Path
import numpy as np
from scipy.signal import lfilter

def r2(y,p):
    v=np.sum((y-y.mean())**2)
    return float(1-np.sum((y-p)**2)/v) if v>1e-12 else None

def filtered(v,k):
    return lfilter([k],[1,-(1-k)],v,zi=[(1-k)*v[0]])[0]

def fit(w,u,dt,tau_ms,cutoff=30):
    lag_k=dt/(dt+tau_ms*.001);k=dt/(dt+1/(2*np.pi*cutoff))
    lag=filtered(np.r_[u[0],u[:-1]],lag_k)
    smooth=filtered(w,k);lag_smooth=filtered(lag,k);rate_smooth=filtered(np.r_[w[0],w[:-1]],k)
    y=np.diff(smooth)/dt
    x=np.column_stack([lag_smooth[1:],rate_smooth[1:],np.ones(len(y))])
    discard=int(.5/dt);x=x[discard:];y=y[discard:];split=int(len(y)*.65)
    if split<200 or len(y)-split<200:raise ValueError("insufficient train/validation duration")
    coef,_,rank,_=np.linalg.lstsq(x[:split],y[:split],rcond=None)
    return dict(tau_ms=tau_ms,b=float(coef[0]),damping=float(coef[1]),bias=float(coef[2]),
        train_r2=r2(y[:split],x[:split]@coef),validation_r2=r2(y[split:],x[split:]@coef),
        validation_rmse_rad_s2=float(np.sqrt(np.mean((y[split:]-x[split:]@coef)**2))),rank=int(rank))

def axis_signals(d,j):
    # Extended logger captures fractional gyro and actual final PID Sum.
    rate=d[f"fdGyro[{j}]"]*.01 if f"fdGyro[{j}]" in d else d[f"gyroADC[{j}] (deg/s)"]
    if f"fdPidSum[{j}]" in d:
        u=d[f"fdPidSum[{j}]"]*.01
    else:
        u=sum(d[f"axis{part}[{j}]"] if f"axis{part}[{j}]" in d else np.zeros(len(rate)) for part in "PIDF")
    return rate*np.pi/180,u

def review(path):
    with path.open() as f:
        names=[n.strip() for n in next(csv.reader(f))]
        a=np.loadtxt(f,delimiter=",")
    d=dict(zip(names,a.T));t=(d["time (us)"]-d["time (us)"][0])*1e-6
    mask=(t>=2)&(t<=t[-1]-2)&(d["flightModeFlags"].astype(int)&1!=0)&(d["failsafePhase"]==0)&(d["rxFlightChannelsValid"]!=0)
    for j in range(4):mask &= d[f"motor[{j}]"]<2047
    # Never join across rejected intervals or recording gaps.
    ii=np.flatnonzero(mask);groups=np.split(ii,np.flatnonzero((np.diff(ii)>1)|(np.diff(t[ii])>.002))+1)
    groups=[g for g in groups if len(g)>1 and t[g[-1]]-t[g[0]]>=6]
    result=dict(file=path.name,duration_s=float(t[-1]),signal_source="fine gyro and final PID Sum" if "fdGyro[0]" in d and "fdPidSum[0]" in d else "rounded gyro and PID component sum",intervals=[])
    for g in groups:
        grid=np.arange(t[g[0]],t[g[-1]],.001)
        segment=dict(start_s=float(grid[0]),end_s=float(grid[-1]),axes={})
        for j,name in enumerate(["roll","pitch","yaw"]):
            raw_w,raw_u=axis_signals(d,j)
            w=np.interp(grid,t[g],raw_w[g]);u=np.interp(grid,t[g],raw_u[g])
            st=dict(rate_std_dps=float(np.std(w)*180/np.pi),pid_std=float(np.std(u)))
            if st["rate_std_dps"]<10 or st["pid_std"]<5:
                st.update(qualified=False,reason="insufficient axis excitation")
            else:
                candidates=[fit(w,u,.001,tau) for tau in [1,2,5,10,15,20,25,40,60,100,150]]
                best=max(candidates,key=lambda x:x["train_r2"] if x["train_r2"] is not None else -1e30)
                plausible=[c["tau_ms"] for c in candidates if c["train_r2"]>=best["train_r2"]-.02]
                ok=best["rank"]==3 and .005<=best["b"]<=100 and best["train_r2"]>=.6 and best["validation_r2"] is not None and best["validation_r2"]>=.6 and best["damping"]<=0
                st.update(best_training_candidate=best,near_best_lags_ms=plausible,qualified=bool(ok),candidates=candidates,
                    reason="candidate for further model review, not flight qualification" if ok else "fit/generalization/physical sign checks failed")
            segment["axes"][name]=st
        result["intervals"].append(segment)
    return result

if __name__=="__main__":
    p=argparse.ArgumentParser();p.add_argument("folder",type=Path);args=p.parse_args()
    reports=[review(f) for f in sorted(args.folder.glob("flight.*.csv")) if re.fullmatch(r"flight\.\d+\.csv",f.name)]
    out=dict(method="30 Hz matched low-pass; train first 65%, validate last 35%; lag selected ONLY on training; assumed 1..150 ms lag sweep; no automatic parameter application",flights=reports)
    (args.folder/"identification-review.json").write_text(json.dumps(out,indent=2,allow_nan=False)+"\n")
    for f in reports:
        print(f["file"],round(f["duration_s"],2),"seconds")
        for s in f["intervals"]:
            print(" interval",round(s["start_s"],2),round(s["end_s"],2))
            for axis,a in s["axes"].items():print(axis,{k:v for k,v in a.items() if k!="candidates"})
