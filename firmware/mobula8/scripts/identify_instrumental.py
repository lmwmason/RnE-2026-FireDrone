#!/usr/bin/env python3
"""Closed-loop instrumental-variable review using pilot RC as instruments.
Results are local candidate models, not guaranteed flight-safe or full-plant fits.
Positive state coefficients are nuisance dynamics, NOT identified physical drag.
"""
import argparse,csv,json
from pathlib import Path
import numpy as np
from scipy.signal import lfilter
from review_rpm_model import read,low
from review_identification import axis_signals

def matrices(w,u,rc,dt,hp,tau_ms):
    t=np.arange(len(w))*dt
    def band(v):return low(v-low(v,dt,hp),dt,15)
    k=dt/(dt+tau_ms*.001)
    lag=lfilter([k],[1,k-1],np.r_[u[0],u[:-1]],zi=[(1-k)*u[0]])[0]
    X=np.column_stack([band(lag),band(w)])
    y=band(np.gradient(w,dt))
    Z=np.column_stack([band(np.interp(t-delay,t,rc)) for delay in [0,.02,.04,.08,.12,.2]])
    return X,y,Z

def estimate(X,y,Z,mask):
    zz=Z[mask].T@Z[mask];zx=Z[mask].T@X[mask];zy=Z[mask].T@y[mask]
    zxhat=np.linalg.lstsq(zz,zx,rcond=1e-10)[0];zyhat=np.linalg.lstsq(zz,zy,rcond=1e-10)[0]
    return np.linalg.lstsq(zx.T@zxhat,zx.T@zyhat,rcond=1e-10)[0]

def assess(X,y,Z,mask,coef):
    xh=Z[mask]@np.linalg.lstsq(Z[mask],X[mask],rcond=None)[0]
    yh=Z[mask]@np.linalg.lstsq(Z[mask],y[mask],rcond=None)[0]
    signal=float(np.sum(yh*yh))
    return {"projected_moment_error":float(np.sum((yh-xh@coef)**2)/signal) if signal>1e-12 else None,
        "input_explained_fraction":float(np.var(xh[:,0])/max(np.var(X[mask,0]),1e-12)),
        "raw_acceleration_r2":float(1-np.sum((y[mask]-X[mask]@coef)**2)/max(np.sum((y[mask]-y[mask].mean())**2),1e-12))}

def fit_fixed(w,u,rc,dt,hp,tau_ms,train,val,blocks=None):
    X,y,Z=matrices(w,u,rc,dt,hp,tau_ms)
    _,sv,vh=np.linalg.svd(Z[train],full_matrices=False)
    if sv[0]<1e-9:raise ValueError("No pilot excitation")
    keep=sv>sv[0]*.01;Z=Z@vh[keep].T
    if sum(keep)<3:raise ValueError("Insufficient instrument rank")
    coef=estimate(X,y,Z,train)
    report={"b":float(coef[0]),"state_coefficient":float(coef[1]),"hp_hz":hp,"lp_hz":15,"tau_ms":tau_ms,
        "instrument_rank":int(sum(keep)),"train":assess(X,y,Z,train,coef),"validation":assess(X,y,Z,val,coef)}
    if blocks is not None:
        rng=np.random.default_rng(20261015);ids=np.unique(blocks[train]);stats=[]
        for ident in ids:
            mask=train&(blocks==ident);stats.append((Z[mask].T@Z[mask],Z[mask].T@X[mask],Z[mask].T@y[mask]))
        samples=[]
        for _ in range(300):
            selected=rng.integers(0,len(stats),len(stats));zz=sum(stats[i][0] for i in selected);zx=sum(stats[i][1] for i in selected);zy=sum(stats[i][2] for i in selected)
            xhat=np.linalg.lstsq(zz,zx,rcond=1e-10)[0];yhat=np.linalg.lstsq(zz,zy,rcond=1e-10)[0]
            b=np.linalg.lstsq(zx.T@xhat,zx.T@yhat,rcond=1e-10)[0][0];samples.append(float(b))
        report["b_block_bootstrap_5_50_95"]=np.percentile(samples,[5,50,95]).tolist();report["bootstrap_positive_fraction"]=float(np.mean(np.array(samples)>0))
    return report

def prepare(path):
    d,h=read(path);tt=(d["time (us)"]-d["time (us)"][0])*1e-6
    if np.any(np.diff(tt)>.002):raise ValueError("Recording gap; segment before identification")
    dt=.004;t=np.arange(3,tt[-1]-2,dt);blocks=np.floor((t-3)/6).astype(int)
    interior=(((t-3)%6)>.5)&(((t-3)%6)<5.5)&(t>5)
    train=interior&(blocks%2==0);val=interior&(blocks%2==1)
    if sum(train)<1000 or sum(val)<1000:raise ValueError("Too short for blocked validation")
    result=[]
    for j in range(3):
        w,u=axis_signals(d,j);result.append((np.interp(t,tt,w),np.interp(t,tt,u),np.interp(t,tt,d[f"rcCommand[{j}]"])))
    return result,dt,train,val,blocks

if __name__=="__main__":
    ap=argparse.ArgumentParser();ap.add_argument("csv",type=Path);ap.add_argument("--out",type=Path,required=True);a=ap.parse_args()
    data,dt,train,val,blocks=prepare(a.csv)
    # Fixed after exploratory training search; these settings are NOT selected on validation.
    params=[(5,40),(5,40),(.5,5)]
    report={"source":str(a.csv),"method":"RC instruments, fixed filters, disjoint 6s blocks, 0.5s margins; 300 training-block bootstrap samples. Validation reports projected moment consistency, not whole-plant prediction R2.","axes":{}}
    for name,signals,(hp,tau) in zip(["roll","pitch","yaw"],data,params):
        report["axes"][name]=fit_fixed(*signals,dt,hp,tau,train,val,blocks);print(name,report["axes"][name],flush=True)
    a.out.write_text(json.dumps(report,indent=2,allow_nan=False)+"\n")
