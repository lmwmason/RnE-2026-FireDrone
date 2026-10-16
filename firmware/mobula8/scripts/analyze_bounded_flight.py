#!/usr/bin/env python3
"""Read-only bounded-controller flight diagnostics. Does not infer causal benefit."""
import argparse,csv,json
from pathlib import Path
import numpy as np
from scipy.signal import find_peaks
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

def analyze(path):
    with path.open() as f:
        names=[v.strip() for v in next(csv.reader(f))];a=np.loadtxt(f,delimiter=",",ndmin=2)
    d=dict(zip(names,a.T));t=(d["time (us)"]-d["time (us)"][0])*1e-6
    with path.with_name(path.stem+".headers.csv").open() as f:h=dict(csv.reader(f))
    flags=d["debug[3]"].astype(int);armed=(d["flightModeFlags"].astype(int)&1)!=0
    corr=np.column_stack([d[f"fdCorrection[{i}]"] for i in range(3)])*.001
    pid=np.column_stack([d[f"fdPidSum[{i}]"] for i in range(3)])*.01
    gyro=np.column_stack([d[f"fdGyro[{i}]"] for i in range(3)])*.01
    sp=np.column_stack([d[f"setpoint[{i}]"] for i in range(3)])
    acc=np.column_stack([d[f"accSmooth[{i}]"] for i in range(3)])/float(h["acc_1G"])
    amag=np.linalg.norm(acc,axis=1);error=np.linalg.norm(gyro-sp,axis=1)
    motor=np.column_stack([d[f"motor[{i}]"] for i in range(4)])
    valid=armed&((flags&1024)!=0);nonzero=np.any(abs(corr)>.0005,axis=1)
    low=flags&1023;changes=np.flatnonzero(np.r_[True,low[1:]!=low[:-1]])
    def fraction(mask,within=armed):return float(np.mean(mask[within])) if np.any(within) else None
    def stats(v,mask=armed):return dict(rms=np.sqrt(np.mean(v[mask]**2,axis=0)).tolist(),max_abs=np.max(abs(v[mask]),axis=0).tolist()) if np.any(mask) else None
    report=dict(file=path.name,duration_s=float(t[-1]),samples=len(t),armed_samples=int(sum(armed)),
        ukf_valid_armed_fraction=fraction(flags&2!=0),ukf_active_armed_fraction=fraction(flags&8!=0),ukf_full_armed_fraction=fraction(flags&32!=0),
        rate_valid_armed_fraction=fraction(flags&1024!=0),correction_nonzero_armed_fraction=fraction(nonzero),
        candidate_clipped_fraction_when_rate_valid=fraction(flags&4096!=0,valid),
        correction=stats(corr),total_pid=stats(pid),gyro_deg_s=stats(gyro),tracking_error_deg_s=stats(gyro-sp),
        correction_limit_exceeded_samples=int(sum(np.any(abs(corr)>.2505,axis=1))),
        correction_without_valid_flag_samples=int(sum(nonzero&((flags&1024)==0))),
        correction_with_recorded_disarm_samples=int(sum(nonzero&~armed)),
        correction_rms_over_total_pid_rms=(np.sqrt(np.mean(corr[armed]**2,axis=0))/np.maximum(np.sqrt(np.mean(pid[armed]**2,axis=0)),1e-9)).tolist() if np.any(armed) else None,
        max_research_us=int(max(d["debug[2]"])),budget_fault_samples=int(sum(flags&1!=0)),observer_stopped_samples=int(sum(flags&4!=0)),
        ukf_rejected_samples=int(sum(flags&16!=0)),hard_fault_samples=int(sum(flags&64!=0)),
        failsafe_samples=int(sum(d["failsafePhase"]!=0)),rx_invalid_armed_samples=int(sum(armed&(d["rxFlightChannelsValid"]==0))),
        motor_upper_saturation_armed_fraction=fraction(np.any(motor>=2047,axis=1)),
        max_acceleration_norm_g=float(max(amag)),gaps_over_2ms=int(sum(np.diff(t)>.002)),
        ukf_transitions=[dict(t_s=float(t[i]),flags=int(low[i]),reason=int((low[i]>>7)&7)) for i in changes],
        limitations=["No matched baseline experiment; cannot attribute recovery to extra controller.","Acceleration peaks indicate candidate disturbances, not known foot-contact times.","No external attitude/position ground truth. Very small bounded correction is not full-authority control."])
    peaks,_=find_peaks(amag,height=2.5,prominence=1,distance=max(1,int(.4/np.median(np.diff(t)))))
    peaks=sorted([i for i in peaks if armed[i]],key=lambda i:amag[i],reverse=True)[:12]
    events=[]
    for i in sorted(peaks):
        window=(t>=t[i]-.1)&(t<=t[i]+1)&armed
        events.append(dict(t_s=float(t[i]),acceleration_g=float(amag[i]),rate_error_peak_deg_s=float(max(error[window])),
            ukf_active_entire_window=bool(np.all(flags[window]&8)),correction_peak=np.max(abs(corr[window]),axis=0).tolist(),
            motor_saturated_in_window=bool(np.any(motor[window]>=2047))))
    report["candidate_disturbances"]=events
    if all(f"fdAttitude[{i}]" in d for i in range(4)):
        attitude=np.column_stack([d[f"fdAttitude[{i}]"] for i in range(4)])*.01
        difference=attitude[:,:2]-attitude[:,2:]
        observed=armed&((flags&2)!=0)&np.all(attitude[:,:2]!=-327.68,axis=1)
        gate=d["fdUkfAccelRejected"].astype(int)
        report["attitude_diagnostics"]={
            "difference_deg":stats(difference,observed),
            "over_8deg_observed_fraction":fraction(np.any(abs(difference)>8,axis=1),observed),
            "over_20deg_observed_fraction":fraction(np.any(abs(difference)>20,axis=1),observed),
            "correction_age_max_us":int(max(d["fdUkfAge"][observed])) if np.any(observed) else None,
            "accel_rejected_observed_fraction":fraction((gate&1)!=0,observed),
            "dynamic_hold_observed_fraction":fraction((gate&2)!=0,observed)}
        for event in events:
            window=(t>=event["t_s"]-.1)&(t<=event["t_s"]+1)&observed
            event["ukf_bf_max_abs_difference_deg"]=np.max(abs(difference[window]),axis=0).tolist() if np.any(window) else None
            event["dynamic_hold_in_window"]=bool(np.any(gate[window]&2))
        step=max(1,len(t)//20000);sl=slice(None,None,step)
        fig,ax=plt.subplots(4,1,figsize=(12,9),sharex=True)
        ax[0].plot(t[sl],amag[sl]);ax[0].set_ylabel("Acceleration norm (g)")
        clean=difference.copy();clean[~observed]=np.nan
        ax[1].plot(t[sl],clean[sl,0],label="Roll");ax[1].plot(t[sl],clean[sl,1],label="Pitch")
        for limit in [-20,-8,8,20]:ax[1].axhline(limit,ls="--",alpha=.35,color="gray")
        ax[1].legend();ax[1].set_ylabel("UKF - BF (deg)")
        ax[2].step(t[sl],((flags[sl]&8)!=0).astype(int),label="UKF selected")
        ax[2].step(t[sl],((flags[sl]&64)!=0).astype(int),label="Hard fallback");ax[2].legend();ax[2].set_ylabel("Selection state")
        ax[3].step(t[sl],(gate[sl]&1)!=0,label="Acceleration rejected")
        ax[3].step(t[sl],(gate[sl]&2)!=0,label="Dynamic hold");ax[3].legend();ax[3].set_xlabel("Seconds since recording start")
        for panel in ax:panel.grid(alpha=.2)
        fig.tight_layout();fig.savefig(path.with_suffix(".attitude.png"),dpi=150);plt.close(fig)
    fig,ax=plt.subplots(5,1,figsize=(12,11),sharex=True)
    step=max(1,len(t)//15000);s=slice(None,None,step)
    ax[0].plot(t[s],amag[s]);ax[0].set_ylabel("Acceleration (g)")
    for j,name in enumerate(["Roll","Pitch","Yaw"]):
        ax[1].plot(t[s],(gyro-sp)[s,j],label=name);ax[2].plot(t[s],corr[s,j],label=name)
    ax[1].set_ylabel("Rate error (deg/s)");ax[1].legend(loc="upper right")
    ax[2].set_ylabel("Added PID units");ax[2].set_ylim(-.3,.3)
    ax[3].plot(t[s],motor[s]);ax[3].set_ylabel("Motor command")
    for shift,name in [(3,"UKF active"),(10,"Rate valid"),(11,"Correcting"),(12,"Clipped")]:ax[4].step(t[s],((flags[s]>>shift)&1)+(shift-3)*.2,label=name)
    ax[4].legend(loc="upper right");ax[4].set_ylabel("State (offset)");ax[4].set_xlabel("Seconds since recording start")
    for z in ax:z.grid(alpha=.2)
    fig.tight_layout();fig.savefig(path.with_suffix(".bounded.png"),dpi=150);plt.close(fig)
    return report

if __name__=="__main__":
    p=argparse.ArgumentParser();p.add_argument("csv",type=Path);a=p.parse_args();r=analyze(a.csv)
    a.csv.with_suffix(".bounded.json").write_text(json.dumps(r,indent=2,allow_nan=False)+"\n")
    print(json.dumps(r,indent=2,allow_nan=False))
