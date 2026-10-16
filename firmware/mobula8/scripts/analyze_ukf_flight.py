#!/usr/bin/env python3
"""Analyze FIREDRONE_UKF_FLIGHT logs; BF-relative differences are not ground truth."""
import argparse,csv,json,re
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
p=argparse.ArgumentParser();p.add_argument("folder",type=Path);p.add_argument("--control",action="store_true",help="Decode FIREDRONE_UKF_CONTROL applied-input differences and control flags");args=p.parse_args();reports=[]
for file in sorted(args.folder.glob("flight.*.csv")):
    if not re.fullmatch(r"flight\.\d+\.csv",file.name):continue
    with file.open() as f:
        reader=csv.reader(f,skipinitialspace=True);names=[n.strip() for n in next(reader)]
        data=np.asarray([[float(x) for x in row] for row in reader])
    if not len(data):continue
    d=dict(zip(names,data.T));t=(d["time (us)"]-d["time (us)"][0])*1e-6
    flags=d["debug[3]"].astype(int);valid=(flags&2!=0)&(d["debug[0]"]!=-32768)&(d["debug[1]"]!=-32768)
    errors=np.column_stack([d["debug[0]"],d["debug[1]"]])*.1
    transitions=np.flatnonzero(np.r_[True,flags[1:]!=flags[:-1]])
    armed=d["flightModeFlags"].astype(int)&1!=0
    interior=(t>=2)&(t<=t[-1]-2)&armed
    usable=interior&valid
    motor=np.column_stack([d[f"motor[{i}]"] for i in range(4)])
    report=dict(file=file.name,duration_s=float(t[-1]),samples=len(t),armed_samples=int(sum(armed)),
        valid_armed_samples=int(sum(valid&armed)),valid_armed_fraction=float(np.mean(valid[armed])) if any(armed) else None,
        flag_transitions=[dict(t_s=float(t[i]),flags=int(flags[i]),max_adapter_us=int(d["debug[2]"][i])) for i in transitions],
        maximum_adapter_us=int(max(d["debug[2]"])),budget_fault_samples=int(sum(flags&1!=0)),stopped_samples=int(sum(flags&4!=0)),
        angle_samples=int(sum(d["flightModeFlags"].astype(int)&2!=0)),
        failsafe_samples=int(sum(d["failsafePhase"]!=0)),rx_invalid_after_first=int(sum(d["rxFlightChannelsValid"][1:]==0)),
        interior_motor_upper_saturation_samples=int(sum(np.any(motor[interior]>=2047,axis=1))),
        interior_difference_rmse_deg=np.sqrt(np.mean(errors[usable]**2,axis=0)).tolist() if any(usable) else None,
        interior_difference_max_abs_deg=np.max(abs(errors[usable]),axis=0).tolist() if any(usable) else None,
        gaps_over_2ms=[dict(t_s=float(t[i]),gap_ms=float(v*1000)) for i,v in enumerate(np.diff(t)) if v>.002],
        reference="UKF minus Betaflight; stock PID controlled flight; not external truth")
    if args.control:
        report.update(control_active_armed_samples=int(sum(armed&(flags&8!=0))),
            control_full_armed_samples=int(sum(armed&(flags&32!=0))),
            control_rejected_samples=int(sum(flags&16!=0)),
            reference="Applied ANGLE measurement minus BF; check active/full flags to establish UKF control")
        report["interior_applied_difference_rmse_deg"]=report.pop("interior_difference_rmse_deg")
        report["interior_applied_difference_max_abs_deg"]=report.pop("interior_difference_max_abs_deg")
    reports.append(report)
    fig,ax=plt.subplots(3,1,figsize=(11,7),sharex=True)
    shown=errors.copy();shown[~valid]=np.nan
    ax[0].plot(t[::10],shown[::10,0],label="Roll");ax[0].plot(t[::10],shown[::10,1],label="Pitch");ax[0].set_ylabel("Applied - BF (deg)" if args.control else "UKF - BF (deg)");ax[0].legend()
    ax[1].plot(t[::10],d["debug[2]"][::10]);ax[1].axhline(312*.35,color="red",ls="--",label="35% of 312 us");ax[1].set_ylabel("Peak adapter (us)");ax[1].legend()
    ax[2].step(t[::10],flags[::10]);ax[2].set_ylabel("Flags: 42=full UKF control" if args.control else "Flags: 2=valid, 5=fault");ax[2].set_xlabel("Time since recording start (s)")
    for a in ax:a.grid(alpha=.2)
    fig.suptitle(file.name);fig.tight_layout();fig.savefig(file.with_suffix(".ukf.png"),dpi=150);plt.close(fig)
(args.folder/"ukf-analysis.json").write_text(json.dumps(reports,indent=2)+"\n");print(json.dumps(reports,indent=2))
