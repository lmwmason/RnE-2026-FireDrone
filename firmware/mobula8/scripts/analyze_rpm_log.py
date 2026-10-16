#!/usr/bin/env python3
"""Validate extended Mobula8 identification logs; never modifies aircraft settings."""
import argparse,csv,json,re
from pathlib import Path
import numpy as np

def analyze(path):
    with path.open() as f:
        names=[x.strip() for x in next(csv.reader(f))]
        a=np.loadtxt(f,delimiter=",",ndmin=2)
    d=dict(zip(names,a.T))
    required=[f"fdErpm[{i}]" for i in range(4)]+[f"fdErpmAge[{i}]" for i in range(4)]+["fdTelemetryMask"]+[f"fdGyro[{i}]" for i in range(3)]+[f"fdPidSum[{i}]" for i in range(3)]
    missing=[k for k in required if k not in d]
    if missing:return {"file":path.name,"valid_schema":False,"missing":missing}
    header=path.with_name(path.stem+".headers.csv")
    with header.open() as f: metadata=dict(csv.reader(f))
    poles=int(metadata["motor_poles"])
    if poles<=0 or poles%2:raise ValueError("Invalid motor pole count")
    rpm=np.column_stack([d[f"fdErpm[{i}]"] for i in range(4)])*200/poles
    age=np.column_stack([d[f"fdErpmAge[{i}]"] for i in range(4)])*.1
    valid=np.column_stack([(d["fdTelemetryMask"].astype(int)&(1<<i)!=0)&(age[:,i]<=10) for i in range(4)])
    armed=(d["flightModeFlags"].astype(int)&1)!=0
    gyro=np.column_stack([d[f"fdGyro[{i}]"] for i in range(3)])*.01
    old=np.column_stack([d[f"gyroADC[{i}] (deg/s)"] for i in range(3)])
    t=d["time (us)"]*1e-6
    report={"file":path.name,"valid_schema":True,"samples":len(t),"duration_s":float(t[-1]-t[0]),"motor_poles":poles,
        "rpm_min":rpm.min(axis=0).tolist(),"rpm_max":rpm.max(axis=0).tolist(),
        "fresh_fraction":valid.mean(axis=0).tolist(),"armed_samples":int(sum(armed)),
        "fresh_armed_fraction":valid[armed].mean(axis=0).tolist() if any(armed) else None,
        "all_motors_positive_fresh_armed_fraction":float(np.mean(np.all(valid[armed]&(rpm[armed]>0),axis=1))) if any(armed) else None,
        "gyro_fine_vs_standard_max_deg_s":np.max(abs(gyro-old),axis=0).tolist(),
        "gaps_over_2ms":int(sum(np.diff(t)>.002)),
        "warning":"RPM age is time since decoding, not ESC sampling; zero RPM may be valid while stopped. Motor poles are configured, not physically verified. Schema validity is not flight/model qualification."}
    return report

if __name__=="__main__":
    ap=argparse.ArgumentParser();ap.add_argument("folder",type=Path);args=ap.parse_args()
    reports=[analyze(p) for p in sorted(args.folder.glob("flight.*.csv")) if re.fullmatch(r"flight\.\d+\.csv",p.name)]
    (args.folder/"rpm-analysis.json").write_text(json.dumps(reports,indent=2,allow_nan=False)+"\n")
    print(json.dumps(reports,indent=2))
