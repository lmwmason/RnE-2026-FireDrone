#!/usr/bin/env python3
"""Estimate rate-axis effectiveness from a selected, unsaturated Blackbox interval.
CSV: time (microseconds), gyroADC[0..2] (deg/s), axisP/I/D/F[0..2] (BF PID units).
Alternatively use time_us, gyro_roll/pitch/yaw_dps, pid_roll/pitch/yaw.
Always inspect fit quality. Estimates are not flight qualification or automatic settings.
"""
import argparse,csv,json,math,pathlib,statistics

def solve(a,b):
    m=[list(row)+[rhs] for row,rhs in zip(a,b)]
    for i in range(len(b)):
        k=max(range(i,len(b)),key=lambda k:abs(m[k][i]));m[i],m[k]=m[k],m[i]
        if abs(m[i][i])<1e-10:raise ValueError("insufficient independent excitation")
        scale=m[i][i];m[i]=[x/scale for x in m[i]]
        for j in range(len(b)):
            if j!=i:
                scale=m[j][i];m[j]=[x-scale*y for x,y in zip(m[j],m[i])]
    return [row[-1] for row in m]

def fit_axis(samples,tau_ms):
    if len(samples)<200:raise ValueError("at least 200 samples per axis required")
    if samples[-1][0]-samples[0][0]<2:raise ValueError("at least two seconds required")
    if statistics.pstdev(s[2] for s in samples)<5:raise ValueError("PID excitation too small")
    if statistics.pstdev(s[1] for s in samples)<math.radians(10):raise ValueError("rate excitation too small")
    tau=tau_ms*.001; lag=samples[0][2]; smooth=samples[0][1]; lagSmooth=lag; rateSmooth=smooth
    rows=[];targets=[]
    for i in range(1,len(samples)):
        t,w,u=samples[i];dt=t-samples[i-1][0]
        if not .00005<=dt<=.02:raise ValueError("non-monotonic time or log gap >20ms")
        lag+=dt/(tau+dt)*(samples[i-1][2]-lag)
        k=dt/(dt+1/(2*math.pi*30));old=smooth;smooth+=k*(w-smooth)
        lagSmooth+=k*(lag-lagSmooth); rateSmooth+=k*(samples[i-1][1]-rateSmooth)
        if t-samples[0][0]>.5:
            rows.append([lagSmooth,rateSmooth,1]);targets.append((smooth-old)/dt)
    a=[[sum(r[i]*r[j] for r in rows) for j in range(3)] for i in range(3)]
    b=[sum(r[i]*v for r,v in zip(rows,targets)) for i in range(3)]
    coef=solve(a,b);pred=[sum(x*y for x,y in zip(r,coef)) for r in rows]
    mse=sum((x-y)**2 for x,y in zip(pred,targets))/len(targets)
    variance=statistics.pvariance(targets)
    r2=1-mse/variance if variance>0 else -math.inf
    good=.005<=coef[0]<=100 and r2>=.6
    return dict(effectiveness_rad_s2_per_pid_unit=coef[0],damping=coef[1],bias=coef[2],
        r_squared=r2,rmse_rad_s2=math.sqrt(mse),tau_ms=tau_ms,
        usable_for_review=good,cli_value=round(coef[0]*100) if good else None)

def read_csv(path,start,end):
    text=path.read_text().splitlines()
    header=next((i for i,l in enumerate(text) if "gyroADC[0]" in l or "gyro_roll_dps" in l),None)
    if header is None:raise ValueError("Blackbox CSV header not found; see --help")
    rows=csv.DictReader(text[header:],skipinitialspace=True); axes=[[],[],[]]
    for row in rows:
        row={k.strip():v.strip() for k,v in row.items() if k is not None and v is not None}
        t=float(row.get("time_us",row.get("time (us)",row.get("time","nan"))))*1e-6
        if not start<=t<=end:continue
        for a,name in enumerate(("roll","pitch","yaw")):
            w=float(row[f"gyro_{name}_dps"] if f"gyro_{name}_dps" in row else row.get(f"gyroADC[{a}] (deg/s)",row.get(f"gyroADC[{a}]")))*math.pi/180
            u=float(row[f"pid_{name}"]) if f"pid_{name}" in row else sum(float(row.get(f"axis{part}[{a}]",0)) if part=="D" and a==2 else float(row[f"axis{part}[{a}]"]) for part in "PIDF")
            if not all(math.isfinite(v) for v in (t,w,u)):raise ValueError("nonfinite CSV sample")
            axes[a].append((t,w,u))
    return axes

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("csv",type=pathlib.Path);p.add_argument("--start",type=float,required=True,help="absolute log time in seconds")
    p.add_argument("--end",type=float,required=True);p.add_argument("--tau-ms",type=float,required=True,help="independently estimated motor lag; not auto-guessed")
    p.add_argument("--out",type=pathlib.Path,required=True);a=p.parse_args()
    if a.start>=a.end or not 1<=a.tau_ms<=200:p.error("invalid interval or motor tau")
    result={}
    for name,samples in zip(("roll","pitch","yaw"),read_csv(a.csv,a.start,a.end)):
        try:result[name]=fit_axis(samples,a.tau_ms)
        except ValueError as e:result[name]={"usable_for_review":False,"error":str(e)}
    result["status"]="ESTIMATE_REQUIRES_REVIEW"
    result["scope"]="Selected baseline interval only; discard clipping, impacts, RPM loss, and rapid throttle changes. No automatic activation."
    a.out.write_text(json.dumps(result,indent=2,allow_nan=False)+"\n");print(a.out.read_text())
if __name__=="__main__":main()
