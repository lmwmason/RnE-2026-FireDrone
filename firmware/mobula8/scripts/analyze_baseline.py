#!/usr/bin/env python3
"""Descriptive Blackbox baseline analysis; never produces flight-ready controller gains."""
import csv,json,argparse
from pathlib import Path
import numpy as np
from scipy import signal
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=argparse.ArgumentParser();p.add_argument('folder',type=Path);p.add_argument('--index',type=int,default=1);p.add_argument('--start',type=float,default=2);p.add_argument('--end',type=float,default=40);p.add_argument('--landing-contact',action='store_true');args=p.parse_args();folder=args.folder
with (folder/f'flight.{args.index:02d}.csv').open() as f:
 reader=csv.reader(f,skipinitialspace=True);names=[x.strip() for x in next(reader)]
 data=np.asarray([[float(x) for x in row] for row in reader])
d=dict(zip(names,data.T));t=d['time (us)']*1e-6;t-=t[0];dt=np.diff(t)
# Explicit conservative interior window, excluding startup and landing contact.
mask=(t>=args.start)&(t<=args.end);tm=t[mask];step=float(np.median(np.diff(tm)));fs=1/step
sp=np.column_stack([d[f'setpoint[{a}]'] for a in range(3)])
gyro=np.column_stack([d[f'gyroADC[{a}] (deg/s)'] for a in range(3)])
motor=np.column_stack([d[f'motor[{a}]'] for a in range(4)])
acc=np.column_stack([d[f'accSmooth[{a}] (g)'] for a in range(3)])
accnorm=np.linalg.norm(acc,axis=1);saturation=np.any(motor>=2047,axis=1)
report={'duration_s':float(t[-1]),'rows':len(t),'decoded_frame_errors':0,
 'median_sample_interval_us':step*1e6,'median_sampling_hz':fs,
 'scheduled_logging_fraction':'1/2; decoder missing-iteration percentage is not packet-loss percentage',
 'analysis_window_s':[args.start,args.end],'landing_contact_confirmed_by_user':args.landing_contact,
 'gaps_over_2ms':[{'from_s':float(t[i]),'duration_ms':float(dt[i]*1000)} for i in np.where(dt>.002)[0]],
 'mode_flags_unique':[int(v) for v in np.unique(d['flightModeFlags'])],
 'mode_interpretation':'Blackbox rcModeActivationMask: ARM bit0, ANGLE bit1, HORIZON bit2.',
 'angle_active_samples':int(np.count_nonzero(d['flightModeFlags'].astype(int)&2)),
 'horizon_active_samples':int(np.count_nonzero(d['flightModeFlags'].astype(int)&4)),
 'failsafe_phase_nonzero_samples':int(np.count_nonzero(d['failsafePhase'])),
 'receiver_invalid_samples_after_first':int(np.count_nonzero(d['rxFlightChannelsValid'][1:]==0)),
 'interior_motor_upper_saturation_samples':int(sum(saturation[mask])),
 'interior_motor_command_percent_mean':list(np.mean((motor[mask]-248)/(2047-248)*100,axis=0)),
 'interior_battery_range_v':[float(np.min(d['vbatLatest (V)'][mask])),float(np.max(d['vbatLatest (V)'][mask]))],
 'landing_peak_acceleration_g':float(np.max(accnorm)),
 'landing_peak_motor_command':float(np.max(motor)),
 'landing_motor_saturation_samples':int(sum(saturation)),
 'landing_min_voltage_v':float(np.min(d['vbatLatest (V)'])),
 'axes':{},'controller_parameters_identified':False,'ukf_enabled':False}
fig,axes=plt.subplots(3,2,figsize=(15,10),constrained_layout=True)
for a,name in enumerate(['roll','pitch','yaw']):
 err=gyro[mask,a]-sp[mask,a]
 grid=np.arange(tm[0],tm[-1],step);g=np.interp(grid,tm,gyro[mask,a]);target=np.interp(grid,tm,sp[mask,a])
 f,psd=signal.welch(g,fs,nperseg=min(4096,len(g)))
 band=(f>=100)&(f<=500);rms=float(np.sqrt(np.trapezoid(psd[band],f[band])))
 correlations=[]
 for lag in range(round(.04/step)+1):
  x=target[:-lag] if lag else target;y=g[lag:] if lag else g
  correlations.append(float(np.corrcoef(x,y)[0,1]) if np.std(x)>1 else None)
 best=int(np.argmax([v if v is not None else -2 for v in correlations]))
 report['axes'][name]={'tracking_rmse_dps':float(np.sqrt(np.mean(err**2))),
 'tracking_abs_error_p95_dps':float(np.percentile(abs(err),95)),
 'target_std_dps':float(np.std(sp[mask,a])), 'target_range_dps':[float(min(sp[mask,a])),float(max(sp[mask,a]))],
 'gyro_100_500hz_rms_dps':rms,'setpoint_to_gyro_correlation_lag_ms':best*step*1000 if correlations[best] is not None else None,
 'lag_correlation':correlations[best], 'lag_warning':'Closed-loop descriptive lag, NOT motor time constant.',
 'excitation_sufficient_for_initial_review':bool(np.std(sp[mask,a])>10)}
 ax=axes[a,0];ax.plot(t[::10],sp[::10,a],label='Target',lw=1);ax.plot(t[::10],gyro[::10,a],label='Gyro',lw=.8,alpha=.8)
 ax.set(title=name.title()+' rate',ylabel='deg/s',xlabel='Seconds after log start');ax.legend(loc='upper left');ax.axvspan(args.end,t[-1],color='orange',alpha=.12)
axes[0,1].plot(t[::10],(motor[::10]-248)/(2047-248)*100,lw=.7);axes[0,1].set(title='Motor command (normalized endpoints)',ylabel='%',xlabel='Seconds');axes[0,1].legend(['M0','M1','M2','M3'])
axes[1,1].plot(t[::10],d['vbatLatest (V)'][::10]);axes[1,1].set(title='Battery voltage',ylabel='V',xlabel='Seconds')
axes[2,1].plot(t[::10],accnorm[::10]);axes[2,1].set(title='Accelerometer magnitude (includes gravity)',ylabel='g',xlabel='Seconds');axes[2,1].axvspan(args.end,t[-1],color='orange',alpha=.12)
fig.suptitle(f'Mobula8 log {args.index} | {t[-1]:.2f} s | stock PID | analysis window {args.start:g}-{args.end:g} s')
fig.savefig(folder/'overview.png',dpi=160);plt.close(fig)
(folder/'analysis.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
