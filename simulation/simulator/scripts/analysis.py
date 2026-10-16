#!/usr/bin/env python3
"""Run every scenario against both firmware builds, score them, draw the report."""
import argparse
import json
import os
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import common

plt.style.use('dark_background')

# pass / fail limits
MAX_TOUCHDOWN_VZ = 1.0     # m/s
MAX_LAND_ERR     = 0.5     # m from the take-off point
MAX_TRACK_ERR    = 5.0     # m, 3-D error to the reference during the mission (sanity bound)
MAX_TILT         = 60.0    # deg
# the firmware lands alone without a pilot: no attitude estimate, so no position bound
BLIND_SCENARIOS  = {'rc_lost_land', 'motor_fail'}
# lost motor: no recovery of the mission, only a bounded impact is expected
LIMITS = {'motor_fail': dict(vz=2.5, tilt=180.0)}


def score(df, scenario=''):
    done     = int(df.phase.iloc[-1]) == 5
    fin      = df.iloc[-1]
    air      = df[(df.t > 3.0) & (df.pz < 0.35)]
    touch_vz = float(air.vz.min()) if len(air) else 0.0
    if 'impact_vz' in df:
        touch_vz = float(df.impact_vz.min())
    crashed = bool(df.crashed.max()) if 'crashed' in df else (not done and fin.pz < 0.35)

    mis = df[df.phase == 3]
    err = np.sqrt((mis.px - mis.rx) ** 2 + (mis.py - mis.ry) ** 2 + (mis.pz - mis.rz) ** 2)
    hov = df[(df.phase == 3) & (df.t > 6)]
    jitter = float(np.mean([hov[f'pwm{i}'].diff().abs().mean() for i in range(1, 5)])) if len(hov) else 0.0
    flying = df[(df.phase >= 2) & (df.phase <= 4)]
    sat = float(np.mean([((flying[f'pwm{i}'] >= 1990) | (flying[f'pwm{i}'] <= 1010)).mean() for i in range(1, 5)])) if len(flying) else 0.0

    r = {
        'outcome':      'CRASHED' if crashed else ('LANDED' if done else 'INCOMPLETE'),
        'flight_time':  float(df.t.iloc[-1]),
        'touchdown_vz': touch_vz,
        'land_err':     float(np.hypot(fin.px, fin.py)),
        'track_rms':    float(np.sqrt((err ** 2).mean())) if len(err) else float('nan'),
        'track_max':    float(err.max()) if len(err) else float('nan'),
        'max_tilt':     float(df.tilt.max()),
        'sat_frac':     sat,
        'pwm_jitter':   jitter,
        'ukf_tilt_med': float(df.ukf_tilt_err[df.t > 3].median()),
        'fault_final':  common.FAULTS[int(fin.fault)],
        'max_fhat':     float(df[['fhat_x', 'fhat_y', 'fhat_z']].abs().max().max()),
    }
    r['pass'] = bool(done and not crashed and abs(touch_vz) <= LIMITS.get(scenario, {}).get('vz', MAX_TOUCHDOWN_VZ) and (r['land_err'] <= MAX_LAND_ERR or scenario in BLIND_SCENARIOS)
                     and (r['track_max'] <= MAX_TRACK_ERR or scenario in BLIND_SCENARIOS) and r['max_tilt'] <= LIMITS.get(scenario, {}).get('tilt', MAX_TILT))
    return r


def table(results, builds, scenarios):
    lines = []
    for b in builds:
        lines.append(f'### {common.BUILD_LABEL[b]}')
        lines.append('')
        lines.append('| scenario | result | touchdown vz | landing err | track max | tilt max | motor sat. | verdict |')
        lines.append('|---|---|---|---|---|---|---|---|')
        for s in scenarios:
            r = results[b][s]
            track = 'n/a' if np.isnan(r['track_max']) else f"{r['track_max']:.2f} m"
            lines.append(f"| {s} | {r['outcome']} | {r['touchdown_vz']:+.2f} m/s | {r['land_err']:.2f} m | "
                         f"{track} | {r['max_tilt']:.0f}° | {100 * r['sat_frac']:.1f} % | "
                         f"{'PASS' if r['pass'] else 'FAIL'} |")
        lines.append('')
    return '\n'.join(lines)


def comparison_plot(results, builds, scenarios, path):
    fig, axes = plt.subplots(len(scenarios), len(builds), figsize=(6.2 * len(builds), 2.1 * len(scenarios)),
                             sharex=True, squeeze=False)
    colors = {'asis': '#f87171', 'patched': '#34d399', 'patched_og': '#fbbf24', 'patched_o2': '#fb923c', 'patched_ukf': '#60a5fa'}
    for j, b in enumerate(builds):
        for i, s in enumerate(scenarios):
            ax = axes[i][j]
            df = common.load(common.csv_path(b, s))
            ax.plot(df.t, df.rz, color='#94a3b8', lw=1, ls='--', label='ref z')
            ax.plot(df.t, df.pz, color=colors[b], lw=1.6, label='z')
            ax.plot(df.t, df.tilt / 10.0, color='#fbbf24', lw=0.9, alpha=0.8, label='tilt/10°')
            r = results[b][s]
            ax.set_ylim(-0.3, 6)
            ax.set_ylabel(s, fontsize=8)
            ax.text(0.99, 0.92, f"{r['outcome']}  {'PASS' if r['pass'] else 'FAIL'}", transform=ax.transAxes,
                    ha='right', va='top', fontsize=8, color=colors[b])
            ax.grid(alpha=0.2)
            if i == 0:
                ax.set_title(common.BUILD_LABEL[b], fontsize=10)
            if i == 0 and j == 0:
                ax.legend(fontsize=7, loc='upper left', ncol=3)
    for ax in axes[-1]:
        ax.set_xlabel('t [s]')
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def detail_plot(df, name, path):
    fig, ax = plt.subplots(3, 3, figsize=(15, 9))
    t = df.t

    a = ax[0][0]
    a.plot(df.rx, df.ry, color='#94a3b8', ls='--', lw=1, label='reference')
    a.plot(df.px, df.py, color='#38bdf8', lw=1.5, label='actual')
    a.set_aspect('equal'); a.set_title('top view [m]'); a.legend(fontsize=7)

    a = ax[0][1]
    a.plot(t, df.rz, color='#94a3b8', ls='--', lw=1)
    a.plot(t, df.pz, color='#38bdf8')
    a.set_title('altitude [m]')

    a = ax[0][2]
    a.plot(t, df.roll, label='roll'); a.plot(t, df.pitch, label='pitch'); a.plot(t, df.yaw, label='yaw')
    a.set_title('attitude [deg]'); a.legend(fontsize=7)

    a = ax[1][0]
    for i in range(1, 5):
        a.plot(t, df[f'pwm{i}'], lw=0.6, label=f'M{i}')
    a.set_title('motor PWM [us] (firmware TIM output)'); a.legend(fontsize=7, ncol=4)

    a = ax[1][1]
    a.plot(t, df.wx, label='p'); a.plot(t, df.wy, label='q'); a.plot(t, df.wz, label='r')
    a.set_title('body rates [rad/s]'); a.legend(fontsize=7)

    a = ax[1][2]
    a.plot(t, df.wind, color='#38bdf8', label='|wind| [m/s]')
    a.plot(t, np.hypot(df.tq_x, df.tq_y) * 20, color='#fbbf24', label='arm torque x20 [Nm]')
    a.plot(t, df.fault, color='#f87171', label='fw fault state')
    a.plot(t, df.imu_fault * 0.5, color='#c084fc', label='IMU fault')
    a.plot(t, (1 - df.link) * 0.7, color='#fb923c', label='RC link down')
    a.set_title('disturbances / firmware state'); a.legend(fontsize=7, ncol=2)

    a = ax[2][0]
    a.plot(t, df.ukf_tilt_err, color='#c084fc')
    a.set_title('UKF tilt error vs truth [deg]')

    a = ax[2][1]
    a.plot(t, df.fhat_x, label='x'); a.plot(t, df.fhat_y, label='y'); a.plot(t, df.fhat_z, label='z')
    a.set_title('ADRC disturbance estimate [rad/s^2]'); a.legend(fontsize=7)

    a = ax[2][2]
    for k in 'xyz':
        a.plot(t, df[f'th2_{k}'], label=f'theta2 {k}')
    a.plot(t, df.th1_x, ls='--', label='theta1 x')
    a.set_title('MRAC parameters'); a.legend(fontsize=7)

    for a in ax.flat:
        a.grid(alpha=0.2)
    fig.suptitle(name, fontsize=12)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--builds', nargs='+', default=['asis', 'patched', 'patched_og', 'patched_o2', 'patched_ukf'])
    ap.add_argument('--scenarios', nargs='+')
    ap.add_argument('--seed', type=int, default=42)
    ap.add_argument('--no-detail', action='store_true')
    args = ap.parse_args()

    names = [n for n, _ in common.list_scenarios(args.builds[0])]
    scenarios = args.scenarios or names
    builds = [b for b in args.builds if os.path.exists(common.BINARY[b])
              and (b not in ('patched_og', 'patched_o2') or common.build_args(b))]

    results = {b: {} for b in builds}
    for b in builds:
        print(f'== {common.BUILD_LABEL[b]}')
        for s in scenarios:
            path, _ = common.run_scenario(b, s, seed=args.seed)
            df = common.load(path)
            r = score(df, s)
            results[b][s] = r
            print(f"  {s:13s} {r['outcome']:10s} {'PASS' if r['pass'] else 'FAIL'}  "
                  f"vz {r['touchdown_vz']:+5.2f}  track max {r['track_max']:5.2f} m  tilt {r['max_tilt']:5.1f}")
            if not args.no_detail:
                detail_plot(df, f'{common.BUILD_LABEL[b]} / {s}', os.path.join(common.OUT, f'{b}_{s}.png'))

    md = table(results, builds, scenarios)
    with open(os.path.join(common.OUT, 'summary.md'), 'w') as f:
        f.write(md + '\n')
    with open(os.path.join(common.OUT, 'summary.json'), 'w') as f:
        json.dump(results, f, indent=2)
    comparison_plot(results, builds, scenarios, os.path.join(common.OUT, 'comparison.png'))
    print()
    print(md)


if __name__ == '__main__':
    main()
