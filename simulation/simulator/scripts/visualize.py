#!/usr/bin/env python3
"""3-D playback of a simulator log: airframe, wind, per-motor thrust, telemetry."""
import argparse
import os
import sys

import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.gridspec import GridSpec
from matplotlib.lines import Line2D
from mpl_toolkits.mplot3d import Axes3D   # noqa: F401

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import common

matplotlib.rcParams['toolbar'] = 'None'
plt.style.use('dark_background')

BODY_SCALE = 9.0          # the real airframe (arm 8 cm) is invisible at mission scale
ARM        = 0.080 * BODY_SCALE
ROTOR_R    = 0.045 * BODY_SCALE
# FLU body frame, motor order M1 rear-right, M2 front-right, M3 rear-left, M4 front-left
MOTOR_POS  = np.array([[-1, -1, 0], [1, -1, 0], [-1, 1, 0], [1, 1, 0]]) * ARM / np.sqrt(2.0)
MOTOR_LBL  = ['M1', 'M2', 'M3', 'M4']

PHASE_COLOR = {0: '#94a3b8', 1: '#94a3b8', 2: '#fbbf24', 3: '#38bdf8', 4: '#34d399', 5: '#94a3b8'}
C_OK, C_WARN, C_BAD = '#34d399', '#fbbf24', '#f87171'
THRUST_CMAP = plt.get_cmap('turbo')


def circle(n=24):
    a = np.linspace(0, 2 * np.pi, n)
    return np.stack([ROTOR_R * np.cos(a), ROTOR_R * np.sin(a), np.zeros_like(a)], 1)


def fault_spans(t, mask):
    spans, start = [], None
    for i, m in enumerate(mask):
        if m and start is None:
            start = t[i]
        if not m and start is not None:
            spans.append((start, t[i])); start = None
    if start is not None:
        spans.append((start, t[-1]))
    return spans


def main():
    global MOTOR_POS, MOTOR_LBL
    ap = argparse.ArgumentParser()
    ap.add_argument('csv', nargs='?')
    ap.add_argument('--build', default='patched', choices=['x8', 'asis', 'patched', 'patched_og', 'patched_o2', 'patched_ukf'])
    ap.add_argument('--scenario', default='nominal')
    ap.add_argument('--save', help='write mp4 / gif instead of opening a window')
    ap.add_argument('--frame', type=float, help='render a single PNG at this time [s]')
    ap.add_argument('--step', type=int, default=8, help='log samples per video frame (200 Hz log)')
    ap.add_argument('--fps', type=int, default=25)
    ap.add_argument('--follow', action='store_true', help='close-up camera that tracks the drone (default for prop/arm scenarios)')
    ap.add_argument('--wide', action='store_true', help='fixed mission-scale camera')
    ap.add_argument('--orbit', action='store_true', help='slowly rotate the camera')
    args = ap.parse_args()

    follow = args.follow or (not args.wide and any(k in (args.csv or args.scenario) for k in ('prop_damage', 'arm_swing', 'extreme')))
    path = args.csv or common.csv_path(args.build, args.scenario)
    if not os.path.exists(path):
        print(f'{path} not found - run ./run.sh first'); sys.exit(1)
    df = common.load(path)
    motors = int(df.motor_count.iloc[0]) if 'motor_count' in df else 4
    if motors == 8:
        upper = MOTOR_POS.copy(); upper[:, 2] = 0.13
        lower = MOTOR_POS.copy(); lower[:, 2] = -0.13
        MOTOR_POS = np.vstack([upper, lower])
        MOTOR_LBL = [f'M{i+1}' for i in range(8)]
    title = os.path.splitext(os.path.basename(path))[0]
    if motors == 8:
        title += ' | X8 UNCALIBRATED | geometry/arm animation schematic'
    wps = common.mission_waypoints(args.build if os.path.exists(common.BINARY[args.build]) else 'asis')

    t   = df.t.values
    pos = df[['px', 'py', 'pz']].values
    R   = common.rotation_matrices(df[['qw', 'qx', 'qy', 'qz']].values)
    wind = df[['wind_x', 'wind_y', 'wind_z']].values
    thr  = df[[f'T{i+1}' for i in range(motors)]].values
    eff  = df[[f'eff{i+1}' for i in range(motors)]].values
    u    = df[[f'u{i+1}' for i in range(motors)]].values
    tq   = df[['tq_x', 'tq_y']].values
    spin_all = np.cumsum(0.25 + 1.5 * u, axis=0)
    tq_max = max(np.abs(tq).max(), 1e-9)
    N = len(df)

    fig = plt.figure(figsize=(16, 9), facecolor='#0b1220')
    gs  = GridSpec(3, 5, figure=fig, left=0.02, right=0.95, top=0.93, bottom=0.07, wspace=0.55, hspace=0.45)
    ax3 = fig.add_subplot(gs[:, :3], projection='3d')
    ax_z = fig.add_subplot(gs[0, 3:])
    ax_a = fig.add_subplot(gs[1, 3:])
    ax_m = fig.add_subplot(gs[2, 3:])

    # ---- 3-D scene ---------------------------------------------------------
    ax3.set_facecolor('#0b1220')
    ax3.set_xlim(-3, 9); ax3.set_ylim(-3, 9); ax3.set_zlim(0, 6)
    ax3.set_box_aspect((12, 12, 6))
    ax3.set_xlabel('East [m]', labelpad=2); ax3.set_ylabel('North [m]', labelpad=2); ax3.set_zlabel('Up [m]', labelpad=2)
    for axis in (ax3.xaxis, ax3.yaxis, ax3.zaxis):
        axis.pane.set_alpha(0.04)
        axis.pane.set_edgecolor('#1e293b')
        axis._axinfo['grid']['color'] = (0.2, 0.25, 0.33, 0.5)
        axis.line.set_color('#475569')
    ax3.tick_params(colors='#94a3b8', labelsize=7)
    ax3.view_init(elev=24, azim=-58)

    gx, gy = np.meshgrid(np.arange(-3, 9.1, 1.0), np.arange(-3, 9.1, 1.0))
    ax3.plot_wireframe(gx, gy, np.zeros_like(gx), color='#1e293b', lw=0.5)

    ax3.plot(wps[:, 0], wps[:, 1], wps[:, 2], color='#475569', ls='--', lw=1.2)
    for k, w in enumerate(wps):
        ax3.scatter(*w, s=36, color='#fbbf24', marker='D', depthshade=False)
        ax3.text(w[0], w[1], w[2] + 0.35, f'WP{k}', color='#fbbf24', fontsize=8, ha='center')
    ax3.scatter(0, 0, 0, s=90, color='#34d399', marker='s', depthshade=False)
    ax3.text(0, 0, 0.35, 'HOME', color='#34d399', fontsize=8, ha='center')

    trail,  = ax3.plot([], [], [], color='#38bdf8', lw=1.6, alpha=0.9)
    shadow, = ax3.plot([], [], [], color='#000000', lw=2.5, alpha=0.5)
    ref_pt, = ax3.plot([], [], [], 'o', color='#f472b6', ms=5, alpha=0.9)
    arms    = [ax3.plot([], [], [], color='#e2e8f0', lw=2.4)[0] for _ in range(motors)]
    heading, = ax3.plot([], [], [], color='#f97316', lw=3.0)
    rotors  = [ax3.plot([], [], [], lw=2.0)[0] for _ in range(motors)]
    blades  = [[ax3.plot([], [], [], lw=3.0, solid_capstyle='round')[0] for _ in range(2)] for _ in range(motors)]
    dmg_txt = [ax3.text(0, 0, 0, '', color=C_BAD, fontsize=8, ha='center', fontweight='bold') for _ in range(motors)]
    # robot arm hanging under the airframe: shoulder at the body, two links, gripper
    arm_links, = ax3.plot([], [], [], color='#fb923c', lw=4.0, solid_capstyle='round')
    arm_joint, = ax3.plot([], [], [], 'o', color='#fed7aa', ms=6)
    arm_tq,    = ax3.plot([], [], [], color='#f43f5e', lw=2.5)
    arm_txt = ax3.text(0, 0, 0, '', color='#fb923c', fontsize=8, ha='center')
    thrust_v, = ax3.plot([], [], [], color='#fde047', lw=1.5)
    wind_l, = ax3.plot([], [], [], color='#7dd3fc', lw=2.5)
    wind_t, = ax3.plot([], [], [], '>', color='#7dd3fc', ms=6)
    wind_txt = ax3.text(-2.6, 8.4, 5.6, '', color='#7dd3fc', fontsize=9)
    hud = ax3.text2D(0.02, 0.97, '', transform=ax3.transAxes, fontsize=10, va='top',
                     family='monospace', color='#e2e8f0')
    fig.suptitle(f'FireDrone SITL  -  {title}', color='#e2e8f0', fontsize=13, y=0.985)

    # ---- telemetry panels -------------------------------------------------
    def style(ax, ylabel):
        ax.set_facecolor('#0f172a'); ax.grid(alpha=0.2)
        ax.set_ylabel(ylabel, fontsize=8); ax.tick_params(labelsize=7)
        ax.set_xlim(0, t[-1])

    style(ax_z, 'altitude [m]')
    ax_z.plot(t, df.rz, color='#64748b', ls='--', lw=1)
    ax_z.plot(t, df.pz, color='#38bdf8', lw=1.3)
    ax_e = ax_z.twinx()
    err = np.sqrt((df.px - df.rx) ** 2 + (df.py - df.ry) ** 2)
    ax_e.plot(t, err, color='#f472b6', lw=0.9, alpha=0.8)
    ax_e.set_ylabel('xy error [m]', fontsize=8, color='#f472b6'); ax_e.tick_params(labelsize=7)

    style(ax_a, 'attitude [deg]')
    ax_a.plot(t, df.roll,  color='#f87171', lw=0.9, label='roll')
    ax_a.plot(t, df.pitch, color='#34d399', lw=0.9, label='pitch')
    ax_a.plot(t, df.tilt,  color='#fbbf24', lw=0.9, ls=':', label='tilt')
    ax_a.legend(fontsize=7, ncol=3, loc='upper right')

    for lbl, mask, col in [('wind / gust', df.wind.values > 5.0, '#7dd3fc'),
                           ('arm torque',  np.hypot(df.tq_x, df.tq_y).values > 1e-6, '#fbbf24'),
                           ('IMU fault',   df.imu_fault.values > 0, '#c084fc'),
                           ('RC link lost', df.link.values == 0, '#fb923c'),
                           ('motor degraded', (eff < 0.999).any(1), '#f87171')]:
        for a, b in fault_spans(t, mask):
            for ax in (ax_z, ax_a):
                ax.axvspan(a, b, color=col, alpha=0.13, lw=0)
    legend_items = [Line2D([0], [0], color=c, lw=6, alpha=0.4, label=l) for l, c in
                    [('wind', '#7dd3fc'), ('arm torque', '#fbbf24'), ('IMU fault', '#c084fc'),
                     ('RC lost', '#fb923c'), ('motor', '#f87171')]]
    ax_a.legend(handles=legend_items + [Line2D([0], [0], color=c, lw=1, label=l) for l, c in
                [('roll', '#f87171'), ('pitch', '#34d399')]], fontsize=6, ncol=4, loc='lower right')

    cur_z = ax_z.axvline(0, color='#e2e8f0', lw=0.8)
    cur_a = ax_a.axvline(0, color='#e2e8f0', lw=0.8)

    ax_m.set_facecolor('#0f172a'); ax_m.set_ylim(0, 1); ax_m.set_xlim(-0.6, motors-0.4)
    ax_m.set_xticks(range(motors)); ax_m.set_xticklabels(MOTOR_LBL, fontsize=8)
    ax_m.set_ylabel('command (PWM out)', fontsize=8); ax_m.tick_params(labelsize=7); ax_m.grid(alpha=0.2, axis='y')
    bars = ax_m.bar(range(motors), np.zeros(motors), color=C_OK, width=0.6)
    btxt = [ax_m.text(i, 0.02, '', ha='center', fontsize=8, color='#0b1220') for i in range(motors)]

    fig.text(0.62, 0.02, '', color='#94a3b8', fontsize=8)

    # ---- per-frame update ---------------------------------------------------
    def draw(i):
        R_i, p = R[i], pos[i]
        world = lambda b: (R_i @ (b * 1.0).T).T + p

        j0 = max(0, i - 700)
        trail.set_data_3d(pos[j0:i + 1, 0], pos[j0:i + 1, 1], pos[j0:i + 1, 2])
        shadow.set_data_3d(pos[j0:i + 1, 0], pos[j0:i + 1, 1], np.zeros(i + 1 - j0))
        ref_pt.set_data_3d([df.rx[i]], [df.ry[i]], [max(df.rz[i], 0.0)])

        circ = circle()
        for m in range(motors):
            tip = world(MOTOR_POS[m:m + 1])[0]
            ctr = world(np.zeros((1, 3)))[0]
            arms[m].set_data_3d([ctr[0], tip[0]], [ctr[1], tip[1]], [ctr[2], tip[2]])
            ring = world(circ + MOTOR_POS[m])
            rotors[m].set_data_3d(ring[:, 0], ring[:, 1], ring[:, 2])
            if eff[i, m] < 0.01:
                col = C_BAD
            elif eff[i, m] < 0.999:
                col = C_WARN
            else:
                col = THRUST_CMAP(0.15 + 0.8 * u[i, m])
            rotors[m].set_color(col)
        # blades: spin phase follows the command, a degraded prop has a shortened blade
        spin = spin_all[i]
        for m in range(motors):
            dead = eff[i, m] < 0.01
            for b in range(2):
                ang = spin[m] * ((1 if m % 4 in (0, 3) else -1) * (1 if m < 4 else -1)) + b * np.pi
                ln  = ROTOR_R * (0.0 if dead else (1.0 if b == 0 else max(eff[i, m], 0.25)))
                tip = np.array([ln * np.cos(ang), ln * np.sin(ang), 0.0]) + MOTOR_POS[m]
                seg = world(np.array([MOTOR_POS[m], tip]))
                blades[m][b].set_data_3d(seg[:, 0], seg[:, 1], seg[:, 2])
                blades[m][b].set_color(C_BAD if dead else (C_WARN if eff[i, m] < 0.999 else '#cbd5e1'))
            lab = world(MOTOR_POS[m:m + 1] + np.array([[0, 0, 0.6]]))[0]
            dmg_txt[m].set_position((lab[0], lab[1])); dmg_txt[m].set_3d_properties(lab[2])
            dmg_txt[m].set_text('' if eff[i, m] >= 0.999 else
                                (f'{MOTOR_LBL[m]} DEAD' if dead else f'{MOTOR_LBL[m]} {100 * eff[i, m]:.0f}%'))

        # arm swing angle from the logged reaction torque (torque ~ -accel of the arm), per axis
        a_x = -1.1 * tq[i, 0] / tq_max     # swing about body x -> moves in y-z
        a_y = -1.1 * tq[i, 1] / tq_max     # swing about body y -> moves in x-z
        L1, L2 = 0.8, 0.7
        d1 = np.array([np.sin(a_y), -np.sin(a_x), -np.cos(a_x) * np.cos(a_y)])
        d1 /= np.linalg.norm(d1)
        d2 = np.array([np.sin(1.4 * a_y), -np.sin(1.4 * a_x), -np.cos(1.4 * a_x) * np.cos(1.4 * a_y)])
        d2 /= np.linalg.norm(d2)
        sh = np.array([0.0, 0.0, -0.12])
        el = sh + L1 * d1
        gr = el + L2 * d2
        chain = world(np.array([sh, el, gr]))
        arm_links.set_data_3d(chain[:, 0], chain[:, 1], chain[:, 2])
        arm_joint.set_data_3d(chain[:, 0], chain[:, 1], chain[:, 2])
        active = np.hypot(*tq[i]) > 1e-6
        if active:
            tv = R_i @ np.array([tq[i, 0], tq[i, 1], 0.0]) / tq_max * 1.2
            arm_tq.set_data_3d([p[0], p[0] + tv[0]], [p[1], p[1] + tv[1]], [p[2], p[2] + tv[2]])
        else:
            arm_tq.set_data_3d([], [], [])
        arm_txt.set_position((chain[2, 0], chain[2, 1])); arm_txt.set_3d_properties(chain[2, 2] - 0.35)
        arm_txt.set_text(f'arm tq {np.hypot(*tq[i]):.2f} Nm' if active else 'arm idle')

        nose = world(np.array([[0, 0, 0], [ARM * 1.15, 0, 0]]))
        heading.set_data_3d(nose[:, 0], nose[:, 1], nose[:, 2])

        z_b = R_i[:, 2] * thr[i].sum() * 0.35
        thrust_v.set_data_3d([p[0], p[0] + z_b[0]], [p[1], p[1] + z_b[1]], [p[2], p[2] + z_b[2]])

        base = np.array([-2.4, 8.2, 4.6])
        tipw = base + wind[i] * 0.35
        wind_l.set_data_3d([base[0], tipw[0]], [base[1], tipw[1]], [base[2], tipw[2]])
        wind_t.set_data_3d([tipw[0]], [tipw[1]], [tipw[2]])
        wind_txt.set_text(f'wind {df.wind[i]:.1f} m/s')

        ph = int(df.phase[i])
        fs = common.FAULTS[int(df.fault[i])]
        arm = 'ARMED' if df.arm[i] else 'DISARMED'
        hud.set_text(f't = {t[i]:6.2f} s   {common.PHASES[ph]:<8s} WP{int(df.wp[i])} lap {int(df.lap[i]) if 'lap' in df else 0}\n'
                     f'pos  {p[0]:+6.2f} {p[1]:+6.2f} {p[2]:+5.2f} m\n'
                     f'spd  {df.speed[i]:5.2f} m/s   tilt {df.tilt[i]:4.1f} deg\n'
                     f'FC   {arm}   fault: {fs}\n'
                     f'RC link {"UP" if df.link[i] else "LOST"}   IMU {"FAULT" if df.imu_fault[i] else "ok"}')
        hud.set_color(C_BAD if df.fault[i] >= 2 or not df.arm[i] and ph in (2, 3, 4) else
                      (C_WARN if df.fault[i] == 1 or not df.link[i] else '#e2e8f0'))

        cur_z.set_xdata([t[i], t[i]]); cur_a.set_xdata([t[i], t[i]])
        for m, b in enumerate(bars):
            b.set_height(u[i, m])
            b.set_color(C_BAD if eff[i, m] < 0.01 else (C_WARN if eff[i, m] < 0.999 else THRUST_CMAP(0.15 + 0.8 * u[i, m])))
            btxt[m].set_text(f'{int(df[f"pwm{m + 1}"][i])}')
            btxt[m].set_y(min(u[i, m], 0.9) * 0.5)

        if follow:
            h = 1.7
            ax3.set_xlim(p[0] - h, p[0] + h); ax3.set_ylim(p[1] - h, p[1] + h); ax3.set_zlim(max(p[2] - h, 0.0), max(p[2] - h, 0.0) + 2 * h)
            ax3.set_box_aspect((1, 1, 1))
        if args.orbit:
            ax3.view_init(elev=24, azim=-58 + 40 * t[i] / t[-1])
        return []

    if args.frame is not None:
        i = int(np.argmin(np.abs(t - args.frame)))
        draw(i)
        out = args.save or os.path.join(common.OUT, f'{title}_t{args.frame:.0f}.png')
        fig.savefig(out, dpi=100, facecolor=fig.get_facecolor())
        print('wrote', out)
        return

    frames = list(range(0, N, args.step))
    anim = animation.FuncAnimation(fig, draw, frames=frames, interval=1000 / args.fps, blit=False, repeat=True)

    if args.save:
        os.makedirs(os.path.dirname(os.path.abspath(args.save)), exist_ok=True)
        writer = (animation.PillowWriter(fps=args.fps) if args.save.endswith('.gif')
                  else animation.FFMpegWriter(fps=args.fps, bitrate=4000))
        print(f'rendering {len(frames)} frames -> {args.save}')
        anim.save(args.save, writer=writer, dpi=90,
                  progress_callback=lambda i, n: print(f'\r  {i + 1}/{n}', end='') if i % 25 == 0 else None)
        print('\ndone')
    else:
        plt.show()


if __name__ == '__main__':
    main()
