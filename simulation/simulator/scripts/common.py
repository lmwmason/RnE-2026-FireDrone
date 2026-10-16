import os
import subprocess
import numpy as np
import pandas as pd

ROOT   = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT    = os.path.join(ROOT, 'out')
BINARY = {
    'x8': os.path.join(ROOT, 'firedrone_x8_sim'),
    'asis':       os.path.join(ROOT, 'firedrone_sim_orig'),
    'patched':    os.path.join(ROOT, 'firedrone_sim'),
    'patched_og': os.path.join(ROOT, 'firedrone_sim'),
    'patched_o2': os.path.join(ROOT, 'firedrone_sim'),
    'patched_ukf': os.path.join(ROOT, 'firedrone_sim'),
}
BUILD_LABEL = {
    'x8': 'coaxial X8 (uncalibrated design study)',
    'asis':       'original firmware (truth diagnostic)',
    'patched':    'current firmware (UKF)',
    'patched_og': 'current + MCU load (-Og, old default)',
    'patched_o2': 'current + MCU load (-O2, new default)',
    'patched_ukf': 'current, attitude from UKF',
}


def build_args(build):
    """Extra simulator flags for a build variant (MCU compute time from timing/)."""
    if build == 'asis':
        return ['--att', 'truth']
    if build == 'patched_ukf':
        return ['--att', 'ukf']
    opt = {'patched_og': 'Og', 'patched_o2': 'O2'}.get(build)
    if opt is None:
        return []
    # fresh measurement if ./run.sh timing was run, else the committed reference numbers
    path = os.path.join(OUT, f'timing_{opt}.json')
    if not os.path.exists(path):
        path = os.path.join(ROOT, 'timing', 'reference', f'timing_{opt}.json')
    if not os.path.exists(path):
        return []
    import json
    t = json.load(open(path))
    return ['--cost-us', f"{t['even_us']:.0f},{t['odd_us']:.0f}"]

PHASES = ['PREARM', 'ARM', 'TAKEOFF', 'MISSION', 'LAND', 'DONE']
FAULTS = ['NORMAL', 'SENSOR_DEGRADED', 'MOTOR_FAULT', 'COMM_LOST', 'EMERGENCY_LAND']


def list_scenarios(build='asis'):
    out = subprocess.run([BINARY[build], '--list'], capture_output=True, text=True).stdout
    res = []
    for line in out.splitlines():
        name, _, desc = line.partition(' ')
        res.append((name, desc.strip()))
    return res


def mission_waypoints(build='asis'):
    out = subprocess.run([BINARY[build], '--mission'], capture_output=True, text=True).stdout
    return np.array([[float(v) for v in l.split()] for l in out.splitlines()])


def csv_path(build, scenario):
    return os.path.join(OUT, f'{build}_{scenario}.csv')


def run_scenario(build, scenario, seed=42, extra=()):
    os.makedirs(OUT, exist_ok=True)
    path = csv_path(build, scenario)
    cmd = [BINARY[build], '--scenario', scenario, '--out', path, '--seed', str(seed), *build_args(build), *extra]
    res = subprocess.run(cmd, capture_output=True, text=True)
    if res.returncode != 0:
        raise RuntimeError(res.stderr or res.stdout)
    return path, res.stdout


def load(path):
    df = pd.read_csv(path)
    q = df[['qw', 'qx', 'qy', 'qz']].values
    w, x, y, z = q.T
    df['roll']  = np.degrees(np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)))
    df['pitch'] = -np.degrees(np.arcsin(np.clip(2 * (w * y - z * x), -1, 1)))   # nose-up positive
    df['yaw']   = np.degrees(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
    df['tilt']  = np.degrees(np.arccos(np.clip(1 - 2 * (x * x + y * y), -1, 1)))
    df['speed'] = np.sqrt(df.vx ** 2 + df.vy ** 2 + df.vz ** 2)
    df['wind']  = np.sqrt(df.wind_x ** 2 + df.wind_y ** 2 + df.wind_z ** 2)
    motors = int(df.motor_count.iloc[0]) if 'motor_count' in df else 4
    for i in range(1, motors + 1):
        df[f'u{i}'] = ((df[f'pwm{i}'] - 1000) / 1000.0).clip(0, 1)

    u = df[['ukf_qw', 'ukf_qx', 'ukf_qy', 'ukf_qz']].values
    ww, xx, yy, zz = u.T
    zt = np.stack([2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)], 1)
    zu = np.stack([2 * (xx * zz + ww * yy), 2 * (yy * zz - ww * xx), 1 - 2 * (xx * xx + yy * yy)], 1)
    df['ukf_tilt_err'] = np.degrees(np.arccos(np.clip((zt * zu).sum(1), -1, 1)))
    return df


def rotation_matrices(q):
    """Body (FLU) -> world (ENU) rotation for an array of quaternions (w,x,y,z)."""
    w, x, y, z = q.T
    R = np.empty((len(q), 3, 3))
    R[:, 0, 0] = 1 - 2 * (y * y + z * z); R[:, 0, 1] = 2 * (x * y - w * z);     R[:, 0, 2] = 2 * (x * z + w * y)
    R[:, 1, 0] = 2 * (x * y + w * z);     R[:, 1, 1] = 1 - 2 * (x * x + z * z); R[:, 1, 2] = 2 * (y * z - w * x)
    R[:, 2, 0] = 2 * (x * z - w * y);     R[:, 2, 1] = 2 * (y * z + w * x);     R[:, 2, 2] = 1 - 2 * (x * x + y * y)
    return R
