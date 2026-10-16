#!/usr/bin/env python3
"""Repeat isolated trials, a multi-lap gauntlet, then the destructive finale.

Each trial is a new process/airframe. A crash never resets the vehicle in flight.
This runner targets an uncalibrated coaxial X8 design study, not flight qualification.
"""
import argparse
import csv
import json
from pathlib import Path
import re
import subprocess

ROOT = Path(__file__).resolve().parents[1]
TRIALS = ("nominal", "gust", "arm_swing", "accel_freeze", "gyro_bias",
          "accel_spikes", "gps_loss", "imu_loss", "rc_loss", "prop_damage", "motor_fail",
          "rc_lost_land", "gauntlet", "all_failures")


def positive(value):
    n = int(value)
    if n < 1:
        raise argparse.ArgumentTypeError("must be positive")
    return n


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", type=Path, default=ROOT / "configs" / "x8_design_study.conf")
    ap.add_argument("--cycles", type=positive, default=3)
    ap.add_argument("--laps", type=positive, default=8, help="gauntlet waypoint circuits")
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--duration", type=float, default=200)
    ap.add_argument("--out", type=Path, default=ROOT / "out" / "campaign")
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    if not args.config.is_file():
        ap.error("config file does not exist")
    import hashlib
    manifest = dict(config_path=str(args.config.resolve()),
                    config_sha256=hashlib.sha256(args.config.read_bytes()).hexdigest(),
                    config_text=args.config.read_text(),
                    binary_sha256=hashlib.sha256((ROOT / "firedrone_x8_sim").read_bytes()).hexdigest(),
                    cycles=args.cycles, laps=args.laps, seed=args.seed,
                    duration=args.duration, status="UNCALIBRATED_DESIGN_STUDY")
    (args.out / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    results = []
    for cycle in range(1, args.cycles + 1):
        folder = args.out / f"cycle_{cycle:03d}"
        folder.mkdir(exist_ok=True)
        for trial in TRIALS:
            path = folder / f"{trial}.csv"
            cmd = [str(ROOT / "firedrone_x8_sim"), "--scenario", trial,
                   "--att", "ukf", "--seed", str(args.seed + cycle - 1),
                   "--duration", str(args.duration), "--config", str(args.config), "--out", str(path)]
            if trial == "gauntlet":
                cmd.extend(["--laps", str(args.laps)])
            run = subprocess.run(cmd, capture_output=True, text=True)
            (folder / f"{trial}.log").write_text(run.stdout + run.stderr)
            if run.returncode:
                raise RuntimeError(f"{trial}: {run.stderr or run.stdout}")
            match = re.search(r"result   : (\w+).*touchdown vz ([-.0-9]+)", run.stdout)
            coverage = re.search(r"coverage : (\d+)/(\d+) events \| laps (\d+)/(\d+)", run.stdout)
            if match is None or coverage is None:
                raise RuntimeError(f"{trial}: missing result/coverage")
            fired, total, laps, requested = map(int, coverage.groups())
            row = dict(cycle=cycle, seed=args.seed + cycle - 1, scenario=trial,
                       airframe="coaxial_x8_uncalibrated", attitude="ukf", outcome=match[1],
                       impact_vz=float(match[2]), events_fired=fired, events_total=total,
                       coverage_complete=fired == total, laps=laps, laps_requested=requested,
                       mission_complete=match[1] == "LANDED" and laps >= requested,
                       csv=str(path))
            results.append(row)
            (args.out / "summary.json").write_text(json.dumps(results, indent=2) + "\n")
            with (args.out / "summary.csv").open("w", newline="") as f:
                writer = csv.DictWriter(f, fieldnames=list(row))
                writer.writeheader()
                writer.writerows(results)
            print(f"cycle {cycle}/{args.cycles} {trial:15s} {match[1]:10s} "
                  f"events {fired}/{total} laps {laps}/{requested}", flush=True)
    print(f"Results: {args.out / 'summary.csv'}")
    # Execution success is not a flight qualification verdict.


if __name__ == "__main__":
    main()
