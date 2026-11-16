# FireDrone

FireDrone is an R&E research project on fault-aware flight-control firmware for a robotic-arm-equipped firefighting drone. It combines two distinct development tracks:

- **Mobula8 research firmware**: a real STM32F405 quadrotor running a patched Betaflight 4.4.2 build, used to validate embedded attitude estimation, bounded rate correction, logging, and safety fallbacks.
- **FireDrone X8 simulator**: a real-time X8/coaxial simulation for design studies involving wind, actuator and sensor faults, and robotic-arm reaction torque.

The project investigates how UKF attitude estimation and bounded INDI·ADRC·MRAC correction can be integrated into an existing flight stack without replacing its flight-proven rate PID, mixer, receiver, or failsafe paths.

> **Research status:** The Mobula8 firmware and associated Blackbox analysis demonstrate implementation, timing, logging, and selected flight behavior. They do **not** establish improved disturbance rejection under a controlled A/B flight experiment. The X8 airframe and robotic-arm feedforward path remain simulation/design work, not flight-validated hardware.

## Repository map

| Path | Purpose |
|---|---|
| [`firmware/mobula8/`](firmware/mobula8/) | Betaflight 4.4.2 research firmware for the real Mobula8 test vehicle, CLI snapshots, patches, host tests, firmware artifacts, and Blackbox logs. |
| [`simulation/simulator/`](simulation/simulator/) | X8 USB-joystick simulator, legacy X4/F405 SITL environment, physics model, controller core, tests, and visualizations. |
| [`PCB-Design/`](PCB-Design/) | KiCad hardware and power-system design files. |
| [`source/`](source/) | Project source material and references. |
| [`report/`](report/) | R&E report source and generated report assets. |
| [`output/`](output/) | Generated deliverables, including PDFs. |

## Control architecture

```text
RC / ANGLE reference
        │
        ▼
UKF attitude estimate ──► Betaflight outer attitude loop
                              │
                              ▼
                    Stock Betaflight rate PID
                              │
      bounded INDI · ADRC · MRAC correction (experimental)
                              │
                              ▼
                     Stock mixer / DShot motors
```

The experimental controller is intentionally **low authority**. It adds a bounded correction to the baseline PID sum only after the expected sensor, receiver, scheduler, UKF, and saturation conditions are valid. Invalid or stale estimates, timing/CPU faults, sensor faults, saturation, and mode/failsafe conditions return control to the baseline Betaflight path.

## Quick start

### X8 manual simulator

The default simulator mode is manual USB-joystick angle-mode flight.

```bash
cd simulation/simulator
./run.sh
```

Press `C` in the simulator to calibrate the joystick. See the [simulator README](simulation/simulator/README.md) for controls, fault scenarios, model assumptions, and test commands.

Useful commands:

```bash
./run.sh fpv --list-devices
./run.sh campaign --cycles 3 --laps 8
./run.sh view gust
./run.sh timing
```

### Mobula8 firmware

The Mobula8 track has hardware-specific configuration and flashing instructions. Read the [Mobula8 README](firmware/mobula8/README.md) before building, configuring, or connecting a flight battery.

```bash
cd firmware/mobula8
python3 scripts/build.py
bash scripts/test.sh
```

The generated candidate firmware is at:

```text
firmware/mobula8/dist/FireDrone_Mobula8_STM32F405_BF4.4.2_CANDIDATE.hex
```

## Evidence and validation boundaries

The project keeps simulator evidence, host tests, bench tests, Blackbox logs, and flight observations separate.

| Evidence | What it supports | What it does not support |
|---|---|---|
| Host tests | Numerical guards, bounds, reset/fallback paths, and configuration handling | Real-flight stability or performance |
| Bench Blackbox logs | Execution time, initialization, logging schema, and inactive-output checks | Closed-loop flight behavior |
| Mobula8 flight logs | Observed behavior of the real test vehicle in recorded flights | Controlled improvement claims without matched baseline trials |
| X8 simulation | Design exploration, fault-path logic, and controller integration | X8 airframe or robotic-arm flight performance |

The current Mobula8 logs and verification records are organized under [`firmware/mobula8/logs/`](firmware/mobula8/logs/). The research report and its generated PDF are in [`report/`](report/) and [`output/pdf/`](output/pdf/).

## Safety

- Remove propellers and disconnect the flight battery before firmware flashing, CLI changes, bench work, or receiver/motor checks.
- The Mobula8 firmware artifact is tied to the captured **FPVM BETAFLIGHTF4 / STM32F405** configuration. Do not flash it to another board variant based only on a similar product name.
- Preserve a full firmware readback and CLI snapshot before changing a real aircraft.
- A successful build, simulator run, host test, or flash readback does not prove safe flight.

## License

Project materials are released under the repository [LICENSE](LICENSE). Betaflight-derived components retain their respective upstream licensing terms.
