# Mobula8 — 실제 비행 중인 기체 설정 기반 Betaflight 연구 펌웨어

**최신 추가 변경: SE 모드 스위치 수정 완료.** 실제 SE는 CH8/AUX4였습니다. 사용자 선택에 따라 ANGLE(1700–2100) / HORIZON(1300–1700) / ACRO(낮은 값)로 연결하고 기존 SE 수동 비퍼 범위를 해제했습니다. 세 위치의 실제 모드 전환 및 다른 설정 불변을 확인했습니다(`dist/se-mode-verification.json`). ARM과 crash-flip은 그대로입니다. 초기 원본 덤프는 보존했으므로, 원본 복원/재플래시 후에는 `cli/05-stock-pid-identification-log.cli`와 `cli/06-se-flight-modes.cli`를 다시 적용해야 현재 상태가 됩니다.

추가 비행 로그는 `logs/02.Stock-PID-Yaw-Input-Flight/report.txt`와 `overview.png`에 있습니다. 10번 로그 39.22초에서 요 입력을 보완했고, 주 분석 구간의 roll/pitch/yaw 추종 RMSE는 2.06/2.04/1.60°/s였습니다. 실험 제어와 UKF는 계속 OFF입니다.


**최신 상태 (2026-10-15): 비행 로그 수집 준비 완료.** 기체의 기존 커스텀 펌웨어를 유지하고 Blackbox를 SPIFLASH / NORMAL / 1/2로 켰습니다. 사용자 허락으로 기존 로그를 삭제해 8 MB 전체 공간을 확보했습니다. 재부팅 후 전체 설정 대조 결과, 지정한 기록 설정 외 변경 없음. fd_mode/fd_blend/fd_ukf는 모두 0이며 CLI 종료를 요청했습니다. 실제 새 비행 로그는 아직 수집하지 않았습니다. 결과: `dist/logging-preparation.json`. 아래 2026-10-14의 기록 OFF 상태는 이전 시험 기록입니다.


2026-10-14 USB에 연결된 기체에서 `version`, `status`, `dump all`, `diff all`, `resource`, `timer`, `dma`, `defaults show`를 읽었습니다. 최초 캡처 단계에서는 읽기만 수행했고 전체 덤프 불변을 확인했습니다. 이후 사용자 요청으로 아래 플래시 작업을 수행했습니다.

## 2026-10-14 기체 상태 — 플래시 완료

2026-10-14 사용자 요청으로 후보 HEX를 실제 기체에 플래시했습니다. 기존 1 MB 내부 flash 원본은 `backups/preflash/stock-full-flash.bin`에 보관했습니다. 플래시 후 1 MB 전체 읽기 검증은 일치했습니다.

첫 부팅에서 기존 EEPROM 내용이 그대로 인식되지 않아 UNCONFIGURED/NOGYRO 상태가 관측됐습니다. 전체 CLI 복원 파일을 실제 기체에 적용하고 저장/재부팅해 해결했습니다. 이후 전체 덤프 비교는 요청한 `blackbox_device = NONE` 변경을 제외하고 원본과 일치했습니다. 모터·PID·Rate·AUX 모드 등 원본 설정은 보존했습니다.

`fd_mode=0`, `fd_blend=0`, `fd_ukf=0`, Blackbox 기록 OFF. BMI270 gyro/acc, MSP OSD, 8 MB flash 인식 정상, 자이로 약 3.2 kHz, USB 벤치 CPU 약 46%. 배터리/수신기 입력이 없는 상태에서 검사했으므로 RXLOSS는 남아 있습니다. 모터 회전·실제 수신기 입력·비행은 시험하지 않았습니다.

검증 결과: `dist/flash-verification.json`, `dist/postflash-verification.json`. 기록 없는 비행 확인 설정은 `cli/04-flight-no-log.cli`입니다. 향후 다시 플래시할 때도 전체 CLI 복원과 재부팅 후 대조가 필요합니다.

## 기준 기체

- STM32F405 / `FPVM BETAFLIGHTF4`, BMI270, 8 MB SPI flash
- Betaflight **4.4.2**, upstream commit `23d066d0824ecdb2cd0986a31eb6603a89d15a3e`
- 실제 저장된 수신기 설정: **SBUS, UART6**. 이전 Pavo 개발 시의 CRSF 설정으로 덮어쓰지 않았습니다.
- QUADX, 보드 roll 180°, gyro CW0, 채널맵 RETA1234
- PID 프로필 0–3 / Rate 프로필 0–3 모두 보존. 현재 선택 **PID 2 / Rate 0**.
- 기체 이름 Mobula8, 조종사 Grass FPV, 기존 보정·배터리·failsafe·OSD·VTX·모드·필터 설정 그대로 보존.

## 모터 설정

| FC 출력 | 핀 | 타이머 | DMA |
|---|---|---|---|
| MOTOR 1 | PB0 | TIM3 CH3 AF2 | DMA1 Stream7 Channel5 / option 0 |
| MOTOR 2 | PB1 | TIM3 CH4 AF2 | DMA1 Stream2 Channel5 / option 0 |
| MOTOR 3 | PC9 | TIM8 CH4 AF3 | DMA2 Stream7 Channel7 / option 0 |
| MOTOR 4 | PC8 | TIM8 CH3 AF3 | DMA2 Stream2 Channel0 / option 0 |

`motor_output_reordering = 2,3,0,1,4,5,6,7`, DSHOT300, bidirectional DShot ON, motor poles 12, DShot idle 1000, burst OFF, EDT OFF, bitbang AUTO, yaw_motors_reversed OFF. 출력 제한·dynamic idle·RPM filter 등 나머지 항목도 전체 덤프에 포함됩니다. 이 표는 FC 리소스이며, 실제 프로펠러 위치/ESC 내부 회전 방향을 새로 측정했다는 뜻은 아닙니다.

## 파일과 검증

- `configs/stock-dump-all.txt`: 기체의 전체 원본 덤프
- `configs/stock-*.txt`: 버전·상태·리소스·타이머·DMA·제조사 기본값·diff 원본
- `configs/full-defaults.txt`: 전체 설정을 담은 펌웨어 내장 기본값
- `cli/00-restore-full-stock.cli`: 동일 보드/버전용 수동 전체 복원 파일. 실행하면 초기화 후 저장/재부팅합니다. 이번 플래시 후 기체에 적용하고 Blackbox 기록을 끈 뒤 저장했습니다.
- `dist/FireDrone_Mobula8_STM32F405_BF4.4.2_CANDIDATE.hex`: 빌드된 연구 펌웨어 후보
- `dist/config-verification.json`: 실제 ARM CLI 재입력·재출력 대조 결과
- `dist/device-readback.json`: 작업 전후 연결 기체 설정 불변 확인
- `dist/manifest.json`: 소스·컴파일러·HEX SHA-256
- `legacy/pavo-pico2/`: 이전 Pavo 작업 보관. 현재 Mobula8 빌드에 사용하지 않습니다.

`diff all`만 적용한 것이 아닙니다. 전체 덤프의 `set` 917개와 리소스·타이머·DMA·모드·포트 등 모든 명령 순서를 보존했습니다. 실행 제어용 `batch start`를 포함해 1,322개이며, 배치 명령을 제외하면 1,321개입니다. 최종 상태 기준 1,308개 항목을 프로필별로 비교합니다. MCU UID는 칩의 읽기 전용 식별자여서 비교에서 제외하며 원본에는 보관합니다.

검증은 빌드된 ARM ELF의 CLI와 설정 저장 구조를 Unicorn에서 실행합니다. 하드웨어 초기화·출력만 대체하며, full defaults 입력과 부팅 시 설정 유효성 검사 후 dump all 값을 비교합니다. 실제 비행/모터 검증은 아닙니다. Intel HEX 내장 설정 바이트 일치, 1 MB 주소 범위, 설정 flash sector 미침범도 검사합니다.

## 재빌드

```sh
python3 scripts/build.py
bash scripts/test.sh
python3 -m venv .venv
.venv/bin/pip install unicorn==2.1.4 pyelftools==0.33
.venv/bin/python scripts/verify_emulated.py
python3 scripts/verify_snapshot.py
```

ARM GCC 10.3.1은 `tools/gcc-arm-none-eabi-10.3-2021.10/`에 있습니다. `prepare.py`가 고정 upstream에 검토 가능한 패치를 적용하고 `research/`와 전체 기본값을 복사합니다. 기존 CLI 기본값 공간 16 KiB는 전체 덤프에 부족하므로 flash 끝 64 KiB를 예약했습니다. EEPROM 주소는 유지합니다. 전체 캡처 기본값을 읽을 때만 simplified tuning 자동 재계산을 생략하여 캡처한 PID/필터 값이 바뀌지 않도록 했습니다. 원래 기체에 있던 Acro Trainer도 활성화했습니다.

## 기체에 넣을 때

이 HEX는 이 기체에서 읽은 **FPVM BETAFLIGHTF4 / STM32F405** 기준입니다. 다른 Mobula8 보드 변형에 모델명만 보고 사용하지 마세요. 현재 기체에는 플래시와 전체 CLI 복원까지 완료했습니다.

1. 프로펠러를 제거하고 원본 덤프를 별도로 보관합니다.
2. Betaflight Configurator에서 로컬 HEX를 선택해 플래시합니다. 전체 erase를 했다면 내장 custom defaults를 적용합니다. 기존 EEPROM을 유지한 경우에는 기존 설정을 읽으므로 새 내장 기본값이 자동 강제 적용되지는 않습니다.
3. 확실히 캡처 시점으로 복원하려면 CLI에 `00-restore-full-stock.cli`를 적용합니다. 연구 기능을 꺼 두려면 `set fd_mode = 0`, `set fd_blend = 0`, `set fd_ukf = 0`, `save`를 적용합니다.
4. 재연결 후 `dump all`을 캡처하여 `configs/stock-dump-all.txt`와 비교하고, 보드 자세·수신기·각 모터 출력 순서와 실제 회전 방향을 확인합니다.

ESC 펌웨어/EEPROM(Bluejay·BLHeli 회전 방향, PWM 주파수, startup power, timing 등), 수신기 자체 펌웨어/바인딩, OSD 폰트, Blackbox 로그는 FC CLI 설정과 별도입니다. 이번에는 읽거나 변경하지 않았습니다. 동일 ESC를 그대로 쓰므로 기존 내부 설정은 유지되며, 다른 ESC까지 복제한 것은 아닙니다. USB 상태에서 배터리 전압은 0 V로 관측되었습니다.

설정 일치가 커스텀 펌웨어의 동일 비행 성능을 증명하지는 않습니다. 공식 일반 STM32F405 빌드는 기존 기체 빌드보다 추가 컴파일 기능/설정이 있으며 대조 보고서에 표시합니다. 캡처된 기능 활성화 마스크는 동일하게 유지됩니다.

## 연구 기능

출고 기본값은 `fd_mode=0`, `fd_blend=0`, `fd_ukf=0`, effectiveness 0입니다. 기본 비행 PID 출력에 실험 보정을 더하지 않습니다. UKF는 disarmed bench 전용이며, 실제 기체 모델 식별 전에는 INDI/ADRC/MRAC 혼합 제어를 활성화하지 않습니다. `cli/01–03`은 선택적 실험 설정이며 원본 설정 복원과 별개입니다.

4.4.2 Blackbox debug 채널은 4개입니다.

| debug_mode | debug[0..3] |
|---|---|
| FIREDRONE | 목표 각속도×10, 측정 각속도×10, 기본 PID 합×10, 후보 출력×10 |
| FIREDRONE_OBSERVER | 외란 추정×10, 보정량×10, 유효 여부, 실행시간 초과 latch |
| FIREDRONE_UKF | UKF roll/pitch, BF roll/pitch (0.1°) |
| FIREDRONE_TIMING | 소요 μs, 최대 μs, 목표 주기 μs, bit0=시간 초과 / bit1=UKF 유효 |

확인 후 CLI를 종료하고 재부팅했습니다. 실제 시험 전 USB를 분리하고 조종기 입력과 모터 동작을 확인하세요.

## 다음 단계: 기존 PID 비행 로그 수집 준비

`cli/05-stock-pid-identification-log.cli`는 실험 제어기/UKF를 OFF로 유지하며 일반 Blackbox 로그만 켭니다. 기존 `01-baseline-log.cli`의 연구 shadow 실행과 구분됩니다. 모터 출력, PID 각 항, 목표/측정 각속도, 조종 입력, 배터리·가속도 데이터를 기록합니다. 현재 약 3.2 kHz PID 주기의 절반 빈도로 기록하도록 설정하며 실제 표본 간격은 로그에서 검증합니다.

적용 전 USB 연결 기체의 시동 해제·버전·전체 설정과 flash 여유 공간을 확인합니다. 기존 로그를 자동 삭제하지 않습니다. CLI 파일을 만들었다는 것만으로 기체에 적용된 것은 아닙니다.

첫 수집은 익숙한 ANGLE 모드에서 사람/장애물과 거리를 확보하고 약 30–45초 정도 수행합니다. 잠깐 안정적으로 호버링하고, 롤·피치·요를 한 축씩 작게 움직였다가 멈추는 입력을 각각 2–3회 줍니다. 급격한 스로틀, 큰 기울임, 플립은 필요 없습니다. 공간이나 조종 여건이 맞지 않으면 평소의 안정적인 짧은 비행으로 충분하며, 데이터 부족 여부는 분석 시 판단합니다.

착륙·시동 해제 후 배터리를 분리하고 USB로 다시 연결하면 로그를 읽어 기록 누락, 출력 포화, 진동, 목표 추종, 축별 입력/응답을 확인할 수 있습니다. 한 번의 짧은 로그가 모든 계수 식별을 보장하지는 않습니다. `identify.py --tau-ms`는 독립적으로 추정한 모터 응답 시간이 필요하므로 임의의 기본값을 실측값으로 간주하지 않습니다. 검증된 값이 생기기 전 실험 제어기를 켜거나 계수를 자동 적용하지 않습니다.

## 2026-10-15 첫 비행 로그 분석 완료

`logs/01.Stock-PID-Baseline-Flight/report.txt`와 `overview.png`에 결과를 보관했습니다. 43.48초 로그에서 중간 2–40초 구간의 roll/pitch 추종 RMSE는 2.05/2.85°/s였고 최대 모터 출력 포화는 없었습니다. 실제 기록 모드는 ACRO였습니다. 착륙 접촉은 사용자가 확인했고, 종료 구간을 주 분석에서 제외했습니다. 요 입력이 부족해 3축 계수 식별은 완료되지 않았습니다. UKF와 실험 제어기는 활성화하지 않았으며 기록 설정은 유지했습니다.


## 2026-10-15 UKF 분할 실행 수정

첫 벤치에서 UKF 포함 최대 실행시간 853 µs가 PID 주기 312 µs를 초과했고, 시작 자세가 BF 대비 roll 약 24° 어긋났습니다. `research/ukf_sliced.c`는 원본 10상태 UKF의 예측/관측 수식을 최대 13회의 호출로 나눕니다. 매 업데이트의 센서 입력은 시작 시점 값으로 고정합니다. 원본 `research/original/ukf.c`는 보존했습니다.

초기화는 자이로가 정지 상태이고 가속도 기반 자세와 BF 자세가 3° 이내로 1초 유지된 후 수행합니다. 기존 가속도 보정을 기준으로 바이어스 초기 공분산을 1e-6, 과정 잡음을 0으로 제한합니다. 분할 실행 중 상태는 내부 작업용이며 완료 시점에만 자세 debug 값을 갱신합니다. ARM 시 계산을 중단하며 비행 PID 입력으로 사용하지 않습니다.

호스트 테스트는 일반 최적화/UBSan 및 fast-math에서 각각 2,000회 원본 수식과 분할 수식의 상태·공분산을 비교하고, 가속도 게이트·잘못된 입력·Cholesky 실패를 확인합니다. 실제 어댑터 테스트는 30° 초기 자세, 1초 초기화 대기, ARM 중단, PID 출력 보존을 확인합니다. 이 테스트는 실기 연산시간이나 비행 검증을 대신하지 않습니다. 새 후보 빌드는 설정 1,308개 항목의 ARM CLI 대조를 통과했습니다. 실기 측정 자료는 `backups/ukf-sliced-20261015/`에 저장합니다.


분할 실행 펌웨어 실기 업로드 완료: 1MB 전체 플래시 읽기 결과가 기록 이미지와 일치했고, 업로드 전후 CLI 설정 차이는 없었습니다. 벤치 측정을 위해 이후 `fd_ukf=1`, `debug_mode=FIREDRONE_UKF`만 변경했습니다. SE 모드·모터·PID·로그 설정은 보존했습니다. 정지 시험의 최대 어댑터 시간은 102 µs(기존 853 µs), 정지 자세 BF 대비 최대 차이는 롤·피치 각각 0.1°였습니다. 35초 손 기울임 시험 409표본의 BF 대비 RMS 차이는 롤 0.586°, 피치 0.588°, 최대 1.1°/2.2°였고 표본에서 관측한 최대 PID 주기는 312 µs였습니다. BF 비교는 독립적인 자세 정답 측정이 아니며, 모든 동적 조건의 최대 실행시간이나 비행 안정성을 증명하지 않습니다. 결과: `dist/ukf-sliced-verification.json`, 그래프: `backups/ukf-sliced-20261015/comparison.png`. UKF는 여전히 disarmed 전용이며 ARM 시 중단됩니다. 새 펌웨어 비행 시험은 아직 수행하지 않았습니다.


## Current stage: UKF flight observation (2026-10-15)

The user reported a successful ANGLE flight with the sliced UKF firmware. The 43.49 s recording in `logs/05.Angle-Mode-Sliced-Firmware-Flight` confirms ANGLE activation, no RX/failsafe events, and no upper motor saturation in the 2–40 s analysis window. UKF was stopped while armed in that firmware.

The next firmware adds `fd_ukf=2`: continue UKF calculation during flight while keeping stock Betaflight attitude and PID in control (`fd_mode=0`, `fd_blend=0`). `fd_ukf=1` retains disarmed-only operation; `0` disables UKF. Mode 2 requires initialization while disarmed. Keep the aircraft stationary for at least five seconds after battery connection before ARM. If it is armed before initialization, if sensor/state validity fails, if the scheduler interval exceeds two target periods, if published output is over 30 ms old, or if measured adapter cost exceeds 35% of the PID period, the observer stops until reboot. This cannot undo time already consumed by the first overrun. No UKF attitude is fed into ANGLE or rate control.

`FIREDRONE_UKF_FLIGHT` records UKF-minus-BF roll/pitch differences in 0.1 degrees in debug[0:2], maximum measured adapter microseconds in debug[2], and flags in debug[3]: bit0 budget fault, bit1 published valid observation, bit2 stopped-until-reboot. Invalid differences use -32768, not a stale angle. `cli/07-ukf-flight-observer.cli` selects this mode and normal flash logging. Firmware safety-path host tests cover early arming, sensor faults, cadence faults, and budget faults, with unchanged PID outputs. Actual armed-flight observer performance is still pending the next user flight.

Logs now use chronological numbered English purpose folders; see `logs/README.md`. After local download/analysis, erase aircraft Blackbox storage before the next test per user preference. Do not erase local archives.


## Current stage: guarded UKF attitude input to ANGLE (2026-10-15)

Two observer flights (`logs/07.UKF-Flight-Observer-Two-Flights`) lasted 65.09/47.06 s. All recorded armed samples reported a valid observer with no stop/budget flags; maximum adapter time was 105 µs. Interior UKF-minus-BF roll/pitch RMS differences were 0.825/0.524° and 0.439/0.335°. These observations do not validate closed-loop UKF control.

`fd_ukf=3` selects UKF roll/pitch as the ANGLE outer-loop measurement. It does not replace gyro rate PID, motor mixing, yaw control, accelerometer trims, HORIZON attitude, or rescue logic. `firedroneLevelAngles()` selects both axes once per PID tick from a completed UKF update; it never reads partially updated sigma state. Entry ramps over one second to 100% UKF. Selection requires ARM, ANGLE without HORIZON/rescue/failsafe, valid receiver, enabled stabilization, no gyro overflow/crash recovery/launch/yaw spin, QUADX, `fd_mode=0`, and `fd_blend=0`.

Selection rejects stale published values (>20 ms), nonfinite values, PID dt outside (0,2 ms], UKF/BF disagreement above 5° on either axis, or absolute roll/pitch above 50° in either estimate. Rejection latches until reboot and restores the original BF ANGLE measurement. Exiting ANGLE returns to the original mode immediately and resets the entry ramp; SE middle HORIZON remains a manual way to use stock BF leveling. Existing UKF timing/cadence faults also stop the observer. A cost fault discovered after computing the current PID output affects the next selection tick; it cannot retrospectively undo that tick.

When inactive, the original BF angle-trim arithmetic is used verbatim. Healthy full-transition UKF control uses the same angle gains/filter and inner PID. `FIREDRONE_UKF_CONTROL` debug[0:2] stores the actual applied measurement difference from BF in 0.1°, debug[2] maximum combined adapter and selection µs, and debug[3] flags: bit0 budget fault, bit1 valid observer, bit2 observer stopped, bit3 UKF level input active, bit4 level input rejected, bit5 full one-second transition complete. A fully active healthy flight reports 42; healthy observation without control reports 2. `cli/08-ukf-angle-control.cli` enables this experimental mode.

Host tests cover entry ramp/full UKF selection, exact stock fallback, mode exit, stale/divergent/nonfinite values, invalid observer, timing limits, and angle envelope; tests also exercise selection against the actual adapter's published UKF state. Host tests and ground timing are not closed-loop flight validation. The first UKF-controlled flight remains pending.


Initial ANGLE-control ground timing reached 111 µs including the level selector, above the retained 35%/312 µs armed budget. This build was not approved for flight. UKF slicing, matrix/quaternion math, adapter and level selection are now compiled with Betaflight's `SPEED_OPTIMISED_SRC` (`-Ofast`) option. The same original-vs-sliced numerical, nonfinite, and fallback tests pass with `-Ofast`; final hardware timing must be remeasured. The 111 µs run is retained as `logs/08.UKF-Angle-Control-Ground-Test/before-optimization-stationary.json`.

Final optimized ANGLE build flashed and full readback matched. Settings preserved; fd_ukf=3, fd_mode=0, fd_blend=0, debug=FIREDRONE_UKF_CONTROL. Ground maximum combined adapter/selection cost: 93 µs. Stationary BF-relative maximum differences after initialization: [0.2, 0.1] degrees (initialization sentinel samples excluded). Blackbox flash empty. Actual UKF-controlled armed flight is not yet validated; verify active/full-control flags in the first flight log. See logs/08.UKF-Angle-Control-Ground-Test/verification.json.


## Controller integration optimization and model review

First UKF ANGLE flight is confirmed in `logs/09.UKF-Angle-Control-First-Flight`: 12.53 s, active flag throughout recorded arming, full UKF input after the entry ramp, no rejection/stop/budget fault, maximum adapter time 93 µs. This is a successful single-flight observation, not broad qualification.

Remaining rate-loop INDI/ADRC/MRAC code is compiled and now caches inverse effectiveness and filter coefficients, refreshes them when dt/config changes, resets observers on retuning, matches the actuator low-pass delay to the INDI measured acceleration, and normalizes MRAC learning while freezing adaptation during warmup, saturation and correction clipping. Original research sources remain preserved. The adapter allows `fd_mode=1` shadow calculations alongside `fd_ukf=3`; shadow output cannot alter the rate PID. Mode 2 combined with armed UKF remains excluded until coefficient/control review. Host tests include nine plant gain/lag combinations, output bounds, saturation learning freeze, retuning, and actual adapter shadow/UKF coexistence. Synthetic plants are not calibrated Mobula8 models.

`identify.py` now accepts the actual decoder's time/gyro unit column names and omitted zero yaw-D field. Across six downloaded flights, none of the candidate lag-sweep fits meets the existing R² >=0.6 review threshold. Results are in `logs/10.Controller-Model-Review/lag-sensitivity.json`. No identified b or tau has been applied. Current aircraft operation remains UKF ANGLE plus stock rate PID (`fd_ukf=3`, `fd_mode=0`, `fd_blend=0`, effectiveness unknown/zero). Additional controllers are compiled, not enabled for actuator output.

Next identification data: ANGLE, separate 30–45 s flights with gentle roll, pitch, yaw inputs one axis at a time, 3–5 input-and-settle repetitions each, including steady hover between inputs. Keep modest tilt and avoid aggressive maneuvers/low battery. Reserve a separate run for validation. A fit can still fail; do not auto-apply unqualified coefficients. Save locally and erase aircraft flash after analysis per user preference.


## Historical stage: UKF recovery and shadow-only candidates (2026-10-15)

This section supersedes earlier current-stage claims. Flight 14 recorded 63.35 s; UKF selection fell back at 45.156777 s while the observer remained valid. The old firmware did not record the precise rejection subcondition.

New selector limits are 8 degrees disagreement and 60 degrees absolute attitude. These envelope violations recover only after 0.5 s continuously within 4 degrees disagreement and 45 degrees attitude, followed by the existing 1 s entry ramp. Invalid/stale (>20 ms), invalid dt/nonfinite estimates, or >20 degrees disagreement remain hard-latched until reboot. Observer/cadence/CPU protections remain unchanged. Debug control flags now additionally encode hardFault in bit6 and reason in bits7..9: 1 invalid/stale, 2 dt, 3 nonfinite, 4 gross disagreement, 5 soft disagreement, 6 soft angle envelope. Flags are a bitmask, not an exact-value enum.

Aircraft: acc_trim_roll=-7, pitch unchanged, fd_ukf=3, fd_mode=1, fd_blend=0. Shadow-only b values [0.14, 0.06, 0.69], common tau20ms, adaptation0.0001. **These are NOT flight-qualified constants.** Roll and pitch estimates failed held-out validation; yaw0.69/tau5ms passed only one local segment. Common tau20ms and learning rate are exploratory design settings, not all-axis identification. INDI/ADRC/MRAC compute when the existing armed/throttle/receiver eligibility permits, but cannot add motor output in mode1. All prior motor/PID/SE settings preserved.

Firmware full readback matches; nine host tests pass including soft recovery and hard faults. Ground max research98us, all94 sampled observations valid. This disarmed check does not measure the armed extra-controller cost or validate flight recovery. Logs14 and15 hold analysis, snapshots and verification. Flash cleared and verified empty. Further direct-output enabling is not justified by these data.


## RPM identification recording (2026-10-15)

The user confirms earlier flights already used separate-axis inputs. The failed model fits do not establish a pilot-input error. Extended logging adds motor eRPM, per-motor telemetry decode age/freshness, fractional filtered gyro (0.01 deg/s), and actual final PID Sum (0.01 units). Existing UKF debug fields, motor commands, setpoints and voltage remain available simultaneously. `configs/identification-recording.json` defines units and limitations. `scripts/analyze_rpm_log.py` checks schema, freshness, configured pole conversion and coarse/fine gyro agreement. `review_identification.py` prefers the fine channels when available and preserves old-log support.

Telemetry is read from the existing cache; logging does not issue motor commands or trigger an extra decoder pass. The DShot decoder stamps valid eRPM batches once per decode. The timestamp is not a physical ESC acquisition timestamp. A fresh zero-RPM sample is valid for a stopped motor; nonzero rotating-motor response and pole count still need verification from actual operation. RPM recording is an added diagnostic, not proof that all constants can be identified.

Current controller/trim settings are preserved. Ground logging uses CLI mode ALWAYS temporarily while disarmed, then restores NORMAL and verifies the full setting snapshot. Local logs are archived before onboard erase. Keep individual acquisition flights short enough to avoid filling the 8MB flash; actual data rate is checked from the ground capture before giving a flight-duration recommendation.

Deployed RPM logger: full flash readback and all settings verified. Ground capture in logs/16 decodes without frame errors, all 15 new fields present; all four stopped ESCs report fresh zero RPM after initial acquisition. Disarmed recording max CPU65%, PID period312us, research96us. CLI recording mode restored to NORMAL. Ground data rate70.6kB/s; next acquisition flight should be short (30–40s) because motion can increase data rate. No rotating-motor or flight identification qualification yet.

## Bounded controller integration after flight 17 (2026-10-15)

This section supersedes earlier current-state descriptions. Flight 17 contains 113.63 s of fresh four-motor RPM and continuous UKF control without timing faults. Ordinary whole-plant least-squares fits still fail; RPM does not make these fits valid. RC instrumental-variable analysis uses alternating six-second train/validation blocks with half-second margins, addressing the previous chronological split of sequential-axis flight. See `scripts/identify_instrumental.py` and `logs/17.RPM-Controller-Identification-Flight/identified-candidates.json`.

Local nominal effectiveness estimates (rad/s² per BF PID unit) are roll 2.543, pitch 2.195, yaw 0.716, quantized for CLI to 2.54/2.19/0.72. Roll/pitch effective lag is 40 ms; RPM motor fits favored 40–50 ms. Yaw torque-path effective lag is 5 ms, not a measured physical motor constant. Earlier exploratory 25 ms body fits produced different effectiveness estimates and are not the deployed model. Train-block bootstrap intervals are [2.341,2.756], [2.029,2.270], [0.393,0.895]. These intervals omit model/systematic error. Independent flight 14 fits are 2.599/1.641/0.692; pitch has substantial cross-flight mismatch and negative raw-acceleration validation R². Projected IV moment error must not be described as raw prediction accuracy or full flight qualification. Nuisance state coefficients are not physical drag estimates and are not applied.

Prepared configuration: UKF3, mode2, blend5%, correction limit5, AUX4 (SE high). Added output is bounded to ±0.25 BF PID units on each axis; stock rate PID and mixer remain active. Full UKF ANGLE selection, existing armed/throttle/RX/saturation/CPU conditions, and 0.5 s rate warmup are required. SE middle HORIZON or low ACRO removes extra output and uses the original attitude behavior for that mode. UKF safety fallback removes extra output as well. Existing UKF recovery/hard-fault conditions remain intact.

INDI produces a bounded incremental candidate; ADRC ESO computes the rate/reference state, but its disturbance is not subtracted separately because that would double-count INDI acceleration compensation. MRAC reference adaptation remains normalized/projected and freezes on clipping/saturation/warmup. Its useful adaptation has not been demonstrated; very small float updates may be ineffective. Observer bandwidth20, rate gain15, derivative30Hz and adaptation0.0001 are retained design settings, NOT constants inferred from this flight. This is an experimental low-authority integration, not all-controller/full-authority validation or proof that every physical constant is identifiable.

Six new Blackbox channels record actual extra output and the unblended candidate. Debug flags bits10/11/12 indicate rate-valid/nonzero-correction/clipped-candidate. A tail-appended yaw lag setting preserves existing EEPROM layout; zero inherits common lag. Host tests cover adapter eligibility, fallback, output caps, unequal axis lag and uncertain synthetic plants, plus a known noisy feedback plant for IV identification. See the next ground-test folder for actual flash/config/schema/timing verification. Ground checks cannot establish armed behavior or improved flight performance.

Deployment complete: HEX SHA256 `ab2c382d296a61e6341ec3b08cd0f76e9b6eab46cbd3d9118ad1ca4718e469be`. Full 1MB readback matches, EEPROM prefix preserved, and only the nine requested settings changed. Ground log18: 19343 frames/12.04s, no decode failures, new six channels present, zero disarmed corrections; UKF valid continuously after initialization, research max98us. Separate stationary sample CPUmax56%, cyclemax312us. NORMAL recording restored; aircraft flash confirmed0/8388608 bytes. No motor commands issued and no armed integration flight performed.

## Impact-response update (2026-10-15)

Flight19 demonstrates software fallback, not full-time research control: flight1 has three soft disagreement exits; flight2 hard-latches at8.611811s (>20deg disagreement), keeping baseline BF control and zero additional output. No observer/CPU latch occurred. The new work addresses a specific timing weakness rather than raising these limits.

The production observer now accumulates quaternion increments from every PID gyro sample using actual elapsed time. At each100Hz snapshot, sigma-point attitudes propagate by that measured rotation (gyro is an exogenous process input; retained Q represents process uncertainty). Original equations remain available via fdUkfStart and original source is untouched. The integrated path intentionally differs from the old rate-state-only prediction. The published attitude propagates every PID tick; after a sliced correction completes, its snapshot-time attitude is propagated through the accumulated rotation since that snapshot. Existing correction-age, cadence and budget checks remain enforced.

Acceleration updates are excluded during measured norm outside0.8–1.2g or rate norm above8rad/s, and for150ms after the last such sample. The existing Mahalanobis acceleration gate is also retained. These are conservative design thresholds, not experimentally optimized constants. Gyro samples above40rad/s on any axis or invalid intervals stop observation under the existing guards; saturation cannot recover missing physical sensor information.

Rate controller saturation now clears the experimental observer/model/adaptation state. After the existing0.5s eligible warmup, added output ramps over0.25s. Baseline rate PID, effectiveness constants, ±0.25 added-PID cap, receiver/motor settings and UKF fallback thresholds are unchanged. Larger authority is not supported by a matched baseline comparison or the uncertain pitch plant model. This firmware does not promise control under arbitrary impact.

Tests include a known6ms/2000deg/s pulse entirely between100Hz samples: integrated prediction captures the12deg rotation, while the old sample path misses it. An actual-adapter test exercises sliced publication and high-rate propagation against synthetic known attitude. Original-versus-sliced legacy regression, fast-math/undefined-behavior checks, failover and model reset tests pass. Replay of both flight19 logs remains finite; new-versus-old pitch differs by up to21.15deg in the second record. Replay has half-rate gyro, reconstructed initialization, synchronous observer updates and no reference attitude: this is sensitivity evidence, not demonstrated accuracy or flight stability.

New fdAttitude[0..3] fields log propagated UKF roll/pitch and BF roll/pitch in0.01deg even when the level selector falls back. fdUkfAge reports correction age; fdUkfAccelRejected distinguishes rejected acceleration and the dynamic-input hold. This lets the next flight distinguish estimator divergence from selection behavior.

Initial impact build ground20 was not released for flight: stationary peak109us was too close to the retained35% PID-loop budget. Revised scheduling keeps quaternion integration every PID tick but alternates Euler-angle conversion with heavy UKF slices at PID intervals below0.5ms. At312us, angles refresh about every0.624ms and13 slices finish in about8.1ms. A3.2kHz actual-adapter regression verifies correction freshness and continuous selection. The later optimized ground test supersedes ground20 timing.

Final optimized impact build is deployed: HEX SHA256 `d133853efcddba5bf45d3bf79ebbd73c8a6f198dba50305d159ec7497500cdf0`. Full1MB readback matches; all live settings preserved. Ground21 decodes19334 frames over12.039s, all new fields present, zero decode failures/corrections/fault flags. UKF stays valid after initialization. Maximum research time across recording and later bench100us; bench CPUmax59%, cyclemax311us. Ground log contains incidental measured motion (maxaxis129.46deg/s); a planned hand-tilt test was declined. Maximum recorded UKF/BF difference1.0/1.2deg is not an external accuracy measurement. Aircraft flash confirmed0/8388608; recording mode NORMAL. Revised firmware has not been flown; first check remains a short ordinary ANGLE flight before interpreting impact performance.


## UKF fixed selection update (2026-10-17)

This section supersedes the earlier selection thresholds. Explicit upload authorization followed the user's earlier pause. Production UKF ANGLE selection no longer exits for 8/20-degree estimator disagreement or 60-degree tilt. The existing one-second entry blend remains. Invalid/nonfinite or stale (>20ms) estimates, invalid dt, observer/cadence/CPU faults and mode/failsafe permissions still protect the original rate PID path. UKF remains an attitude estimator; the stock inner rate PID is retained. Additional bounded controller settings and output limits are unchanged.

Normal disarm clears the level selector and experimental rate state and requests UKF reinitialization after the existing one-second quiet-ground checks. Observer-stop and CPU fault latches still require reboot. UKF3 now inhibits arming with UKF_INIT until a fresh valid estimate is published; this does not disarm in flight. The new arming flag is appended at bit26 so existing flag indices remain unchanged, and OSD ARMSWITCH filtering uses its explicit flag rather than assuming it is last.

All14 host tests pass, including actual-adapter rearming/quiet initialization, repeated large-attitude selection, retained invalid/stale protections, integrated output limits and gyro impulse propagation. Production-selector replay selects every valid row in flight22 (25273/55800/44713 rows), conditional on nominal dt and permission. This tests selection only: it is not a flight trajectory simulation, a reference-attitude validation or proof of impact recovery.

Deployed HEX SHA256 `4cda832eca54b2c78a8652373dbe6b435f95851133933934d702421281bbd29a`. Full1MB readback matches; all1456 live setting entries preserved. Ground23 contains12.044s/19344 decoded frames, continuous UKF validity after initialization, no research fault flags, zero disarmed additional output, maximum101us across recording and bench, CPUmax62%, PIDcyclemax313us. There is one decoder failure and31.674ms gap after the first row before UKF initialization; the remaining adjacent sample gaps are at most656us. This acquisition limitation is preserved in verification.json. Fifteen normal-MSP samples confirm UKF_INIT clears with valid UKF; CLI bulk dumps can transiently leave the initialization guard set, so CLI status alone is not the readiness test. Recording mode restored to NORMAL and aircraft flash verified0/8388608bytes. No motors were commanded and this revision has not yet been flight-tested.
