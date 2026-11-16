# 외란 환경에서 quadrotor attitude estimation 및 low-authority adaptive control의 구현과 검증

## 초록

소형 quadrotor는 외부 충격, 급격한 attitude 변화, 프로펠러 추력 변화에 의해 attitude estimation과 제어 성능이 함께 저하될 수 있다. 본 연구에서는 실제 비행 중인 Mobula8 quadrotor를 시험 기체로 사용하여, Betaflight 4.4.2의 기존 flight-control architecture를 보존한 상태에서 Unscented Kalman Filter(UKF) 기반 attitude estimation과 low-authority INDI·ADRC·MRAC correction control을 구현하였다. UKF는 gyro와 accelerometer를 결합해 roll·pitch attitude를 추정하며, 큰 가속도·충격 구간에서는 accelerometer correction을 제한하도록 설계하였다. 제어 구조에서는 기존 rate PID와 motor mixer를 주 제어기로 유지하고, 추가 correction output은 축별 ±0.25 Betaflight PID unit으로 제한하였다.

총 14개의 host test, firmware 전체 readback 비교, 설정 대조, bench Blackbox 기록과 실제 flight log 분석을 수행하였다. 최신 firmware는 전체 1 MB flash readback 결과가 build image와 일치했고, 1,456개 설정 항목도 보존되었다. 정지 bench test에서 UKF는 initialization 뒤 지속적으로 valid했으며, 연구 코드의 최대 execution time은 101 μs였다. 이전 disturbance flight log에서는 최대 12.469 g의 충격 기록 중에도 CPU·observer·receiver 오류 없이 동작한 구간을 확인하였다. 다만 최신 UKF selection 수정본은 bench validation까지만 완료되었으므로, 본 연구는 이를 최종 flight performance의 증명으로 해석하지 않는다.

**주요어:** quadrotor, attitude estimation, Unscented Kalman Filter, INDI, ADRC, MRAC, Betaflight

---

# 1. 이론적 배경

## 1.1 Quadrotor attitude와 control architecture

Quadrotor는 네 개의 rotor thrust를 서로 다르게 조절하여 기체의 attitude와 이동을 제어하는 비행체이다. 기체 중심 좌표계의 x, y, z축 회전을 각각 roll (`phi`), pitch (`theta`), yaw (`psi`)라고 한다. roll과 pitch는 thrust vector의 방향을 바꾸어 수평 이동을 만들고, yaw는 기체의 heading을 바꾼다. 안정적인 비행을 위해서는 세 축의 angle과 angular rate를 짧은 주기 안에 정확히 estimate하고 control해야 한다.

일반적인 ANGLE mode의 quadrotor control은 outer attitude loop와 inner rate loop로 구성된다. 조종기 입력에서 attitude reference `eta sub ref`를 만들고, estimated attitude `hat eta`와의 error를 rate reference `omega sub ref`로 변환한다. 이후 rate PID controller가 실제 angular rate `omega`를 reference에 추종시키는 output을 계산하며, mixer가 이 output을 네 motor command로 배분한다.

`e sub eta = eta sub ref - hat eta`

`omega sub ref = C sub level (e sub eta)`

`e sub omega = omega sub ref - omega`

`u sub PID = K sub P e sub omega + K sub I int e sub omega dt + K sub D {d e sub omega} over {dt}`

본 연구에서는 이미 flight-validated된 Betaflight의 inner rate PID, motor mixer, receiver·failsafe 설정을 교체하지 않았다. 대신 outer attitude loop에 제공되는 roll·pitch estimate를 UKF로 개선하고, 추가 control은 기존 PID output에 작고 제한된 correction으로 더하도록 구성하였다.

## 1.2 IMU와 attitude representation

시험 기체의 BMI270 IMU는 gyro와 accelerometer로 구성된다. gyro는 angular rate `omega = [p, q, r] sup T`를 빠르게 측정하지만, 작은 bias도 적분되면 attitude error로 누적된다. accelerometer는 정지 또는 일정한 속도 운동에서 gravity direction을 제공하므로 roll·pitch의 장기 error를 보정할 수 있다. 반면 기체가 가속하거나 thrust가 급격히 변할 때에는 중력 이외의 acceleration과 vibration이 함께 측정된다. 따라서 두 sensor의 성질을 함께 고려하는 sensor fusion이 필요하다.

Euler angle은 직관적이지만 pitch가 `plusminus 90 DEG`에 가까워질 때 gimbal lock 문제가 발생한다. 본 연구는 내부 attitude state를 unit quaternion `q = [q sub 0, q sub 1, q sub 2, q sub 3] sup T`로 표현하였다.

`abs q = 1`

`dot q = {1} over {2} q times [0, p, q, r] sup T`

Quaternion은 큰 회전을 연속적으로 표현할 수 있으므로 빠른 attitude 변화에 적합하다. 최종 ANGLE control에는 roll·pitch angle로 변환한 값을 사용하지만, UKF의 prediction과 correction은 quaternion space에서 수행하였다.

## 1.3 Unscented Kalman Filter 기반 자세 추정

Kalman filter는 system model의 prediction과 sensor measurement를 각 uncertainty에 따라 결합하는 state estimator이다. UKF는 nonlinear function을 미분·선형화하는 대신, state distribution을 대표하는 sigma point를 nonlinear function에 통과시켜 mean과 covariance를 근사한다. 큰 회전과 quaternion normalization이 포함되는 attitude estimation에 적합한 방법이다.

본 연구의 state vector는 다음과 같이 attitude quaternion, angular rate, accelerometer bias로 구성하였다.

`x = [q sub 0, q sub 1, q sub 2, q sub 3, omega sub x, omega sub y, omega sub z, b sub ax, b sub ay, b sub az] sup T`

10차원 state에 대해 21개의 sigma point를 생성하고, gyro 기반 rotation model로 propagate하였다.

`chi sub i sup {k+1 | k} = f(chi sub i sup k, DELTA t)`

`hat x sub {k+1 | k} = sum from i=0 to 2n W sub i sup {(m)} chi sub i sup {k+1 | k}`

measurement vector는 gyro와 accelerometer로 구성된다.

`z = [omega sub x, omega sub y, omega sub z, a sub x, a sub y, a sub z] sup T`

measurement update에서는 measurement residual `z - hat z`와 Kalman gain `K`를 사용해 state를 correct한다.

`K = P sub xz S sup {-1}`

`hat x sup + = hat x sup - + K(z - hat z)`

accelerometer는 기동 중에 중력만을 나타내지 않는다. 따라서 acceleration magnitude가 0.8–1.2 g 범위를 벗어나거나 angular-rate magnitude가 8 rad/s를 넘을 때, 그리고 그 상태 뒤 150 ms 동안에는 accelerometer attitude correction을 보류하였다. measurement residual의 Mahalanobis distance도 사용해 비정상 acceleration measurement를 한 번 더 판별하였다. 이때도 gyro 기반 prediction은 계속 수행해 빠른 회전의 연속성을 유지한다.

PID loop는 약 3.2 kHz로 동작하므로, 10차원 UKF의 모든 matrix operation을 매 cycle에 수행하면 computational load가 커질 수 있다. 이에 무거운 UKF calculation은 약 100 Hz로 나누어 수행하고, 그 사이 모든 PID cycle의 gyro increment는 quaternion으로 누적하였다. UKF correction이 완료되면 correction 시점의 attitude를 현재 시점까지 누적 rotation만큼 propagate하였다.

## 1.4 INDI·ADRC·MRAC 기반 low-authority correction control

quadrotor의 angular-rate dynamics는 `dot omega = B u + d`로 근사할 수 있다. 여기서 `B`는 control effectiveness, `d`는 모델에 포함되지 않은 disturbance이다. INDI는 입력 증분, ADRC는 extended state observer (ESO), MRAC는 reference model 추종을 이용해 correction candidate를 만든다.

`DELTA u approx B sup {-1} (dot omega sub des - dot omega)`

본 연구에서는 이를 main controller로 쓰지 않고 기존 PID에 더하는 low-authority correction으로 제한했다. 축별 출력은 ±0.25 Betaflight PID unit을 넘지 않으며, saturation·warm-up·invalid condition에서는 adaptation을 멈춘다.

## 1.5 안전 및 검증 원칙

UKF는 motor control을 대체하지 않고 ANGLE mode의 roll·pitch input만 제공한다. 기존 Betaflight rate PID와 motor mixer는 유지하며, UKF input은 1초에 걸쳐 전환한다. estimate가 invalid·stale하거나 sensor·timing·CPU·failsafe 조건이 비정상이면 UKF를 해제한다. 반면 valid UKF는 큰 tilt나 Betaflight estimate와의 차이만으로 해제하지 않는다. 비행 종료 뒤에는 약 1초의 안정 상태를 확인해 재initialize하고, 준비 전에는 `UKF_INIT`로 ARM을 막는다.

---

# 2. 연구 방법 및 절차

## 2.1 시험 기체와 개발 환경

시험 기체는 BMI270과 8 MB SPI flash를 갖는 STM32F405 기반 Mobula8이다. Betaflight 4.4.2의 DSHOT300·bidirectional DShot·motor·receiver·PID·filter·mode 설정은 유지했다. 연구 코드는 UKF, UKF selection, low-authority correction 모듈로 분리해 Betaflight PID loop에 연결했다.

## 2.2 시뮬레이터 구현과 역할

실기체 전에는 F405 firmware를 C 기반 6-DOF physics engine에 연결한 SITL과 FireDrone X8 real-time simulator를 사용했다. SITL은 IMU·GPS·RC input과 firmware PWM output을 closed loop로 연결했고, X8 simulator는 coaxial motor, robot-arm torque, wind, failure, RC loss를 포함했다. 시뮬레이터는 logic·fault path 검증용이며, 실제 비행 성능의 증거로 사용하지 않았다.

## 2.3 기존 설정 보존과 펌웨어 제작

기체의 `dump all`과 resource·timer·DMA 정보를 보관한 뒤, EEPROM을 덮지 않는 HEX를 빌드했다. 체크섬·주소 범위·CLI 설정과 1 MB flash readback을 대조했으며, 최신 이미지와 기존 1,456개 설정 항목은 모두 일치했다.

## 2.4 UKF 구현 및 실행 시간 제어

UKF는 1초의 안정된 disarmed state 뒤 initialize한다. gyro increment는 모든 PID cycle에 누적하고 sigma-point calculation은 분할 실행한다. UKF input은 1초에 걸쳐 전환하며, invalid·stale estimate, abnormal timing, CPU budget, sensor fault만 release condition으로 사용했다.

## 2.5 INDI·ADRC·MRAC의 firmware 통합

추가 제어는 Betaflight rate PID를 교체하지 않고, PID가 계산한 baseline output `u sub base`에 작은 correction `u sub corr`을 더하는 구조로 통합했다. 최종 PID sum은 다음과 같다.

`u sub final = u sub base + u sub corr`

각 axis에서 gyro rate를 derivative low-pass filter로 미분해 measured angular acceleration `alpha`를 구했다. motor command에는 first-order lag를 적용하고, 같은 phase의 filter를 acceleration 경로에 적용해 INDI가 서로 다른 delay를 비교하지 않도록 했다. INDI candidate는 다음처럼 계산한다.

`u sub INDI = u sub lag + {alpha sub des + d hat - alpha} over {B}`

여기서 `B`는 control effectiveness이며, `u sub lag`는 lag를 반영한 이전 command이다. `alpha sub des`는 ADRC·MRAC에서 만든 desired angular acceleration, `d hat`는 ESO가 estimate한 lumped disturbance이다. INDI 식 안에서 이미 measured acceleration을 이용하므로, ESO disturbance를 별도 output에서 한 번 더 빼지 않도록 구현했다.

ADRC의 extended state observer (ESO)는 rate state `z sub 1`과 disturbance state `z sub 2`를 update한다. `z sub 1`은 measured rate에 맞추고, `z sub 2`는 aerodynamic change·battery voltage·unmodeled torque를 하나의 disturbance로 추정한다. observer bandwidth는 20, rate gain은 15로 두었으나, 이 값들은 flight log로 identification된 physical constant가 아니라 안정적인 bounded experiment를 위한 design setting이다.

MRAC는 reference model에 대한 rate error로 adaptive parameter `theta sub 1`, `theta sub 2`를 update한다. 적용 시 gradient를 rate와 setpoint magnitude로 normalize하고, `theta sub 1`은 0.8–1.2, `theta sub 2`는 0–0.2 범위로 projection했다. warm-up·motor saturation·candidate clipping에서는 adaptation을 freeze해 transient 또는 infeasible output이 parameter drift로 누적되지 않게 했다.

Mode 1은 shadow mode로 candidate·disturbance·correction만 log하고 output을 바꾸지 않는다. Mode 2에서만 `u sub corr`을 적용한다. armed, throttle, receiver, scheduler cadence, UKF selection, mixer saturation 조건이 모두 valid해야 하며, 0.5 s warm-up 뒤 0.25 s 동안 부드럽게 engage한다. final correction은 ±0.25 Betaflight PID unit, blend는 5%로 제한했다. AUX4/SE가 high가 아니거나 UKF safety fallback이 발생하면 즉시 baseline PID output으로 돌아간다.

## 2.6 System identification과 parameter 해석

Blackbox에는 rate reference, gyro, baseline PID sum, motor eRPM, eRPM freshness, INDI candidate, ESO disturbance, applied correction을 기록했다. 축을 하나씩 자극한 flight log에서 RC input을 instrumental variable로 사용해 control effectiveness와 effective delay를 estimate했다. training과 validation block은 시간적으로 분리하고, block 경계 0.5 s는 제외했다.

후보 `B`는 roll 2.543, pitch 2.195, yaw 0.716 rad/s² per Betaflight PID unit이고 effective delay는 roll·pitch 40 ms, yaw 5 ms였다. 하지만 roll·pitch validation raw-acceleration `R sup 2`는 0.157, -0.131로 낮았다. 따라서 이 값은 bounded correction의 initialization에만 사용했으며, rotor inertia·motor time constant·airframe parameter 같은 physical constant로 해석하지 않았다.

## 2.7 검증 절차

검증은 14개 host test, propeller·battery를 분리한 bench Blackbox test, flight-log analysis, 최신 selector의 과거-log replay 순으로 수행했다. host test는 output cap, saturation 시 state reset, warm-up, parameter retuning, unequal axis lag, invalid input을 포함했다. 기록은 시간순 영어 폴더명으로 관리했다.

---

# 3. 연구 결과

## 3.1 펌웨어와 설정 보존 결과

최신 HEX의 SHA-256은 `4cda832eca54b2c78a8652373dbe6b435f95851133933934d702421281bbd29a`이다. 업로드 뒤 1 MB readback은 build image와 일치했고 EEPROM과 1,456개 CLI setting도 보존됐다.

## 3.2 지상 UKF 검증 결과

최신 선택 수정본으로 12.044초의 비무장 정지 Blackbox 기록을 획득했다. 결과는 표 1과 같다.

| 항목 | 결과 |
|---|---:|
| 기록 프레임 수 | 19,344개 |
| UKF 최초 유효 시점 | 기록 시작 후 2.561초 |
| 초기화 뒤 UKF 연속 유효 | 예 |
| 연구 코드 최대 실행 시간 | 99 μs |
| 별도 MSP 벤치 포함 최대 실행 시간 | 101 μs |
| PID 주기 최대값 | 313 μs |
| CPU 최대 사용률 | 62% |
| UKF–Betaflight 최대 차이(롤/피치) | 0.3° / 0.3° |
| 추가 보정 출력 | 0 (비무장 상태) |
| research safety fault | 0회 |

MSP sample 15회에서 `UKF_INIT` 해제와 UKF validity를 확인했다. 시작 직후 decoder error 1회와 31.674 ms gap이 있었으나 UKF initialization 전 구간이었고, 이후 최대 gap은 656 μs였다.

## 3.3 기존 UKF 자세제어 비행 및 외란 기록

초기 UKF ANGLE flight에서는 12.53초 동안 UKF input이 유지됐고 최대 연구 코드 시간은 93 μs였다. 표 2의 disturbance flight는 최신 selector 수정 전 기록이다.

| 기록 | 비행 시간 | 최대 가속도 | UKF 활성 비율 | 최대 연구 코드 시간 | 주요 관찰 |
|---|---:|---:|---:|---:|---|
| Flight 1 | 15.720 s | 5.879 g | 100.00% | 98 μs | 추가 보정 출력이 76.31% 구간에서 0이 아니었음 |
| Flight 2 | 34.648 s | 12.469 g | 95.56% | 98 μs | 마지막 전복·지면 접촉 뒤 hard latch 발생 |
| Flight 3 | 27.771 s | 10.132 g | 0.00% | 98 μs | 이전 비행의 hard latch가 재부팅 없이 이어짐 |

Flight 2의 8.871 g·9.975 g impact candidate에서는 UKF selection이 유지됐으나, 전복·ground contact 뒤 hard latch가 발생했다. Flight 3은 이 latch가 지속돼 UKF와 correction output이 0이었다. 통제된 A/B test가 아니므로 impact performance의 인과 증명은 아니다.

추가 제어가 적용된 구간에서는 Blackbox에 unblended INDI candidate와 실제 applied correction을 함께 남겼다. Flight 1에서 applied correction이 0이 아니었던 구간은 전체의 76.31%였고, correction limit·blend·validity flag를 함께 점검했다. 이 결과는 bounded output path가 동작했음을 뜻하지만, 동일한 disturbance를 stock PID와 반복 비교한 결과가 아니므로 INDI·ADRC·MRAC이 안정성을 향상시켰다는 결론에는 사용하지 않았다.

## 3.4 최신 UKF 선택 로직의 로그 재생 결과

Flight 22 replay에서 25,273·55,800·44,713개 valid row는 모두 UKF selected 상태였다. 이는 tilt·estimate difference만으로 release되지 않음을 보이지만, closed-loop flight나 absolute accuracy 검증은 아니다.

## 3.5 실제 비행 실험의 단계별 진행

실험은 baseline flight, UKF observation, UKF ANGLE input, system identification, low-authority correction, impact analysis 순으로 진행했다.

| 단계 | 실험 목적 | 핵심 결과 | 다음 단계로 이어진 판단 |
|---|---|---|---|
| 01–02 | 기존 PID 기준선과 요 축 입력 확보 | 정상 구간 각속도 추종 오차와 모터 여유를 기록 | 실험 제어 전 비교 기준 확보 |
| 05 | 분할 UKF 펌웨어 첫 비행 | 무장 중 UKF 관측 동작 확인 | UKF를 제어 입력으로 쓰기 전 관측 단계 분리 |
| 07 | UKF 비행 관측 2회 | 관측기 유효·시간 예산 범위 확인 | ANGLE 자세 입력 실험 진행 |
| 09 | UKF ANGLE 첫 비행 | 12.53초 동안 UKF 자세 입력 유지 | 제어기 모델 식별 로그 수집 |
| 12·14·17 | 축별 입력·RPM 기록·모델 식별 | 효과도와 지연의 후보값 산출, 롤·피치 모델 검증 한계 확인 | 보정 권한을 작게 제한 |
| 19 | 저권한 보정 출력 비행 | 출력 한도와 보호 상태를 로그로 확인 | 충격 상황의 추정 지연 개선 필요성 확인 |
| 22 | 외란·충격 비행 | 최대 12.469 g 기록과 전복 뒤 latch 지속 문제 확인 | 자이로 증분 누적·선택 조건·재초기화 수정 |
| 23 | 최신 선택 수정본 지상 시험 | 설정 보존, UKF 초기화, 최대 101 μs 확인 | 최신 비행 검증은 후속 과제 |

각 단계의 log 분석 결과를 다음 firmware 수정과 host·bench test에 반영했다.

---

# 4. 고찰

## 4.1 UKF 적용의 의미와 한계

UKF는 모든 PID cycle의 gyro increment와 약 100 Hz correction을 결합해 ANGLE input을 제공한다. 최대 101 μs는 STM32F405에서의 실행 가능성을 보이지만, 공통 IMU를 쓰는 Betaflight와의 일치만으로 absolute accuracy를 증명할 수는 없다.

## 4.2 큰 자세 차이에 대한 선택 조건 수정

초기 selector는 large tilt 또는 estimate difference에서 UKF를 release했고, impact flight에서 hard latch가 다음 flight까지 지속됐다. 최신 selector는 invalid input·timing·CPU fault만 release condition으로 남기고, 정상 disarm 뒤 재initialize한다. 최신 revision은 bench test와 replay까지만 검증됐다.

## 4.3 INDI·ADRC·MRAC 적용의 해석

세 제어기는 순차적으로 역할을 분리했다. MRAC가 rate setpoint에 대한 reference를 조정하고, ADRC ESO가 rate와 lumped disturbance를 estimate하며, INDI가 measured angular acceleration과 effectiveness로 incremental command를 계산한다. 이 구조에서 Betaflight PID와 mixer는 그대로 main control path를 유지한다. 따라서 추가 제어의 실패가 곧바로 motor allocation 전체를 바꾸지 않도록 했다.

다만 roll·pitch validation `R sup 2`가 낮아 identified effectiveness를 정밀 physical model로 해석할 수 없다. 이에 output authority를 ±0.25 PID unit과 5% blend로 제한하고, warm-up·saturation·clipping·invalid receiver/UKF state에서는 correction 및 adaptation을 freeze 또는 reset했다. Flight log는 output cap과 fallback path의 동작 근거이지만, 동일한 maneuver·battery·disturbance 조건의 stock comparison이 없으므로 control-performance improvement의 증명은 아니다.

## 4.4 연구의 한계와 후속 검증

최신 firmware의 flight validation, controlled A/B impact test, independent attitude reference가 남아 있다. 이후 반복 flight와 external camera 또는 motion capture로 UKF accuracy와 correction effect를 분리해 검증해야 한다.

---

# 5. 결론

Mobula8에 UKF attitude input과 low-authority correction을 구현하면서 기존 Betaflight setting을 보존했다. 1 MB readback과 1,456개 setting 대조, 101 μs bench execution, 최대 12.469 g disturbance log, selector replay를 확보했다. 다만 최신 firmware의 flight validation과 controlled impact comparison은 남아 있으므로, 현재 결론은 구현·bench·log 수준의 검증에 한정한다.

---

# 참고문헌

[1] Julier, S. J., & Uhlmann, J. K. (2004). Unscented filtering and nonlinear estimation. *Proceedings of the IEEE, 92*(3), 401–422.

[2] Wan, E. A., & van der Merwe, R. (2000). The unscented Kalman filter for nonlinear estimation. In *Proceedings of Symposium 2000 on Adaptive Systems for Signal Processing, Communication and Control* (pp. 153–158).

[3] Smeur, E. J. J., Chu, Q. P., & de Croon, G. C. H. E. (2016). Adaptive incremental nonlinear dynamic inversion for attitude control of micro air vehicles. *Journal of Guidance, Control, and Dynamics, 39*(3), 450–461.

[4] Han, J. (2009). From PID to active disturbance rejection control. *IEEE Transactions on Industrial Electronics, 56*(3), 900–906.

[5] Ioannou, P. A., & Sun, J. (1996). *Robust adaptive control*. Prentice Hall.

[6] Betaflight. (2026). *Betaflight 4.4 documentation and source code*. https://github.com/betaflight/betaflight

# 부록: 근거 자료 위치

- 최신 지상 검증: `firmware/mobula8/logs/23.UKF-Fixed-Selection-Ground-Test/verification.json`
- 외란 비행 분석: `firmware/mobula8/logs/22.Impact-Robustness-Disturbance-Flight/summary.json`
- 외란 비행 해석: `firmware/mobula8/logs/22.Impact-Robustness-Disturbance-Flight/report.txt`
- 모델 식별 결과: `firmware/mobula8/logs/17.RPM-Controller-Identification-Flight/identified-candidates.json`
- 배포 펌웨어 정보: `firmware/mobula8/dist/manifest.json`

# 부록: 실제 비행 로그 그래프

다음 그래프는 각 비행의 Blackbox 기록에서 생성한 원본 분석 그림이다. 비행마다 목적·모드·펌웨어 단계가 다르므로, 그래프의 차이를 제어기 성능의 직접적인 우열로 해석하지 않는다.

## 기준 PID 비행

**그림 A-1.** 기존 PID 기준 비행의 목표 각속도, 자이로, 모터 출력 기록.

![기준 PID 비행 그래프](../firmware/mobula8/logs/01.Stock-PID-Baseline-Flight/overview.png)

**그림 A-2.** 요 입력을 보강한 기존 PID 비행 기록.

![요 입력 기준 비행 그래프](../firmware/mobula8/logs/02.Stock-PID-Yaw-Input-Flight/overview.png)

## UKF 관측 및 ANGLE 자세 입력 비행

**그림 A-3.** 분할 UKF 펌웨어 비행 기록.

![분할 UKF 비행 그래프](../firmware/mobula8/logs/05.Angle-Mode-Sliced-Firmware-Flight/overview.png)

**그림 A-4, A-5.** UKF 비행 관측 두 기록. 이 단계에서는 기존 Betaflight 자세·PID가 제어를 담당했다.

![UKF 관측 비행 1](../firmware/mobula8/logs/07.UKF-Flight-Observer-Two-Flights/flight.01.ukf.png)

![UKF 관측 비행 2](../firmware/mobula8/logs/07.UKF-Flight-Observer-Two-Flights/flight.02.ukf.png)

**그림 A-6.** UKF 자세를 ANGLE 바깥 루프에 입력한 첫 비행 기록.

![UKF ANGLE 첫 비행](../firmware/mobula8/logs/09.UKF-Angle-Control-First-Flight/flight.01.ukf.png)

## 모델 식별과 저권한 보정 제어 비행

**그림 A-7–A-10.** 축별 입력, 롤 트림, RPM 기록을 포함한 모델 식별 비행 그래프.

![제어기 식별 비행](../firmware/mobula8/logs/12.Controller-Identification-Flights/flight.01.ukf.png)

![롤 트림 식별 비행 1](../firmware/mobula8/logs/14.Roll-Trim-Controller-Identification-Flight/flight.01.ukf.png)

![롤 트림 식별 비행 2](../firmware/mobula8/logs/14.Roll-Trim-Controller-Identification-Flight/flight.02.ukf.png)

![RPM 식별 비행](../firmware/mobula8/logs/17.RPM-Controller-Identification-Flight/flight.01.ukf.png)

**그림 A-11.** 4개 모터 eRPM과 신선도 진단 기록.

![RPM 진단](../firmware/mobula8/logs/17.RPM-Controller-Identification-Flight/rpm-diagnostic.png)

**그림 A-12, A-13.** 저권한 보정 출력이 포함된 외란 비행 기록. 추가 출력은 기존 PID에 더해진 값이며, 축별 한도가 적용되었다.

![저권한 보정 외란 비행 1](../firmware/mobula8/logs/19.Bounded-Controllers-Disturbance-Flight/flight.01.bounded.png)

![저권한 보정 외란 비행 2](../firmware/mobula8/logs/19.Bounded-Controllers-Disturbance-Flight/flight.02.bounded.png)

## 충격 대응 최적화 비행

**그림 A-14–A-19.** 충격 대응 최적화 전 비행 기록. 각 기록은 UKF–Betaflight 자세 비교와 저권한 보정 출력을 함께 나타낸다. Flight 2의 마지막 사건은 사용자가 확인한 전복 또는 지면 접촉 뒤 ARM 해제와 일치하며, 일반적인 공중 비행 실패로 단정하지 않았다.

![충격 비행 1 자세](../firmware/mobula8/logs/22.Impact-Robustness-Disturbance-Flight/flight.01.attitude.png)

![충격 비행 1 보정 출력](../firmware/mobula8/logs/22.Impact-Robustness-Disturbance-Flight/flight.01.bounded.png)

![충격 비행 2 자세](../firmware/mobula8/logs/22.Impact-Robustness-Disturbance-Flight/flight.02.attitude.png)

![충격 비행 2 보정 출력](../firmware/mobula8/logs/22.Impact-Robustness-Disturbance-Flight/flight.02.bounded.png)

![충격 비행 3 자세](../firmware/mobula8/logs/22.Impact-Robustness-Disturbance-Flight/flight.03.attitude.png)

![충격 비행 3 보정 출력](../firmware/mobula8/logs/22.Impact-Robustness-Disturbance-Flight/flight.03.bounded.png)
