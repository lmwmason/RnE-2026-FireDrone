# Ⅱ. 이론적 배경

## 2.1 쿼드로터의 자세와 비행 제어 구조

쿼드로터는 네 개의 로터가 만드는 추력의 크기를 서로 다르게 조절하여 기체의 자세와 이동을 제어하는 비행체이다. 기체 중심에 고정된 좌표계에서 x, y, z축 주위의 회전을 각각 롤(roll, \(\phi\)), 피치(pitch, \(\theta\)), 요(yaw, \(\psi\))라고 한다. 롤과 피치는 추력 벡터의 방향을 바꾸어 수평 이동을 만들고, 요는 기체의 방위각을 바꾼다. 따라서 안정적인 비행을 위해서는 세 축의 각도와 각속도를 빠르고 정확하게 알아야 한다.

본 연구에 사용한 Mobula8 비행제어기에는 자이로스코프와 가속도계가 포함된 BMI270 관성측정장치(IMU)가 탑재되어 있다. 자이로스코프는 각속도 \(\boldsymbol{\omega}=[p,q,r]^T\)를 직접 측정하므로 빠른 회전에 민감하게 반응하지만, 작은 바이어스도 시간에 따라 적분되어 자세 오차가 누적된다. 가속도계는 정지 또는 일정한 속도 운동에서 중력 방향을 제공하므로 롤과 피치의 장기 오차를 보정할 수 있다. 그러나 기체가 가속하거나 프로펠러 추력이 변할 때에는 중력 외의 비관성 가속도와 진동도 함께 측정한다. 즉, 어느 한 센서만으로는 비행 중 자세를 안정적으로 추정하기 어렵다.

일반적인 쿼드로터 제어는 바깥쪽 자세 루프와 안쪽 각속도 루프로 나뉜다. ANGLE 모드에서는 조종기 스틱 입력으로 목표 자세 \(\boldsymbol{\eta}_{ref}\)를 만들고, 추정 자세 \(\hat{\boldsymbol{\eta}}\)와의 차이로 목표 각속도 \(\boldsymbol{\omega}_{ref}\)를 계산한다. 이어서 각속도 PID 제어기가 실제 각속도 \(\boldsymbol{\omega}\)를 목표값에 추종시키는 모터 명령을 만든다.

\[
\boldsymbol{e}_{\eta}=\boldsymbol{\eta}_{ref}-\hat{\boldsymbol{\eta}},\qquad
\boldsymbol{\omega}_{ref}=C_{level}(\boldsymbol{e}_{\eta})
\]

\[
\boldsymbol{e}_{\omega}=\boldsymbol{\omega}_{ref}-\boldsymbol{\omega},\qquad
\boldsymbol{u}_{PID}=K_P\boldsymbol{e}_{\omega}+K_I\int\boldsymbol{e}_{\omega}dt+K_D\frac{d\boldsymbol{e}_{\omega}}{dt}
\]

여기서 \(C_{level}\)은 자세 오차를 목표 각속도로 변환하는 레벨 제어부이고, \(\boldsymbol{u}_{PID}\)는 각 축의 제어 출력이다. Betaflight는 이 출력에 믹서를 적용해 네 모터의 명령으로 변환한다. 본 연구는 이 검증된 내부 각속도 PID와 모터 믹서를 교체하지 않았다. 대신 ANGLE 모드의 바깥 자세 루프에 입력되는 \(\hat{\boldsymbol{\eta}}\)를 UKF 추정값으로 개선하고, 별도의 저권한 보정 제어를 기존 PID 출력에 제한적으로 더하는 구조를 사용하였다.

## 2.2 사원수 기반 자세 표현

오일러각 \((\phi,\theta,\psi)\)은 사람이 이해하기 쉬우나, 피치가 \(\pm90^\circ\) 부근에 가까워지면 축이 겹쳐지는 짐벌락 문제가 발생한다. 또한 큰 회전에서는 오일러각의 단순 차이가 실제 회전 오차를 잘 나타내지 못할 수 있다. 본 연구는 내부 자세 상태를 단위 사원수(quaternion)로 표현하였다.

\[
\boldsymbol{q} = [q_0,q_1,q_2,q_3]^T,\qquad
\|\boldsymbol{q}\|=1
\]

사원수의 시간 변화는 각속도 \(\boldsymbol{\omega}\)에 대해 다음과 같이 나타낼 수 있다.

\[
\dot{\boldsymbol{q}}=\frac{1}{2}\boldsymbol{q}\otimes[0,p,q,r]^T
\]

여기서 \(\otimes\)는 사원수 곱이다. 사원수는 모든 방향의 회전을 연속적으로 나타낼 수 있으므로 빠른 롤·피치 운동과 충격이 포함된 비행에 적합하다. 최종적으로 조종기 ANGLE 제어에는 사람이 사용하는 롤·피치각으로 변환해 제공하되, 필터의 예측·보정 과정은 사원수 공간에서 수행하였다.

## 2.3 Unscented Kalman Filter를 이용한 자세 추정

칼만 필터는 시스템의 예측값과 센서 측정값을 각 불확실도에 따라 결합하는 상태 추정기이다. 확장 칼만 필터(EKF)는 비선형 함수를 선형화하여 사용하지만, 큰 회전과 사원수 정규화가 포함된 자세 문제에서는 선형화 오차가 커질 수 있다. Unscented Kalman Filter(UKF)는 상태 평균과 공분산을 여러 개의 시그마점(sigma point)으로 전파한 뒤 다시 결합하는 방식으로, 미분을 직접 계산하지 않고 비선형성의 평균과 분산을 근사한다.

본 연구의 UKF 상태벡터는 다음의 10개 상태로 구성하였다.

\[
\boldsymbol{x}=[q_0,q_1,q_2,q_3,\omega_x,\omega_y,\omega_z,b_{ax},b_{ay},b_{az}]^T
\]

앞의 네 항은 자세 사원수, 다음 세 항은 각속도, 마지막 세 항은 가속도계 바이어스이다. 10차원 상태에 대해 \(2n+1=21\)개의 시그마점을 만들고, 각 시그마점을 자이로스코프 기반 회전 모델로 전파한다. 예측 단계의 개념은 다음과 같다.

\[
\chi_i^{k+1|-}=f(\chi_i^k,\Delta t),\qquad
\hat{\boldsymbol{x}}_{k+1|-}=\sum_{i=0}^{2n}W_i^{(m)}\chi_i^{k+1|-}
\]

\[
P_{k+1|-}=\sum_{i=0}^{2n}W_i^{(c)}(\chi_i^{k+1|-}-\hat{\boldsymbol{x}})(\chi_i^{k+1|-}-\hat{\boldsymbol{x}})^T+Q
\]

측정벡터는 자이로스코프와 가속도계로 구성하였다.

\[
\boldsymbol{z}=[\omega_x,\omega_y,\omega_z,a_x,a_y,a_z]^T
\]

자이로스코프 측정은 각속도 상태를 보정하고, 가속도계 측정은 중력 벡터의 기체좌표계 성분을 통해 롤과 피치의 장기 드리프트를 보정한다. 측정 업데이트는 예측 측정치 \(\hat{\boldsymbol{z}}\), 측정 공분산 \(S\), 상태-측정 교차공분산 \(P_{xz}\)로 칼만 이득 \(K\)를 계산하여 수행한다.

\[
K=P_{xz}S^{-1},\qquad
\hat{\boldsymbol{x}}^+=\hat{\boldsymbol{x}}^-+K(\boldsymbol{z}-\hat{\boldsymbol{z}})
\]

### 2.3.1 비행 중 가속도계 신뢰도 판단

가속도계는 정지 상태에서만 중력 방향을 직접 나타낸다. 급기동, 충격, 추력 변화 중에는 \(\|\boldsymbol{a}\|\)가 1 g에서 벗어나며, 그 값을 자세 보정에 그대로 사용하면 오히려 기체가 기울어졌다고 잘못 판단할 수 있다. 따라서 본 연구에서는 가속도 크기가 0.8–1.2 g 범위를 벗어나거나 각속도 크기가 8 rad/s를 넘는 경우, 그리고 그러한 상태 뒤 150 ms 동안 가속도계 자세 보정을 보류하였다. 이때도 자이로스코프 기반 예측과 각속도 상태의 갱신은 계속 수행한다. 또한 측정 잔차의 마할라노비스 거리로 비정상 가속도 측정을 한 번 더 판별하였다.

이는 가속도계를 완전히 버리는 방법이 아니라, 정적인 중력 정보로 해석할 수 있는 순간에만 큰 가중치를 주는 방법이다. 따라서 빠른 조종 및 외란 중에는 자이로 적분의 연속성이 중요하고, 안정 구간에서는 가속도계가 누적 오차를 줄이는 상호보완 구조가 된다.

### 2.3.2 PID 주기와 UKF 갱신 주기의 분리

본 기체의 PID 루프는 약 3.2 kHz로 동작하지만, 10차원 UKF의 행렬 연산을 매 주기에 모두 수행하면 비행제어기에 과도한 계산 부하가 생길 수 있다. 이에 UKF의 무거운 시그마점 계산은 약 100 Hz 단위로 나누어 수행하고, 그 사이 모든 PID 주기의 자이로 증분은 사원수로 누적하였다. UKF 보정이 끝나면 보정 시점의 자세를 누적 회전만큼 현재 시점으로 전파한다.

이 방식은 느린 UKF 보정과 빠른 자이로 적분을 결합한다. 특히 100 Hz 보정 사이에 발생하는 짧은 회전을 버리지 않으므로, 충격이나 빠른 회전에서 발생하는 자세 지연을 줄이는 데 목적이 있다. 지상 검증에서 연구 코드의 최대 실행 시간은 101 \(\mu s\)로 측정되었으며, 이는 3.2 kHz PID 주기 안에서 운용 가능한지 확인하기 위한 시간 검증이다. 이는 곧바로 모든 비행 상황에서의 정확도나 안정성을 보장한다는 의미는 아니다.

## 2.4 INDI, ADRC, MRAC 기반 각속도 보정 제어

쿼드로터의 각속도 동역학은 간단히 다음과 같이 나타낼 수 있다.

\[
\dot{\boldsymbol{\omega}}=B\boldsymbol{u}+\boldsymbol{d}
\]

여기서 \(B\)는 모터 명령이 각가속도로 변환되는 유효성(effectiveness) 행렬이고, \(\boldsymbol{d}\)는 공력 변화, 배터리 전압 저하, 프로펠러 효율 변화, 외부 충격처럼 정확히 모델링하기 어려운 외란을 뜻한다. 실제 소형 드론에서는 \(B\)와 \(\boldsymbol{d}\)가 비행 조건에 따라 달라지므로 하나의 고정 모델만으로 정확한 제어를 보장하기 어렵다.

INDI(Incremental Nonlinear Dynamic Inversion)는 절대적인 모델보다 직전 제어 입력과 실제 각가속도 변화를 이용하여 필요한 제어 입력의 증분을 계산한다. 개념적으로는 측정 각가속도 \(\dot{\boldsymbol{\omega}}\)와 원하는 각가속도 \(\dot{\boldsymbol{\omega}}_{des}\)의 차이를 유효성의 역수로 환산하여 보정한다.

\[
\Delta\boldsymbol{u}\approx B^{-1}(\dot{\boldsymbol{\omega}}_{des}-\dot{\boldsymbol{\omega}})
\]

ADRC(Active Disturbance Rejection Control)는 확장 상태 관측기(ESO)를 이용해 모델에 포함되지 않은 외란을 하나의 상태로 추정하고, 그 추정치를 참고하여 목표 응답을 만든다. MRAC(Model Reference Adaptive Control)는 실제 각속도가 기준 모델의 응답을 따라가도록 제어 파라미터를 조정한다. 본 구현에서는 적응 파라미터가 포화, 워밍업, 출력 제한 구간에서 변하지 않도록 하여, 불충분한 데이터나 실현할 수 없는 모터 출력 때문에 파라미터가 누적되는 것을 막았다.

그러나 이 세 기법은 자동으로 항상 더 좋은 제어를 만들어 주는 방법이 아니다. INDI에는 축별 유효성 및 지연 시간 추정이 필요하고, ADRC에는 관측기 대역폭 선택이 필요하며, MRAC에는 충분한 자극과 검증 가능한 기준 모델이 필요하다. 본 연구의 식별 결과에서 롤·피치 모델은 독립 검증 기준을 통과하지 못했으며, 따라서 이를 ‘정확한 물리 상수’로 해석하지 않았다. 현재 실험 펌웨어에서는 기존 PID를 주 제어기로 유지하고, 추가 출력은 각 축에서 최대 ±0.25 Betaflight PID 단위로 제한하였다. 이는 새로운 제어기의 효과를 관찰하면서도 기존 비행 제어 구조를 보존하기 위한 설계이다.

## 2.5 본 연구의 안전·검증 구조

본 연구의 핵심은 UKF가 PID를 대체한다는 것이 아니라, UKF가 ANGLE 모드의 자세 입력을 제공하고 기존 Betaflight의 내부 각속도 PID가 모터 제어를 계속 담당한다는 점이다. UKF 선택은 한 번에 1초 동안 부드럽게 전환되어 급격한 자세 입력 변화가 모터 출력 변화로 이어지는 것을 줄인다.

실험 펌웨어는 추정값이 유효하지 않거나 20 ms 이상 오래되었을 때, 계산 시간 또는 센서 입력이 비정상일 때, PID 주기 조건이 충족되지 않을 때 UKF 출력을 사용하지 않도록 설계하였다. 비행 종료 후에는 다음 이륙 전에 기체가 약 1초 동안 안정된 상태임을 확인한 뒤 UKF를 다시 초기화한다. 이 준비가 끝나기 전에는 `UKF_INIT` 상태로 ARM을 막는다. 반면, 정상적으로 유효한 UKF가 큰 기울기나 Betaflight 추정값과 차이를 보인다는 이유만으로 자동 해제하지 않도록 개선하였다. 이는 큰 외란 환경에서 서로 다른 추정기의 오일러각 차이만으로 UKF를 배제하는 문제를 줄이기 위한 것이다.

이러한 보호 조건은 충격을 포함한 모든 상황에서의 비행 성공을 보장하는 장치는 아니다. 따라서 실험 결과에서는 (1) 펌웨어 전체 읽기 비교와 설정 보존, (2) 지상 연산 시간과 UKF 유효성, (3) Blackbox 비행 로그의 자세·각속도·출력 분석을 서로 구분하여 평가하였다. 이 구분을 통해 지상 검증, 로그 기반 재생 검증, 실제 비행 검증의 증거 수준을 혼동하지 않도록 하였다.

## 참고문헌

1. S. J. Julier and J. K. Uhlmann, “Unscented Filtering and Nonlinear Estimation,” *Proceedings of the IEEE*, vol. 92, no. 3, pp. 401–422, 2004.
2. E. A. Wan and R. van der Merwe, “The Unscented Kalman Filter for Nonlinear Estimation,” *Proceedings of Symposium 2000 on Adaptive Systems for Signal Processing, Communication and Control*, pp. 153–158, 2000.
3. A. G. Parus, M. S. Selig, and M. B. Tischler, “Incremental Nonlinear Dynamic Inversion Control for a Small Unmanned Aerial Vehicle,” *AIAA Guidance, Navigation, and Control Conference*, 2012.
4. J. Han, “From PID to Active Disturbance Rejection Control,” *IEEE Transactions on Industrial Electronics*, vol. 56, no. 3, pp. 900–906, 2009.
5. P. A. Ioannou and J. Sun, *Robust Adaptive Control*, Prentice Hall, 1996.
6. Betaflight, “Betaflight 4.4 Documentation and Source Code,” https://github.com/betaflight/betaflight (accessed 2026-10-19).
