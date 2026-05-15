# LC-EKF VIO: Cam + IMU 융합으로 Pose 를 추정하는 정확한 개념과 수식

> **2026-05-14** · 본 프로젝트 `D:\02_research\04_cpp_LC-EKF_VIO` 의 LC-EKF 구현이 실제로 어떻게 두 센서를 결합해 pose 를 만드는지를 코드–수식–직관–EKF 의미–C++ 노트가 한 자리에 묶인 형태로 정리.
>
> 대상 코드: `include/ekf/imu_propagator.hpp`, `src/ekf/imu_propagator.cpp`, `include/ekf/lc_ekf.hpp`, `src/ekf/lc_ekf.cpp`, `apps/run_vio.cpp` (메인 루프).

---

## 0. 한 페이지 개요 — Loose Coupling 의 골격

### 📌 요약

이 프로젝트의 **LC-EKF (Loosely-Coupled Extended Kalman Filter) VIO** 는

1. **IMU 만으로** 고주파(100~200 Hz)에서 pose 를 *예측 (predict)* 하고,
2. **카메라(스테레오 VO) 가 만들어낸 pose 결과물** 을 저주파(10 Hz) **외부 측정 (measurement)** 으로 받아 *보정 (update)* 한다.

"Loosely-coupled" 는 **카메라 raw pixel 이나 feature 가 EKF state 안으로 들어오지 않는다** 는 뜻이다. VO 가 자기 안에서 PnP 까지 다 풀어 `p_world_cam0` 라는 3D 위치 한 개만 EKF 에게 전달한다. EKF 의 입장에서 카메라는 *3축 위치 센서*다.

→ 즉 융합 방식의 본질은 두 줄이다:

$$
\boxed{\;\;\text{Predict: }\; \dot{x}=f(x,u_\text{imu})\;\;|\;\;\text{Update: }\;z_\text{vo}=h(x)+n\;\;}
$$

### 💡 직관

- IMU 는 **빠르지만 누적 오차 (drift)** 가 생긴다. 가속도 두 번 적분 → 위치는 시간 제곱으로 발산.
- VO 는 **느리지만 절대 위치를 본다** (스케일은 스테레오 baseline 으로 고정). 다만 텍스쳐 부족·동적물체로 가끔 튄다.
- 두 단점이 정확히 상호 보완: 빠른 IMU 가 frame 사이를 메우고, 느린 VO 가 IMU 의 누적을 끌어내린다.
- **공분산 P** 가 둘 사이의 가중치를 결정한다. 시간이 지날수록 IMU 측 P 가 커지고, VO 가 들어오면 P 가 줄어든다.

### 🎯 EKF 의미

- "Tightly-coupled" 라면 feature 픽셀 잔차가 직접 EKF measurement model 에 들어간다 (예: MSCKF, OpenVINS).
- "Loosely-coupled" 인 우리 시스템은 **VO 가 풀어준 `p_world_cam0` 만 받는다**. 그래서 measurement model 이 단순한 3차원 위치 모델이 된다 → 안정적이지만 정보 손실이 있음.

### 🔧 코드 — 메인 루프 골격 (`apps/run_vio.cpp`)

```cpp
// apps/run_vio.cpp:892-916  (요약)
for (const auto& cam : cam_data) {
    // (1) 카메라 timestamp 이전까지 IMU 샘플을 모두 propagate
    while (imu_idx < imu_data.size() &&
           imu_data[imu_idx].timestamp <= cam.timestamp) {
        const auto& imu = imu_data[imu_idx++];
        ekf.propagate(imu.timestamp, imu.gyro, imu.accel);   // ← prediction
    }
    // (2) 스테레오 이미지 → VO pose
    const auto pose = tracker.process(img_l, img_r);
    const Eigen::Vector3d p_world_cam = R_WC0 * pose.t + t_WC0;
    if (pose.valid && frame_count > 0) {
        ekf.update_vo(p_world_cam);                          // ← update
    }
}
```

**한 줄 요약:** *IMU 가 올 때마다 `propagate`, 카메라가 올 때마다 `update_vo`. 그 외에는 아무 일도 안 일어난다.*

---

## 1. State 정의 — Nominal × Error 의 분리

### 📌 요약

이 시스템은 **error-state EKF** 다. 즉 EKF 가 직접 다루는 변수는 "현재 추정값(`p`, `v`, `R`, `b_g`, `b_a`)" 이 아니라 **그것의 작은 오차 δx** 이다. 큰 값은 *nominal state* 가 들고, **선형화는 작은 δx 에 대해서만** 한다 → 회전 다양체(SO(3))의 비선형성을 깔끔하게 회피.

### 🧮 수식

**Nominal state** (직접 적분되는, 큰 값):

$$
x = (\,\mathbf{p}_{wI}\in\mathbb{R}^3,\; \mathbf{v}_{wI}\in\mathbb{R}^3,\; R_{wI}\in SO(3),\; \mathbf{b}_g\in\mathbb{R}^3,\; \mathbf{b}_a\in\mathbb{R}^3\,)
$$

**Error state** (EKF 가 다루는, 15차원 벡터):

$$
\delta x = (\delta\mathbf{p},\;\delta\mathbf{v},\;\delta\boldsymbol{\theta},\;\delta\mathbf{b}_g,\;\delta\mathbf{b}_a)\in\mathbb{R}^{15}
$$

두 state 의 관계:

$$
\mathbf{p} \leftarrow \mathbf{p} + \delta\mathbf{p},\quad
\mathbf{v} \leftarrow \mathbf{v} + \delta\mathbf{v},\quad
R \leftarrow R\cdot \mathrm{Exp}(\delta\boldsymbol{\theta}),\quad
\mathbf{b} \leftarrow \mathbf{b}+\delta\mathbf{b}
$$

여기서 $\mathrm{Exp}(\boldsymbol{\theta})$ 는 SO(3) 의 지수 사상 (Rodrigues 공식).

### 🔧 코드 (`src/ekf/lc_ekf.cpp:71-78`)

```cpp
void LcEkf::apply_correction(const Vec15& dx) {
    imu_.p  += dx.segment<3>(0);                     // p ← p + δp
    imu_.v  += dx.segment<3>(3);                     // v ← v + δv
    imu_.R   = imu_.R * so3::Exp(dx.segment<3>(6));  // R ← R · Exp(δθ)
    imu_.R   = Eigen::Quaterniond(imu_.R).normalized().toRotationMatrix();
    imu_.bg += dx.segment<3>(9);
    imu_.ba += dx.segment<3>(12);
}
```

이 함수가 끝나면 **δx ≡ 0 으로 재설정** (error-state EKF 의 "reset" 단계). 즉 다음 step 에서 δx 는 다시 작은 값에서 출발한다.

### 💡 직관

회전을 그냥 행렬로 EKF state 에 넣으면 9개 성분 사이의 직교 조건이 깨져 발산한다. 큰 회전은 $R$(외부, 다양체 위) 으로 두고, **작은 회전 보정만 3차원 벡터 δθ 로** 다룬다 → 차원 축소 + 다양체 정합.

### 🎯 EKF 의미

이 분리가 **15차원 P (공분산)** 가 가능한 이유다. 만약 quaternion 을 직접 EKF state 에 넣었다면 16차원이고 단위노름 제약을 매번 깨뜨려야 한다. error-state 는 **회전을 다양체 위에 두면서 EKF 의 선형성 가정도 유지**하는 표준 트릭.

### 📚 C++ 노트

- `Eigen::Matrix<double, 15, 15>` (`Mat15` 타입 alias) 가 P. `Eigen::Matrix<double, 15, 1>` (`Vec15`) 가 δx.
- `dx.segment<3>(idx)` 는 **컴파일 타임에 크기가 결정되는** 3차원 sub-vector → 동적할당 0, SIMD 가능.
- `imu_.R = Eigen::Quaterniond(imu_.R).normalized().toRotationMatrix();` 한 줄로 SVD 없이 SO(3) re-orthogonalization. 누적 부동소수점 오차로 R 이 살짝 직교성을 잃을 때 표준적 해법.

---

## 2. Prediction — IMU 가 어떻게 state 를 밀고 나가는가

### 📌 요약

매 IMU 샘플마다:

1. **Nominal state 적분** (RK4): 가속도→속도→위치, 각속도→회전.
2. **Error covariance propagation** (1차 Euler): $P \leftarrow \Phi P \Phi^\top + Q_d$.

핵심 식은 두 줄.

### 🧮 수식 — Nominal state ODE

bias 보정된 측정값을 $\omega_c = \omega_\text{raw} - \mathbf{b}_g$, $\mathbf{a}_c = \mathbf{a}_\text{raw} - \mathbf{b}_a$ 라 두면:

$$
\dot{\mathbf{p}} = \mathbf{v},\qquad
\dot{\mathbf{v}} = R_{wI}\,\mathbf{a}_c + \mathbf{g},\qquad
\dot{R}_{wI} = R_{wI}\,[\omega_c]_\times
$$

- $\mathbf{g}=(0,0,-9.81)$ : world frame 중력.
- $[\omega]_\times$ : skew-symmetric matrix (`so3::hat`). 이게 곧 $\dot{R}=R\Omega$ 형식의 SO(3) 운동방정식.

이를 RK4 로 적분.

### 🔧 코드 (`src/ekf/imu_propagator.cpp:30-66`)

```cpp
ImuPropagator::State
ImuPropagator::state_derivative(const State& s,
                                 const Vector3d& gyro_c,
                                 const Vector3d& accel_c) const {
    State ds;
    ds.p = s.v;
    ds.v = s.R * accel_c + g_;        //  v̇ = R·a + g
    ds.R = s.R * so3::hat(gyro_c);    //  Ṙ = R·[ω]×
    return ds;
}
```

```cpp
void ImuPropagator::rk4_step(double dt,
                              const Vector3d& gyro_c,
                              const Vector3d& accel_c) {
    State s0{p, v, R};
    auto k1 = state_derivative(s0, gyro_c, accel_c);
    // ... k2, k3, k4 (각 단계에서 R 은 so3::Exp(0.5*dt*ω) 로 갱신)
    p = p + (dt/6.0)*(k1.p + 2*k2.p + 2*k3.p + k4.p);
    v = v + (dt/6.0)*(k1.v + 2*k2.v + 2*k3.v + k4.v);
    R = R * so3::Exp(dt * gyro_c);    // 회전은 다양체 위에서 한 번에
    R = Eigen::Quaterniond(R).normalized().toRotationMatrix();
}
```

### 🧮 수식 — Error-state ODE (선형화)

$\delta x = (\delta\mathbf{p},\delta\mathbf{v},\delta\boldsymbol{\theta},\delta\mathbf{b}_g,\delta\mathbf{b}_a)$ 의 연속 시간 ODE 는

$$
\dot{\delta x} = F\,\delta x + G\,\mathbf{n},\qquad \mathbf{n}\sim\mathcal{N}(0,Q_c)
$$

여기서 $F$ 는 15×15 block matrix:

$$
F = \begin{bmatrix}
0 & I_3 & 0 & 0 & 0 \\
0 & 0 & -R\,[\mathbf{a}_c]_\times & 0 & -R \\
0 & 0 & -[\omega_c]_\times & -R & 0 \\
0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0
\end{bmatrix}
$$

**행별로 읽으면:**
- 1행: $\dot{\delta\mathbf{p}}=\delta\mathbf{v}$ — 위치 오차의 도함수는 속도 오차.
- 2행: $\dot{\delta\mathbf{v}}=-R[\mathbf{a}_c]_\times\,\delta\boldsymbol{\theta} - R\,\delta\mathbf{b}_a$ — attitude error 와 accel bias error 가 속도 오차를 만든다.
- 3행: $\dot{\delta\boldsymbol{\theta}}=-[\omega_c]_\times\,\delta\boldsymbol{\theta} - R\,\delta\mathbf{b}_g$ — gyro bias 가 attitude error 를 만든다.
- 4,5행: bias 는 random walk → 결정론적 미분은 0.

이산화 (1차 Euler): $\Phi = I + F\,dt$.

### 🔧 코드 (`src/ekf/imu_propagator.cpp:89-124`)

```cpp
Mat15 F = Mat15::Zero();
F.block<3,3>(0,3)  =  Matrix3d::Identity();          // δṗ = δv
F.block<3,3>(3,6)  = -R * so3::hat(accel_c);         // δv̇ ← -R[a×]δθ
F.block<3,3>(3,12) = -R;                             // δv̇ ← -R·δba
F.block<3,3>(6,6)  = -so3::hat(gyro_c);              // δθ̇ ← -[ω×]δθ
F.block<3,3>(6,9)  = -R;                             // δθ̇ ← -R·δbg
Mat15 Phi = Mat15::Identity() + F * dt;

Eigen::Matrix<double,15,12> G = Eigen::Matrix<double,15,12>::Zero();
G.block<3,3>(3,3)  = R;                              // accel noise → δv
G.block<3,3>(6,0)  = R;                              // gyro  noise → δθ
G.block<3,3>(9,6)  = Matrix3d::Identity();           // gyro  walk  → δbg
G.block<3,3>(12,9) = Matrix3d::Identity();           // accel walk  → δba

// Qc: 4채널 (gyro_noise, accel_noise, gyro_walk, accel_walk) 의 diag
// Qd = G·Qc·Gᵀ·dt
Mat15 Qd = (G * Qc * G.transpose()) * dt;
P = Phi * P * Phi.transpose() + Qd;
```

### 💡 직관

- $\Phi P \Phi^\top$ 가 "**기존 불확실성이 dynamics 를 따라 어떻게 흘러가는가**" 를 표현.
- $Q_d$ 가 "**이번 dt 동안 IMU 가 새로 만들어낸 노이즈**". 두 항의 합이 새로운 P. 시간이 갈수록 단조 증가 → drift.

### 🎯 EKF 의미

이 단계가 끝난 직후 P 는 거의 항상 커진 상태다. P 의 **position block** trace 가 직선적으로(또는 더 빠르게) 자라는 것을 보면 IMU drift 가 진행 중임을 알 수 있다. VO update 가 도착해야 다시 줄어든다.

### 📚 C++ 노트

- `Eigen::Matrix<double,15,12>` 같은 **rectangular fixed-size** 행렬도 stack 에 잡힌다 → 매 propagate 마다 동적할당 없음.
- `F.block<3,3>(row,col)` 는 **lvalue reference** 라 좌변/우변 모두 가능. 비주얼적으로 수식 블록과 1:1 매핑.
- `noise_.gyro_noise` 같은 단위는 *continuous-time spectral density* (`rad/s/√Hz`). 이산 분산은 $\sigma^2\cdot dt$ → 코드의 `* dt` 가 바로 그 변환.

---

## 3. Update — Camera (VO) 가 어떻게 state 를 보정하는가

### 📌 요약

스테레오 트래커가 한 프레임의 카메라 절대 위치 $\mathbf{p}_{w\mathrm{C}0}$ 를 만들면, EKF 는 이를 **3차원 위치 측정** 으로 받는다. 측정모델 → 잔차 → 공분산 → Kalman gain → 보정 → reset 순.

### 🧮 수식 — Measurement model

`p_world_cam0` 는 *카메라 원점이 world 에서 어디 있는가* 다. IMU 위치 $\mathbf{p}_{wI}$ 와 IMU 자세 $R_{wI}$ 로 표현하면:

$$
\mathbf{z} \;=\; h(x) + \mathbf{n}
\;=\; R_{wI}\,\mathbf{p}_{IC} + \mathbf{p}_{wI} + \mathbf{n}
$$

- $\mathbf{p}_{IC}$ : **IMU body frame 에서 본 카메라 원점** (lever arm). 캘리브레이션 상수.
- $\mathbf{n}\sim\mathcal{N}(0,\sigma_{vo}^2 I_3)$ : VO 잔차의 isotropic 모델.

### 🧮 수식 — Jacobian (선형화)

작은 회전 보정 $R\leftarrow R\,\mathrm{Exp}(\delta\boldsymbol{\theta})\approx R(I+[\delta\boldsymbol{\theta}]_\times)$ 를 넣고 1차항만 남기면:

$$
h(x\boxplus\delta x) \;\approx\; h(x) + \delta\mathbf{p} \;+\; R_{wI}\,[\delta\boldsymbol{\theta}]_\times\,\mathbf{p}_{IC}
$$

$[\delta\boldsymbol{\theta}]_\times\,\mathbf{p}_{IC} = \delta\boldsymbol{\theta}\times \mathbf{p}_{IC} = -[\mathbf{p}_{IC}]_\times\,\delta\boldsymbol{\theta}$ 항등식으로 정리하면:

$$
\boxed{\;
H = \frac{\partial h}{\partial \delta x}
= \begin{bmatrix} I_3 \;\; 0 \;\; -R_{wI}[\mathbf{p}_{IC}]_\times \;\; 0 \;\; 0 \end{bmatrix}_{3\times 15}
\;}
$$

- $\partial h/\partial \delta\mathbf{p}=I_3$ — 위치 오차는 측정에 그대로.
- $\partial h/\partial \delta\boldsymbol{\theta}=-R_{wI}\,[\mathbf{p}_{IC}]_\times$ — **자세 오차가 lever arm 만큼 위치 측정에 새어 들어옴**. lever arm 이 0 이면 이 블록도 0 (자세는 위치 측정으로 직접 보정 안 됨).
- velocity, biases 블록은 0 — 즉 VO 위치 한 번으로는 속도·바이어스를 직접 못 본다. **간접적으로만 P 의 cross-covariance 를 통해 보정**.

### 🧮 수식 — 표준 EKF Update

$$
\mathbf{y} = \mathbf{z} - h(x) \;\; (\text{innovation})
$$
$$
S = H P H^\top + R_\text{noise},\quad R_\text{noise}=\sigma_{vo}^2 I_3
$$
$$
K = P H^\top S^{-1} \;\; (\text{Kalman gain},\; 15\times 3)
$$
$$
\delta x = K\,\mathbf{y}
$$
$$
P \leftarrow (I - KH)\,P\,(I-KH)^\top + K R_\text{noise} K^\top \;\;(\text{Joseph form})
$$

마지막 식이 **Joseph form** — 수치적으로 P 의 대칭성과 PSD 를 보존. 표준 $P\leftarrow(I-KH)P$ 는 더 짧지만 부동소수점에서 비대칭이 누적된다.

### 🔧 코드 (`src/ekf/lc_ekf.cpp:26-69`)

```cpp
bool LcEkf::update_vo(const Vector3d& p_world_cam0) {
    // 측정 예측 h(x) = R_wI · p_IC + p_wI
    const Matrix3d& R_wI = imu_.R;
    const Vector3d& p_wI = imu_.p;
    Vector3d h = R_wI * p_IC_ + p_wI;

    Vector3d innov = p_world_cam0 - h;                      // y = z - h

    // Measurement Jacobian H (3×15)
    Eigen::Matrix<double, 3, 15> H = Eigen::Matrix<double, 3, 15>::Zero();
    H.block<3,3>(0,0) = Matrix3d::Identity();               // ∂h/∂δp
    H.block<3,3>(0,6) = -R_wI * so3::hat(p_IC_);            // ∂h/∂δθ

    Eigen::Matrix3d R_noise = (sigma_vo_*sigma_vo_) * Matrix3d::Identity();
    Eigen::Matrix3d S = H * imu_.P * H.transpose() + R_noise;
    Eigen::Matrix<double,15,3> K = imu_.P * H.transpose() * S.inverse();

    Vec15 dx = K * innov;                                    // δx 계산
    apply_correction(dx);                                    // nominal 에 흡수

    // Joseph form covariance update
    Mat15 I_KH = Mat15::Identity() - K * H;
    imu_.P = I_KH * imu_.P * I_KH.transpose()
           + K * R_noise * K.transpose();
    return true;
}
```

여기서 `p_IC_` 는 생성자에서 단 한 번 계산:

```cpp
// src/ekf/lc_ekf.cpp:10-18
LcEkf::LcEkf(ImuPropagator& imu,
              const Eigen::Matrix4d& T_cam0_imu,
              double sigma_vo)
    : imu_(imu), sigma_vo_(sigma_vo)
{
    Eigen::Matrix4d T_imu_cam0 = T_cam0_imu.inverse();
    p_IC_ = T_imu_cam0.block<3,1>(0,3);  // cam0 origin in IMU body frame
}
```

### 💡 직관

- 잔차 `innov` 가 크면 → K 가 잔차를 흡수 → δx 가 크게 잡힘 → nominal 이 많이 움직임.
- **P 가 작으면 (IMU 가 자신 있으면)** S 가 R_noise 주도 → K 가 작음 → VO 무시 경향.
- **P 가 크면 (IMU drift 누적)** S 가 H P Hᵀ 주도 → K → I 에 가까움 → VO 를 거의 그대로 받아들임.
- 이게 곧 EKF 가 두 센서를 **공분산 기반 최적 가중평균** 하는 메커니즘.

### 🎯 EKF 의미

- `sigma_vo` 의 실제 효과: 매우 크게 하면 (`1e9`) K≈0 → IMU-only 와 동일. 매우 작게 하면 (`0.01`) K 가 H 의 의사역 행렬 가까이 가서 VO 를 거의 100% 신뢰. **현실에서는 0.3–0.5 m 가 KITTI scale 에서 안정** (이미 `260430_lc_ekf_vio_insights.md` 에서 확인).
- 한 번의 update 로 보정되는 건 직접적으로 `p, θ` 와 (cross-covariance 를 통해) `v, b_g, b_a` 까지 — 즉 **EKF 의 진짜 위력은 H 블록이 0 인 성분까지 P 의 off-diagonal 을 통해 보정** 한다는 점. observability 분석의 출발점.

### 📚 C++ 노트

- `Eigen::Matrix<double, 15, 3>` 처럼 행/열 수가 다른 정적 행렬도 자유롭게 만들 수 있음. Eigen 의 표현식 템플릿이 곱셈 형식 자동 추론.
- `S.inverse()` — 3×3 이라 안전. 만약 큰 행렬이면 `S.ldlt().solve(...)` 로 바꿔야 한다. 여기선 그대로 OK.
- 코드의 `imu_.P` 접근 패턴: `LcEkf` 는 `ImuPropagator&` 만 들고 있고 P 는 `ImuPropagator` 의 `public` 멤버. **소유권 없이 reference 로 들고, 같은 P 를 양쪽이 modify** 하는 구조. friend 안 쓰고도 협력하게 만든 단순한 설계.

---

## 3½. 초기 Anchor — 시스템이 "어디서, 어느 자세로" 시작되는가

### 📌 요약

EKF 가 첫 propagate 를 시작하기 *전에* 결정되어야 하는 것:
$(\mathbf{p}_0, \mathbf{v}_0, R_0, \mathbf{b}_{g,0}, \mathbf{b}_{a,0})$ 그리고 **VO 좌표계를 EKF world 에 묶는 카메라 anchor** $(R_{WC_0}, \mathbf{t}_{WC_0})$.

이 시스템은 *모든 anchor 를 IMU 정지 가정* 하나로 결정한다 — 첫 1초 동안 카메라는 안 본다, IMU 만 본다.

### 🧮 수식 — 정지 가정의 의미

시작 순간 IMU 가 정지 ($\mathbf{a}_\text{true}=0$, $\boldsymbol{\omega}_\text{true}=0$) 라 가정하면:

$$
\boldsymbol{\omega}_\text{raw} \;=\; \underbrace{\boldsymbol{\omega}_\text{true}}_{=0} + \mathbf{b}_g + \mathbf{n}_g \;\Rightarrow\; \boxed{\;\hat{\mathbf{b}}_g = \langle \boldsymbol{\omega}_\text{raw}\rangle\;}
$$

$$
\mathbf{a}_\text{raw} \;=\; R_{IW}\,(\underbrace{\mathbf{a}_\text{true}}_{=0} - \mathbf{g}_W) + \mathbf{b}_a + \mathbf{n}_a \;\Rightarrow\; \langle\mathbf{a}_\text{raw}\rangle \approx -R_{IW}\,\mathbf{g}_W
$$

(가속도 bias 와 중력은 정지만으로 분리 불가능 → $\hat{\mathbf{b}}_{a,0}=0$ 로 두고 EKF 의 갱신에 맡긴다.)

$\mathbf{g}_W = (0,0,-g)$ 라 두었으므로 $-\mathbf{g}_W = +g\hat{\mathbf{z}}_W$. 정지 시 측정된 specific force 방향은 곧 "**world 의 +Z 가 body 의 어느 방향에 있는가**" 를 가리킨다.

→ $R_{WI}$ 를 구하는 조건: $R_{WI}\,\hat{\mathbf{a}}_\text{body} = \hat{\mathbf{z}}_W$. 두 단위벡터 사이의 최단 회전 → quaternion `FromTwoVectors`.

**관측 가능성**: roll, pitch 결정. **yaw 는 unobservable** (중력 한 축으로 yaw 분리 불가).

### 🔧 코드 — 단계별 분해

**Step A. 정지 윈도우 IMU 평균** (`apps/run_vio.cpp:292-335`)

```cpp
InitialState initialize_from_imu(const std::vector<ImuData>& imu_data,
                                 double t0,
                                 const InitParams& init_params) {
    InitialState init;
    init.t0      = t0;
    init.g_world = Eigen::Vector3d(0.0, 0.0, -init_params.gravity_norm);

    Eigen::Vector3d gyro_sum  = Eigen::Vector3d::Zero();
    Eigen::Vector3d accel_sum = Eigen::Vector3d::Zero();
    int sample_count = 0;
    for (const auto& imu : imu_data) {
        if (imu.timestamp < t0) continue;
        if (imu.timestamp > t0 + init_params.static_window_sec) break;
        gyro_sum  += imu.gyro;
        accel_sum += imu.accel;
        ++sample_count;
    }
    init.bg0 = gyro_sum / static_cast<double>(sample_count);     // ← b_g 초기값
    const Eigen::Vector3d accel_mean = accel_sum / static_cast<double>(sample_count);
    init.R0 = estimate_initial_orientation(accel_mean);          // ← R_WI 초기값
    return init;
}
```

`InitialState` 의 다른 필드는 struct default initializer 로 모두 0:
- `init.p0 = 0` — world 원점 자체를 *IMU 의 초기 위치* 로 정의 (위치는 항상 상대적).
- `init.v0 = 0` — 정지 가정에서 직접 따라옴.
- `init.ba0 = 0` — 정지로는 중력과 분리 불가능, 추후 EKF 가 알아서 추정.

**Step B. 중력 정렬 회전** (`apps/run_vio.cpp:280-290`)

```cpp
Eigen::Matrix3d estimate_initial_orientation(const Eigen::Vector3d& accel_mean) {
    if (accel_mean.norm() < 1e-6) {
        return Eigen::Matrix3d::Identity();
    }
    Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(
        accel_mean.normalized(),
        Eigen::Vector3d::UnitZ());
    q.normalize();
    return q.toRotationMatrix();
}
```

`FromTwoVectors(a, b)` → "$q$ such that $q \cdot a = b$" 의 *최단* 회전 quaternion. 여기서:
- $a$ = body frame 에서 본 중력 반대방향 (정지 시 가속도계 측정).
- $b = \hat{\mathbf{z}}_W$.
- $q$ = $R_{WI}$ (world ← body).

**Step C. 카메라 anchor 합성** (`apps/run_vio.cpp:862-864`)

```cpp
// VO 는 첫 cam0 frame 을 자기 좌표계 원점으로 삼는다.
// 그 원점을 EKF world 의 어디로 잡을지를 결정하는 게 anchor.
const Eigen::Matrix4d T_IC = cam0.T_cam_imu.inverse();           // cam0 → IMU
const Eigen::Matrix3d R_WC0 = init.R0 * T_IC.block<3,3>(0,0);
const Eigen::Vector3d t_WC0 = init.R0 * T_IC.block<3,1>(0,3) + init.p0;
```

수식:

$$
R_{WC_0} \;=\; R_{WI}\cdot R_{IC},\qquad
\mathbf{t}_{WC_0} \;=\; R_{WI}\cdot \mathbf{p}_{IC} + \mathbf{p}_{WI}
$$

→ 이게 **EKF measurement model $h(x)=R_{wI}\mathbf{p}_{IC}+\mathbf{p}_{wI}$ 와 형식이 동일**한 것이 핵심. 즉 *t=0 순간의 카메라 위치를 EKF 의 관측 공식으로 계산해서 그걸 VO 좌표계의 원점에 부여* — 그래서 첫 프레임의 잔차가 자연스럽게 0 근처에서 시작한다.

**Step D. VO 좌표 → EKF world 변환** (매 프레임, `apps/run_vio.cpp:912-916`)

```cpp
const auto pose = tracker.process(img_l, img_r);
const Eigen::Vector3d p_world_cam = R_WC0 * pose.t + t_WC0;   // VO local → EKF world
if (pose.valid && frame_count > 0) {
    ekf.update_vo(p_world_cam);
}
```

### 💡 직관

- **왜 t=0 에 update 가 안 들어가나?** 조건 `frame_count > 0`. 첫 프레임은 `pose.t = 0`, anchor 변환을 거쳐도 $\mathbf{t}_{WC_0}$ → 곧 측정값이 측정 예측과 *정의상* 일치 → 정보 없음. 두 번째 프레임부터 의미가 생긴다.
- **anchor 가 잘못되면?** init.R0 가 부정확하면 (예: 시작 시 차량이 사실은 흔들리고 있었다면) yaw 외에도 roll/pitch 가 살짝 틀어진 채로 출발. VO 가 들어와도 EKF 가 보는 모든 잔차가 *systematic* 하게 한쪽으로 치우쳐 → drift 처럼 보이는데 사실은 초기화 오차.
- **`static_window_sec` 의 trade-off**: 길수록 평균 노이즈 ↓ 하지만 *진짜로 정지인 구간* 이 그만큼 보장돼야 함. KITTI 처럼 차량이 처음부터 굴러가는 데이터셋이면 이 가정이 깨져 bg0 가 오염될 수 있다.

### 🎯 EKF 의미

- 초기화는 *EKF 의 prior* 결정. 평균 (nominal state) 과 P (covariance) 의 초기값이 모두 여기서 정해진다.
- P 초기값은 `ImuPropagator` 생성자에서: position $10^{-4}$, velocity $10^{-2}$, attitude $10^{-4}$, bg $10^{-6}$, ba $10^{-4}$ (단위는 각 변수의 분산).
- **attitude P 가 작게 잡힌 게 흥미로움** — `1e-4` rad² ≈ (0.01 rad)² ≈ (0.57°)². 즉 "중력 정렬이 0.5° 이내" 라는 신뢰. 가속도 노이즈와 1초 평균을 생각하면 합리적.
- yaw 는 unobservable 인데도 같은 분산을 줬다 → **yaw drift 는 covariance 가 작아 보여도 실제로는 계속 커지는** 종류의 오차. observability 분석을 안 하면 함정.

### 📚 C++ 노트

- `Eigen::Quaterniond::FromTwoVectors(a, b)` — 두 벡터 사이 최단 회전 quaternion. SLERP 의 출발점이고, 중력 정렬 같은 1-벡터 attitude estimation 에서 표준.
- `struct InitialState { ... = Eigen::Vector3d::Zero(); ... };` 같은 **default member initializer** — 깜빡 잊고 setter 를 안 호출해도 안전한 기본값. 본 프로젝트의 거의 모든 struct 가 이 패턴.
- `cam0.T_cam_imu.inverse()` — `Eigen::Matrix4d::inverse()` 는 일반 inverse. rigid transform 인 게 명확하면 $T^{-1}=\bigl[\,R^\top\;\;-R^\top\mathbf{t}\,\bigr]$ 분해가 더 안전하지만, 캘리브레이션 행렬이 종종 살짝 비-rigid 라 그대로 inverse 가 보수적 선택.
- `if (sample_count == 0) { ... fallback ... }` — `t0` 이전 IMU 데이터만 있는 경우의 백업 경로. 데이터셋 마다 timestamp 정의가 달라 자주 깨지는 부분이라 의도적으로 가드.

---

## 4. Lever Arm — 왜 카메라 측정이 IMU pose 를 정확히 보정하나

### 📌 요약

카메라와 IMU 는 **물리적으로 다른 위치**에 있다. 측정모델 $h = R_{wI}\mathbf{p}_{IC}+\mathbf{p}_{wI}$ 이 정확히 그 **공간 분리 (lever arm $\mathbf{p}_{IC}$)** 를 표현하고 있고, Jacobian 의 $-R_{wI}[\mathbf{p}_{IC}]_\times$ 항이 **자세 오차가 위치 측정에 새어 들어오는 통로**를 만든다. 이 통로 덕분에 위치 측정 한 종류로 attitude 까지 부분적으로 관측 가능해진다.

### 🧮 수식

YAML 에서 받은 `T_cam_imu` 는 *IMU → cam* 변환행렬:

$$
T_{CI} = \begin{bmatrix} R_{CI} & \mathbf{t}_{CI} \\ \mathbf{0}^\top & 1 \end{bmatrix}
$$

이 역행렬의 translation 부분이 **IMU body frame 에서 본 카메라 원점**:

$$
\mathbf{p}_{IC} \;=\; -R_{CI}^\top\,\mathbf{t}_{CI}
$$

코드는 그냥 행렬 역변환으로 한 줄에 처리. (위 `LcEkf` 생성자 코드 참조.)

### 🔧 코드 (`apps/run_vio.cpp:862-868`)

```cpp
// VO 는 첫 cam0 frame 기준 좌표. 이걸 EKF world 와 anchor.
const Eigen::Matrix4d T_IC = cam0.T_cam_imu.inverse(); // cam0 -> IMU
const Eigen::Matrix3d R_WC0 = init.R0 * T_IC.block<3,3>(0,0);
const Eigen::Vector3d t_WC0 = init.R0 * T_IC.block<3,1>(0,3) + init.p0;
```

그리고 매 프레임:

```cpp
// apps/run_vio.cpp:912-916
const auto pose = tracker.process(img_l, img_r);
const Eigen::Vector3d p_world_cam = R_WC0 * pose.t + t_WC0;  // VO 로컬 → EKF 월드
if (pose.valid && frame_count > 0) {
    ekf.update_vo(p_world_cam);
}
```

### 💡 직관

- **VO 가 출력하는 `pose.t`** 는 *VO 자신의 좌표계* (첫 카메라 프레임 = 원점) 에서의 카메라 위치다. EKF world 좌표계와 다르다.
- 그래서 첫 프레임에서 `R_WC0`, `t_WC0` 를 anchor 로 잡아 두고, 매 프레임 `R_WC0 * pose.t + t_WC0` 로 EKF world 로 옮긴다.
- 이 anchor 가 어긋나면 (예: init.R0 가 부정확) **EKF 가 처음부터 IMU 와 VO 의 systematic mismatch 를 본다** → drift 처럼 보임. 초기화가 정밀해야 하는 이유.

### 🎯 EKF 의미

- $\mathbf{p}_{IC}=0$ 이라면 H 의 attitude 블록도 0 → 위치 측정으로 attitude 가 직접 관측되지 않음.
- $\mathbf{p}_{IC}\neq 0$ 이면 **회전 운동이 lever arm 끝(카메라)에 contribution 을 만들고**, 그 contribution 이 위치 측정에 보인다. 결과적으로 attitude 도 (lever arm 방향의 수직 성분만) 부분적으로 관측 가능.
- 이게 LC-EKF VIO 가 **yaw/roll/pitch 일부를 카메라만으로 묶을 수 있는 이유**다. 정밀한 lever arm 캘리브레이션이 곧 자세 관측 가능성.

### 📚 C++ 노트

- `Matrix4d.block<3,3>(0,0)` , `.block<3,1>(0,3)` — 동차변환 행렬을 분해할 때 가장 자주 쓰는 패턴.
- `Eigen::Matrix4d::inverse()` — 일반 4×4 inverse 지만, rigid transform 이면 사실 $T^{-1}=\begin{bmatrix}R^\top & -R^\top t\\0&1\end{bmatrix}$ 로 더 안전·빠르게 만들 수 있다. 현재 코드는 일반 inverse — 캘리브레이션 행렬에 작은 비-rigid 성분 있어도 fail 안 하는 장점이 있음.

---

## 5. Reset — Error-state 가 0 으로 돌아가는 과정

### 📌 요약

`apply_correction(dx)` 이 끝나는 순간 δx 는 nominal 에 흡수되어 **개념적으로 0** 으로 리셋된다. 코드에는 δx 를 보관하는 변수 자체가 없고, P 만 남는다. *Error-state EKF 의 핵심 디자인 결정.*

### 🧮 수식

리셋 직후:

$$
x^+ = x \boxplus \delta x,\qquad \delta x \leftarrow 0,\qquad P^+ = (I-KH)P(I-KH)^\top + KRK^\top
$$

엄밀히 따지면 attitude error 의 reset 은 P 의 attitude 블록에도 작은 회전 jacobian (\text{Jr}^{-1}(\delta\boldsymbol{\theta})) 보정이 필요하지만, **|δθ|** 가 작은 정상동작 영역에서는 무시해도 무방. 우리 구현은 이 보정을 생략 (대부분의 실제 VIO 구현 동일).

### 🔧 코드

리셋이라는 명시적 함수가 없다. `apply_correction` 의 마지막 줄 직후 함수가 끝나면, 다음 `propagate` 가 호출될 때 δx 변수 자체가 다시 0 으로 출발 (애초에 변수도 없다 — propagation 은 nominal 만 적분).

### 💡 직관

> "EKF state 가 δx 이지만, 코드에서 δx 라는 변수가 안 보이는 이유는?"
> → 매 update 직후 즉시 nominal 로 흡수되기 때문. 보관할 필요가 없다.

### 🎯 EKF 의미

- error-state EKF 가 standard EKF 와 다른 가장 큰 두 가지: (1) Jacobian 이 작은 회전 주변에서 평가됨 (선형화 오차 ↓). (2) **state 자체가 항상 0 근처에 머문다** → 수치 동역학이 안정.

### 📚 C++ 노트

- `Eigen::Quaterniond(R).normalized().toRotationMatrix()` — `apply_correction` 마지막에 호출. δθ 가 누적되며 R 의 직교성이 약간 무너질 수 있는데, quaternion 으로 변환→정규화→다시 행렬 로 한 줄에 복구.
- `imu_.bg += dx.segment<3>(9);` — bias 는 단순 vector addition. 다양체 고려 없음.

---

## 6. 전체 흐름 시퀀스 — 한 카메라 프레임 동안 무슨 일이?

### 📌 요약

카메라 한 프레임이 도착하기 직전까지 IMU 가 여러 번 propagate 되어 P 가 커진다. 카메라가 도착하면 VO 가 `p_world_cam` 을 만들고 EKF update 가 한 번 일어나 P 가 다시 줄어든다. 이 사이클이 반복.

### 🧮 수식 — 사이클 한 회

$t_{k-1}$ 에서 $t_k$ (카메라 프레임 시각) 사이에 IMU 가 $M$ 번 들어오면:

$$
\text{for } i=1..M:\;\; x \leftarrow \text{RK4}(x, u_i, dt_i),\; P \leftarrow \Phi_i P \Phi_i^\top + Q_{d,i}
$$

그 다음 카메라:

$$
\mathbf{z}_k = R_{WC_0}\,\mathbf{p}_{C_0 \to \text{cam}}^{\text{VO}} + \mathbf{t}_{WC_0}
$$
$$
x \leftarrow x \boxplus K_k\,(\mathbf{z}_k - h(x)),\quad P \leftarrow \text{Joseph}(P, K_k, H, R_\text{noise})
$$

### 🔧 코드 (`apps/run_vio.cpp:892-916`, 핵심만)

```cpp
for (const auto& cam : cam_data) {
    while (imu_idx < imu_data.size() &&
           imu_data[imu_idx].timestamp <= cam.timestamp) {
        const auto& imu = imu_data[imu_idx++];
        ekf.propagate(imu.timestamp, imu.gyro, imu.accel);    // M번 prediction
    }

    const auto pose = tracker.process(img_l, img_r);          // VO
    const Eigen::Vector3d p_world_cam = R_WC0 * pose.t + t_WC0;
    if (pose.valid && frame_count > 0) {
        ekf.update_vo(p_world_cam);                           // 1번 update
    }
}
```

### 💡 직관

- IMU 가 100 Hz, 카메라가 10 Hz 라면 한 카메라 frame 당 IMU 가 10 번 들어옴 → propagation 이 10번, update 가 1번.
- 그래서 EKF 는 **"빠르게 예측, 가끔 보정"** 패턴.

### 🎯 EKF 의미

- VO 가 떨어져 (`pose.valid==false`) update 가 한동안 안 들어오면 P 가 계속 커진다. 다음 valid VO 가 오면 그 한 번이 매우 강하게 보정 (P 가 크니 K 도 크다). 즉 시스템이 자동으로 dropouts 를 복구.
- 반대로 VO 가 systematic 하게 한쪽으로 치우치면 EKF 가 그 bias 를 그대로 따라간다 — outlier 가드 (`innov.norm() > 5.0` 경고) 만으로는 충분치 않음. **다음 단계로 χ² gating 도입이 자연스러운 확장**.

### 📚 C++ 노트

- `while (imu_idx < imu_data.size() && imu_data[imu_idx].timestamp <= cam.timestamp)` — IMU 와 카메라가 **time-synchronized 되어 있다는 가정**. EuRoC, KITTI 처럼 dataset 가 정렬돼 있을 때만 안전.
- 만약 실시간 시스템이라면 buffer + sliding window 가 필요. 지금 구조는 **batch / offline 분석에 최적화** 되어 있음.

---

## 7. 종합 — Cam + IMU 가 만드는 한 줄

### 📌 한 줄 요약

> **IMU 가 짧은 시간동안 어림하고, 카메라가 가끔 와서 어림을 고친다. 둘 다 자기 불확실성을 P 라는 한 그릇에 담아 두기 때문에, EKF 가 둘을 최적으로 가중평균할 수 있다.**

### 🧮 수식

$$
\hat{x}_k = \underbrace{f(\hat{x}_{k-1}, u_\text{imu})}_{\text{prediction (IMU)}} \;\;\boxplus\;\; \underbrace{K_k\,[\mathbf{z}_\text{vo} - h(\cdot)]}_{\text{correction (cam)}}
$$

여기서 두 항의 가중치 $K_k = P H^\top (HPH^\top + R)^{-1}$ 가 시간에 따라 동적으로 변하는 것이 EKF VIO 의 본질.

### 🎯 다음 학습 방향

| 주제 | 어디로 |
|---|---|
| **Observability** — lever arm 이 0 일 때 못 보는 자세 성분은? | Hesch et al. 2014 (FEJ) |
| **Tight coupling** 으로의 확장 | MSCKF (Mourikis 2007), OpenVINS 코드 |
| **Outlier rejection (χ² gate)** | 현재 `innov_norm > 5.0` 단순 경고 → Mahalanobis 거리 기반으로 교체 |
| **Initialization** — `estimate_initial_orientation` 가 가속도만으로 자세 정렬하는 방법 | `apps/run_vio.cpp:280-290`, IMU static initialization 문헌 |
| **Bias observability** — 정지 시에는 왜 bias 가 잘 안 잡히는가 | error-state F 의 4,5행 분석 |

### 📚 C++ 학습 누적 (이 문서로 다시 돌아왔을 때 빠르게 복습할 토픽)

- `Eigen::Matrix<double, N, M>` 정적 크기 행렬 — heap allocation 0, SIMD friendly.
- `.block<r,c>(i,j)` — sub-matrix lvalue. 수식 블록을 코드와 1:1 매핑하는 핵심 도구.
- `.segment<n>(i)` — fixed-size sub-vector. Vec15 에서 (δp, δv, δθ, δbg, δba) 추출.
- `Eigen::Quaterniond(R).normalized().toRotationMatrix()` — SO(3) re-orthogonalization 의 표준 한 줄.
- `friend` 없이 `ImuPropagator&` reference 와 public 멤버 P 로 협력하게 만든 설계 — "단순함이 정답일 때가 많다".
- `Eigen::umeyama` — Sim(3) alignment (run_vio.cpp 의 ATE 계산용). 별 관련 없어도 같은 헤더에 있어서 자주 마주침.

---

## 부록: 변수·기호 사전

| 기호 | 의미 | 차원 | 코드 |
|---|---|---|---|
| $\mathbf{p}_{wI}$ | IMU 위치 (world frame) | $\mathbb{R}^3$ | `imu_.p` |
| $\mathbf{v}_{wI}$ | IMU 속도 (world frame) | $\mathbb{R}^3$ | `imu_.v` |
| $R_{wI}$ | IMU 자세 (world ← body) | $SO(3)$ | `imu_.R` |
| $\mathbf{b}_g,\mathbf{b}_a$ | gyro, accel bias | $\mathbb{R}^3$ | `imu_.bg`, `imu_.ba` |
| $\delta x$ | 15차원 error state | $\mathbb{R}^{15}$ | `Vec15 dx` |
| $P$ | error covariance | $15\times 15$ | `imu_.P` |
| $\mathbf{p}_{IC}$ | lever arm (IMU body 에서 본 cam0) | $\mathbb{R}^3$ | `p_IC_` |
| $H$ | measurement Jacobian | $3\times 15$ | `H` |
| $K$ | Kalman gain | $15\times 3$ | `K` |
| $\sigma_\text{vo}$ | VO noise std | scalar | `sigma_vo_` |
| $\mathbf{p}_{w\mathrm{C}0}$ | 카메라 원점 (world frame), 측정값 | $\mathbb{R}^3$ | `p_world_cam0` |

관련 문서: [[260430_lc_ekf_vio_insights]] (sigma_vo 의미 / IMU-only 모드), [[20260501_stereo_rectification_left_feature_state]] (VO 가 어떻게 `pose.t` 를 만드는가).
