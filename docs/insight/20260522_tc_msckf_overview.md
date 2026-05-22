---
title: "TC MSCKF VIO: 전체 학습 spine — 개념·수식 정리"
date: 2026-05-22
type: insight
tags: [vio, tc-msckf, msckf, overview, error-state-ekf]
related:
  - "[[20260514_lc_ekf_cam_imu_fusion]]"
  - "[[20260520_vio_lvio_msckf_thinking_cheatsheet]]"
  - "[[260430_lc_ekf_vio_insights]]"
status: draft
---

# TC MSCKF VIO 전반 — 한 자리에 정리한 개념·수식 spine

> **2026-05-22** · 이 문서는 후속 7편의 deep dive (Stage 1a~3) 가 가지를 뻗어 나갈 **줄기 (spine)** 다. 토픽별 OpenVINS 코드 인용은 deep dive 로 미루고, 여기서는 **(왜 / 무엇이 / 어떻게 — 수식 수준에서)** 까지만 정리한다.
>
> 학습 위치: 이미 LC-EKF VIO 를 구현·정리 ([[20260514_lc_ekf_cam_imu_fusion]]) 했고, cheatsheet ([[20260520_vio_lvio_msckf_thinking_cheatsheet]]) Part 2.7 / 4.2 / 5 에서 TC MSCKF 를 *한 문단씩* 만난 상태. 본 문서는 그 한 문단들을 **수학적 완결성** 까지 끌어올린다.

---

## 0. 이 문서의 사용법

본 문서는 **개념·수식만**. 코드 (OpenVINS / 본인 향후 구현) 는 별도 deep dive 에서.

각 섹션 구조: 📌 요약 / 🧮 수식 / 💡 직관 / 🎯 EKF 의미. (LC 캐논 문서의 [코드/C++노트] 두 층은 deep dive 의 몫.)

학습 지도:

| 본 문서 § | 후속 deep dive | 파일명 (예정) |
|---|---|---|
| §2 State 구조 | (Stage 0 orientation 완) | — |
| §3 IMU Predict | (LC 와 동일, 신규 학습 없음) | — |
| §4 Clone Augmentation | Stage 1a | `tc_msckf_clone_augment.md` |
| §5 Per-feature 측정 모델 | Stage 2a | `tc_msckf_feature_jacobian.md` |
| §6 Null-space Projection | Stage 2b | `tc_msckf_nullspace.md` |
| §7 Chi² + Compression + Update | Stage 2c | `tc_msckf_compress_update.md` |
| §8 Marginalization | Stage 1b | `tc_msckf_marginalize.md` |
| §9 전체 사이클 + LC vs TC | Stage 3 | `tc_msckf_orchestration.md` |

---

## 1. 한 페이지 개요 — *왜* TC MSCKF 인가

### 📌 요약

**LC (Loosely-Coupled)** 는 카메라가 *이미 적분한 결과* (예: VO pose) 만 EKF 에 던진다. 이 한 변환에서 *픽셀 잔차의 공분산 구조*가 사라지고, 카메라 trajectory 의 누적 오차가 한 덩어리로 압축돼 들어온다 → 정보 손실 (information loss). 본인 LC-EKF KITTI 결과 ATE ~153cm 가 그 한계.

**TC (Tightly-Coupled) MSCKF (Multi-State Constraint Kalman Filter)** 는 *raw pixel residual 자체를 EKF measurement 로* 직접 쓴다. 다만 한 feature 는 *여러 카메라 pose* 에서 관측되므로, "한 측정 ≠ 한 시점" 이 된다 → **여러 시점의 IMU pose 를 state 에 동시에 들고 있어야** 한다 (sliding window of clones).

문제: feature 의 3D 위치 자체를 state 에 넣으면 차원이 폭발 (EKF-SLAM 의 O(n²)). MSCKF 의 **정수** 는 *feature 의 3D 위치는 state 에 넣지 않고도* multi-view constraint 를 update 로 활용 — **null-space projection** 이 그 트릭이다.

### 💡 직관 (한 줄)

> "TC = 픽셀 잔차를 EKF 가 직접 본다. MSCKF = feature 위치를 state 에 넣지 않고도 픽셀 잔차의 정보를 흡수한다."

### 🎯 EKF 의미

TC MSCKF 는 EKF 6-식 ($\hat{x}^- = f(\hat{x}, u)$, $P^- = FPF^T + Q$, $y = z - h$, $S = HPH^T + R$, $K = PH^TS^{-1}$, $\hat{x}^+, P^+$ Joseph form) 의 *변형이 아니라 동일한 6 식*. 다른 점:

1. **state 가 시간에 따라 차원이 커지고 작아짐** (clone augment / marginalize).
2. **한 update 의 measurement 차원이 매우 큼** (수십~수백 픽셀).
3. **measurement model $h$ 에 *간접 변수* (feature 위치) 가 끼어 있어**, $h$ 의 Jacobian 을 그대로 못 쓰고 *feature 변수 차원 제거* (null-space) 가 필요.

---

## 2. State 구조 — *무엇을 추정하는가*

### 📌 요약

TC MSCKF 의 state 는 LC 의 IMU state 위에 *카메라 pose clone* 을 sliding window 로 얹은 형태. (선택적으로 SLAM feature, 캘리브레이션 변수 추가.)

$$
\boxed{\;\mathbf{x} \;=\; \bigl[\,\mathbf{x}_{IMU};\;\;\mathbf{x}_{\text{clones}};\;\;\mathbf{x}_{\text{features}}\,\bigr]\;}
$$

### 🧮 수식 — 각 구성요소

**1) IMU 부분** (LC 와 동일, 15 DoF error state):

$$
\mathbf{x}_{IMU} = \bigl(\;q_{GI}\in SO(3),\; \mathbf{p}_{IinG}\in\mathbb{R}^3,\; \mathbf{v}_{IinG}\in\mathbb{R}^3,\; \mathbf{b}_g\in\mathbb{R}^3,\; \mathbf{b}_a\in\mathbb{R}^3\;\bigr)
$$

Error state: $\delta\mathbf{x}_{IMU} = (\delta\boldsymbol{\theta},\;\delta\mathbf{p},\;\delta\mathbf{v},\;\delta\mathbf{b}_g,\;\delta\mathbf{b}_a)\in\mathbb{R}^{15}$.

**2) Clone 부분** (sliding window, $N$ 개):

$$
\mathbf{x}_{\text{clones}} = \bigl\{\;(q_{GI_k},\; \mathbf{p}_{I_kinG})\;:\; k = 1, 2, \ldots, N\;\bigr\}
$$

각 clone 은 *과거 카메라 노출 시각 $t_k$ 의 IMU pose*. Pose 만 (속도·바이어스 ❌) 보관 — 측정 시점의 *기하* 만 필요하므로.

각 clone error: $\delta\mathbf{x}_k = (\delta\boldsymbol{\theta}_k, \delta\mathbf{p}_k)\in\mathbb{R}^6$. 전체 clone error 차원: $6N$.

**3) Feature 부분** (선택, MSCKF 본질은 ❌, SLAM feature 만 ⭕):

$$
\mathbf{x}_{\text{features}} = \bigl\{\;\mathbf{p}_{f_jinG}\in\mathbb{R}^3\;:\; j = 1, 2, \ldots, M_{\text{slam}}\;\bigr\}
$$

본 문서의 핵심 학습은 MSCKF feature (state 에 ❌, null-space 로 흡수) 이므로 $M_{\text{slam}}=0$ 인 pure MSCKF 모드 가정.

**전체 error state 차원**:

$$
\dim(\delta\mathbf{x}) = 15 + 6N + 3M_{\text{slam}}\quad\text{(가변)}
$$

공분산 $P\in\mathbb{R}^{\dim\times\dim}$ 도 가변. **이게 LC ($P\in\mathbb{R}^{15\times 15}$, 고정) 와의 가장 큰 형상 차이.**

### 💡 직관

- IMU pose 는 *현재 시점만* 하나, clone 은 *과거 시점들* 여러 개. 둘이 같은 종류의 *기하* 지만 *시간상 역할이 다름*.
- Clone 에 *속도와 바이어스를 안 넣는* 이유: 측정 모델에 안 들어감. Feature 가 관측될 때 필요한 건 "그 시점의 카메라 pose" 뿐. 속도·바이어스는 *현재 IMU* 의 propagation 에만 쓰임.
- Clone 개수 $N$ 의 trade-off: 많을수록 *긴 트랙* 의 feature 도 살릴 수 있지만 $P$ 비용 ↑. 보통 $N = 10\sim 20$.

### 🎯 EKF 의미

State 차원이 가변 → $P$ 의 **행/열 인덱싱** 이 동적. LC 처럼 `segment<3>(0)=p` 식의 고정 offset 으로 못 씀. 따라서 *각 state 변수가 자기 위치 인덱스를 들고 다니는* 구조 (OpenVINS 의 `Type::id()`).

이게 곧 "왜 `StateHelper` 라는 별도 클래스가 필요한가" 의 이유. LC 에서는 직접 P 의 블록을 슬라이싱했지만, TC 에서는 *state 변수가 covariance 의 어디 있는지* 를 매번 조회해야 함.

---

## 3. Predict — IMU 가 state 를 미는 단계

### 📌 요약

IMU propagation 자체는 LC 와 **수식·코드 모두 동일**. 단 한 가지 다른 점: 카메라 노출 시각이 오면 *propagation 끝에 clone augmentation* 이 따라붙는다.

따라서 본 문서에서는 **간략히만**. (자세히는 LC 캐논 문서 [[20260514_lc_ekf_cam_imu_fusion]] §2 참조.)

### 🧮 수식 — Nominal state ODE

Bias 보정된 측정값 $\boldsymbol{\omega}_c = \boldsymbol{\omega}_m - \mathbf{b}_g$, $\mathbf{a}_c = \mathbf{a}_m - \mathbf{b}_a$ 라 두면:

$$
\dot{\mathbf{p}}_{IinG} = \mathbf{v}_{IinG},\qquad
\dot{\mathbf{v}}_{IinG} = R_{GI}\,\mathbf{a}_c + \mathbf{g}_G,\qquad
\dot{R}_{GI} = R_{GI}\,[\boldsymbol{\omega}_c]_\times
$$

$$
\dot{\mathbf{b}}_g = \mathbf{n}_{b_g},\qquad \dot{\mathbf{b}}_a = \mathbf{n}_{b_a}\quad (\text{random walk})
$$

### 🧮 수식 — Error-state ODE (15×15)

$$
\dot{\delta\mathbf{x}}_{IMU} = F_{15\times 15}\,\delta\mathbf{x}_{IMU} + G_{15\times 12}\,\mathbf{n}
$$

$F$ 의 비-0 블록은 정확히 LC 와 동일 — $F_{31}=I$, $F_{32}=-R[\mathbf{a}_c]_\times$, $F_{34}=-R$, $F_{22}=-[\boldsymbol{\omega}_c]_\times$, $F_{23}=-R$. (Quaternion JPL 컨벤션이냐 Hamilton 이냐에 따라 부호 한두 군데 바뀜.)

이산화: $\Phi_{15} = I + F\,dt$ (1차) 또는 정확한 행렬 지수.

### 🧮 수식 — Covariance propagation in *전체* state

전체 $P$ 는 블록 구조:

$$
P = \begin{bmatrix} P_{II} & P_{IC} \\ P_{CI} & P_{CC} \end{bmatrix}
$$

여기서 $I = IMU$, $C = $ clones (있는 모든 clone 합쳐서). Propagation:

$$
\Phi_{\text{total}} = \begin{bmatrix} \Phi_{15} & 0 \\ 0 & I_{6N} \end{bmatrix},\quad
Q_{\text{total}} = \begin{bmatrix} Q_d & 0 \\ 0 & 0 \end{bmatrix}
$$

→ **clone 부분은 propagate 동안 가만히** 있고 (과거의 pose 니까), IMU 부분만 정상적으로 propagate. 단 **cross-term $P_{IC}$ 는 $\Phi_{15}$ 가 좌측 곱해지므로 함께 변화**:

$$
P_{IC}^+ = \Phi_{15}\,P_{IC}^-,\qquad P_{CC}^+ = P_{CC}^-,\qquad P_{II}^+ = \Phi_{15}\,P_{II}^-\,\Phi_{15}^\top + Q_d
$$

### 💡 직관

- Clone 은 *시간이 멈춘 사진*. IMU 만 시간을 따라 움직인다.
- 다만 IMU 의 *불확실성* 이 커지면 clone 과의 *상대* 도 불확실해져야 함 → cross-term $P_{IC}$ 만 변화하고 $P_{CC}$ 는 안 변.

### 🎯 EKF 의미

LC 의 propagation 은 한 줄: $P \leftarrow \Phi P \Phi^T + Q$. TC 도 **수식은 한 줄**이지만 *블록 구조* 를 보면 의미가 다르다. 핵심: **cross-term 보존이 모든 information transfer 의 근원**. Stage 2 (update) 에서 한 clone 의 변화를 다른 clone 으로 전파시키는 것도 결국 이 cross-term 의 KH 형태 변환.

---

## 4. Clone Augmentation — sliding window 의 탄생

### 📌 요약

새 카메라 노출 시각 $t_k$ 가 오면, **현재 IMU pose 의 복사본** 을 state 에 추가:

$$
\mathbf{x} \;\leftarrow\; \bigl[\,\mathbf{x};\;\; q_{GI_k}\!:=\!q_{GI}(t_k),\;\; \mathbf{p}_{I_kinG}\!:=\!\mathbf{p}_{IinG}(t_k)\,\bigr]
$$

단순 복사가 아니라 **공분산 $P$ 도 함께 augment** 해야 한다 — 새 clone 은 *현재 IMU 와 완전히 상관* 있으므로 cross-term 을 0 으로 두면 정보 손실.

### 🧮 수식 — Augmentation Jacobian

새 clone 의 error 와 IMU error 의 관계 (clone = 복사이므로 error 도 동일):

$$
\delta\mathbf{x}_{\text{new clone}} \;=\; J_{\text{aug}}\,\delta\mathbf{x}_{\text{old}}
$$

$J_{\text{aug}}$ 는 (6 × old_dim) 차원. IMU pose 블록에 $I_6$, 나머지 0:

$$
J_{\text{aug}} = \bigl[\;\underbrace{I_6}_{\text{IMU pose}}\;\;\;\mathbf{0}\;\;\bigr]
$$

(엄밀히는 IMU 의 첫 6 블록 [$\delta\boldsymbol{\theta},\delta\mathbf{p}$] 에 매핑되도록 indexing.)

새 공분산:

$$
P^{\text{aug}} = \begin{bmatrix} P & P\,J_{\text{aug}}^\top \\ J_{\text{aug}}\,P & J_{\text{aug}}\,P\,J_{\text{aug}}^\top \end{bmatrix}
$$

블록 단위로 풀면:

- 우상단 $P\,J_{\text{aug}}^\top$ → "기존 state ↔ 새 clone" cross-term. *기존 P 의 IMU pose 컬럼 그대로 복사*.
- 우하단 $J_{\text{aug}}\,P\,J_{\text{aug}}^\top$ → "새 clone ↔ 새 clone" → *기존 P 의 IMU pose 블록 그대로 복사*.

### 💡 직관

> "Clone augmentation 은 *복사* 다. State 의 한 줄을 그대로 늘리고, 공분산도 그 줄에 해당하는 모든 cross-term 을 그대로 늘린다."

만약 cross-term 을 0 으로 두면? → 새 clone 이 *기존 IMU 와 독립이라고 거짓말* 함. 첫 update 에서 K 가 비정상적으로 작아져 정보가 안 흡수됨.

### 🎯 EKF 의미

Augmentation 자체는 *measurement-less* 한 state 변환. 정보 보존 (entropy 불변) 이므로 새 covariance 가 기존보다 *작아지지도, 커지지도 않음*. 그저 차원이 늘었을 뿐.

이게 곧 cheatsheet Part 4.2 의 "**공분산 행렬도 같이 augmented: 새 행/열은 IMU pose 와 *상관* 을 가짐. 단순 zero-pad 면 정보 손실**" 의 수학적 본체.

---

## 5. Per-feature 측정 모델 — pixel residual & Jacobian

### 📌 요약

한 feature $f_j$ 가 $M$ 개 clone $\{t_{k_1}, \ldots, t_{k_M}\}$ 에서 관측되면, *시점별 픽셀 측정* $\mathbf{z}_{jm}\in\mathbb{R}^2$ 가 $M$ 개 모인다. Measurement model:

$$
\mathbf{z}_{jm} = \pi\bigl(\,R_{C_kI_k}\,R_{GI_k}^\top\,(\mathbf{p}_{f_jinG} - \mathbf{p}_{I_kinG}) + \mathbf{p}_{IinC}\,\bigr) + \mathbf{n}_{jm}
$$

여기서:
- $\pi(\mathbf{X}_C) = (X_C/Z_C,\; Y_C/Z_C)$ 의 정규화 후 intrinsic 적용 (간단히 정규화 좌표 + $f, c$).
- $(R_{C_kI_k}, \mathbf{p}_{IinC})$ : IMU-camera 캘리브레이션 (보통 고정).

### 🧮 수식 — 잔차 & Jacobian 분리

선형화 (작은 perturbation):

$$
\mathbf{r}_{jm} = \mathbf{z}_{jm} - \hat{\mathbf{z}}_{jm} \;\approx\; H_{x,jm}\,\delta\mathbf{x}_{\text{clone}_k} \;+\; H_{f,jm}\,\delta\mathbf{p}_{f_jinG} \;+\; \mathbf{n}_{jm}
$$

여기서:

- $H_{x,jm}\in\mathbb{R}^{2\times 6}$ : clone $k$ 의 pose error $(\delta\boldsymbol{\theta}_k, \delta\mathbf{p}_k)$ 에 대한 Jacobian.
- $H_{f,jm}\in\mathbb{R}^{2\times 3}$ : feature 의 global 위치 error 에 대한 Jacobian.

Chain rule 분해:

$$
H_{x,jm} = \frac{\partial\pi}{\partial\mathbf{X}_C}\bigg|_{\hat{\mathbf{X}}_C} \cdot \frac{\partial\mathbf{X}_C}{\partial\delta\mathbf{x}_{\text{clone}_k}},\qquad
H_{f,jm} = \frac{\partial\pi}{\partial\mathbf{X}_C}\bigg|_{\hat{\mathbf{X}}_C} \cdot R_{C_kI_k}\,R_{GI_k}^\top
$$

$\partial\pi/\partial\mathbf{X}_C$ 는 표준 핀홀 (정규화 후):

$$
\frac{\partial\pi}{\partial\mathbf{X}_C} = \frac{1}{Z_C}\begin{bmatrix} 1 & 0 & -X_C/Z_C \\ 0 & 1 & -Y_C/Z_C \end{bmatrix}
$$

### 🧮 수식 — feature $j$ 전체 stack ($M$ 개 측정)

$j$ 번째 feature 의 *모든 측정* 을 세로로 쌓으면:

$$
\underbrace{\mathbf{r}_j}_{2M\times 1} \;\approx\; \underbrace{H_{x,j}}_{2M\times \dim(\delta\mathbf{x})}\,\delta\mathbf{x} \;+\; \underbrace{H_{f,j}}_{2M\times 3}\,\delta\mathbf{p}_{f_j} \;+\; \mathbf{n}_j
$$

$H_{x,j}$ 는 *대부분 0*. 오직 feature 가 관측된 $M$ 개 clone 의 컬럼 (각 6열) 에만 non-zero block. 이 sparsity 가 measurement compression 단계 (Stage 2c) 의 핵심 자원.

### 💡 직관

- 한 feature 의 픽셀 측정 = "*그 시점의 카메라 pose* 와 *feature 의 절대 위치*" 둘 다의 함수.
- 우리가 *원하는 것*: pose 보정. 우리가 *방해받는 것*: feature 위치도 함께 미지수.
- → 다음 섹션 (null-space) 의 동기.

### 🎯 EKF 의미

LC 에서는 *측정 모델 자체가 state 의 일부* (3D pose) 만의 함수였다 ($\mathbf{z}_{vo} = R_{GI}\mathbf{p}_{IC} + \mathbf{p}_{IinG}$). 그래서 Jacobian 이 깔끔히 $H\in\mathbb{R}^{3\times 15}$.

TC 에서는 측정 모델에 *state 가 아닌 변수* (feature 위치) 가 끼어든다. 그래서 Jacobian 이 $(H_x, H_f)$ 로 분리되고, 이 *외부 변수* 를 제거해야 표준 EKF update 가 가능 — 이게 다음 섹션의 정수.

---

## 6. Null-space Projection — TC MSCKF 의 정수

### 📌 요약

feature 위치 $\delta\mathbf{p}_{f_j}$ 의 의존성을 *수학적으로 제거*. 방법: $H_{f,j}$ 의 *좌측 null-space* 행렬 $N_j$ ($N_j^\top H_{f,j} = 0$) 를 양변에 곱한다.

$$
\boxed{\;N_j^\top\,\mathbf{r}_j \;=\; N_j^\top\,H_{x,j}\,\delta\mathbf{x} \;+\; N_j^\top\,\mathbf{n}_j\;}
$$

이제 *feature 항은 사라졌고*, 표준 EKF measurement equation 모양.

### 🧮 수식 — Null-space 의 존재와 차원

$H_{f,j}\in\mathbb{R}^{2M\times 3}$ 의 rank 는 일반적으로 $\min(2M, 3) = 3$ (충분히 많은 시점에서 본 경우).

→ 좌측 null-space $\mathcal{N}(H_{f,j}^\top) = \{\mathbf{v}\in\mathbb{R}^{2M} : \mathbf{v}^\top H_{f,j} = 0\}$ 의 차원은 $2M - 3$.

따라서 $N_j\in\mathbb{R}^{2M\times(2M-3)}$ 의 컬럼을 그 null-space basis 로 잡을 수 있다.

**필수 조건**: $M \geq 2$ (즉 최소 2 시점에서 본 feature). $M = 1$ 이면 $H_{f,j}$ 가 full column rank 인지 보장 안 됨 — triangulation 도 불가.

### 🧮 수식 — Null-space 계산 (QR via Givens)

$H_{f,j}$ 의 thin QR 분해:

$$
H_{f,j} = Q_j\,\begin{bmatrix} R_{1,j} \\ 0 \end{bmatrix},\qquad Q_j = [\,Q_{1,j}\;\;Q_{2,j}\,],\quad Q_{1,j}\in\mathbb{R}^{2M\times 3},\;\; Q_{2,j}\in\mathbb{R}^{2M\times(2M-3)}
$$

여기서 $R_{1,j}\in\mathbb{R}^{3\times 3}$ 은 upper-triangular. 이때 $Q_{2,j}^\top H_{f,j} = 0$ 이므로 **$N_j = Q_{2,j}$**.

OpenVINS 가 *Householder/SVD 가 아닌 Givens rotation* 을 쓰는 이유: 행 단위로 $H_{f,j}$ 의 한 열 한 열 0 으로 만들어가면서 *그 회전을 $H_{x,j}$ 와 $\mathbf{r}_j$ 에 그대로 적용* → $Q_j^\top$ 을 *명시적으로 만들 필요 없이* in-place 변환. 메모리 절약.

### 🧮 수식 — Projection 후 잔차의 통계량

$$
\mathbf{r}'_j = N_j^\top\,\mathbf{r}_j,\qquad H'_{x,j} = N_j^\top\,H_{x,j},\qquad \mathbf{n}'_j = N_j^\top\,\mathbf{n}_j
$$

$N_j$ 의 컬럼이 *직교 정규* 이면 (QR 의 $Q_2$ 는 자동 직교 정규):

$$
\text{Cov}(\mathbf{n}'_j) = N_j^\top\,\sigma_{\text{pix}}^2 I_{2M}\,N_j = \sigma_{\text{pix}}^2\,I_{2M-3}
$$

→ 노이즈가 여전히 *isotropic* — 이것이 Givens/QR null-space 의 또 다른 장점. SVD 로 잡아도 같지만 굳이.

### 💡 직관 (네 가지 비유)

1. **물리적 직관**: $H_{f,j}$ 의 *열 공간* (column space) 은 "feature 가 움직일 때 잔차가 움직이는 방향". 그 **수직** 방향에서 본 잔차는 feature 의 움직임과 무관 → pose 만의 정보.

2. **선형대수 직관**: $H_{f,j}\,\delta\mathbf{p}_f$ 라는 항은 $\mathbb{R}^{2M}$ 의 *부분공간* (열공간, 3-dim) 안에서만 흔들린다. 그 부분공간에 수직인 방향 ($N_j$ 가 펼치는, $2M-3$-dim) 의 잔차는 *feature 위치와 무관*.

3. **Schur complement 동치**: 만약 feature 위치를 *state 에 잠시 넣고* Kalman update 후 *바로 marginalize* 한다면, 결과적으로 *동일* 한 효과 (수학적으로 equivalent). 즉 null-space projection = "*feature 를 state 에 넣었다가 즉시 빼는* 압축 절차".

4. **정보 흐름 직관**: feature 의 픽셀 정보가 *그 feature 위치 추정* 과 *카메라 pose 보정* 두 곳으로 갈라지는데, 우리는 pose 만 원함. 그래서 *feature 방향 채널* (3-dim) 을 잘라내고 *pose 방향 채널* ($2M-3$-dim) 만 남긴다.

### 🎯 EKF 의미

LC 의 measurement 차원은 항상 3 (한 시점의 3D pose). TC + null-space 후 measurement 차원은 *feature 마다 $2M-3$*. 여러 feature 를 쌓으면 수십~수백 행의 *키 큰 (tall)* measurement equation 이 된다. 이게 곧 Stage 2c (compression) 의 동기.

**왜 이 트릭이 강력한가**: feature 를 state 에 안 넣었음에도 *feature 정보의 pose-relevant 부분만은 살림*. State 차원 증가 없이 multi-view constraint 의 본질적 이득을 챙긴다. → EKF-SLAM 의 O(n²) 비용을 피하면서 BA 수준의 정보 활용에 근접.

---

## 7. Chi² gating + Measurement Compression + EKF Update

### 📌 요약

Null-space projection 까지 끝낸 *feature-별* 잔차 $\mathbf{r}'_j$ 와 Jacobian $H'_{x,j}$ 를 가지고:

1. **per-feature chi² gate** — 이상치 feature 제거.
2. 남은 feature 를 *모두 세로로 stack* → tall 시스템 $(\mathbf{r}_{\text{big}}, H_{\text{big}})$.
3. **Measurement compression** — QR 로 행 수를 state 차원까지 줄임 (정보 보존).
4. 최종 EKF update (Joseph form).

### 🧮 수식 — Per-feature Chi² Gate

feature $j$ 의 마할라노비스 거리:

$$
\chi^2_j \;=\; (\mathbf{r}'_j)^\top \bigl(H'_{x,j}\,P\,(H'_{x,j})^\top + \sigma_{\text{pix}}^2 I\bigr)^{-1}\,\mathbf{r}'_j
$$

자유도 $d_j = 2M_j - 3$ 에서 95% chi² 임계값 $\chi^2_{0.95}(d_j)$ 보다 크면 그 feature 폐기.

LC 의 `innov.norm() > 5.0` 이 *isotropic threshold* 였다면, 이건 *공분산-aware* threshold. 차원 다른 잔차들을 *통계적으로 동일 기준* 으로 비교 가능.

### 🧮 수식 — Stacking & Measurement Compression

남은 $K$ 개 feature 의 잔차·Jacobian 을 세로로 쌓음:

$$
\mathbf{r}_{\text{big}} = \begin{bmatrix} \mathbf{r}'_1 \\ \vdots \\ \mathbf{r}'_K \end{bmatrix} \in\mathbb{R}^{D\times 1},\qquad
H_{\text{big}} = \begin{bmatrix} H'_{x,1} \\ \vdots \\ H'_{x,K} \end{bmatrix} \in\mathbb{R}^{D\times n}
$$

여기 $D = \sum_j (2M_j - 3)$ (전체 측정 차원), $n = \dim(\delta\mathbf{x})$ (state 차원).

보통 $D \gg n$ (수백 행, 수십~수백 컬럼). 그대로 EKF update 하면 $K = PH^\top S^{-1}$ 의 $S$ 역행렬이 $D\times D$ → 매우 비쌈.

**Trick**: $H_{\text{big}}$ 의 thin QR:

$$
H_{\text{big}} = Q\,\begin{bmatrix} T_H \\ 0 \end{bmatrix},\quad Q = [Q_1\;\;Q_2],\;\; T_H\in\mathbb{R}^{n\times n}\text{ upper-triangular}
$$

$Q^\top$ 을 양변에 곱하면:

$$
Q^\top \mathbf{r}_{\text{big}} = \begin{bmatrix} T_H \\ 0 \end{bmatrix}\delta\mathbf{x} + Q^\top \mathbf{n}_{\text{big}}
$$

상단 $n$ 행만 남기면 (하단은 $0\cdot\delta\mathbf{x} + \text{noise}$ 라 *state 정보 없음*):

$$
\boxed{\;\mathbf{r}_{\text{comp}} = Q_1^\top \mathbf{r}_{\text{big}},\;\; H_{\text{comp}} = T_H,\;\; R_{\text{comp}} = \sigma^2 I_n\;}
$$

이제 measurement 차원이 $n$ 으로 줄었다 (정보 손실 0). EKF update 비용도 $O(n^3)$.

### 🧮 수식 — EKF Update (Joseph form)

$$
S = H_{\text{comp}}\,P\,H_{\text{comp}}^\top + R_{\text{comp}}
$$
$$
K = P\,H_{\text{comp}}^\top\,S^{-1}
$$
$$
\delta\mathbf{x} = K\,\mathbf{r}_{\text{comp}}
$$
$$
\mathbf{x} \;\boxplus=\; \delta\mathbf{x},\qquad P \leftarrow (I - K H_{\text{comp}})\,P\,(I - K H_{\text{comp}})^\top + K R_{\text{comp}} K^\top
$$

마지막 줄이 Joseph form — LC 와 동일. $\boxplus$ 는 manifold 합 (회전은 $R\cdot\exp(\delta\boldsymbol{\theta})$).

### 💡 직관

- Chi² gate: "이 feature 가 *현재의 P 와 R 을 가정하면* 발생 확률 5% 이하면 outlier."
- Measurement compression: tall 행렬을 *수직 방향으로 회전* 시켜 *정보가 있는 $n$ 행* 만 남기고 *정보 없는 (state 와 무관한) 잡음 행* 을 잘라냄. 정보 ✓, 비용 ✓.
- EKF update: 결국 *LC 의 update 와 동일한 6 식*. 다른 점은 *measurement 차원 $n$ × state 차원 $n$* 의 정사각 시스템이라는 것뿐.

### 🎯 EKF 의미

이 단계의 EKF math 는 LC 와 *완전히 동일*. *데이터 준비 단계* 가 다를 뿐이다. 즉:

| 단계 | LC | TC |
|---|---|---|
| Measurement 준비 | 한 줄 ($z_{vo} = R\mathbf{p}_{IC} + \mathbf{p}_{IinG}$) | per-feature triangulation + Jacobian + null-space + chi² + stack + compress |
| EKF math | $K, S, x^+, P^+$ | $K, S, x^+, P^+$ (동일) |

→ "TC = LC + *measurement 준비의 파이프라인이 길어진 것*" 으로 봐도 무방.

---

## 8. Marginalization — clone 의 안전한 제거

### 📌 요약

Sliding window 의 크기를 일정하게 유지하려면, 새 clone 이 들어올 때 *오래된 clone 하나를 빼야* 한다. 핵심 질문: **빼면서 정보를 잃지 않으려면 어떻게?**

답: 그 clone 이 *관련된 모든 measurement 를 이미 update 에 흡수* 했다면, 단순히 $P$ 의 해당 행·열을 *삭제* 해도 무방. OpenVINS 는 이를 이렇게 보장 — *update 후* marginalize.

### 🧮 수식 — Drop vs. Schur 의 동치

만약 clone $k$ 에 *아직 처리하지 않은 measurement* 가 있다면, 단순 drop 은 정보 손실. 정보 보존 제거는 *Schur complement* 가 표준.

State 를 두 블록으로 나눔 ($r$ = remain, $m$ = marginalize):

$$
P = \begin{bmatrix} P_{rr} & P_{rm} \\ P_{mr} & P_{mm} \end{bmatrix}
$$

$r$ 블록의 새 공분산:

$$
P_{rr}^{\text{new}} = P_{rr} - P_{rm}\,P_{mm}^{-1}\,P_{mr}\quad(\text{Schur complement})
$$

만약 $m$ 블록과 $r$ 블록이 *현재 상태에서 사실상 독립* ($P_{rm}\to 0$) → $P_{rr}^{\text{new}} \to P_{rr}$ → drop 과 동일.

MSCKF 의 통상 흐름:
1. clone $k$ 가 관련된 모든 feature 트랙이 *끝났을 때* (또는 max window 도달 시) update 발동.
2. update 가 *그 clone 의 픽셀 정보를 모두 흡수* (cross-term 을 통해 다른 clone, IMU 로 전파).
3. 흡수 후엔 clone $k$ 의 *추가 정보* 가 없으므로 drop ≈ Schur.

OpenVINS 는 한 발 더 신중 — *Schur 식을 그대로 사용* 하여 항상 정보 보존을 수학적으로 보장 (drop 의 가정에 실수가 없도록).

### 💡 직관

- "clone 을 *기억에서 지운다*. 단 그 clone 이 만든 *기여* 는 이미 다른 변수의 평균·공분산에 누적된 채로 두고."
- 정보이론적: Schur complement 은 *조건부 공분산* 의 식. 즉 "$m$ 변수가 정확히 알려졌을 때 $r$ 변수의 공분산" — 하지만 실제로는 $m$ 을 *적분 (marginalize) 해서 없애는* 것이 목적. 두 연산이 가우시안에서는 *같은 식* 으로 떨어지는 마법.
- LC 에는 이 개념 자체가 없음. LC 의 measurement 는 한 시점이 즉시 흡수되고 끝.

### 🎯 EKF 의미

Marginalize 의 비용: $P_{mm}$ 의 역행렬 ($6\times 6$, 또는 작은 블록). 거의 무료.

**위험**: $P_{mm}$ 이 *수치적으로 singular* 에 가까우면 Schur 가 폭발. 보통 정상 동작에서는 일어나지 않지만, FEJ 같은 consistency trick 의 동기 중 하나가 이거.

---

## 9. 전체 사이클 + LC vs TC 비교

### 📌 요약

한 카메라 프레임이 도착했을 때:

```
1. (도착 직전까지 들어온 IMU 들로) Propagate
2. Clone augmentation (현 IMU pose 복사)
3. Image tracking (feature association)
4. Feature 별:
   a. Triangulation
   b. Jacobian H_f, H_x, residual r
   c. Null-space project
   d. Chi² gate
5. 모든 feature stack → measurement compression
6. EKF update (Joseph form)
7. (필요시) Marginalize old clone
```

### 🧮 수식 — 한 cycle 의 정보 흐름

$$
\underbrace{P(t_{k-1}^+)}_{\text{이전 update 직후}} \xrightarrow{\text{propagate}} P^-(t_k) \xrightarrow{\text{augment}} P^{\text{aug}} \xrightarrow{\text{update}} P(t_k^+) \xrightarrow{\text{marg}} P(t_k^{++})
$$

차원 변화:
- propagate: 차원 그대로.
- augment: $+6$.
- update: 그대로 (정보 ↑).
- marg: $-6$ (가장 오래된 clone 제거).

정상 동작에서 *총 차원이 일정* ($N$ clone 유지).

### LC vs TC 종합 비교

| 항목 | LC (`lc_ekf.cpp`) | TC MSCKF |
|---|---|---|
| **State** | $15$ (IMU만, 고정) | $15 + 6N$ (IMU + clones, 가변) |
| **Measurement** | 한 시점의 3D pose ($\in\mathbb{R}^3$) | per-feature 픽셀들 (feature 별 $2M-3$, 모든 feature stack → 수십~수백 행) |
| **Measurement model** | $\mathbf{z} = R\mathbf{p}_{IC} + \mathbf{p}$ | $\mathbf{z}_{jm} = \pi(R_{CI}R_{GI}^\top(\mathbf{p}_f - \mathbf{p}_I) + \mathbf{p}_{IC})$ |
| **Feature** | EKF 밖 (VO 내부) | EKF 안 (triangulation 만, state ❌, null-space 로 흡수) |
| **Sliding window** | 없음 | 있음 (clone 10~20 개 유지) |
| **Outlier** | norm threshold | chi² gate (per-feature) |
| **Update 비용** | $O(15^3)$ per cam frame | $O((15+6N)^3)$ per cam frame |
| **정보 흐름 깊이** | 픽셀 → VO → pose → EKF (3 단계 손실) | 픽셀 → null-space → EKF (1 단계 손실) |
| **수식 본질** | EKF 6 식 + 위치 측정 | EKF 6 식 + 측정 준비 pipeline (triangulation→Jacobian→null-space→compress) |

### 💡 한 줄 요약 (학습 종착점)

> "TC MSCKF 는 *EKF 자체가 바뀐 게 아니다*. measurement 의 준비 과정이 *카메라 raw 픽셀에서 시작* 하도록 늘어났을 뿐. 그 늘어난 파이프라인의 정수가 *null-space projection* — feature 위치를 state 에 넣지 않고도 픽셀 정보를 흡수하는 트릭."

---

## 부록 A. 변수·기호 사전

| 기호 | 의미 | 차원 |
|---|---|---|
| $q_{GI}, R_{GI}$ | IMU 자세 (global ← IMU) | quaternion / SO(3) |
| $\mathbf{p}_{IinG}, \mathbf{v}_{IinG}$ | IMU 위치·속도 (global) | $\mathbb{R}^3$ |
| $\mathbf{b}_g, \mathbf{b}_a$ | gyro / accel bias | $\mathbb{R}^3$ |
| $q_{GI_k}, \mathbf{p}_{I_kinG}$ | clone $k$ 의 pose | 6 DoF |
| $\mathbf{p}_{f_jinG}$ | feature $j$ 의 global 위치 | $\mathbb{R}^3$ |
| $R_{C_kI_k}, \mathbf{p}_{IinC}$ | IMU↔camera 캘리브레이션 | 6 DoF (고정) |
| $\pi(\cdot)$ | 카메라 projection (정규화 좌표 + intrinsic) | $\mathbb{R}^3\to\mathbb{R}^2$ |
| $\mathbf{z}_{jm}$ | feature $j$, clone $m$ 의 픽셀 측정 | $\mathbb{R}^2$ |
| $\mathbf{r}_{jm} = \mathbf{z}_{jm} - \hat{\mathbf{z}}_{jm}$ | 픽셀 잔차 | $\mathbb{R}^2$ |
| $H_{x,jm}, H_{f,jm}$ | Jacobian (clone pose, feature 위치) | $2\times 6$, $2\times 3$ |
| $N_j$ | feature $j$ 의 $H_f$ 좌측 null-space | $2M\times(2M-3)$ |
| $\mathbf{r}'_j, H'_{x,j}$ | null-space projection 후 잔차·Jacobian | $(2M-3)\times 1$, $(2M-3)\times n$ |
| $\chi^2_j$ | per-feature 마할라노비스 거리² | scalar |
| $T_H$ | QR 압축 후 upper-triangular Jacobian | $n\times n$ |
| $\Phi, Q, P, K, S$ | EKF 표준 기호 | (각자) |
| $\delta\mathbf{x}, \boxplus$ | error state, manifold 합 | — |

## 부록 B. 다음 학습 — Deep Dive 지도

본 spine 문서를 기준으로 Stage 1~3 의 deep dive 는 *코드·구체 알고리즘·C++ 노트* 를 추가:

- **Stage 1a** (§4 deep): `Propagator::propagate_and_clone()` + `StateHelper::augment_clone()` 의 *공분산 augmentation Jacobian 의 실제 구현*, `Type::id()` 인덱싱.
- **Stage 1b** (§8 deep): `StateHelper::marginalize_old_clone()` 의 Schur 실제 코드, `_variables` 재정렬.
- **Stage 2a** (§5 deep): `UpdaterHelper::get_feature_jacobian_full()` 의 *chain rule 구현*, `FeatureInitializer` 의 triangulation (DLT + Gauss-Newton).
- **Stage 2b** (§6 deep): `UpdaterHelper::nullspace_project_inplace()` 의 *Givens rotation in-place* 구현. 왜 Householder 가 아닌가.
- **Stage 2c** (§7 deep): `measurement_compress_inplace()` + chi² table + `StateHelper::EKFUpdate()` 의 Joseph form 실제 코드.
- **Stage 3** (§9 deep): `VioManager::do_feature_propagate_update()` 의 전체 시퀀스 walk-through, 시간 측정, 디버깅 포인트.

이후 *별도 사이클* 로 ov_plane 의 plane-aided 변형 분석 + (선택) tc_msckf_clean_room 클린룸 재구현.

---

## 부록 C. 본 문서가 *명시적으로 다루지 않는* 것

본 문서의 의도적 단순화:

- **FEJ (First-Estimate Jacobians)** — observability consistency 트릭. 핵심 흐름 익힌 뒤 별도 학습.
- **Inverse-depth representation** — feature 의 *parameterization* 선택. 본 문서는 global cartesian 만.
- **SLAM features** — state 에 들어가는 long-tracked feature. 본 문서는 pure MSCKF feature (state ❌) 만.
- **Online camera/IMU calibration** — `do_calib_*`. 본 문서는 캘리브레이션 고정 가정.
- **Multi-camera fusion** — stereo 의 두 카메라가 *같은 IMU clone* 을 공유하지만 별도 측정. 본 문서는 mono 로 단순화.
- **Plane constraint (Seg-aided)** — 본인 연구의 차별점이지만, *TC MSCKF 본질* 학습 이후의 *확장*. 본 문서는 vanilla MSCKF.

이들 항목은 차례차례 후속 사이클에서 추가 — 본 spine 위에 *가지* 로 자라남.

---

> 학습 점검: 본 문서를 다 읽었을 때 다음 세 문장을 *백지에서* 쓸 수 있는가?
>
> 1. "MSCKF state 는 ___ 인데, 한 카메라 프레임마다 차원이 ___ 늘어난다. 늘어난 부분은 ___ 의 복사본이고, 공분산도 ___ 식으로 확장된다."
> 2. "feature 의 픽셀 잔차의 Jacobian 은 ___ 와 ___ 의 두 부분으로 갈라지는데, 우리는 ___ 의 좌측 null-space 를 곱해 ___ 항을 제거한다. 그 결과 측정 차원이 $2M$ 에서 $2M-3$ 으로 줄지만 ___ 정보는 보존된다."
> 3. "EKF update 자체는 LC 와 ___ 한 6 식이지만, TC 의 measurement 준비 과정은 ___ → ___ → ___ → ___ → ___ 의 5단계를 거친다."
>
> 위 세 문장이 막힌다면, 막힌 섹션 (§2, §6, §9) 로 돌아가 다시 읽을 것.
