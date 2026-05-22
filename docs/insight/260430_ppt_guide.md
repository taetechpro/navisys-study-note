---
title: "LC-EKF VIO 발표 구성 가이드"
date: 2026-04-30
type: insight
tags: [vio, lc-ekf, presentation]
related:
  - "[[260430_lc_ekf_vio_insights]]"
status: reviewed
---

# LC-EKF VIO 발표 구성 가이드 — 2026-04-30

## 핵심 프레임: "정보 흐름"으로 소개

역할이 아니라 정보 흐름으로 각 모듈을 소개한다.

```
StereoTracker  →  VO 위치 측정 z
ImuPropagator  →  IMU 기반 상태 예측 x⁻, P⁻
LcEkf          →  z로 x, P 보정 → x⁺, P⁺
```

수식은 모듈당 핵심 1~2개만 박스 처리. 나머지는 설명으로 대체.

---

## Slide 1: 전체 파이프라인

```
Stereo Images ──→ StereoTracker ──→ p_C^W
IMU Samples   ──→ ImuPropagator ──→ x⁻, P⁻
                          ↓
                     LcEkf Update
                          ↓
                     x⁺, P⁺
```

---

## Slide 2: Core State 정의

공통 언어 슬라이드 — 세 모듈을 연결하는 역할.

$$x = \{p,\ v,\ R,\ b_g,\ b_a\}$$

$$\delta x = [\delta p,\ \delta v,\ \delta\theta,\ \delta b_g,\ \delta b_a]^T \in \mathbb{R}^{15}$$

---

## Slide 3: StereoTracker

**목표**: stereo image → VO pose → EKF measurement

**핵심 과정**:
1. Rectification
2. Stereo matching / triangulation
3. Temporal tracking (KLT)
4. PnP RANSAC
5. camera position $p_C^W$

**핵심 수식** (PnP objective):

$$\min_{R_{CW},\, t_{CW}} \sum_i \left\| u_i - \pi\!\left(K(R_{CW}P_i^W + t_{CW})\right) \right\|^2$$

**출력**:

$$z_k = p_{C_k}^W$$

> StereoTracker는 EKF 입장에서 **위치 측정값 생성기**.

---

## Slide 4: ImuPropagator

**목표**: IMU → 다음 상태 예측

**Bias 제거**:

$$\omega = \omega_m - b_g, \quad a = a_m - b_a$$

**공칭 상태 전파**:

$$\dot{p} = v, \quad \dot{v} = Ra + g, \quad \dot{R} = R[\omega]_\times$$

**Bias random walk**:

$$\dot{b}_g = n_{bg}, \quad \dot{b}_a = n_{ba}$$

**Covariance 전파**:

$$P^-_{k+1} = \Phi P^+_k \Phi^T + GQG^T, \quad \Phi \approx I + F\Delta t$$

> 슬라이드 메시지: **IMU는 빠르지만 drift가 누적된다.**

---

## Slide 5: LcEkf

**목표**: VO position measurement로 IMU drift 보정

**측정 모델**:

$$z = p_C^W, \quad h(x) = p_I^W + R_I^W\, p_C^I$$

**Innovation**:

$$r = z - h(x)$$

**Jacobian**:

$$H = \begin{bmatrix} I_3 & 0_3 & [R_I^W p_C^I]_\times & 0_3 & 0_3 \end{bmatrix}$$

**Kalman Update**:

$$K = PH^T(HPH^T + R)^{-1}$$

$$\delta x = Kr$$

$$P^+ = (I - KH)P^-(I - KH)^T + KRK^T \quad \text{(Joseph form)}$$

> Joseph form: 수치 안정성을 위해 사용.

---

## 모듈별 발표용 한 줄 요약

| 모듈 | 한 줄 |
|---|---|
| StereoTracker | sparse feature 기반 VO 위치 측정 생성 |
| ImuPropagator | IMU로 15-state error-state EKF 예측 수행 |
| LcEkf | VO position measurement로 IMU drift 보정 |

---

## 제일 중요한 메시지

> **이 프로젝트는 Tight VIO가 아니라 Loosely-Coupled VIO다.**

- Tightly-Coupled: feature residual을 EKF에 직접 사용
- **Loosely-Coupled (이 코드)**: Stereo VO가 만든 camera position을 EKF measurement로 사용

**PPT 제목 추천**:
> *Core Modules of LC-EKF VIO*
> *Stereo VO Measurement + IMU Propagation + EKF Fusion*
