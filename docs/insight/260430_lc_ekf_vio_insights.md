---
title: "LC-EKF VIO 분석 인사이트"
date: 2026-04-30
type: insight
tags: [vio, lc-ekf, baseline, ekf]
related:
  - "[[260430_ppt_guide]]"
  - "[[20260514_lc_ekf_cam_imu_fusion]]"
status: reviewed
---

# LC-EKF VIO 분석 인사이트 — 2026-04-30

## 1. sigma_vo 의미와 IMU-only 구현

### sigma_vo 값의 실제 의미

`sigma_vo`는 VO 계측 노이즈 표준편차 [m]. EKF update 식에서:

```
R_noise = sigma_vo² * I₃
S = H·P·Hᵀ + R_noise
K = P·Hᵀ·S⁻¹
```

- `sigma_vo` 작을수록 VO를 강하게 신뢰 → IMU drift 억제
- `sigma_vo` 크면 Kalman gain K → 0 → VO update 사실상 무시

### IMU-only 구현법

```yaml
ekf:
  sigma_vo: 1.0e9   # Kalman gain ≈ 0 → 사실상 IMU-only
```

`sigma_vo: 1`은 IMU-only가 **아님** — VO update는 여전히 호출됨, 다만 약하게 신뢰할 뿐.

진짜 IMU-only = `sigma_vo: 1.0e9` (재빌드 불필요, config만 수정)

---

## 2. output 경로 수정이 반영 안 된 이유

config YAML은 런타임에 읽히는 파일 → **재빌드 불필요**.  
문제는 편집 후 저장이 안 된 것 (output 경로 변경이 누락됨).  
항상 실행 전 config 파일을 직접 확인할 것.

---

## 3. IMU-only 결과 해석

### 좌표계 차이로 초기 위치가 다른 이유

| | Estimated (EKF) | GT (KITTI oxts) |
|---|---|---|
| 원점 | IMU 초기화 시 p=(0,0,0) + camera lever arm | GPS Mercator local ENU |
| 방향 | 중력 Z-up 정렬 | ENU (East-North-Up) |

- **VO 활성** 시: VO measurement가 두 좌표계를 암묵적으로 연결 → GT와 비슷한 공간 유지
- **IMU-only** 시: 좌표계 연결 없음 → 처음부터 offset 발생

ATE는 Umeyama alignment로 계산하므로 수치는 의미 있음. 그래프는 raw 좌표 그대로.

### IMU-only drift 규모 (drive_0117, 660 frames ≈ 70s)

```
Frame 100 (t≈10s):  ~9m
Frame 300 (t≈30s):  ~640m
Frame 600 (t≈60s):  ~3675m
ATE RMSE:           ~1702m
```

VO fusion 시 ATE: **4.78m** → IMU-only: **1702m** → **VO의 drift 억제 효과 약 355배**

---

## 4. KITTI raw _sync 데이터의 IMU/Camera 주파수

```
IMU (oxts): dt ≈ 0.11s → ~9 Hz
Camera:     dt ≈ 0.10s → ~10 Hz
```

**둘 다 ~10 Hz** — `_sync` 폴더는 모든 센서를 카메라 프레임에 동기화하기 때문.

| 데이터 | raw (비동기) | _sync (사용 중) |
|---|---|---|
| Camera | 10 Hz | 10 Hz |
| IMU (oxts) | **100 Hz** | **10 Hz** |

### VIO 관점에서의 문제

정상적인 EKF-VIO 구조:
- IMU 100 Hz → high-rate propagation (dt=0.01s, 적분 오차 작음)
- Camera 10 Hz → low-rate update

_sync 사용 시:
- IMU도 10 Hz → dt=0.1s로 적분 → **drift 10배 빠르게 누적**
- IMU-only 결과가 특히 나쁜 주요 원인 중 하나

제대로 된 IMU 성능 평가 → raw 100Hz oxts 데이터 필요

---

## 5. Per-frame Error의 U자형 원인

### Umeyama alignment의 부작용

```python
# 현재 per-frame error 계산
est_aligned, R, t = rigid_align(est_pos, gt_interp)  # 전체 프레임 최적화
per_frame_err = ||est_aligned_i - gt_i||
```

Umeyama는 **전체 trajectory를 한꺼번에** 최소자승으로 정렬.  
IMU drift가 선형적으로 쌓일 때, 전역 정렬 후:

```
t=0s  : aligned 궤적과 GT가 벌어짐 → error 큼
t=40s : aligned 궤적이 GT를 교차하는 지점 → error ≈ 0
t=70s : 반대 방향으로 벌어짐 → error 다시 커짐
```

→ **선형 drift + 전역 정렬 = 항상 U자형 error curve**

### Raw drift error (실제 누적 drift)

```python
# 좌표계 회전만 보정 + t=0에서 translation 맞춤
est_rot = (R @ est_pos.T).T
shift   = gt_interp[0] - est_rot[0]
raw_err = ||est_rot + shift - gt_interp||  # 단조증가
```

| 곡선 | 의미 | 형태 |
|---|---|---|
| Umeyama-aligned error | ATE 계산용, 전역 최적 | U자형 |
| Raw drift (origin-aligned) | 실제 drift 누적 | 단조증가 |

---

## 6. 코드 변경 사항 요약

### C++ (`apps/run_euroc.cpp`, `include/ekf/lc_ekf.hpp`)

- `LcEkf::velocity()` accessor 추가
- `rot_to_euler_deg()`: ZYX Euler angle [deg] 변환
- `write_state_log()`: `state_log.txt` 저장
  - 컬럼: `timestamp px py pz vx vy vz roll_deg pitch_deg yaw_deg qx qy qz qw`
- 메인 루프에서 velocity, euler, quaternion 수집 및 저장

### Python (`tools/plot_trajectory.py`)

- `load_state_log()`: state_log.txt 파싱 (quat 없는 구버전 호환)
- `plot_state()` → `state_plot.png` 생성 (5개 패널):
  1. Position XYZ (est + GT)
  2. Velocity XYZ (est only)
  3. Euler Angles Roll/Pitch/Yaw
  4. Quaternion qx/qy/qz/qw
  5. Per-frame error: Umeyama + Raw drift 동시 표시
- `origin_align_error()`: raw drift 계산 함수 추가

---

## 7. Euler angle vs Quaternion

| | Euler Angle | Quaternion |
|---|---|---|
| 직관성 | 좋음 (° 단위) | 나쁨 (qx, qy, qz, qw) |
| Gimbal lock | 있음 (pitch=±90°) | 없음 |
| 연속성 | ±180° wrap-around | 연속 |
| 플롯 가독성 | 좋음 | 나쁨 |

KITTI 지상 차량: pitch/roll이 작아 Gimbal lock 실용적으로 발생 안 함  
→ 둘 다 `state_log.txt`에 저장, `state_plot.png`에 모두 표시

---

## 8. 결론 및 향후 과제

- **현재 문제**: `_sync` 데이터로 IMU가 10Hz → 진짜 IMU 성능 평가 불가
- **개선 방향**:
  - raw 100Hz oxts 데이터 파이프라인 구축
  - IMU propagation과 camera update 비율 100:1로 복원
- **LC-EKF의 효과**: VO fusion만으로 drift 355배 억제 확인
- **ATE 해석 주의**: Umeyama alignment 후 per-frame error는 U자형 → raw drift와 함께 해석
