# `plane_constraint/` — Plane Pseudo-Measurement (예정 모듈)

이 폴더는 비어 있다. `semantic/` 에서 분류된 floor/wall plane normal 을 받아, EKF 의 **pseudo-measurement** 형태로 변환해 update 단계에 투입하는 모듈이다.

## 의도

Floor 위에 있어야 한다는 기하 제약, wall 에 평행 운동한다는 기하 제약을 EKF measurement equation 으로 수식화한다.

```
floor :  z_meas = n_f^T · p + b_f = 0  → Z drift 억제
wall  :  z_meas = n_w^T · p + b_w = 0  → heading drift 억제
```

이 모듈은 `lidar/lidar_segmenter.hpp` 와 `depth/stereo_depth_segmenter.hpp` 의 출력 (`SegmentedCloud`) 을 입력으로 받고, EKF (LC 또는 MSCKF) 의 update 인터페이스에 맞는 measurement / Jacobian 을 산출한다.

## 예정 파일

| 후보 파일 | 책임 |
|---|---|
| `plane_pseudo_measurement.hpp` | `SegmentedCloud` → `(z, H, R)` 튜플 변환 |
| `plane_jacobian.hpp` | floor/wall Jacobian (state 에 대해) |

## 의존성

- `semantic/segment_types.hpp` — `SegmentedCloud`, `SegmentLabel`
- `core/` — state types, SO(3)
- (interface) `ekf/` 또는 `msckf/` 의 update 함수 시그니처

## 관련 메모리

- `[[tc-msckf-pivot]]` — 본 모듈이 contribution 의 핵심 (Seg-aided)
