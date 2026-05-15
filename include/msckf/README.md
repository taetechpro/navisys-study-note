# `msckf/` — Tightly-Coupled MSCKF (예정 모듈)

이 폴더는 비어 있다. 차주 (2026-05-16~) `ov_plane` (ICRA 2023, RPNG) 의 `UpdaterMSCKF.cpp` 를 분석한 후, 본 프로젝트로 옮겨 채울 예정이다. fork 방식 vs 직접 구현 방식은 ov_plane 코드 한 번 읽고 결정한다.

## 의도

LC EKF (`include/ekf/lc_ekf.hpp`) 가 카메라의 적분된 position 만 measurement 로 받는 데 비해, MSCKF 는 **raw pixel residual** 을 매 프레임 EKF 에 직접 연결한다. 이를 통해 IMU bias 추정 정확도를 높이고 KITTI ATE ~153cm baseline (LC) 을 개선하는 것이 목표.

## 예정 파일 (차주 결정 후 추가)

| 후보 파일 | 책임 | 대응되는 ov_plane 코드 |
|---|---|---|
| `sliding_window.hpp` | camera pose clones (state 확장) | `State.h::clones_IMU` |
| `feature_track.hpp` | multi-frame KLT feature 누적 + bookkeeping | `TrackBase`, `TrackKLT` |
| `msckf_update.hpp` | nullspace projection + chi-squared gate + EKF update | `UpdaterMSCKF::update` |

## 의존성

- `core/` — SO(3), state types
- `frontend/` — stereo feature tracking (LC 와 공유)
- `semantic/segment_types.hpp` — `SegmentedCloud` 등 plane constraint 와 인터페이스 시 사용
- (선택) `plane_constraint/` — pseudo-measurement 결합

## 관련 메모리

- `[[tc-msckf-pivot]]` — 전환 결정과 contribution 3-tier 로드맵
- `[[ov-plane-benchmark]]` — 비교 대상 논문 분석
