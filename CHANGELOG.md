# Changelog

연구 마일스톤 기록. 각 버전은 git tag 와 1:1 대응. 형식은 semver 와 유사하나 *연구 단계* 기준 (검증된 baseline → 검증된 신규 contribution).

## [Unreleased] — P6 cycle 2 (accuracy debug, in progress)

- Port `ov_init::InitializerHelper::gram_schmidt` so the adapter can build
  R_GtoI exactly as OpenVINS expects (Global z aligned with body's
  gravity-opposite axis).
- Replace adapter's R0_wi-from-LC path with mean_accel-driven gram_schmidt
  init. Validate stationary v.z stays near zero.
- Re-measure KITTI 0117 ATE; target LC parity (~153 cm) or better.

진행 추적: [`docs/Practice/tc_msckf_port_progress.md`](docs/Practice/tc_msckf_port_progress.md)

## [v0.3.0-msckf-runs] — 2026-05-26

### Added (P3 → P5)
- OpenVINS 포트 **P3**: `include/msckf/update/{UpdaterMSCKF, UpdaterHelper, UpdaterOptions}`,
  `include/msckf/feat/{Feature, FeatureInitializer, FeatureInitializerOptions}` (10 파일).
  FeatureHelper / FeatureDatabase 의도적 제외 (UpdaterMSCKF 직접 의존 없음).
- OpenVINS 포트 **P4**: `include/msckf_pipeline/msckf_pipeline.hpp` + cpp
  (Hamilton ↔ JPL quat 변환, monocular MSCKF, feature DB direct mgmt,
  CamRadtan with rectified intrinsics + zero distortion).
- `include/frontend/stereo_tracker.hpp/cpp`: `prev_track_ids_` + `next_id_`
  추가. KLT / PnP / ORB 모든 분기에서 track ID 동기. LC 영향 없음.
- `apps/run_vio.cpp`: `--engine=lc|msckf` CLI 옵션, parallel engine wiring.

### Changed
- `CMakeLists.txt` — Boost::date_time 추가, msckf_pipeline.cpp + P3 cpp 4개 등록.
- `libvio_core.a` 1.99 MB → **3.0 MB**.

### Build & runtime status
- Build clean (warnings 0, errors 0).
- KITTI 0117 660-frame **crash-free** on `--engine=msckf` after 3 boundary fixes:
  first-frame dt=0 bypass (StateHelper::augment_clone), upfront IMU feed
  for OpenVINS Propagator, pre-t0 IMU also buffered for boundary interp.
- LC EKF baseline preserved: ATE **152.96 cm** (메모리의 ~153 cm 와 정확히 일치).

### Known issues (recorded, deferred to next milestone)
- MSCKF ATE 53,279 cm (diverges). Root cause analysis ongoing:
  - G1 (gravity sign): rejected — flipping made it worse (1.33M cm).
  - G2 (R1 quat round-trip): verified — |R_state - R_wi^T| = 1.13e-16.
  - G3 (frame convention): suspected. Stationary v.z accumulates 0 → 0.148 m/s
    over 5 frames; pattern matches uncancelled residual gravity.
- See [`docs/insight/20260522_tc_msckf_port_journal.md`](docs/insight/20260522_tc_msckf_port_journal.md)
  §"P6 debug cycle 1" for the full evidence trail.

## [v0.2.0-msckf-port-p2] — 2026-05-22

### Added
- OpenVINS 포트 P1: `include/msckf/{types,utils,cam}/` 15 헤더 + `src/msckf/{types,utils}/` 2 cpp
- OpenVINS 포트 P2: `include/msckf/state/` 4 헤더 + `src/msckf/state/` 3 cpp (~1825 LOC)
- `docs/insight/20260522_tc_msckf_overview.md` — TC MSCKF 개념·수식 spine (~580 줄)
- `docs/insight/20260522_tc_msckf_port_plan.md` — P1~P6 실행 플랜 + 리스크 레지스터
- `docs/insight/20260522_tc_msckf_port_journal.md` — P1+P2 실행 저널
- `docs/Practice/tc_msckf_port_progress.md` — 진행 추적 (사고 변화 표, 누적 자산 인덱스)
- `docs/insight/20260520_vio_lvio_msckf_thinking_cheatsheet.{md,pdf,tex}` — VIO/LVIO/MSCKF 시각 치트시트
- Obsidian vault 구조: `docs/.obsidian/` config + `docs/OBSIDIAN.md` 가이드
- `practice/lidar_clean_room/main_test.cpp` — Stage A~H 마무리, KITTI 첫 프레임 OK
- `docs/Practice/stage_d_ransac.md` — RANSAC 수식↔코드 매핑

### Changed
- `CMakeLists.txt` — Boost::filesystem 의존 추가, msckf 포트 소스 5 cpp 등록, `ROS_AVAILABLE=0` 정의
- `libvio_core.a` 519 KB → 1.99 MB
- `docs/README.md` — vault 마스터 MOC 로 재구성, 빠른 진입 표 + Structure 표 갱신
- 기존 insight 노트 (260430~20260514, 8개) frontmatter / wiki-link 정규화
- `docs/Practice/session_progress.md` → `docs/Practice/LidarSegmenter session_progress.md` 로 rename (트랙별 분리)
- `PLAN/README.md` → `docs/PLAN/README.md` 로 이동 (Obsidian vault 안으로 흡수)

### Build status
- WSL gcc 빌드 통과, warnings 0, errors 0
- 런타임 검증 ❌ — P5 에서 수행 예정

### Repo migration note
- `scene-aware-vio` (origin) 가 GitHub 에서 archived 됨 (의도된 결정)
- 본 repo 의 새 active remote: `navisys-study-note` (origin 으로 rename)
- 기존 archived repo 는 `scene-aware-vio-archived` remote 로 history 참조용 유지

## [v0.1.0-lc-ekf] — 2026-04-30 (retroactive)

LC EKF VIO baseline 완성. TC MSCKF 와의 비교군으로 영구 보존.

### Added
- `src/io/{euroc_reader, kitti_raw_reader}` — EuRoC / KITTI raw 데이터 로더
- `src/ekf/{imu_propagator, lc_ekf}` — IMU 예측 + LC update (Hamilton quaternion)
- `src/frontend/stereo_tracker` — Stereo KLT
- `src/lidar/lidar_segmenter` — RANSAC floor/wall plane fit
- `src/depth/stereo_depth_segmenter` — Stereo depth segmentation
- `apps/run_vio.cpp` — EuRoC + KITTI 공통 진입점

### Measurements
- EuRoC ATE ~59 cm
- KITTI ATE ~153 cm (TC MSCKF 전환 결정의 직접 근거)

### Insights
- `docs/insight/260430_lc_ekf_vio_insights.md`
- `docs/insight/20260501_stereo_rectification_left_feature_state.md`
- `docs/insight/20260514_lc_ekf_cam_imu_fusion.md`

## 미래 마일스톤 (planted)

- **v0.3.0-msckf-update-p3** — Update + Feat 모듈 포트 완료, 단위 동작 확인
- **v0.4.0-msckf-runs-kitti-p5** — KITTI 0117 에서 첫 trajectory 출력
- **v1.0.0-msckf-verified-p6** — KITTI 0117 ATE < 153 cm (LC baseline) 확인
- **v1.1.0-plane-constraint** — TC + plane pseudo-measurement, degenerate 구간 개선 확인
- **v2.0.0-paper-draft** — 논문 초안 1개
