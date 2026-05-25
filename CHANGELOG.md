# Changelog

연구 마일스톤 기록. 각 버전은 git tag 와 1:1 대응. 형식은 semver 와 유사하나 *연구 단계* 기준 (검증된 baseline → 검증된 신규 contribution).

## [Unreleased] — P3 진행 예정

- TC MSCKF Stage P3: `update/{UpdaterMSCKF, UpdaterHelper, UpdaterOptions}` + `feat/{Feature, FeatureInitializer, FeatureInitializerOptions, FeatureHelper}` 포트
- Stage P4: msckf_pipeline 어댑터 (LC EKF 의 Hamilton ↔ JPL quaternion 변환)
- Stage P5: KITTI 0117 첫 빌드·실행·디버그
- Stage P6: LC 대비 ATE 비교

진행 추적: [`docs/Practice/tc_msckf_port_progress.md`](docs/Practice/tc_msckf_port_progress.md)

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
