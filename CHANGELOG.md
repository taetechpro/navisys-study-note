# Changelog

연구 마일스톤 기록. 각 버전은 git tag 와 1:1 대응. 형식은 semver 와 유사하나 *연구 단계* 기준 (검증된 baseline → 검증된 신규 contribution).

## [Unreleased] — Cycle 5 (stereo TC MSCKF, next)

Mono backend 의 한계 (KITTI/EuRoC stereo 데이터 + LC stereo baseline 과의
architectural mismatch) 확인. Cycle 5 에서 stereo MSCKF 로 재구축.

## [v0.3.2-cycle4-mono-final] — 2026-05-27

> 협업 AI 진단으로 mono backend 의 blocking divergence 해결. mono 라인의
> 마지막 마일스톤. cycle 5 stereo 진입 전 학습용 reference 로 영구 보존.

### Reframing
- Cycle 3 H1 (motion contamination 부정) 의 reasoning trap 확인:
  `a_var < 0.5` 은 *jerk 없음* 일 뿐 *stationary* 가 아님. KITTI 0117 GT 가
  첫 5초에 35.5 m 이동 (등속 ~7 m/s). [[feedback-a-var-is-not-stationarity]]
- → 진짜 원인은 *등속 운동 + v0=0 init 의 scale collapse*.

### Applied (5 fixes, 함께 작동해야 효과)
1. **H5 init covariance**: OV `StaticInitializer` 식으로 `_imu` 의 cov 명시
   (q/bg/ba 0.02², p 0.05², v 0.01²). `StateHelper::set_initial_covariance` 사용.
2. **Rectified cam0 extrinsic**: MSCKF 가 rectified 픽셀 받으므로
   `T_rectcam0_imu = R_rectcam0_cam0 * T_cam0_imu` 로 변환해 전달.
3. **Stereo v0 seed** ← 결정적. 첫 stereo frontend 변위 → IMU velocity 초기화.
4. **max_clone_size** 11 → 30 (`MSCKF_MAX_CLONES` env override).
5. **sigma_pix** 1.5 → 5.0 (`MSCKF_SIGMA_PIX` env). `MSCKF_CHI2_MULT`,
   `MSCKF_SIGMA_A` env 도 같이 노출.

### Evidence
| Variant | 50f ATE | Full 660f ATE |
|---|---:|---:|
| cycle 3 H3b (mono, ba=0) | 956 cm | 57,861 cm |
| + H5 cov | 961 cm | (kept out) |
| + stereo v0 seed | 94.5 cm | 9,137 cm |
| + max_clone=30 | — | 5,486 cm |
| + sigma_pix=5 | **67.9 cm** | **2,821.5 cm** |

### Success criteria
- Primary `< 500 cm` ✅ (full 2,821, 50f 68)
- Stretch `< 5,000 cm` ✅ (full 2,821)
- **50f LC parity** ✅ (MSCKF 68 < LC 103)
- Full LC parity `< 200 cm` ❌ (LC 152 vs MSCKF 2,821, ~18× gap)

### Known limit
남은 18× gap 의 성격: *mono backend* 가 stereo 데이터의 cam1 정보를 버림.
Static-init tuning 으로는 풀리지 않음 → cycle 5 stereo MSCKF.

### Architectural lesson
[[feedback-match-baseline-architecture]]: KITTI/EuRoC stereo 데이터 +
LC stereo baseline 인데 MSCKF 를 *암묵적* mono 로 결정한 게 sunk cost
사이클 1~4 의 근본 원인. 새 구현은 baseline 의 sensor 구성을 *명시적으로*
유지해야.

## [v0.3.1-cycle3-locked-out] — 2026-05-26

> 첫 번째 *방법론 적용* 마일스톤. [[feedback-problem-first-then-opensource]]
> 의 Step1 (정의) → Step2 (opensource 참고) → Step3 (가설별 검증) 을
> 각 단계 별도 commit 으로 분리. cycle 1+2 의 *즉시 적용* 패턴 탈피.

### Added
- `tools/probe_kitti_imu_variance.cpp` (140줄, standalone). OV 식 a_var 를
  sliding window 로 계산. OV gate ref 와 비교. 단독 binary.
- `msckf_pipeline.cpp` update accept/drop 카운터 (영구).

### Changed
- `msckf_pipeline.cpp`:
  - init `ba` cycle 2 OV-식 → `Vector3d::Zero()` (cycle 1 init revert).
    `ba_ov_would_be` 는 진단 print 보존.
  - `noises.sigma_a` 2.0e-3 → **5.886e-3** (KAIST yaml, OXTS RT3003 매칭).
  - `upd_opts_->sigma_pix` 1.0 → **1.5** (KAIST yaml).

### Found
- **H1 부정** — KITTI 0117 first 1s 의 70.8% sliding window 가 KAIST gate
  (0.5) 통과. *motion contamination 아님*.
- **H2 적중** — update accept_pct 8.4% (lock-out). chi² gate 가 state
  covariance 가 작아서 most measurements drop.
- **H3 부분 적중** — accept 8.4% → 33.3% (4×) 까지 회복. 50f ATE 940→956,
  full ATE 53k→57k cm. **chi² gate 조정만으로는 ATE 안 떨어짐**.
- 새 패턴: full run frame 0–250 cumul accept 70% → 250+ 재차 lock-out.
  *수렴 후* P 가 다시 작아짐 — H5 (init cov ba) 또는 H6 (marginalize 정보 수축) 후보.

### Methodology gain
- Step1 commit (`06d026b`): 문제 정의만, 코드 0줄
- Step2 commit (`64233b0`): OV source 체계 read, 코드 0줄
- Step3 commits (3개): 가설별 변경 + 측정
- → cycle 1/2 의 *시행착오 즉시 적용* (롤백 비용 큼) 패턴 탈피

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
