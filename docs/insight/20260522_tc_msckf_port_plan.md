---
title: "OpenVINS → src/msckf/ 포트 실행 플랜 (P1~P6)"
date: 2026-05-22
type: insight
tags: [tc-msckf, port, openvins, implementation-plan]
related:
  - "[[20260522_tc_msckf_overview]]"
  - "[[20260514_lc_ekf_cam_imu_fusion]]"
status: draft
---

# OpenVINS 핵심 → `src/msckf/` 포트 실행 플랜

> **2026-05-22** · 본 문서는 *검증된 구현* (OpenVINS) 의 MSCKF 핵심을 본 repo 의 `src/msckf/` + `include/msckf/` 로 옮기는 작업의 **파일 단위 실행 플랜**. `PLAN/README.md` (9주 전략 로드맵) 의 Phase 1 (W2~W4) 을 구체화한 문서.
>
> **타겟 결과물**: KITTI raw 2011_09_26_drive_0117 에서 동작하는 stereo-IMU TC MSCKF binary. LC EKF 의 ATE 153cm 와 비교 가능한 수준.

---

## 1. 포트 전략 한 줄

> **"OpenVINS 의 ROS-free 알고리즘 코어 ~30 파일만 가져오고, ROS·visualization·VioManager 는 본 repo 의 기존 KITTI loader + stereo_tracker + Rerun 으로 대체한다."**

→ 가져오는 것: state, propagator, updater, feature triangulation, types, cam models, quat utils.
→ 대체하는 것: VioManager, ROS publishers, opencv_yaml_parse, TrackKLT, ov_init.
→ 유지하는 것: 본 repo 의 KITTI/EuRoC reader, stereo_tracker, IMU 정지 초기화, Rerun 시각화, evaluation script.

---

## 2. 의존 그래프 요약 (탐색 결과 종합)

### 2.1 포트 대상 — *반드시 복사* (~30 파일, ~6000 줄)

**State & Propagation** (`src/msckf/state/` 로 배치):
- `ov_msckf/src/state/State.h`, `State.cpp`
- `ov_msckf/src/state/StateOptions.h`
- `ov_msckf/src/state/Propagator.h`, `Propagator.cpp`
- `ov_msckf/src/state/StateHelper.h`, `StateHelper.cpp`
- `ov_msckf/src/utils/NoiseManager.h`

**Update (MSCKF 본체)** (`src/msckf/update/`):
- `ov_msckf/src/update/UpdaterMSCKF.h`, `UpdaterMSCKF.cpp`
- `ov_msckf/src/update/UpdaterHelper.h`, `UpdaterHelper.cpp`
- `ov_msckf/src/update/UpdaterOptions.h`

**Feature** (`src/msckf/feat/`):
- `ov_core/src/feat/Feature.h`, `Feature.cpp`
- `ov_core/src/feat/FeatureInitializer.h`, `FeatureInitializer.cpp`
- `ov_core/src/feat/FeatureInitializerOptions.h`
- `ov_core/src/feat/FeatureHelper.h`

**Types** (`src/msckf/types/`):
- `ov_core/src/types/Type.h`
- `ov_core/src/types/IMU.h`
- `ov_core/src/types/Vec.h`
- `ov_core/src/types/PoseJPL.h`
- `ov_core/src/types/JPLQuat.h`
- `ov_core/src/types/Landmark.h`, `Landmark.cpp`
- `ov_core/src/types/LandmarkRepresentation.h`

**Camera** (`src/msckf/cam/`):
- `ov_core/src/cam/CamBase.h`
- `ov_core/src/cam/CamRadtan.h`
- `ov_core/src/cam/CamEqui.h`

**Utils** (`src/msckf/utils/`):
- `ov_core/src/utils/quat_ops.h`
- `ov_core/src/utils/print.h`, `print.cpp`
- `ov_core/src/utils/colors.h`
- `ov_core/src/utils/sensor_data.h`

### 2.2 *복사 안 함* — ROS 종속 또는 본 repo 가 이미 대체 보유

- `ov_msckf/src/core/` — VioManager, VioManagerHelper. **본 repo 의 `apps/run_vio.cpp` 가 대체**.
- `ov_msckf/src/ros/`, `ros1_*`, `run_simulation.cpp`, `run_subscribe_msckf.cpp` — ROS 전용.
- `ov_init/` — Ceres 기반 정밀 초기화. **본 repo 의 정지 IMU 초기화 (`apps/run_vio.cpp:280-335`) 가 대체**.
- `ov_core/src/track/TrackKLT.*`, `TrackBase.*`, `TrackAruco.*` — **본 repo 의 `frontend/stereo_tracker` 가 대체**.
- `ov_core/src/utils/opencv_yaml_parse.h` — ROS 조건부. **본 repo 가 이미 yaml-cpp 직접 사용** → 별도 thin loader 작성.
- 모든 visualization (`docs/img/`, ROS visualizers) — Rerun 사용.

### 2.3 외부 라이브러리 의존

본 repo 에 이미 있음 / 추가 없음:
- Eigen 3.3 ✅
- OpenCV 4 ✅
- yaml-cpp ✅
- Boost — **추가 필요** (date_time, math, filesystem). OpenVINS 포트 파일이 `boost::posix_time` (타이밍), `boost::math::chi_squared` (chi² gate) 사용.
- Ceres — **불필요**. OpenVINS 의 Ceres 의존은 `ov_init/` 만이고 우리는 정지 IMU 초기화로 대체.

---

## 3. 디렉토리 매핑

```
src/msckf/                 ← 새로 채움
├── state/
│   ├── State.{hpp,cpp}
│   ├── StateOptions.hpp
│   ├── Propagator.{hpp,cpp}
│   └── StateHelper.{hpp,cpp}
├── update/
│   ├── UpdaterMSCKF.{hpp,cpp}
│   ├── UpdaterHelper.{hpp,cpp}
│   └── UpdaterOptions.hpp
├── feat/
│   ├── Feature.{hpp,cpp}
│   ├── FeatureInitializer.{hpp,cpp}
│   └── FeatureInitializerOptions.hpp
├── types/
│   ├── Type.hpp
│   ├── IMU.hpp
│   ├── Vec.hpp
│   ├── PoseJPL.hpp
│   ├── JPLQuat.hpp
│   ├── Landmark.{hpp,cpp}
│   └── LandmarkRepresentation.hpp
├── cam/
│   ├── CamBase.hpp
│   ├── CamRadtan.hpp
│   └── CamEqui.hpp
├── utils/
│   ├── NoiseManager.hpp
│   ├── quat_ops.hpp
│   ├── sensor_data.hpp
│   └── print.{hpp,cpp}
└── msckf_pipeline.{hpp,cpp}    ← 본 repo 신규 작성 (apps/run_vio 와 OpenVINS State 사이 어댑터)

include/msckf/             ← (선택) public header 만 별도. 또는 위 .hpp 를 직접 include.
```

**파일명 컨벤션**: OpenVINS 는 `.h` 사용, 본 repo 는 `.hpp` 사용. 포트 시 `.h` → `.hpp` 일괄 변경 + include 경로 보정.

---

## 4. ROS 의존 제거 전략

OpenVINS 의 ROS 흔적은 *알고리즘 코어에서는 매우 얇음*. 다음 셋만 처리:

### 4.1 `opencv_yaml_parse.h`

원본은 `#if ROS_AVAILABLE` 가드 안에 ROS 파라미터 서버 로딩 코드. 포트 시 둘 중:

- (옵션 a) 파일을 가져와 **`-DROS_AVAILABLE=0`** 매크로로 빌드 → ROS 부분 사라짐. 그러나 `cv::FileStorage` 기반 fallback 이 동작해야 함.
- (옵션 b) 파일을 *복사 안 하고* 본 repo 의 `src/io/` 에 thin loader 만 작성. UpdaterOptions, StateOptions 등을 yaml-cpp 로 직접 채움.

→ **권장 (b)**: 본 repo 가 이미 yaml-cpp 사용 중. include 한 줄 줄이는 게 깔끔.

### 4.2 `print.h/cpp` 의 ROS 매크로

`PRINT_DEBUG`, `PRINT_INFO`, `PRINT_WARNING`, `PRINT_ALL` 매크로 — 원본은 ROS_INFO 와 stdout 양쪽 분기. **stdout-only 로 단순화** (`fprintf(stderr, ...)`).

### 4.3 `boost::posix_time` 타이밍

ROS 무관. 그대로 사용. `<boost/date_time/posix_time/posix_time.hpp>` 헤더 추가하면 끝.

---

## 5. Frontend 통합 계약

OpenVINS `UpdaterMSCKF::update()` 가 요구하는 `Feature` 의 필드:

| 필드 | 타입 | 의미 |
|---|---|---|
| `featid` | `size_t` | feature ID |
| `timestamps[cam_id]` | `std::unordered_map<size_t, std::vector<double>>` | 측정 시점들 |
| `uvs[cam_id]` | `std::unordered_map<size_t, std::vector<Eigen::VectorXf>>` | raw 픽셀 좌표 |
| `uvs_norm[cam_id]` | `std::unordered_map<size_t, std::vector<Eigen::VectorXf>>` | 정규화 좌표 |
| `anchor_cam_id` | `int` | (관련된 경우만) |
| `anchor_clone_timestamp` | `double` | (관련된 경우만) |
| `p_FinA, p_FinG` | `Eigen::Vector3d` | triangulation 결과 |
| `to_delete` | `bool` | 처리 후 삭제 표시 |
| `clean_old_measurements(times)` 메소드 | — | clone 정렬 |

→ **본 repo 의 `stereo_tracker` 가 매 프레임 left+right 픽셀 매칭을 산출**. 어댑터 `msckf_pipeline.cpp` 가 그 결과를 `Feature` 객체에 누적 (track id 기반).

→ **정규화 좌표 `uvs_norm`**: stereo_tracker 의 raw 픽셀에 `cam::CamRadtan::undistort_d()` 적용. 이 어댑터 한 함수가 frontend ↔ MSCKF 사이의 *모든 책임*.

---

## 6. CMake 통합

기존 `CMakeLists.txt` 에 다음 추가:

```cmake
# ---- new: Boost (msckf core 가 요구) ----
find_package(Boost REQUIRED COMPONENTS date_time)

# ---- vio_core 에 msckf 소스 추가 ----
add_library(vio_core
    # ... (기존 항목 유지) ...
    src/msckf/state/State.cpp
    src/msckf/state/Propagator.cpp
    src/msckf/state/StateHelper.cpp
    src/msckf/update/UpdaterMSCKF.cpp
    src/msckf/update/UpdaterHelper.cpp
    src/msckf/feat/Feature.cpp
    src/msckf/feat/FeatureInitializer.cpp
    src/msckf/types/Landmark.cpp
    src/msckf/utils/print.cpp
    src/msckf/msckf_pipeline.cpp        # 신규 어댑터
)
target_link_libraries(vio_core PUBLIC
    # ... 기존 ...
    Boost::date_time
)
target_compile_definitions(vio_core PRIVATE
    ROS_AVAILABLE=0
)
```

`include_directories(include)` 가 이미 있으므로 `#include "msckf/..."` 경로 동작 — 단 OpenVINS 의 원본 include 경로 (`#include "state/State.h"`) 를 `#include "msckf/state/State.hpp"` 로 보정 필요. **이 보정은 단순 sed 작업이 아님** (헤더 가드, 상대경로 등) — 파일별로 직접 수정.

---

## 7. 실행 단계 — P1~P6 마일스톤

### Stage P1 — Types + Utils 포트 (1세션, ~3-4시간)

**목표**: 가장 의존 ↓ 인 정의들부터. 빌드 가능한 *작은 정적 라이브러리* 한 덩이.

**포트 대상**:
- `types/{Type, IMU, Vec, PoseJPL, JPLQuat, Landmark, LandmarkRepresentation}`
- `utils/{quat_ops, sensor_data, print, NoiseManager}`
- `cam/{CamBase, CamRadtan, CamEqui}`

**작업**:
1. 파일 복사 + `.h` → `.hpp` 일괄.
2. include 경로 보정 (`"state/..."` → `"msckf/state/..."`).
3. `print.cpp` 의 ROS 분기 제거 (stdout 직접 출력).
4. CMake 에 위 6 cpp 추가 + Boost::date_time 링크.

**완료 조건**: `cmake --build build --target vio_core` 성공. 사용처는 아직 없음.

**리스크**: JPL quaternion 컨벤션 ≠ Eigen Hamilton 컨벤션 — 본 repo 의 LC EKF 가 어느 쪽인지 [[20260514_lc_ekf_cam_imu_fusion]] §1 확인 (Eigen Hamilton). LC 와 TC 의 quaternion convention 충돌 가능성 → 어댑터에서 변환 필요.

---

### Stage P2 — State + Propagator 포트 (1세션, ~4-6시간)

**목표**: 시간 적분 + clone augmentation 까지.

**포트 대상**:
- `state/{State, StateOptions, Propagator, StateHelper}`

**작업**:
1. 파일 복사 + include 보정.
2. `Propagator::feed_imu()` 의 ROS 측 IMU 메시지 변환 제거 — 본 repo 의 `ImuData` struct 가 직접 들어오도록.
3. `StateHelper::set_initial_covariance` 등의 옵션 로딩 부분 — yaml-cpp 직접 채움.
4. `state/StateHelper.cpp:579` `augment_clone()` 검증 — 이게 [[20260522_tc_msckf_overview]] §4 의 *복사 + cross-term* 구현.

**완료 조건**: 단위 테스트 (별도 `tests/test_msckf_propagator.cpp`):
- 정지 IMU 100 샘플 propagate → state 변화 < 0.01m, P trace 증가 확인.
- Augment clone 후 P 의 차원이 +6, 새 clone 블록이 IMU pose 블록과 동일.

**리스크**: `IMU::set_value()` 가 *quaternion 표현* 을 받는 형태. 본 repo 의 `imu_propagator` 의 R (rotation matrix) → JPL quaternion 변환 어댑터 필요.

---

### Stage P3 — Updater + Feature 포트 (1~2세션, ~6-10시간)

**목표**: MSCKF update 동작.

**포트 대상**:
- `update/{UpdaterMSCKF, UpdaterHelper, UpdaterOptions}`
- `feat/{Feature, FeatureInitializer, FeatureInitializerOptions, FeatureHelper}`

**작업**:
1. 파일 복사 + include 보정.
2. `UpdaterMSCKF.cpp:42` 의 chi² 테이블 초기화 — Boost::math::chi_squared 가 OK.
3. `FeatureInitializer` 의 triangulation — Ceres 의존 ❌ 확인 (있다면 Gauss-Newton 자체 구현으로 대체, 본 repo 의 `stereo_depth_segmenter` 에 이미 유사 코드 있음).
4. Nullspace projection (`UpdaterHelper.cpp:426`) — Givens rotation 그대로 가져옴.

**완료 조건**: 단위 테스트 (`tests/test_msckf_updater.cpp`):
- 10 clone + 1 feature, 5 관측 의 toy data 로 `update()` 호출 → 위치 보정량 < 노이즈, P trace 감소.
- chi² gate 가 큰 outlier 잘 거름.

**리스크**: `Feature` 의 `uvs_norm` 채우는 책임이 *원본에서는 TrackKLT* 였음. 우리는 어댑터에서 채워야 함 → P4 에서 처리.

---

### Stage P4 — Frontend 어댑터 (1세션, ~4-5시간)

**목표**: 본 repo 의 `stereo_tracker` 가 산출한 left/right matches 를 `Feature` 객체로 변환하는 어댑터 작성.

**신규 파일**:
- `src/msckf/msckf_pipeline.{hpp,cpp}` — 본 repo 의 `apps/run_vio.cpp` 와 OpenVINS State/Updater 사이의 *유일한* 접점.

**역할**:
1. `stereo_tracker::process(left, right)` → matched feature pairs.
2. track id 별로 누적 → `ov_core::Feature` 객체에 timestamps/uvs/uvs_norm 채움.
3. 필요한 frame 마다 `UpdaterMSCKF::update(state, features)` 호출.
4. 결과 (clone 들의 trajectory) 를 본 repo 의 형식으로 dump.

**완료 조건**: KITTI 0117 의 첫 10 프레임 처리 무사고. State 가 발산하지 않음.

---

### Stage P5 — 첫 빌드·실행·디버그 (1~3세션, 가변)

**목표**: KITTI 0117 시퀀스 전체 동작.

**작업**:
1. `apps/run_vio.cpp` 옆에 `apps/run_msckf.cpp` 신설 — KITTI loader + msckf_pipeline 호출.
2. CMake 에 `add_executable(run_msckf ...)` 추가.
3. 첫 빌드 — 헤더 경로·심볼·linker 오류 정리.
4. 첫 실행 — segfault, NaN, 무한 발산 등 디버그.
5. Rerun 으로 trajectory 실시간 확인.

**완료 조건**:
- `run_msckf data/kitti_raw/2011_09_26_drive_0117 results/msckf_0117_v1/` 가 segfault 없이 시퀀스 끝까지 진행.
- 출력 trajectory 가 *대략* GT 와 같은 모양 (정밀도는 미정).

**리스크 (가장 큼)**: 좌표계 컨벤션 불일치 (KITTI 의 camera frame, IMU frame, world frame 정의 vs. OpenVINS 의 정의). LC EKF 가 이미 다룬 부분이라 그 코드의 변환 행렬 그대로 재사용.

---

### Stage P6 — KITTI 0117 검증 + LC 비교 (1세션)

**목표**: 정량 평가.

**작업**:
1. `evo_ape` 또는 본 repo 의 ATE 계산기로 0117 결과 평가.
2. 이전 LC ATE (153cm) 와 비교.
3. 결과 정리: `docs/insight/20260523_tc_msckf_first_kitti_run.md`.

**완료 조건**:
- TC ATE < 153cm (LC 보다 낫다는 첫 증거) — 이상적.
- 적어도 *완주* + *어떤 수치든 산출* 가능 — 최소 목표.

---

## 8. 리스크 레지스터

| # | 리스크 | 가능성 | 영향 | 완화 |
|---|---|---|---|---|
| R1 | JPL ↔ Hamilton quaternion convention 충돌 | 높음 | 중간 (signs flipped) | P2 에서 어댑터 함수 명시적 작성 + 단위 테스트 |
| R2 | KITTI 의 camera-IMU extrinsic 컨벤션 ≠ OpenVINS 기대 | 높음 | 큼 (trajectory 폭주) | LC EKF 의 변환 그대로 재사용 |
| R3 | Windows MSVC vs OpenVINS Linux 가정 코드 (boost 헤더, 매크로) | 중간 | 중간 (빌드 실패) | P1 에서 컴파일 통과를 first goal |
| R4 | OpenVINS 의 chi² 임계값이 KITTI scale 에 부적합 | 중간 | 작음 (튜닝) | P5 에서 multiplier 조정 |
| R5 | Triangulation Gauss-Newton 의 수렴 실패 — KITTI outdoor 깊은 feature | 중간 | 중간 | OpenVINS 의 `single_triangulation_1d` 폴백 사용 |
| R6 | Feature track lifecycle ≠ OpenVINS 의 sliding window 가정 — feature 가 너무 짧게 살아감 | 낮음 | 중간 | stereo_tracker 의 track id 유지 정책 검증 |

---

## 9. 검증 게이트 (단계별)

각 Stage 통과 조건:

- **P1 통과**: `vio_core` 라이브러리에 types+utils+cam 컴파일 + 링크 ✅, warning 0 또는 의도된 것만.
- **P2 통과**: 정지 IMU propagation 단위 테스트 통과, augment clone 단위 테스트 통과.
- **P3 통과**: Toy data MSCKF update 단위 테스트 통과, chi² gate 작동 확인.
- **P4 통과**: KITTI 첫 10 프레임 처리 무사고, Feature 객체에 uvs_norm 정상.
- **P5 통과**: KITTI 0117 시퀀스 끝까지 segfault 없이 완주.
- **P6 통과**: ATE 수치 산출 + LC 와 비교 표 작성.

---

## 10. 본 플랜에 *없는* 것 (의도적)

- **ov_plane 의 plane updater**: 본 플랜 종료 후 별도 사이클. 본인 연구 차별점 ([[tc-msckf-pivot]]) 이지만, vanilla TC MSCKF 가 먼저 동작해야 의미 있음.
- **FEJ (First-Estimate Jacobians)**: OpenVINS 옵션이지만 `do_fej=false` 로 시작. 일관성 trick 은 동작 확인 후.
- **SLAM features**: `_features_SLAM` 비워두고 pure MSCKF feature 만. `do_slam=false`.
- **Online camera/IMU calibration**: `do_calib_*=false`. 고정 extrinsic.
- **Multi-camera fusion (>2)**: stereo 2 카메라만.
- **ZUPT (zero velocity update)**: 별도 모듈, 본 플랜 외.

이들은 *동작하는 v1* 이후의 *확장* 으로 분리. 

---

## 11. 학습 자산과의 관계

본 포트 작업과 *동시에* [[20260522_tc_msckf_overview]] 가 reference 로 사용됨:

- P2 (Propagator + State) 작업 시 → overview §3, §4 다시 읽기.
- P3 (Updater) 작업 시 → overview §5, §6, §7 다시 읽기.
- P5 (실행) 시 → overview §9 (전체 사이클) 다시 읽기.

기존 학습 플랜 (`lc-enchanted-pillow.md` 의 7-편 deep dive) 은 **보류**. 포트 도중 코드를 직접 읽기 때문에 별도 insight 문서 작성은 *포트 후* 정리 단계에서 부산물로.

---

## 12. 다음 액션

본 플랜 확정 후 즉시:

1. **Stage P1 시작 신호 대기**. 사용자가 "P1 시작" 발화 → 30분 안에:
   - OpenVINS 의 types/utils/cam 파일 복사
   - include 경로 보정
   - CMake 갱신
   - vio_core 빌드 성공 확인

2. P1~P6 진행 중 발견되는 모든 위험·결정은 `docs/insight/20260522_tc_msckf_port_journal.md` (저널 형식) 에 누적.

3. P6 완료 시 별도 결과 doc + PLAN/README 갱신.

---

> **본 플랜의 정신**: *작동하는 검증된 코드* 가 *완벽한 자기 구현* 보다 빠르고 안전하다. 학습은 포트 과정에서 부산물로 발생. 막히면 OpenVINS 원본을 보고 정답을 알 수 있다는 점이 가장 큰 안전망.
