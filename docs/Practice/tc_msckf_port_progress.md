---
title: "TC MSCKF (OpenVINS) 포트 — 진행 상태"
date: 2026-05-22
type: practice
tags: [tc-msckf, port, openvins, cpp-learning, moc]
related:
  - "[[../insight/20260522_tc_msckf_overview]]"
  - "[[../insight/20260522_tc_msckf_port_plan]]"
  - "[[../insight/20260522_tc_msckf_port_journal]]"
  - "[[../insight/20260514_lc_ekf_cam_imu_fusion]]"
  - "[[LidarSegmenter session_progress]]"
status: in_progress
---

# TC MSCKF (OpenVINS) 포트 — 🚧 P5 통과, P6 정확도 디버깅 진행 중 (2026-05-25)

> **시작일:** 2026-05-22 · **누적 작업:** 약 5 시간 (overview + P1~P5)
> **상태:** P1+P2+P3+P4+P5 (5/6 stage). KITTI 0117 660 frame **crash-free**. LC 153 cm vs MSCKF 53279 cm (발산).
> **다음:** `P6 디버그 시작` — R1 quat 변환, chi² gate, FEJ, NoiseManager 추적.
> **전체 플랜:** [[../insight/20260522_tc_msckf_port_plan]] (P1~P6 마일스톤)

---

## 진행률 한눈에

```
[P1] types + utils + cam 포트 .............. ✅ 완료 (2026-05-22)
[P2] state + Propagator + StateHelper ...... ✅ 완료 (2026-05-22)
[P3] update + feat 포트 .................... ✅ 완료 (2026-05-25)
[P4] msckf_pipeline 어댑터 ................. ✅ 완료 (2026-05-25)
[P5] 첫 빌드·실행·디버그 ................... ✅ crash-free (2026-05-25)
[P6] KITTI 0117 검증 + LC 비교 ............. 🚧 발산 디버깅 단계
```

진행률: **5/6 stage 통과 (83%)**, P6 의 *crash-free 단계* 까지 도달. *ATE 개선* 단계 진행 중.

빌드 상태: `libvio_core.a` **519 KB → 3.0 MB** (누적 +2.5 MB), warnings 0, errors 0.

KITTI 0117 첫 결과:
- **LC EKF**: ATE **152.96 cm** (기존 메모리의 ~153 cm 와 정확히 일치, baseline 보존됨)
- **MSCKF**: ATE **53279 cm** (발산 — R1 본격 발현 + 어댑터 튜닝 필요)
- 660 frame 모두 crash 없이 완주. MSCKF update 매 frame 호출 (11~52 features/frame).

---

## 사고 변화 (Before / After)

이번 세션을 거치며 머릿속에 자리잡은 변화. 다음 stage 에서 *코드를 읽을 때* 의 기준선이 달라짐.

| 토픽 | 이전 (LC EKF 시점) | 현재 (P1+P2 후) |
|---|---|---|
| **State 형상** | 15-dim 고정 (`Eigen::Matrix<double,15,15> P`) | 가변 (`15 + 6N + 3M`), `_clones_IMU` 가 `map<double, PoseJPL>` |
| **Covariance 접근** | `imu_.P.block<3,3>(0,3)` 직접 슬라이싱 | `Type::id()` 기반 동적 슬라이싱, `StateHelper` 가 friend 로 보호 |
| **Quaternion** | Eigen Hamilton 만 (`Quaterniond`) | JPL `(x,y,z,w)` 컨벤션도 인지 — 충돌은 P4 에서 처리 예정 (R1) |
| **Feature 위치** | EKF 밖 (VO 가 흡수 후 pose 만 전달) | EKF 안 (triangulation) → **null-space projection 으로 제거** |
| **측정 모델** | 3D pose 직접 (`H ∈ ℝ³ˣ¹⁵`) | per-feature 픽셀 (`H_x`, `H_f` 분리, 2M×... ) |
| **Outlier 거름** | `innov.norm() > 5.0` (norm threshold) | chi² gate (Mahalanobis 거리, dof-aware) |
| **빌드 방식** | LC 단일 라이브러리 | `vio_core` 에 msckf 포트 모듈 통합 |
| **작업 모드** | "처음부터 짠다 (clean-room)" | "검증된 코드를 가져와 의존 정리한다 (port)" |
| **빌드 환경** | 의식 없이 사용 | **WSL `/mnt/d/...`** 표준으로 명시화 |

---

## P1 결과 — types + utils + cam (2026-05-22)

### 산출물 (17 파일)

| 위치 | 파일 수 | 내용 |
|---|---|---|
| `include/msckf/types/` | 7 hpp | Type, Vec, JPLQuat, PoseJPL, IMU, Landmark, LandmarkRepresentation |
| `include/msckf/utils/` | 5 hpp | quat_ops, print, colors, sensor_data, NoiseManager |
| `include/msckf/cam/` | 3 hpp | CamBase, CamRadtan, CamEqui |
| `src/msckf/types/` | 1 cpp | Landmark.cpp (representation 변환) |
| `src/msckf/utils/` | 1 cpp | print.cpp (Printer 본체) |

### 수치 변화
- `libvio_core.a`: (없음) → **519 KB**
- 컴파일 단위: +3 (Landmark, print, smoke_check)
- Warnings 0, Errors 0

### 적용한 패치 (요약)
1. `.h` → `.hpp` 일괄 변경 (본 repo 컨벤션)
2. Include 경로 sed 일괄 — `"Type.h"` → `"msckf/types/Type.hpp"` 형식
3. GCC 가변 매크로 (`PRINT_*(x...)`) → **표준 (`PRINT_*(...) ... __VA_ARGS__`)** — MSVC/clang 호환
4. CMakeLists 에 신규 cpp 추가

### 발견·교훈
- **R7 발견 (빌드 환경 path 컨벤션)**: PowerShell/Git Bash 에서 cmake 호출 시 기존 WSL build cache 와 path 충돌. 사용자의 표준 빌드 환경 = **WSL** (`/mnt/d/...`) 으로 명시화. 향후 모든 빌드는 `wsl -- bash -c "cd /mnt/d/... && cmake --build build ..."`.
- 17 파일 중 ROS 헤더 만지는 파일 **0개** → 포트가 예상보다 깔끔.
- p1_smoke_check.cpp 를 추가하여 모든 헤더 instantiation 검증 → P2 진입 시 삭제됨 (state cpp 가 자연스럽게 흡수).

---

## P2 결과 — state + Propagator + StateHelper (2026-05-22)

### 산출물 (8 파일)

| 위치 | 파일 | 줄 수 |
|---|---|---|
| `include/msckf/state/` | State.hpp | 196 |
| `include/msckf/state/` | StateOptions.hpp | 179 |
| `include/msckf/state/` | Propagator.hpp | 458 |
| `include/msckf/state/` | StateHelper.hpp | 242 |
| `include/msckf/utils/` | opencv_yaml_parse.hpp (StateOptions 의존) | (포트) |
| `src/msckf/state/` | State.cpp | 166 |
| `src/msckf/state/` | Propagator.cpp | 1015 |
| `src/msckf/state/` | StateHelper.cpp | 644 |

**소스 코드 추가량**: ~1825 줄 (cpp 만). 헤더까지 합쳐 ~3000줄.

### 수치 변화
- `libvio_core.a`: 519 KB → **1.99 MB** (`+1.5 MB`)
- 신규 심볼: `Propagator` / `StateHelper` / `augment_clone` 관련 **62개**
- 컴파일 단위: 12개 모두 통과 (full rebuild 후)
- Warnings 0, Errors 0

### 적용한 패치
1. Include 경로 sed 일괄 (P1 과 동일 패턴)
2. **opencv_yaml_parse 의 ROS 가드 활용** — 파일 본문 0 줄 수정, CMake 에 `target_compile_definitions(vio_core PUBLIC ROS_AVAILABLE=0)` 추가하여 `#if ROS_AVAILABLE == 1/2` 블록 무력화
3. CMake 의존 추가: `find_package(Boost REQUIRED COMPONENTS filesystem)` (opencv_yaml_parse 의 `boost::filesystem::exists()` 1회 사용)

### 발견·교훈
- **opencv_yaml_parse 의 ROS 가드 깔끔함** — `#if ROS_AVAILABLE` 가드가 *모든* ROS 사용을 깔끔히 둘러쌈. 매크로 한 줄로 ROS 의존 제거. 포트 플랜 §4.1 의 (옵션 a) 가 (옵션 b yaml-cpp 어댑터) 보다 더 깔끔한 것으로 판명.
- **첫 빌드 시도에서 통과** — 1825 줄을 패치 없이 컴파일. 본 포트가 *어쩌면 예상보다 훨씬 빠를 수 있음* 을 시사.
- **R1 (JPL ↔ Hamilton 충돌) 아직 미발생** — P2 의 state 코드와 LC EKF 코드가 *격리 컴파일* 되기 때문. P4 어댑터에서 처음 충돌 예정.

---

## P3 결과 — update + feat 포트 (2026-05-25)

### 산출물 (10 파일)

| 위치 | 파일 수 | 내용 |
|---|---|---|
| `include/msckf/update/` | 3 hpp | UpdaterMSCKF, UpdaterHelper, UpdaterOptions |
| `include/msckf/feat/` | 3 hpp | Feature, FeatureInitializer, FeatureInitializerOptions |
| `src/msckf/update/` | 2 cpp | UpdaterMSCKF.cpp, UpdaterHelper.cpp |
| `src/msckf/feat/` | 2 cpp | Feature.cpp, FeatureInitializer.cpp |

### 좁힌 범위 결정 (FeatureHelper 제외)

원 plan 의 P3 목록에는 `FeatureHelper.h` 가 포함되어 있었으나 포트 제외:
- UpdaterMSCKF 의 시그니처가 `update(state, std::vector<Feature>&)` — Feature vector 만 직접 받음
- FeatureHelper 는 본 repo 의 `frontend/stereo_tracker` 가 대체할 영역
- FeatureHelper 포트하면 `FeatureDatabase.{h,cpp}` 까지 의존성 끌려 들어옴 → P4 어댑터 영역으로 미룸

결과: P3 의 의존성 그래프가 *닫힘* — 모든 include 가 P1/P2/P3 안에서 해결.

### 수치 변화
- `libvio_core.a`: 1.99 MB → **2.9 MB** (`+0.9 MB`)
- 컴파일 단위 +4 (UpdaterMSCKF, UpdaterHelper, Feature, FeatureInitializer)
- 신규 export 심볼 14개 (UpdaterMSCKF::update, nullspace_project_inplace, measurement_compress_inplace 등)
- Warnings 0, Errors 0

### 적용한 패치
1. Include 경로 sed 일괄 (P1+P2 패턴에 P3 신규 패턴 32개 추가)
2. CMakeLists.txt: `find_package(Boost ... date_time)` 추가, `Boost::date_time` 링크, 4 cpp 등록
3. `boost::math` 은 header-only — component 등록 불필요
4. ROS 가드 사용처 0개 — 추가 어댑터 작업 없음

### 발견·교훈
- **첫 빌드 시도에서 통과** — 패치 없이 2.9 MB 라이브러리 완성. P1+P2 의 패턴이 P3 에 그대로 적용됨.
- **chi² 임계값 표 lazy load 패턴** — UpdaterMSCKF 생성자에서 i=1..500 까지 미리 `boost::math::quantile` 으로 채워둠. 런타임 update 시점에는 표 lookup 만.
- **R8 새로 발견 — Feature ↔ stereo_tracker 어댑터 필요** — FeatureHelper 를 제외했으므로 P4 에서 본 repo 의 stereo_tracker 출력을 `std::vector<std::shared_ptr<Feature>>` 로 변환하는 코드 작성 필요. Feature 의 (uvs, uvs_norm, timestamps, anchor_*) 필드 채우기.

---

## 누적 자산 인덱스 (이번 사이클로 생긴 모든 것)

### 코드 파일 (35개, 본 repo)

| 경로 | 종류 | P 단계 |
|---|---|---|
| `include/msckf/types/Type.hpp` ~ `Vec.hpp` (7) | 헤더 | P1 |
| `include/msckf/utils/quat_ops.hpp` 등 (5) | 헤더 | P1 |
| `include/msckf/cam/CamBase.hpp` 등 (3) | 헤더 | P1 |
| `src/msckf/types/Landmark.cpp` | 구현 | P1 |
| `src/msckf/utils/print.cpp` | 구현 | P1 |
| `include/msckf/state/State.hpp` ~ `StateHelper.hpp` (4) | 헤더 | P2 |
| `include/msckf/utils/opencv_yaml_parse.hpp` | 헤더 | P2 |
| `src/msckf/state/State.cpp` (166), `Propagator.cpp` (1015), `StateHelper.cpp` (644) | 구현 | P2 |
| `include/msckf/update/{UpdaterMSCKF,UpdaterHelper,UpdaterOptions}.hpp` (3) | 헤더 | P3 |
| `include/msckf/feat/{Feature,FeatureInitializer,FeatureInitializerOptions}.hpp` (3) | 헤더 | P3 |
| `src/msckf/update/{UpdaterMSCKF,UpdaterHelper}.cpp` (2) | 구현 | P3 |
| `src/msckf/feat/{Feature,FeatureInitializer}.cpp` (2) | 구현 | P3 |
| `CMakeLists.txt` (편집: +Boost::{filesystem,date_time}, +ROS_AVAILABLE=0, +10 cpp) | 빌드 | P1+P2+P3 |

### 인사이트·플랜 문서 (4개, 이번 세션 신규)

| 경로 | 역할 |
|---|---|
| [[../insight/20260522_tc_msckf_overview]] | 개념·수식 spine (~580줄) |
| [[../insight/20260522_tc_msckf_port_plan]] | P1~P6 파일 단위 실행 플랜 |
| [[../insight/20260522_tc_msckf_port_journal]] | P1, P2 실행 저널 (실시간 발견·결정) |
| [[tc_msckf_port_progress]] | **본 문서** — 성장 추적 |

### 메모리 항목 (4개, `~/.claude/projects/.../memory/`)

| 메모리 | 역할 |
|---|---|
| `project_tc_msckf_port.md` | 포트 작업 인덱스 (현재 메인 작업) |
| `project_tc_msckf_curriculum.md` | 학습 커리큘럼 (보류) |
| `feedback_ov_plane_is_benchmark_not_target.md` | ov_plane 프레이밍 정정 |
| `project_tc_msckf_pivot.md` (편집) | 2026-05-22 정정 반영 |

### MOC 갱신
- `docs/README.md` — 빠른 진입 표 + planted seeds 영역

---

## 다음 액션 (P6 정확도 디버깅 — cycle 2)

### Cycle 1 결과 (2026-05-26)

| 가설 | 결과 | 비고 |
|---|---|---|
| **G1: gravity_mag 부호 반전** | ❌ 악화 (ATE 53,279 → 1,332,060 cm) | gravity_mag = +9.81 자체는 옳음 |
| **G2: R1 quat round-trip 오류** | ✅ R1 OK (`\|R_state - R_wi^T\| = 1.13e-16`) | quat 변환 정확 |
| **G3: frame convention mismatch** | 🚧 의심 → cycle 2 에서 *반대 방향* 으로 드러남 | LC world z-up *맞음*, OpenVINS Global z 도 up |

### Cycle 2 결과 (2026-05-26, 같은 날)

| 시도 | 결과 | 발견 |
|---|---|---|
| **gram_schmidt 단독** (mean_accel 으로 R_GtoI 빌드) | 50f ATE 579 cm | OpenVINS Global z = up *진실 발견*. 단 yaw 180° flip — gram_schmidt 가 e_2 cross z 로 임의 yaw 선택 |
| **Hybrid** (LC R0 + OV bg/ba) | 50f 990 cm, **full 1,599,060 cm (악화)** | yaw align 회복 ✅, v.z residual 감소 (0.148→0.044) ✅, 그러나 ba.z=+0.26 (init window motion contamination) → ATE 30x 폭주 |

**핵심 발견**:
- ✅ OpenVINS Global z = up (cycle 1 추측 반대)
- ✅ gram_schmidt 함수 자체는 *yaw 자유* (4-DOF unobservable) — GT 비교 위해서는 LC R0 의 yaw 보존 필요
- ⚠️ KITTI 0117 의 *첫 1초 init window* 가 *진짜 정지 아님* — OV 식 ba 계산이 *motion contamination* 으로 오염

자세한 흐름: [[../insight/20260522_tc_msckf_port_journal#P6 debug cycle 2]]

### 누적된 부분 성과 (cycle 1 + 2)
- `include/msckf/init/InitializerHelper.hpp` 신규 — gram_schmidt 포트
- 어댑터 ctor 시그니처 = `(R0_wi, mean_accel, mean_gyro, gravity_mag, ...)` — *hybrid init* 옵션
- apps/run_vio.cpp 에 *mean_accel/mean_gyro 직접 계산* (LC init window 와 sync)

### 트리거 단어
**`P6 debug cycle 3`** — KITTI 의 *진짜 정지 구간* + update 효율.

### Cycle 3 시도 순서
1. KITTI 0117 IMU 첫 5초의 *accel variance* plot — 정지 구간 시각화.
2. window 확대 (1→3→5초) 시도.
3. 효과 없으면 *cycle 1 init revert* (ba=0) 후 update 단독 디버깅.
4. msckf_updates_ effective 횟수 + chi² gate dropping rate 측정.
5. NoiseManager sigma_pix 1→3 픽셀 튜닝 (KITTI ORB 가 더 noisy).

### 다음 액션 (백업)

P6 디버깅이 막힐 경우 (예: 2-3 시간 안에 ATE 가 LC 153cm 근처로 안 떨어지면):
- 옵션 A: stereo mode (cam_id=1 도 추가). multi-camera baseline 으로 triangulation 안정화.
- 옵션 B: OpenVINS TrackKLT 포트 — 본 repo stereo_tracker 대신 *검증된 frontend*. ~2000 줄 추가.
- 옵션 C: ov_msckf 의 *VioManager 의 KITTI config* 가 있는지 검색 — 만약 있다면 NoiseManager / StateOptions 의 *KITTI-specific tuning* 가져옴.

### P6 의 *crash-free* 단계는 완료. *정확도* 단계가 남음. 자세한 디버깅 순서는 [[../insight/20260522_tc_msckf_port_journal]] 의 P5 entry 참고.

---

## 메타 — 본인의 학습 습관 관찰 (이번 세션)

### 패턴 1: 큰 결정을 *같은 날 두 번* 내림
오늘 (2026-05-22) 안에:
1. LC → TC MSCKF 학습 시작 결정
2. 같은 날 "학습 7편 → 검증된 코드 포트" 로 전환

→ 의사결정 속도 ↑, 비효율 줄임. 단점: 사이드이펙트로 *방금 합의된 사항* 도 빨리 잊을 수 있음 → 본 문서 같은 *외부 기억 장치* 가 필수.

### 패턴 2: 추상 학습 → 즉시 코드 작업 으로 점프
lidar 클린룸 때는 *Stage A~H 차근차근* 패턴이었으나, 이번에는:
- overview spine 문서 1번 → 바로 P1 포트 실행 → 즉시 빌드
- "이해되는 만큼만 보고 나머지는 코드가 작동하는지로 검증"

→ 학습 ↔ 빌드 사이클의 *그립* 이 짧아짐. lidar 때 8 단계 10일 vs. 이번 P1+P2 1.5시간.

### 패턴 3: 메타 지식의 외부화 의식
WSL 빌드 환경 발견 같은, *세션 끝까지 묻혀 있을 수 있는 메타 지식* 을 즉시 저널 (`port_journal.md` R7 항목) 에 기록. 한 번만 다친 곳에서 두 번 다치지 않게 하는 방어 자세.

### 패턴 4: 추적의 미덕
본 문서 자체가 이 패턴의 산물. *"내가 뭐 했는지 감이 안 잡힘"* 의 명료한 자각 → 즉시 외부화 요청. 이 자각이 빨라질수록 작업 효율의 비손실량이 커짐.

---

## 어떻게 사용하나

- 새 세션 시작 시: §1 (진행률) + §6 (다음 액션) 만 1분 확인 → 바로 작업 재개.
- 막혔을 때: §3, §4 (각 P 결과) 의 *발견·교훈* 부분 다시 읽기 → 동일 실수 방지.
- 멘토·동료 보여주기: §2 (사고 변화) 단독으로 충분.
- 졸업논문 timeline 쓸 때: §3, §4 의 산출물 표를 재료로.

---

## 종료 시점 (P6 완료 시 본 문서 갱신)

P6 완료 시 본 문서가 다음 상태로 전환:
- `status: in_progress` → `status: completed`
- 헤더의 🚧 → 🎉
- 진행률 6/6 + 100%
- 새 섹션 **"최종 실행 검증"** 추가 (KITTI 0117 ATE 수치 + LC 153cm 와의 비교)

[[LidarSegmenter session_progress]] 가 종료 시 변한 형식이 모범 사례.
