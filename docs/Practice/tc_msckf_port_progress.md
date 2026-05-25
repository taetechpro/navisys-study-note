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

# TC MSCKF (OpenVINS) 포트 — 🚧 P3 완료 / 다음 P4 (2026-05-25)

> **시작일:** 2026-05-22 · **누적 작업:** 약 2.5 시간 (overview 작성 + P1 + P2 + P3)
> **상태:** P1 + P2 + P3 통과 (3/6 stage), `libvio_core.a` 컴파일 OK
> **다음:** `P4 시작` — msckf_pipeline 어댑터 (Hamilton↔JPL 변환, stereo_tracker→Feature)
> **전체 플랜:** [[../insight/20260522_tc_msckf_port_plan]] (P1~P6 마일스톤)

---

## 진행률 한눈에

```
[P1] types + utils + cam 포트 .............. ✅ 완료 (2026-05-22)
[P2] state + Propagator + StateHelper ...... ✅ 완료 (2026-05-22)
[P3] update + feat 포트 .................... ✅ 완료 (2026-05-25)
[P4] msckf_pipeline 어댑터 ................. ⬜ 대기
[P5] 첫 빌드·실행·디버그 ................... ⬜ 대기
[P6] KITTI 0117 검증 + LC 비교 ............. ⬜ 대기
```

진행률: **3/6 stage 완료 (50%)**.

빌드 상태: `libvio_core.a` **519 KB → 2.9 MB** (`+2.4 MB` 누적), warnings 0, errors 0.

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

## 다음 액션 (P4)

### 트리거 단어
**`P4 시작`** ← 사용자 발화하면 즉시 진행.

### P4 범위 — msckf_pipeline 어댑터
- `include/msckf_pipeline/msckf_pipeline.hpp` 신규 — OpenVINS VioManager 역할 대체
- `src/msckf_pipeline/msckf_pipeline.cpp` 신규
- **Quaternion 변환 어댑터** (Hamilton↔JPL) — R1 본격 처리
- **stereo_tracker → Feature vector** 변환 (R8) — feat/FeatureHelper 를 안 가져왔으므로 본 repo 의 track 출력을 직접 `std::vector<std::shared_ptr<Feature>>` 로 구성
- IMU 정지 초기화 → State 초기 공분산
- `apps/run_vio.cpp` 에 `--engine=msckf` 옵션 추가 (LC 와 공존)

### 예상 시간
포트 플랜은 8-12 시간 추정. **P1~P3 의 실제 속도 + R1+R8 가 본격화** → 1-3 시간 가능 (어댑터는 단순 패치보다 *설계* 가 필요).

### 발생 가능 리스크
- **R1 (JPL↔Hamilton)** — P4 의 핵심. 변환 함수의 부호·축 순서 실수가 무성성으로 누적.
- **R8 (Feature 필드 구성)** — Feature 의 `uvs[cam_id]`, `uvs_norm[cam_id]`, `timestamps[cam_id]`, `anchor_cam_id`, `anchor_clone_timestamp` 정확히 채우기. 잘못하면 triangulation 실패.
- **R6 (state size 폭주)** — clone 추가/제거 정책이 잘못되면 cov 무한 증가.

### P4 완료 시 예상 상태
- `libvio_core.a` ≈ **3.2-3.5 MB**
- `apps/run_vio.cpp` 가 LC 와 MSCKF 둘 다 진입 가능
- *실행*까지는 OK (`run_vio --engine=msckf` 가 첫 frame 처리 시도). 트래젝토리 정확도는 P5 에서.

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
