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

# TC MSCKF (OpenVINS) 포트 — 🚧 P2 완료 / 다음 P3 (2026-05-22)

> **시작일:** 2026-05-22 (오늘) · **누적 작업:** 약 1.5 시간 (overview 작성 + P1 + P2)
> **상태:** P1 + P2 통과 (2/6 stage), `libvio_core.a` 컴파일 OK
> **다음:** `P3 시작` — update + feat 포트
> **전체 플랜:** [[../insight/20260522_tc_msckf_port_plan]] (P1~P6 마일스톤)

---

## 진행률 한눈에

```
[P1] types + utils + cam 포트 .............. ✅ 완료 (2026-05-22)
[P2] state + Propagator + StateHelper ...... ✅ 완료 (2026-05-22)
[P3] update + feat 포트 .................... ⬜ 대기
[P4] msckf_pipeline 어댑터 ................. ⬜ 대기
[P5] 첫 빌드·실행·디버그 ................... ⬜ 대기
[P6] KITTI 0117 검증 + LC 비교 ............. ⬜ 대기
```

진행률: **2/6 stage 완료 (33%)**.

빌드 상태: `libvio_core.a` **519 KB → 1.99 MB** (`+1.5 MB`), warnings 0, errors 0.

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

## 누적 자산 인덱스 (이번 사이클로 생긴 모든 것)

### 코드 파일 (25개, 본 repo)

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
| `CMakeLists.txt` (편집: +Boost::filesystem, +ROS_AVAILABLE=0, +6 cpp) | 빌드 | P1+P2 |

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

## 다음 액션 (P3)

### 트리거 단어
**`P3 시작`** ← 사용자 발화하면 즉시 진행.

### P3 범위
- `ov_msckf/src/update/{UpdaterMSCKF, UpdaterHelper, UpdaterOptions}` (3 헤더 + 2 cpp)
- `ov_core/src/feat/{Feature, FeatureInitializer, FeatureInitializerOptions, FeatureHelper}` (4 헤더 + 2 cpp)
- 7 파일 쌍, 총 ~1500 줄 추가 예상

### 예상 시간
포트 플랜은 6-10 시간 추정. **실제 P1+P2 의 속도 (15분/단계)** 를 감안하면 30분~1시간 가능.

### 발생 가능 리스크
- **R5 (Triangulation Gauss-Newton 수렴 실패)** — 본문 미사용이라 미발생.
- Ceres 의존 — Feature/Triangulation 코드에 Ceres 가 있는지 P3 시작 시 첫 확인 항목.
- chi² gate 에서 Boost::math 의존 — `find_package(Boost ... math)` 추가 여부 확인.

### P3 완료 시 예상 상태
- `libvio_core.a` ≈ **3 MB**
- 컴파일 단위 +5 (UpdaterMSCKF, UpdaterHelper, Feature, FeatureInitializer + 작은 cpp들)
- 모든 "vanilla MSCKF" 본체 컴파일 완료 — *동작* 은 아직 ❌ (P4 어댑터 필요)

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
