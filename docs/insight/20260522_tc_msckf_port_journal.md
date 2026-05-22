---
title: "OpenVINS → src/msckf/ 포트 진행 저널"
date: 2026-05-22
type: insight
tags: [tc-msckf, port, openvins, journal]
related:
  - "[[20260522_tc_msckf_port_plan]]"
  - "[[20260522_tc_msckf_overview]]"
status: active
---

# 포트 진행 저널

> 각 Stage 의 실제 실행 기록 — 결정·이슈·우회·관찰. 플랜은 [[20260522_tc_msckf_port_plan]] 참조.

---

## P1 — Types + Utils + Cam 포트 (2026-05-22)

### 산출물
- `include/msckf/types/` — Type, Vec, JPLQuat, PoseJPL, IMU, Landmark, LandmarkRepresentation (7 헤더)
- `include/msckf/utils/` — quat_ops, print, colors, sensor_data, NoiseManager (5 헤더)
- `include/msckf/cam/` — CamBase, CamRadtan, CamEqui (3 헤더)
- `src/msckf/types/Landmark.cpp` — Landmark 의 representation 변환 본체
- `src/msckf/utils/print.cpp` — Printer 클래스 본체
- `src/msckf/p1_smoke_check.cpp` — 모든 헤더 파싱 검증용 (P2 진입 시 삭제 예정)

총 **17 파일 (15 헤더 + 2 cpp + 1 smoke)** 포트.

### 적용한 패치

1. **확장자 일괄 변경**: `.h` → `.hpp` (본 repo 컨벤션).
2. **Include 경로 재작성** (sed 일괄):
   - `#include "Type.h"` → `#include "msckf/types/Type.hpp"` 형식.
   - `#include "utils/quat_ops.h"` → `#include "msckf/utils/quat_ops.hpp"` 형식.
   - `#include "CamBase.h"` → `#include "msckf/cam/CamBase.hpp"`.
   - `#include "print.h"` (print.cpp 의 same-dir) → 동일하게 `msckf/utils/` 풀패스.
3. **GCC 가변 매크로 → 표준 형식** (`include/msckf/utils/print.hpp`):
   - `#define PRINT_*(x...) ... , x` → `#define PRINT_*(...) ... , __VA_ARGS__`
   - 이유: MSVC/clang 호환성. 현재 빌드는 WSL gcc 이지만 추후 native Windows 빌드 가능성 대비.
4. **CMakeLists.txt** 의 `vio_core` 라이브러리에 신규 cpp 3개 추가.

### 변경 없음 (의도)

- ROS 의존 코드 ❌ — 본 P1 의 17 파일 중 ROS 헤더를 만지는 파일은 **0개**. `print.cpp` 의 `__ANDROID__` 가드만 있고 ROS 가드는 없음.
- Namespace 보존: `ov_type`, `ov_core`, `ov_msckf` 그대로 유지 → 향후 OpenVINS 와의 코드 변경 추적 용이.
- `print.cpp` 의 `extern "C" void __assert(...)` 보존: 잠재적 libc 충돌 우려가 있으나 WSL gcc 빌드에서는 무사 통과.

### 빌드 환경 발견

- **사용자의 표준 빌드 환경 = WSL Ubuntu** (`/mnt/d/...`). PowerShell / Git Bash (`/d/...` 또는 `D:\...`) 로 cmake 호출 시 기존 `build/CMakeCache.txt` 와 path 불일치 오류.
- 향후 모든 빌드 호출은 `wsl -- bash -c "cd /mnt/d/02_research/04_cpp_seg_msckf_vio && cmake --build build ..."` 형식으로.
- 표준 빌드 가이드 출처: `docs/guides/kitti_full_pipeline_guide.md:1040-1058`.

### 빌드 결과

```
[  9%] Building CXX object CMakeFiles/vio_core.dir/src/msckf/p1_smoke_check.cpp.o
[ 18%] Building CXX object CMakeFiles/vio_core.dir/src/msckf/types/Landmark.cpp.o
[ 27%] Building CXX object CMakeFiles/vio_core.dir/src/msckf/utils/print.cpp.o
[ 36%] Linking CXX static library libvio_core.a
[100%] Built target vio_core
```

`libvio_core.a` 519 KB (이전 + 신규 ~50KB). **Warnings 0, errors 0**.

### Smoke check 검증 범위

`p1_smoke_check.cpp` 가 다음을 모두 통과:
- 17개 헤더 파싱 ✅
- `ov_core::skew_x`, `exp_so3`, `quatnorm` 함수 instantiation ✅
- `ov_type::Vec(3)`, `JPLQuat`, `PoseJPL`, `IMU`, `Landmark(3)` 생성자 ✅
- `ov_msckf::NoiseManager` 기본 생성 ✅
- `ov_core::CamRadtan(640, 480)`, `CamEqui(640, 480)` 생성자 ✅

### P2 로 넘어가기 전 한 줄 메모

- P2 (Propagator + State) 가 시작되면 이 헤더들이 *실제로 사용* 되며 진짜 검증이 일어난다.
- `p1_smoke_check.cpp` 는 P2 작업 첫 단계에서 삭제 (Propagator.cpp 가 같은 헤더들을 더 깊게 사용하게 됨).

### 리스크 업데이트 (포트 플랜 §8 의 리스크 레지스터 대비)

- **R1 (JPL ↔ Hamilton 컨벤션)**: 아직 미관측. P2 의 Propagator 가 본 repo 의 LC EKF (Hamilton/`Eigen::Quaterniond`) 와 교차할 때 발생 예정.
- **R3 (Windows MSVC)**: 표면 미관측. 본 빌드는 WSL gcc. Native MSVC 빌드 시도 시 `__assert` 와 `__ANDROID__` 처리 재검토 필요.
- 새 리스크 **R7 (빌드 환경 path 컨벤션)**: 발견. 모든 후속 빌드는 WSL `wsl -- bash -c "..."` 로 호출. 본 저널 §빌드환경발견 에 기록.

### 다음 트리거

**`P2 시작`** — `state/{State, StateOptions, Propagator, StateHelper}` 포트.

예상 작업: 4 파일 (.h + .cpp 4쌍) + `ov_msckf::State`, `ov_msckf::Propagator`, `ov_msckf::StateHelper` 의 정의·구현. include 경로 패치, 본 repo `imu_propagator.cpp` 와의 *공존* 가능하도록 namespace 격리, `_clones_IMU` augmentation 단위 테스트 작성. 예상 ~4-6 시간.

---

## P2 — State + Propagator + StateHelper 포트 (2026-05-22)

### 산출물
- `include/msckf/state/` — State, StateOptions, Propagator, StateHelper (4 헤더)
- `src/msckf/state/` — State.cpp, Propagator.cpp, StateHelper.cpp (3 cpp, 총 1825 줄)
- `include/msckf/utils/opencv_yaml_parse.hpp` — StateOptions 의 YamlParser 의존성을 위해 동반 포트
- `src/msckf/p1_smoke_check.cpp` 삭제 (state cpp 들이 P1 헤더를 모두 사용하므로 불필요)

총 **신규 8 파일 + 1 삭제**.

### 적용한 패치

1. **Include 경로 재작성** — sed 일괄로 `#include "state/State.h"`, `#include "StateOptions.h"`, `#include "types/...h"`, `#include "utils/opencv_yaml_parse.h"`, `#include "cam/CamBase.h"` 등을 `msckf/` 풀패스로.
2. **opencv_yaml_parse 의 ROS 가드 활용** — 파일 자체는 그대로 가져오고, CMake 에 `target_compile_definitions(vio_core PUBLIC ROS_AVAILABLE=0)` 추가. `#if ROS_AVAILABLE == 1/2` 블록이 *모두* 비활성화되어 ROS 헤더 (`ros/ros.h`, `rclcpp/rclcpp.hpp`) 가 컴파일에서 빠짐. ROS 없이 cv::FileStorage + boost::filesystem 만 사용.
3. **CMake 의존 추가** — `find_package(Boost REQUIRED COMPONENTS filesystem)` + `target_link_libraries(... Boost::filesystem)`. opencv_yaml_parse 가 `boost::filesystem::exists()` 만 사용.

### 변경 없음 (의도)

- State.cpp, Propagator.cpp, StateHelper.cpp 본문 손대지 않음. 원본 그대로.
- Quaternion 컨벤션은 JPL 유지 (R1 충돌은 P4 어댑터에서 처리).
- FEJ, IMU intrinsics, online calibration 코드는 *전부 컴파일 됨* (기본 옵션 `do_fej=true, do_calib_*=false` 로 동작).

### 빌드 결과

```
[  7%] euroc_reader → 100% Linking libvio_core.a
모든 12 컴파일 단위 통과 (full rebuild)
신규: State.cpp, Propagator.cpp, StateHelper.cpp
libvio_core.a: 519 KB → 1.99 MB (+1.5 MB)
Warnings 0, Errors 0
```

`nm` 으로 확인: Propagator / StateHelper / augment_clone 관련 심볼 **62개** 라이브러리에 정상 포함.

### 검증 범위

- P2 신규 3 cpp 가 P1 의 모든 헤더를 transitively include 하므로 P1 smoke check 의 역할을 자연스럽게 흡수.
- 단 *동작 검증* 은 ❌. 다음 가능:
  - State 생성자: `IMU` + 모든 옵션 변수 등록 + 공분산 초기화 — 미실행.
  - Propagator::propagate_and_clone: IMU 적분 + clone augmentation — 미실행.
  - StateHelper::augment_clone, marginalize_old_clone, EKFUpdate — 미실행.
- P5 (KITTI 첫 실행) 에서 실제 호출 시 검증됨.

### 리스크 업데이트

- **R1 (JPL ↔ Hamilton 컨벤션)**: P2 에서는 미발생 (LC EKF 와 격리 컴파일). P4 어댑터에서 *반드시* 충돌 — Hamilton `Eigen::Quaterniond` 를 JPL `Eigen::Vector4d (x,y,z,w)` 로 변환하는 어댑터 함수 작성 필요.
- **R3 (Windows MSVC)**: 미관측. WSL gcc 빌드 통과.
- **R7 (빌드 환경 path)**: P1 에서 발견·기록. 본 P2 빌드도 WSL 통과.
- 새 발견: **opencv_yaml_parse 의 ROS_AVAILABLE 매크로** 가 *정확히* 의도한대로 동작 — 가드가 깔끔하게 #if 로 둘러쌓여 있어 별도 어댑터 불필요. 포트 플랜 §4.1 의 (옵션 a) 가 (옵션 b) 보다 더 깔끔한 것으로 판명.

### 다음 트리거

**`P3 시작`** — `update/{UpdaterMSCKF, UpdaterHelper, UpdaterOptions}` + `feat/{Feature, FeatureInitializer, FeatureInitializerOptions, FeatureHelper}` 포트.

예상 작업 (~6-10 시간):
- 7 파일 쌍 복사 + include 패치
- chi² gating 동작 확인 (Boost::math 의존성 확인 필요)
- Nullspace projection (Givens) 코드 그대로
- Triangulation 의 Ceres 의존 ❌ 확인 (있다면 어떻게 처리할지 결정)
- 모든 P2 코드와 함께 빌드 → libvio_core.a ~3 MB 예상
