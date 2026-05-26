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

---

## P4 + P5 — 어댑터 + 첫 실행 (2026-05-25)

### 산출물
- `include/frontend/stereo_tracker.hpp/.cpp` — `prev_track_ids_` + `next_id_` 추가, KLT/PnP/ORB 모든 분기에서 ID 동기. LC 의 pose 출력은 변경 없음.
- `include/msckf_pipeline/msckf_pipeline.hpp` (97 줄) + `src/msckf_pipeline/msckf_pipeline.cpp` (~210 줄) — OpenVINS VioManager 역할 대체.
- `apps/run_vio.cpp` — `--engine=lc|msckf` CLI, 두 engine 병렬 wiring.

### 좁힌 frontend 결정 (3-옵션 분석)
사용자 선택 (A): StereoTracker 확장. (B) OpenVINS TrackKLT 추가 포트 (~2000 줄) 은 별도 stage. (C) single-view feature 는 MSCKF 본질 상실. (A) 가 LC 영향 최소 + MSCKF/LC 가 같은 frontend 공유.

### 어댑터 핵심 변환
- **Hamilton ↔ JPL** (R1 본격): `IMU::set_value` 는 16-dim `[q(4), p(3), v(3), bg(3), ba(3)]`. JPL q 는 *I_R_G* (IMU ← Global) 표현. 따라서 `q = rot_2_quat(R0_wi.transpose())`.
- **Camera extrinsic**: `_calib_IMUtoCAM[0]->set_value([q_CtoI, p_IinC])`. T_cam0_imu (cam ← imu) 의 R/t 를 그대로 7-dim.
- **Camera intrinsic**: rectified 픽셀 사용. CamRadtan with `[fx, fy, cx, cy, 0, 0, 0, 0]`.
- **monocular MSCKF**: cam_id=0 만 사용. uvs_norm = `((u-cx)/fx, (v-cy)/fy)`.

### P5 — 첫 실행 디버그 (3 boundary fix)

KITTI 0117 첫 실행 시 *두 번째 frame 에서 abort*. 진단 출력 추가하며 좁힌 결과 3개 문제:

1. **첫 frame 의 dt=0 propagate**: Propagator::propagate_and_clone 가 dt<=0 면 `std::exit(EXIT_FAILURE)`. 어댑터 생성자가 state_->_timestamp = t0 설정 → feed_camera(t0) 에서 같은 시점 호출. **해결**: first_camera 분기에서 propagate_and_clone 우회, `StateHelper::augment_clone(state, 0)` 직접 호출.
2. **IMU buffer 의 boundary sample 부재**: main loop 의 incremental IMU feed (`imu.t <= cam.t` 까지) 가 *time1 이후 boundary IMU 없음* → assert. **해결**: `--engine=msckf` 일 때 *전체 IMU 스트림* 을 시작 시 한 번에 feed.
3. **t0 이전 IMU 도 필요**: KITTI IMU rate (~6Hz) < cam rate (~10Hz). select_imu_readings 가 time0 interp 위해 *time0 이전 IMU 1개* 필요. **해결**: pre-t0 IMU 도 buffer 에 feed (skip 안 함).

### 빌드 + 실행 결과

- `libvio_core.a` 3.0 MB (+0.1 MB from P3), warnings 0, errors 0.
- KITTI 0117 660 frame **crash-free**. MSCKF update 매 frame 호출됨 (to_update 11~52 features).
- **LC ATE 153 cm** (기존 baseline 일치). **MSCKF ATE 53279 cm** — **발산**.

### 정확도 미달의 원인 후보 (다음 디버깅 사이클)

frame 600 시점: LC pos = (181, -108, 15) m, MSCKF pos = (1469, -599, -211) m. 약 8x 발산.

가능 원인:
- **R1 quat 변환** (가장 큰 의심) — 부호 오류면 propagate 시 gravity 가 *반대 방향* 으로 더해짐 → 지수 발산
- **Camera extrinsic 부호** — _calib_IMUtoCAM 의 q/t 컨벤션 잘못 시 update 가 wrong correction
- **chi² gate** — `chi2_multipler=5` 가 KITTI 에 안 맞을 가능성. Effective update 횟수 측정 필요
- **NoiseManager default** — 검사 결과 KITTI config 와 동일. 의심 낮음
- **FEJ 영향** — `do_fej=true` 가 KITTI 의 큰 initial uncertainty 에 안 맞을 가능성

### 다음 트리거

**`P6 디버그 시작`** — 정확도 추적. 권장 순서:
1. R1 검증: 첫 frame 의 state_->_imu->Rot() 가 R0_wi^T 와 같은지 print 비교.
2. msckf_updates_ 횟수 vs frame 수 비교 — chi² gate 가 dropping rate.
3. 첫 10 frame 의 MSCKF pos vs LC pos plot — 발산 시작 시점 확인.
4. FEJ 끄고 (`do_fej=false`) 재실행. 5. NoiseManager sigma_a/sigma_w 를 KITTI 에 더 맞게 (KITTI 가 noisier).

---

## P6 debug cycle 1 — R1 quat OK, frame convention 의심 (2026-05-26)

**가설 1 (rejected): gravity_mag 부호 반전**
- 의심: OpenVINS Propagator `_gravity << 0,0,gravity_mag` (line 57) 가 (0,0,+9.81) z-down, LC 는 (0,0,-9.81) z-up. 부호 반대.
- 시도: `init.g_world.z()` (= -9.81) 그대로 어댑터에 전달.
- 결과: **악화**. 660-frame ATE 53279 cm → 1,332,060 cm (13320m). frame 600 z = +36766 m (z-up 폭주, 부호 양쪽 다 발산).
- 결론: gravity_mag = +9.81 이 *옳음*. _gravity = (0,0,+9.81) 자체는 OpenVINS convention 에서 정상.

**가설 2 (R1 quat 변환 자체): 검증 결과 OK**
- 진단 코드: 어댑터 생성자에 `|R_state_IG - R_wi^T|` print + 첫 5 frame state 출력.
- 결과:
  ```
  |R_state_IG - R_wi^T| = 1.13e-16    ← quat round-trip 완벽
  f0  p=(0,0,0)        v=(0,0,0)             updates=0
  f1  p=(2e-4,4e-3,1e-4)  v=(0.005, 0.067, 0.005)   updates=0
  f2  p=(1e-3, 1e-2, 1e-3) v=(0.008, 0.108, 0.034)  updates=1
  f3  p=...               v=(0.010, 0.121, 0.090)   updates=2
  f4  p=...               v=(0.011, 0.143, 0.148)   updates=3
  ```
- 핵심 패턴: **v.z 가 매 frame 기하급수적 증가** — 0 → 0.005 → 0.034 → 0.090 → 0.148. gravity 보정 잔차가 *누적 적분*.
- 결론: R1 변환 OK. quat 자체 부호 오류 아님.

**가설 3 (frame convention mismatch, 다음 사이클 의심)**
- 정지 시 body accel = (0.91, 0.37, +10.02). z = +10 이라는 건 *body z 가 gravity 의 반대* = body z up.
- OpenVINS Propagator 의 `R_Gtoi.T * a_local - _gravity * dt` 가 정상 작동하려면 *Global frame z = body z* 가 *gravity 와 같은 방향* (down) 이어야 cancel.
- 우리 R0_wi (LC world z up) 를 R_IG 로 박았는데, OpenVINS 는 Global z down 가정 → 정지 시 a.z 가 *cancel 안 됨*, residual 약 +0.2 m/s² 가 *매 frame 누적*.
- 가능한 fix:
  - (3a) body↔world 변환 행렬 R_z = diag(1,1,-1) 을 R0_wi 에 곱해서 *OpenVINS convention* 으로 변환.
  - (3b) 어댑터 안에서 *Propagator 의 _gravity 를 (0,0,-9.81) 로 set* — protected 접근 → friend 또는 가상 ctor.
  - (3c) `gravity_mag` 만 받지 말고 vector 형태 ctor 추가 (OpenVINS source 패치).

### 다음 트리거

**`P6 debug cycle 2`** — 가설 3 검증 + fix:
1. 가설 3a 가 가장 비침습. R0_wi 에 z-flip 행렬 곱한 후 정지 init 시 *어떤 R0* 가 나오는지 검증.
2. 그래도 발산하면 (3b) friend 접근 추가.
3. 첫 5 frame v.z 가 *0 근처* 로 수렴하는지 확인 — 그게 *frame convention fix 의 성공 지표*.

---

## P6 debug cycle 2 — ov_init port + frame convention 진실 (2026-05-26)

### 시도 1 — gram_schmidt 단독 (rejected, but enlightening)
- 포트: `include/msckf/init/InitializerHelper.hpp` (ov_init/utils/helper.h 의 gram_schmidt 50줄 발췌).
- 어댑터 ctor 를 *mean_accel + mean_gyro* 입력으로 변경, R_GtoI = gram_schmidt(mean_accel), bg=mean_gyro, ba = mean_accel - R*g_inG (OV 식 line 131).
- *진실 발견*: OpenVINS Global z **= up** (cycle 1 의 z=down 추측 *반대*). gram_schmidt 의 `R_GtoI.col3 = z_axis = a_avg/|a|` 는 *body up direction (Global z의 body 표현)* — *Global z 와 body up 평행*.
- 결과: 50f ATE 579 cm (cycle 1 의 940 cm 보다 *개선*). 그러나 `Rwi_diag = (-0.996, -0.999, +0.995)` — **180-deg yaw flip**. gram_schmidt 가 *e_2 cross z* 로 임의 yaw 선택 → KITTI car forward (+x) 와 반대.
- 의미: *propagate 자체는 internal frame 으로 일관* (yaw 는 unobservable). 그러나 *trajectory vs GT* 비교 시 frame 불일치 → ATE 측정 오염.

### 시도 2 — Hybrid (LC R0 + OV ba) — also rejected
- 가설: gram_schmidt 의 *yaw 만 LC 와 align* 하면 ATE 개선될 것. R_GtoI = R0_wi.transpose() (LC R0 유지, gram_schmidt 사용 안 함), bg/ba 만 OV 식.
- 결과: yaw align 회복 (`Rwi_diag = (+0.996, +0.999, +0.995)`), v.z 발산 감소 (0.148 → 0.044 m/s @ f4). 그러나 **ATE *악화*: 50f 990 cm, full 1,599,060 cm (1599 m)**. cycle 1 의 53,279 cm 보다 30x 더 크게 발산.
- 진단: ba = mean_accel - R*g_inG = (0.023, 0.009, **0.257**). z 성분이 *0.26 m/s²* — propagate 가 매 frame 그만큼 *과보정*. KITTI 0117 의 *첫 1초 init window* 가 진짜 정지 아닌 *천천히 움직임* — mean_accel 에 *motion_avg* 가 *gravity 와 함께* 섞여 ba 가 *오염*.

### 핵심 발견 (cycle 2)
- **OpenVINS Global z = up** (gram_schmidt source 확인). cycle 1 의 가정 반대.
- **gram_schmidt 는 z 만 정확, x/y 는 임의 yaw** — Global frame 정의가 *VIO 의 4-DOF unobservable* 의 yaw 부분 까지 *자유* 라는 OpenVINS 의 의도된 동작.
- **R0_wi (LC 정지 init 결과) 가 *yaw 와 frame align*** — LC 가 *world = first IMU pose* 가정으로 만들었기 때문. GT 와 같은 yaw.
- **OV 식 ba = mean_accel - R*g_inG 는 init window 가 *진짜 정지* 일 때만 정확**. KITTI 0117 처럼 *시작 시 천천히 출발* 데이터 에서는 *motion contamination* 으로 *ba 가 motion 흡수* → 잘못된 IMU bias.

### 회수 결정
- 어댑터 ctor 는 *hybrid (R0+mean_accel+mean_gyro)* 시그니처 유지 — *향후 init 옵션 비교 용*.
- 다음 cycle 진입 전 일단 *cycle 1 의 init* (`ba=0`) 로 *baseline 복원* 하는 commit 도 고려.

### 다음 트리거

**`P6 debug cycle 3`** — init data 의 *진짜 정지 구간* 확인 + update 효율 분석:
1. KITTI 0117 의 IMU 측정 첫 *5초* 의 *variance* plot — 어디까지 정지인지 시각화.
2. init window 를 *true stationary* 구간으로 확대 시도.
3. *그래도 안 되면*: msckf_updates_ effective 횟수 측정. chi² gate 가 dropping 율 정량화.
4. NoiseManager 의 sigma_pix 조정 (현재 1.0, KITTI 의 ORB feature 에 비해 작을 수 있음 — feature update weight 작음).
5. *fallback*: cycle 1 baseline (53k cm) 으로 revert 후 update 단독 디버깅.

---

## P3 — Update + Feat 포트 (2026-05-25)

### 산출물
- `include/msckf/update/` — UpdaterMSCKF, UpdaterHelper, UpdaterOptions (3 헤더)
- `include/msckf/feat/` — Feature, FeatureInitializer, FeatureInitializerOptions (3 헤더)
- `src/msckf/update/` — UpdaterMSCKF.cpp, UpdaterHelper.cpp (2 cpp)
- `src/msckf/feat/` — Feature.cpp, FeatureInitializer.cpp (2 cpp)

총 **10 파일 (6 헤더 + 4 cpp)** 포트.

### 좁힌 범위 결정 (FeatureHelper / FeatureDatabase 제외)

원 plan §2.1 의 P3 목록에는 `feat/FeatureHelper.h` 가 포함되어 있었으나, **포트하지 않음**:

- UpdaterMSCKF.cpp 의 시그니처는 `update(state, std::vector<std::shared_ptr<Feature>> &feature_vec)` — Feature 의 vector 만 직접 받음.
- FeatureHelper 는 본 repo 의 `frontend/stereo_tracker` 가 대체할 영역 (track database 관리). P4 어댑터에서 stereo_tracker 출력을 `std::vector<Feature>` 로 변환하면 됨.
- FeatureHelper 를 포함하면 `FeatureDatabase.{h,cpp}` 까지 의존성이 끌려 들어옴 — 이는 별도 컴포넌트로 P4 또는 별도 stage 에서 결정.

결과: P3 의 의존성 그래프가 *닫힘* (P3 의 모든 include 가 P1/P2 또는 P3 안에서 해결됨). 빌드가 깔끔.

### 적용한 패치

1. **확장자 변경 + 복사**: `.h` → `.hpp`. 4 디렉토리 (`include/msckf/{update,feat}`, `src/msckf/{update,feat}`) 신규 생성.
2. **Include 경로 재작성** (sed 일괄): 32개 패턴. 새로 추가된 패턴:
   - `feat/Feature.h` → `msckf/feat/Feature.hpp` 외 4개
   - `UpdaterMSCKF.h`, `UpdaterHelper.h`, `UpdaterOptions.h` (same-dir) → `msckf/update/` 풀패스
   - 잔여 unrewritten quoted include **0개** (검증 완료).
3. **CMakeLists.txt** 갱신:
   - `find_package(Boost REQUIRED COMPONENTS filesystem date_time)` ← `date_time` 추가
   - `target_link_libraries(... Boost::date_time)` 추가
   - 신규 cpp 4개 등록 (Feature, FeatureInitializer, UpdaterHelper, UpdaterMSCKF)
   - `boost::math` 은 header-only 라 component 등록 불필요

### 변경 없음 (의도)

- UpdaterMSCKF.cpp, UpdaterHelper.cpp, Feature.cpp, FeatureInitializer.cpp 본문 그대로.
- chi² gating, null-space projection (Givens QR), measurement compression QR 모두 OpenVINS 원본 그대로.
- chi² 임계값 표 (`chi_squared_table[i]`) 생성자에서 미리 채워둠 — 호출 시점 boost::math::quantile 안 불러서 빠름.
- ROS 가드 0개 (P3 sources 에는 ROS_AVAILABLE 매크로 사용처 없음).

### 빌드 결과

```
[ 76%] Building CXX ... src/msckf/feat/Feature.cpp.o
[ 88%] Building CXX ... src/msckf/feat/FeatureInitializer.cpp.o
[ 88%] Building CXX ... src/msckf/update/UpdaterHelper.cpp.o
[ 94%] Building CXX ... src/msckf/update/UpdaterMSCKF.cpp.o
[100%] Linking CXX static library libvio_core.a
[100%] Built target vio_core
```

`libvio_core.a` **1.99 MB → 2.9 MB** (+0.9 MB). **Warnings 0, errors 0**.

`nm` 검증: P3 핵심 함수 14개 (UpdaterMSCKF::update, UpdaterHelper::get_feature_jacobian_full / nullspace_project_inplace / measurement_compress_inplace, FeatureInitializer 등) 정상 export.

### 검증 범위

- 컴파일 + 링크 ✅
- 런타임 동작 ❌. P5 에서:
  - UpdaterMSCKF 의 chi² gating: i=1..500 까지 임계값 표 초기화 (생성자) → 첫 update 호출 시점 0이 아닌 임계값 사용 확인
  - Feature 의 anchor frame timestamp 검색 (`timestamps[cam_id]` 순회)
  - FeatureInitializer 의 multi-view triangulation 수렴 여부

### 리스크 업데이트

- **R1 (JPL ↔ Hamilton 컨벤션)**: P3 단계에서 *접촉* — UpdaterMSCKF 가 Pose JPL state 와 직접 상호작용. 그러나 LC EKF 의 Hamilton 코드와는 여전히 격리. P4 어댑터에서 본격 충돌.
- **R3 (Windows MSVC)**: 미관측. WSL gcc 빌드 통과.
- **R7 (빌드 환경 path)**: 영향 없음. 모든 빌드 WSL.
- **새 발견 (R8) — Feature ↔ stereo_tracker 어댑터 필요**: P3 의 FeatureHelper 를 의도적으로 제외했으므로, P4 에서 본 repo `frontend/stereo_tracker` 의 track 출력을 OpenVINS `std::vector<std::shared_ptr<ov_core::Feature>>` 로 변환하는 어댑터 작성 필요. Feature 의 필드 (uvs, uvs_norm, timestamps, anchor_*) 채우기 + JPL 변환.

### 다음 트리거

**`P4 시작`** — `msckf_pipeline` 어댑터 작성. 본 repo 의 `io/`, `frontend/`, `apps/run_vio.cpp` 와 OpenVINS State / Propagator / UpdaterMSCKF 를 연결.

예상 작업 (~8-12 시간):
- `include/msckf_pipeline/msckf_pipeline.hpp` 신규 — VioManager 대체
- `src/msckf_pipeline/msckf_pipeline.cpp` 신규
- Quaternion 변환 어댑터 (Hamilton ↔ JPL) — R1 본격 해결
- stereo_tracker → Feature vector 변환 (R8)
- IMU 정지 초기화 → State 초기 공분산
- `apps/run_vio.cpp` 에 `--engine=msckf` 옵션 추가 (LC 와 공존)
- 빌드 통과 후 run_vio 실행만 가능한 상태가 목표 (트래젝토리 출력은 P5)
