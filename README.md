# C++ Seg-aided  연구 노트

> **연구 주제**: Stereo + IMU gravity 기반 floor/wall segmentation 으로 추출한 plane constraint 를 TC MSCKF 에 통합해 texture-poor / dynamic 환경에서도 drift 를 억제하는 VIO.
> **Baseline**: 본 프로젝트 안에 LC-EKF VIO (KITTI ATE ~153 cm) 가 비교군으로 보존됨.
> **Roadmap**: 본 폴더의 `PLAN/README.md` (9주 일자별 로드맵) 참조.
>
> **현재 단계 (2026-05-15)**: Phase 1 — LC-EKF VIO + lidar/stereo floor-wall segmentation 작업이 마무리된 시점. Phase 2 (TC MSCKF 전환) 직전. 본 스냅샷은 `taetechpro/LC-VIO_EKF` 에 박제되어 있으며, 이후 작업은 본 main 브랜치 (origin: `taetechpro/scene-aware-vio`) 에서 이어진다.

현재 실행기는 `dataset_type` 설정으로 `EuRoC`와 `KITTI raw`를 모두 지원하며,
기본 초기화는 `GT 없이 IMU 정지구간` 기반으로 동작한다.

---

## 목차

1. [배경 지식: VIO 아키텍처 비교](#1-배경-지식)
2. [프로젝트 구조](#2-프로젝트-구조)
3. [핵심 이론 요약](#3-핵심-이론-요약)
4. [C++ 빌드 시스템 완전 이해](#4-c-빌드-시스템-완전-이해) ← **C++ 입문자 필독**
5. [코드 1대1 흐름 설명](#5-코드-1대1-흐름-설명) ← **Python↔C++ 1:1 대조**
6. [빌드 & 실행 명령어 상세](#6-빌드--실행-명령어-상세)
7. [구현 중 발견한 버그 & 교훈](#7-버그--교훈)
8. [실험 결과](#8-실험-결과)
9. [다음 단계: Tight MSCKF](#9-다음-단계-tight-msckf)

---

## 1. 배경 지식

### VIO 아키텍처 비교

| 방식 | 대표 시스템 | 측정값 | 복잡도 | 정확도 |
|------|-----------|--------|--------|--------|
| **LC-EKF** (이 프로젝트) | VINS-Fusion LC | VO position | 낮음 | 중 |
| **Tight MSCKF** | OpenVINS, MSCKF | feature reprojection | 높음 | 높음 |
| **Tight optimization** | ORB-SLAM3, VINS-Mono | feature + IMU factor | 매우 높음 | 매우 높음 |

### LC-EKF vs Tight MSCKF

```
LC-EKF (Loosely-Coupled):
  IMU propagation ──→ EKF state
  VO position ──────→ EKF measurement
  
  장점: 구현 단순, VO 교체 용이
  단점: VO 오차가 그대로 EKF에 전달됨

Tight MSCKF (Tightly-Coupled):
  IMU propagation ──→ EKF state (IMU + sliding-window camera poses)
  raw features ─────→ EKF measurement (reprojection residual)
  
  장점: feature 레벨에서 기하 제약 → 더 높은 정확도
  단점: Null-space projection, Jacobian 복잡
```

---

## 2. 프로젝트 구조

```
04_cpp_seg_msckf_vio/
├── CMakeLists.txt
├── README.md                    ← 이 파일
├── config/
│   └── euroc_v101.yaml          ← 캘리브레이션 + 실행 파라미터
├── include/
│   ├── core/
│   │   ├── types.hpp            ← 데이터 구조체 (ImuData, CamData, GtData 등)
│   │   └── so3.hpp              ← SO(3) 연산: hat, Exp, Log
│   ├── io/
│   │   └── euroc_reader.hpp     ← EuRoC ASL CSV 로더
│   ├── frontend/
│   │   └── stereo_tracker.hpp   ← KLT + Stereo triangulation + PnP VO
│   └── ekf/
│       ├── imu_propagator.hpp   ← 15-state Error-state IMU 전파기
│       └── lc_ekf.hpp           ← Loosely-Coupled EKF 업데이트
├── src/                         ← 위 헤더의 구현부
├── apps/
│   └── run_vio.cpp              ← 공통 VIO 메인 루프 (EuRoC/KITTI raw)
├── tools/
│   └── plot_trajectory.py       ← 결과 시각화 (matplotlib)
└── results/                     ← 실행 결과 저장 위치
```

### 의존성 (CMake find_package)

| 라이브러리 | 용도 | WSL 설치 |
|-----------|------|---------|
| **Eigen3** | 행렬 연산 | `apt install libeigen3-dev` |
| **OpenCV 4** | KLT, PnP, 이미지 | `apt install libopencv-dev` |
| **yaml-cpp** | 설정 파일 | `apt install libyaml-cpp-dev` |

---

## 3. 핵심 이론 요약

### 3.1 SO(3) — 회전 행렬 연산 (`so3.hpp`)

```
hat(v)   : ℝ³ → 3×3 skew-symmetric  [v]×
Exp(ω)  : ℝ³ → SO(3)  (Rodrigues 공식)
Log(R)  : SO(3) → ℝ³  (angle-axis 추출)
```

EKF에서 회전 오차 δθ ∈ ℝ³ 로 parameterize:
```
R_corrected = R_nominal * Exp(δθ)
```

### 3.2 15-state Error-state EKF (`imu_propagator.hpp`)

**공칭 상태 (nominal state)**:
```
x = [p(3), v(3), R(3×3), b_g(3), b_a(3)]
```

**오차 상태 (error state)**:
```
δx = [δp, δv, δθ, δb_g, δb_a]  ∈ ℝ¹⁵
```

**연속시간 오차 동역학**:
```
d/dt [δp]   = [0   I   0        0    0 ] [δp  ]
     [δv]     [0   0  -R[a×]   0   -R ] [δv  ]
     [δθ]     [0   0  -[ω×]   -I    0 ] [δθ  ]
     [δb_g]   [0   0   0        0    0 ] [δb_g]
     [δb_a]   [0   0   0        0    0 ] [δb_a]
```

**공분산 이산 전파**:
```
P ← Φ·P·Φᵀ + G·Qd·Gᵀ   (first-order Euler)
Φ = I + F·dt
```

**RK4 공칭 상태 적분**:
```cpp
// k1 = f(s0),  k2 = f(s0 + dt/2*k1),  ...
p ← p + dt/6 * (k1.p + 2k2.p + 2k3.p + k4.p)
v ← v + dt/6 * (k1.v + ...)
R ← R * Exp(dt * ω_corrected)
```

### 3.3 LC-EKF 업데이트 (`lc_ekf.hpp`)

VO가 카메라 위치 `p_cam_world`를 측정.  
EKF 예측: `h = R_WI * p_IC + p_WI` (lever arm 포함)

```
혁신:   z  = p_cam_measured - h
H 행렬: [I₃ | 0₃ | [R_WI·p_IC]× | 0₃ | 0₃]  (3×15)
게인:   K  = P·Hᵀ·(H·P·Hᵀ + R_noise)⁻¹
보정:   δx = K·z
업데이트: P ← (I-K·H)·P·(I-K·H)ᵀ + K·R·Kᵀ  (Joseph form)
```

### 3.4 Stereo VO (`stereo_tracker.hpp`)

```
매 프레임:
1. KLT temporal tracking (prev_img_l → curr_img_l)
2. solvePnPRansac(map_pts_3d, curr_pts_2d) → T_cam_world
3. KLT stereo tracking (curr_img_l → curr_img_r)
4. triangulatePoints(P0, P1, ud_l, ud_r) → pts_3d_cam
5. pts_3d_world = R_wc * pts_3d_cam + t_wc
6. 새 feature 검출 (FAST) + stereo triangulation → map 추가

불변식: prev_pts_l_.size() == map_pts_world_.size() (항상 1:1 aligned)
```

---

## 4. C++ 프로젝트 빌드 시스템 완전 이해

> C++을 처음 접한다면 "코드를 어떻게 실행 가능한 프로그램으로 만드나?"부터  
> 이해해야 한다. Python은 `python script.py`로 바로 실행되지만  
> C++은 **컴파일 → 링크 → 실행** 단계를 거친다.

### 3.1 헤더(`.hpp`) vs 소스(`.cpp`) — 역할 분리

```
include/ekf/lc_ekf.hpp    ← "이런 함수가 있다"고 선언만 함 (설계도)
src/ekf/lc_ekf.cpp        ← 실제 코드 구현 (공사)
apps/run_vio.cpp           ← 위 모든 것을 사용하는 메인 프로그램
```

**Python과 비교**:
```python
# Python: 선언과 구현이 한 파일
class LcEkf:
    def update_vo(self, p): ...   # 한 파일에 다 있음
```

```cpp
// C++ 헤더 (lc_ekf.hpp) — "이런 함수가 있어요"
class LcEkf {
public:
    bool update_vo(const Eigen::Vector3d& p);  // 선언만
};

// C++ 소스 (lc_ekf.cpp) — "이렇게 동작해요"
bool LcEkf::update_vo(const Eigen::Vector3d& p) {
    // 실제 구현
}
```

**왜 나누나?** 다른 파일에서 `#include "ekf/lc_ekf.hpp"` 한 줄로  
구현 내용 없이 인터페이스만 가져올 수 있어서 컴파일이 빠르고 구조가 명확해짐.

---

### 3.2 CMakeLists.txt — 빌드 설명서

`cmake`는 Python의 `setup.py`나 JS의 `package.json`과 비슷한 역할.  
"어떤 파일을 어떻게 묶어서 실행파일로 만들지" 기술한다.

```cmake
# CMakeLists.txt 핵심 부분 설명

cmake_minimum_required(VERSION 3.16)   # cmake 최소 버전
project(cpp_seg_msckf_vio CXX)        # 프로젝트 이름, C++ 사용
set(CMAKE_CXX_STANDARD 20)            # C++20 문법 사용

# 외부 라이브러리 찾기 (apt로 설치된 것들)
find_package(Eigen3 REQUIRED)         # 행렬 연산 라이브러리
find_package(OpenCV 4 REQUIRED)       # 이미지 처리 라이브러리
find_package(yaml-cpp REQUIRED)       # YAML 파일 파서

# ── 라이브러리 만들기 ──────────────────────────────
# src/ 안의 .cpp 파일들을 묶어서 "lc_vio_core"라는 라이브러리로 만듦
add_library(lc_vio_core
    src/io/euroc_reader.cpp         # EuRoC 데이터 로더
    src/frontend/stereo_tracker.cpp # VO 모듈
    src/ekf/imu_propagator.cpp      # IMU 전파
    src/ekf/lc_ekf.cpp              # EKF 업데이트
)
# 이 라이브러리가 Eigen, OpenCV, yaml-cpp를 사용한다고 명시
target_link_libraries(lc_vio_core PUBLIC Eigen3::Eigen ${OpenCV_LIBS} yaml-cpp)

# ── 실행 파일 만들기 ────────────────────────────────
# apps/run_vio.cpp를 "run_vio"라는 실행파일로 만들되
# 위에서 만든 lc_vio_core 라이브러리를 붙임
add_executable(run_vio apps/run_vio.cpp)
target_link_libraries(run_vio PRIVATE lc_vio_core)

# 기존 명령어 호환용 별칭: 같은 main()을 run_euroc 이름으로도 빌드
add_executable(run_euroc apps/run_vio.cpp)
target_link_libraries(run_euroc PRIVATE lc_vio_core)
```

---

### 3.3 빌드 3단계: configure → generate → compile

```
소스코드(.cpp/.hpp)
      │
      ▼  cmake ..   (configure + generate)
  Makefile 생성    ← "어떻게 컴파일할지" 규칙 파일
      │
      ▼  make -j4  (compile + link)
  run_vio          ← 실행 가능한 바이너리
      │
      ▼  ./run_vio config.yaml
  결과 출력
```

#### `mkdir -p build && cd build`
```
build/         ← 컴파일 중간 산출물(.o 파일, Makefile 등)을 여기 격리
               Python의 __pycache__ 같은 개념
               소스코드 디렉토리를 깨끗하게 유지 (out-of-source build)
```

#### `cmake .. -DCMAKE_BUILD_TYPE=Release`
```
..             ← 상위 폴더의 CMakeLists.txt를 읽음
-DCMAKE_BUILD_TYPE=Release
               ← Release: 최적화(-O2) 켬, 디버그 심볼 제거 → 실행 빠름
                  Debug:   최적화 끔, GDB 사용 가능 → 개발 중에 사용
```
실행 후 `build/` 안에 `Makefile`이 생성된다.

#### `make -j4`
```
make           ← Makefile 읽고 컴파일 시작
-j4            ← CPU 코어 4개 병렬 사용 (빌드 시간 단축)
               .cpp 파일 하나가 .o(오브젝트) 파일로 변환됨
               마지막에 모든 .o를 링크해서 run_vio 실행파일 완성
```

컴파일 출력 예:
```
[ 14%] Building CXX object .../euroc_reader.cpp.o   ← 각 .cpp 컴파일
[ 28%] Building CXX object .../stereo_tracker.cpp.o
[ 57%] Linking CXX static library liblc_vio_core.a   ← 라이브러리 묶음
[100%] Linking CXX executable run_vio                 ← 최종 실행파일
```

#### `./run_vio ../config/euroc_v101.yaml`
```
./run_vio      ← 현재 폴더(build/)의 run_vio 실행
               (Python의 python script.py 에 해당)
../config/...  ← 설정파일 경로를 명령행 인자로 전달
               (Python의 sys.argv[1] 에 해당)
```

---

### 3.4 `#include` — 파일을 연결하는 방법

```cpp
// run_vio.cpp 상단
#include <Eigen/Core>          // < >: 시스템/라이브러리 헤더 (apt로 설치된 것)
#include <opencv2/imgcodecs.hpp>
#include "io/euroc_reader.hpp"  // " ": 이 프로젝트 내 헤더 (include/ 폴더 기준)
#include "ekf/lc_ekf.hpp"
```

컴파일러가 `#include "io/euroc_reader.hpp"` 를 만나면  
`include/io/euroc_reader.hpp` 파일 내용을 그 자리에 복사해 넣는 것과 같다.

---

## 5. 코드 1대1 흐름 설명

> `apps/run_vio.cpp` 메인 루프를 따라가며  
> 각 줄이 어떤 C++ 개념이고 왜 있는지 설명한다.

### 4.1 데이터 로드 (C++ 객체 생성)

```cpp
// Python: data = EuRoCReader(dataset_root)
EuRoCReader data(dataset_root);
//           ^^^^ 클래스 인스턴스(객체) 생성
//                Python의 __init__ = C++의 생성자(constructor)
//                생성자 안에서 CSV 파일을 읽고 IMU/Cam/GT 벡터를 채움

const auto& imu_data = data.imu();  // IMU 샘플 전체 목록
const auto& cam_data = data.cam();  // 카메라 프레임 전체 목록
const auto& gt       = data.gt();   // Ground Truth 전체 목록
// auto& = 타입을 컴파일러가 자동 추론 (Python의 동적 타입과 유사)
// const = 이 변수는 수정 안 함 (읽기 전용)
// &     = 복사 없이 원본 참조 (메모리 효율)
```

### 4.2 Eigen 행렬 — NumPy 대응

```cpp
// Python/NumPy:
// p0 = np.array([0.787, 2.177, 1.062])
// R0 = np.eye(3)

Eigen::Vector3d p0 = it->p;       // 3×1 double 벡터
Eigen::Matrix3d R0 = it->q.toRotationMatrix();  // 3×3 회전 행렬
// Eigen::Vector3d ← 고정 크기(3), double 타입, 컴파일 타임에 크기 확정
//                   NumPy와 달리 크기가 고정이라 매우 빠름
```

**자주 쓰는 Eigen 연산**:
```cpp
Eigen::Vector3d a(1.0, 2.0, 3.0);  // 3D 벡터 초기화
Eigen::Vector3d b = a * 2.0;       // 스칼라 곱 (NumPy: a * 2)
double dot = a.dot(b);             // 내적 (NumPy: np.dot(a,b))
Eigen::Vector3d cross = a.cross(b); // 외적 (NumPy: np.cross(a,b))
double norm = a.norm();            // 크기 (NumPy: np.linalg.norm(a))

Eigen::Matrix3d M = Eigen::Matrix3d::Identity(); // 단위행렬
Eigen::Vector3d v = M * a;        // 행렬-벡터 곱 (NumPy: M @ a)
Eigen::Matrix3d Mt = M.transpose(); // 전치 (NumPy: M.T)
```

### 4.3 중력 추정

```cpp
// 정지 상태에서: f_body = -R_IW * g_world
// → g_world = -R_WI * f_body
Eigen::Vector3d accel_sum = Eigen::Vector3d::Zero();
int accel_cnt = 0;
for (const auto& imu : data.imu()) {     // for imu in data.imu():
    if (imu.timestamp < t0) continue;    //     if ...: continue
    if (imu.timestamp > t0 + 0.5) break; //     if ...: break
    accel_sum += imu.accel - bg0;        //     accel_sum += ...
    accel_cnt++;
}
Eigen::Vector3d g_world = -(R0 * (accel_sum / accel_cnt));
// R0 * vec : 3×3 행렬 × 3×1 벡터 = 3×1 벡터
```

### 4.4 메인 루프 — IMU + 카메라 인터리빙

```cpp
// Python: for cam in cam_data:
for (const auto& cam : cam_data) {

    // 이 카메라 프레임 이전까지의 IMU 샘플을 모두 전파
    // Python: while imu_data[imu_idx].timestamp <= cam.timestamp:
    while (imu_idx < imu_data.size() &&
           imu_data[imu_idx].timestamp <= cam.timestamp) {
        const auto& imu = imu_data[imu_idx++];  // imu_idx 가져오고 +1
        ekf.propagate(imu.timestamp, imu.gyro, imu.accel);
    }

    // 이미지 로드
    cv::Mat img_l = cv::imread(cam.img_l, cv::IMREAD_COLOR);
    // cv::Mat = OpenCV의 이미지/행렬 타입 (NumPy array와 유사)
    // cv::imread = 파일에서 이미지 읽기

    // VO 처리 (KLT 추적 + PnP)
    auto pose = tracker.process(img_l, img_r);
    // auto = 리턴 타입을 컴파일러가 추론 (StereoTracker::Pose)
    // pose.t = 카메라 위치 (3D 벡터), pose.valid = 성공 여부

    // VO 좌표계 변환 (VO-local → GT world)
    Eigen::Vector3d p_world_cam = R_WC0 * pose.t + t_WC0;

    // EKF 업데이트
    if (pose.valid && frame_count > 0)
        ekf.update_vo(p_world_cam);

    // 결과 저장
    traj_est.push_back(ekf.position());
    // std::vector::push_back = Python list.append()
}
```

### 4.5 `std::vector` — Python 리스트 대응

```cpp
// Python: traj = []
std::vector<Eigen::Vector3d> traj_est;

// Python: traj.append(pos)
traj_est.push_back(ekf.position());

// Python: traj[0]
traj_est[0];

// Python: len(traj)
traj_est.size();

// Python: for pos in traj:
for (const auto& pos : traj_est) { ... }
```

### 4.6 클래스 메서드 호출 — Python과 비교

```cpp
// Python: ekf.propagate(t, gyro, accel)
ekf.propagate(imu.timestamp, imu.gyro, imu.accel);
// C++도 동일한 문법. 차이: 타입이 컴파일 타임에 고정

// Python: p = ekf.position()
Eigen::Vector3d p = ekf.position();
// 리턴값 타입(Eigen::Vector3d)을 반드시 받는 변수 타입과 맞춰야 함

// Python: pose.valid
if (pose.valid) { ... }
// 구조체/클래스 멤버 접근: . 연산자 (Python과 동일)

// 포인터로 접근할 때만 다름:
// Python: obj.method()
// C++ (포인터): obj->method()   ← 이 프로젝트에서는 거의 안 씀
```

### 4.7 파일 저장 (`std::ofstream`)

```cpp
// Python: with open(path, 'w') as f:
std::ofstream f(output_dir + "/trajectory_tum.txt");
//              ^^^^^^^^ 파일 경로 (std::string + "/" + std::string)

// Python: f.write("# header\n")
f << "# timestamp tx ty tz qx qy qz qw\n";
// << 연산자 = Python의 print(... , file=f) 또는 f.write()

// Python: f.write(f"{ts:.9f} {x} {y} {z}\n")
f << std::fixed << std::setprecision(9) << traj_ts[i] << " "
  << traj_est[i].x() << " "   // .x() = 벡터의 x 성분
  << traj_est[i].y() << " "
  << traj_est[i].z() << "\n";
// 파일은 ofstream 소멸 시 자동으로 닫힘 (Python with문 같은 효과)
```

---

## 6. 빌드 & 실행 명령어 상세

### WSL Ubuntu 24.04 기준

```bash
# WSL 진입
wsl

# 의존성 (최초 1회)
sudo apt install -y build-essential cmake libeigen3-dev libopencv-dev libyaml-cpp-dev

# 빌드
cd /mnt/d/02_research/04_cpp_seg_msckf_vio
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4

# 실행 (EuRoC 전체 시퀀스)
./run_vio ../config/euroc_v101.yaml

# 실행 (KITTI raw)
./run_vio ../config/kitti_raw_2011_09_26_drive_0117.yaml

# Rerun .rrd 저장 (선택 기능)
cd /mnt/d/02_research/04_cpp_seg_msckf_vio
cmake -S . -B build_rerun -DENABLE_RERUN=ON -DCMAKE_BUILD_TYPE=Release
cmake --build build_rerun -j4
./build_rerun/run_vio \
  config/kitti_raw_2011_09_26_drive_0117.yaml \
  --rerun-save results/kitti_raw_2011_09_26_drive_0117/vio_rerun.rrd \
  --rerun-image-every 10

# Windows Python에 rerun-sdk가 설치되어 있으면
python -m rerun_cli D:\02_research\04_cpp_seg_msckf_vio\results\kitti_raw_2011_09_26_drive_0117\vio_rerun.rrd

# Rerun world/trajectory_est_aligned는 ATE와 같은 Umeyama 정렬을 적용한 궤적
# raw 좌표는 trajectory_tum.txt, 정렬 좌표는 trajectory_aligned_tum.txt에 저장됨
# IMU raw 그래프는 imu/gyro_*_rad_s, imu/accel_*_mps2 entity로 기록되고 imu_log.txt에도 저장됨

# 빠른 테스트 (max_frames: 200 으로 변경 후)
./run_vio ../config/euroc_v101.yaml
```

### 결과 시각화

```bash
# WSL에서 나가서 Windows Python으로
cd D:\02_research\04_cpp_seg_msckf_vio
python tools/plot_trajectory.py results/v101
```

### 주요 config 파라미터 (`config/euroc_v101.yaml`)

```yaml
dataset: "/mnt/d/..."    # EuRoC ASL CSV 경로 (WSL path)
output:  "/mnt/d/..."    # 결과 저장 경로
max_frames: 0            # 0=전체, 200=200프레임 (테스트용)
ekf:
  sigma_vo: 0.1          # VO 측정 노이즈 [m] (작을수록 VO를 더 신뢰)
```

---

## 7. 버그 & 교훈

구현 과정에서 발견한 핵심 버그들. 실수하기 쉬운 패턴.

### Bug 1: Stereo 페어링 threshold 너무 작음

- **증상**: `CAM=8` (2912가 되어야 함)
- **원인**: cam0-cam1 timestamp 차이 최대 12.6ms, threshold=5ms
- **수정**: nearest-neighbor 방식 + threshold=25ms (카메라 주기 50ms의 절반)

### Bug 2: Bootstrap 삼각측량에서 undistort 누락

- **증상**: 초기 3D 맵 포인트가 완전히 틀림 → PnP 결과 이상
- **원인**: `stereo_triangulate(kept_l, kept_r)` 에 원시 pixel 좌표 전달  
  P 행렬이 undistorted pixel 공간 기준인데 distorted pixel 사용
- **수정**: `undistort(kept_l, K0, dist0)` 먼저 적용 후 triangulate

### Bug 3: StereoTracker 인덱스 정렬 파괴

- **증상**: `tracked_count` 가 168 → 330 → 497 (비정상 증가), VO 발산
- **원인**: `detect_new()`이 `prev_pts_l_`에 append하는 동시에  
  `curr_pts_l.size()`를 `before`로 사용 → 인덱스 어긋남  
  `prev_pts_l_`와 `map_pts_world_`의 1:1 정렬이 깨짐
- **수정**: StereoTracker 전체 재설계.  
  **불변식**: `prev_pts_l_.size() == map_pts_world_.size()` 항상 유지  
  새 feature는 별도로 stereo triangulate 후 양쪽에 동시 append

### Bug 4: EuRoC IMU-GT 중력 방향 불일치

- **증상**: EKF z-위치가 -14 m/s² 가속으로 폭발 (10프레임 내 수십 미터 낙하)
- **원인**: GT 쿼터니언으로 예측한 body-frame 중력 = `(-0.456, -0.137, 9.798)`  
  실제 IMU 측정값 = `(9.088, 0.131, -3.694)` → **~90° 불일치**  
  (EuRoC 데이터셋 좌표계 변환 이슈로 추정)
- **수정**: IMU 첫 0.5초 평균으로 중력 방향 경험적 추정
  ```cpp
  g_world = -(R0 * mean(accel_body_static))
  // → (-8.625, -2.090, 4.189) m/s², |g|=9.81 확인
  ```
  정적 구간에서 `a_world=0` → `g_world = -R_WI * f_body` 관계 이용

---

## 8. 실험 결과

### EuRoC V1_01_easy (2912 프레임, 145초)

| 시스템 | ATE RMSE | 비고 |
|--------|---------|------|
| **이 프로젝트 LC-EKF VIO (Phase 1-B)** | **266 cm** | Stereo rectification + **ORB descriptor 매칭** + 에피폴라 제약 |
| 이 프로젝트 LC-EKF VIO (Phase 1-A, KLT) | 361 cm | KLT 스테레오 매칭 (반복 패턴 실패) |
| OpenVINS MSCKF (공식, 목표) | **7 cm** | Tight MSCKF — Phase 2에서 도달 목표 |

> ATE: translation-only alignment (첫 프레임 오프셋 보정) 후 RMS 오차  
> EuRoC V1_01_easy 전체 시퀀스 (2912 프레임, 145초)

### 분석

- **LC-EKF의 한계**: VO position을 측정값으로 사용 → VO 오차가 그대로 EKF에 전달됨
- **MSCKF의 강점**: raw feature reprojection 잔차를 직접 사용 → 50배 이상 정확
- **개선 여지 (Phase 1 내)**: epipolar 필터링으로 stereo depth 개선, gravity 초기화 정교화
- **근본 해결 (Phase 2)**: Tight MSCKF 전환

---

## 9. 다음 단계: Tight MSCKF

### LC-EKF → MSCKF 전환 경로

**Phase 2** 구현 목표: `lc_ekf.hpp/.cpp` → `msckf.hpp/.cpp` 교체

#### 상태 확장

```
LC-EKF:  δx ∈ ℝ¹⁵  [δp, δv, δθ, δbg, δba]
MSCKF:   δx ∈ ℝ^(15+6N)  + N개 camera pose clones
```

#### MSCKF 측정 모델

```
1. Camera clone 추가 (at each keyframe)
   x_state ← [x_imu | x_cam1 | x_cam2 | ... | x_camN]

2. Feature triangulation (multi-view)
   관측된 모든 frame에서 feature 위치 → linear triangulation

3. Reprojection residual
   r_i = z_i - π(R_Ci, t_Ci, p_f)  (픽셀 잔차)

4. Left null-space projection (feature 소거)
   T_H · H_x · δx = T_H · r  (feature pose 제거)

5. Chi2 gating + EKF update
```

#### 참고 자료

- [MSCKF 원논문] Mourikis & Roumeliotis, ICRA 2007
- [OpenVINS 소스] `official_openvins_ros2/ov_msckf/src/` 참조
- [이 프로젝트 관련 MSCKF 분석] `D:\05_agent\codex_ver\OpenVINS\` 내 Python 구현 참고

---

## 빠른 참조

### WSL 원라이너

```bash
# 빌드 + 실행 + 플롯 한번에
cd /mnt/d/02_research/04_cpp_seg_msckf_vio && \
  mkdir -p build && cd build && cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_BUILD_RPATH_USE_ORIGIN=ON > /dev/null && make -j4 && \
  ./run_vio ../config/euroc_v101.yaml
```

```bash
# 플롯 (Windows Python)
python D:\02_research\04_cpp_seg_msckf_vio\tools\plot_trajectory.py D:\02_research\04_cpp_seg_msckf_vio\results\v101
```

### 파일별 핵심 함수

| 파일 | 핵심 함수 | 역할 |
|------|----------|------|
| `so3.hpp` | `Exp(ω)`, `Log(R)`, `hat(v)` | SO(3) 연산 |
| `euroc_reader.cpp` | `load_imu`, `load_cam` | ASL CSV 파싱 |
| `stereo_tracker.cpp` | `process(img_l, img_r)` | VO 출력 |
| `imu_propagator.cpp` | `propagate(t, gyro, accel)` | RK4 + 공분산 |
| `lc_ekf.cpp` | `update_vo(p_world_cam)` | Kalman 보정 |
| `run_vio.cpp` | `main()` | 전체 파이프라인 |
