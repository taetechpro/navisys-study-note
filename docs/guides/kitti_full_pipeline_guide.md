---
title: "KITTI raw 풀 파이프라인 운용 가이드"
date: 2026-05-14
type: guide
tags: [kitti, pipeline, build, run, vio]
related:
  - "[[rerun_cpp_visualization_guide]]"
status: reviewed
---

# KITTI raw 풀 파이프라인 운용 가이드

> **목적**: 본 문서는 `04_cpp_seg_msckf_vio` 프로젝트를 **외부 도움 없이**
> 처음부터 끝까지(데이터 다운로드 → 캘리브레이션 추출 → config 작성 → 빌드 →
> 단일 시퀀스 실행 → 다중 시퀀스 일괄 테스트 → 결과 검증) 굴려보고 싶은 사용자를
> 위한 **운용 매뉴얼 + 코드 내부 워크스루**다.
>
> SO(3)·Error-state EKF 수식 도출, Stereo VO 알고리즘 디테일은
> [`README.md`](../README.md) 가 담당한다. 본 가이드의 §5 는 그 위에서
> "어디서 무엇이 호출되고 어떤 행렬이 계산되는가" 만 짚어 준다.

---

## 처음 읽는 사람을 위한 30분 개관

> 이 절은 VIO·SLAM·EKF·KITTI를 **처음 들어보는 사람**이 백지 상태에서
> "이게 뭐 하는 거고 내가 뭘 할지" 를 30분 안에 잡도록 쓴 도입부다.
> 이 절을 먼저 읽고 §0 사전 요구사항으로 넘어가면 손이 멈추지 않는다.
> (이미 VIO 하던 사람이면 §0 부터 바로 시작하면 된다.)

### A. 왜 이런 시스템이 필요한가 — 위치를 안다는 문제

자율주행/드론/로봇청소기 모두 "내가 지금 어디 있는가"를 매 0.1초마다 알아야 한다.
방법별 정밀도/한계:

| 방법 | 정밀도 | 한계 |
|------|--------|------|
| **GPS** (스마트폰) | 1~10 m | 터널/실내/숲 무력. 1 Hz 라 차량 동역학에 너무 느림 |
| **GPS-RTK** (정밀) | 1~10 cm | 기지국 인프라 필요. 비싸고 GPS 신호 끊기면 똑같이 무력 |
| **휠 오도메트리** | 단기 정밀 | 미끄러짐/타이어 압력으로 누적 오차 |
| **VIO** (이 프로젝트) | 누적 0.5~5% | **GPS 없는 환경에서도 동작**, 인프라 불필요, 실시간 |

**VIO = Visual-Inertial Odometry** — 카메라(visual)와 IMU(inertial)만으로 **6-DoF
pose**(3D 위치 + 3D 회전)를 매 순간 추정. GPS-덜 의존, 실내 가능, 카메라+센서만
있으면 어디서든 동작.

### B. 왜 카메라랑 IMU를 같이 쓰는가

각 센서 단독의 약점이 명확하고 서로 상보적이라서.

**카메라(스테레오) 단독**:
- ✅ 절대 위치 정밀 (특징점이 충분하면 0.1 m 수준)
- ✅ **미터 스케일 자동 확보** (스테레오 베이스라인이 calibration 됨)
- ❌ 빠른 회전 시 모션 블러로 추적 실패
- ❌ 어두운 터널, 단조 텍스처(고속도로 차선만 있는 경우)에서 약함
- ❌ 10 Hz 정도가 한계 (이미지 처리 비용)

**IMU 단독**:
- ✅ 100~200 Hz 고주파 (차량 동역학 매끄럽게 캡처)
- ✅ 빛/날씨/텍스처 무관
- ❌ **누적 오차로 발산**: 가속도를 두 번 적분해 위치를 얻으므로
  3~5초에 1 m, 1분이면 수십~수백 m 오차

**둘을 합치면**:
- 카메라가 절대 기준 역할 → IMU 드리프트를 매 프레임 잡아줌
- IMU가 카메라 사이 빈 시간을 부드럽게 채움
- 한쪽이 일시 실패해도 다른 쪽이 버팀

### C. 왜 스테레오(좌+우 카메라 두 장)인가

**모노 카메라 한 개**의 본질적 한계: 50 cm 떨어진 작은 사과와 5 m 떨어진 큰 사과를
픽셀로는 구별 못 한다. → 모노 VO의 trajectory는 **임의의 스케일** ("100 m 갔는지
1 km 갔는지" 모름).

**스테레오 두 카메라**:
- 좌↔우 픽셀 차이(disparity) + 미리 측정한 두 카메라 사이 거리(**baseline**)
- 깊이 공식: **`Z = (fx · baseline) / disparity`**
- 우리 KITTI calib의 baseline = **53.7 cm** (KITTI 측이 레이저로 잰 물리값)

→ 모든 미터 스케일이 이 53.7 cm 한 값에서 파생. 좌측이 cam0 (`image_00`), 우측이
cam1 (`image_01`). 컬러(`image_02`, `image_03`)도 있지만 본 코드는 **흑백 스테레오
페어 (image_00/01)** 만 사용 (KLT 추적 안정성).

### D. 왜 EKF인가 — 두 측정값을 통계적으로 섞기

EKF = Extended Kalman Filter. 비선형 시스템에서 **각 센서의 잡음 모델을 들고**
가중평균:

```
"IMU 적분으로 위치는 (10.0, 5.0, 0.1) 일 것이다 (불확실성 ±0.5 m)"
"VO가 측정한 위치는 (10.5, 4.7, 0.2) 다 (잡음 ±0.1 m)"
   ↓ EKF가 σ로 가중해 합침
"통합 추정: (10.45, 4.73, 0.19)"  ← VO에 더 가중치 (잡음 작음)
```

내부적으로 15차원 상태 (위치 3 + 속도 3 + 자세 3 + 자이로 bias 3 + 가속도 bias 3)
와 그 15×15 공분산 행렬 `P` 를 들고 다닌다. (상세는 §5)

본 프로젝트는 **LC-EKF (Loosely-Coupled)** — VO의 출력 위치를 측정값으로 받음.
더 정밀한 변종은 **Tight MSCKF** (특징점 픽셀을 직접 측정값으로) 인데,
구현이 복잡하고 본 프로젝트의 다음 마일스톤이다 (README §9).

### E. KITTI 데이터셋이 뭐고 왜 쓰나

- 독일 KIT (카를스루에 공과대) + 미국 TTIC (토요타 시카고 연구소) 가 2012년 공개
- 차량에 장착: 흑백 스테레오 + 컬러 스테레오 + Velodyne LiDAR + **OXTS RT3003 GPS/IMU 통합 유닛**
- 도시/주거지/도로/고속도로/캠퍼스 다양한 환경 50시간+
- **VIO/SLAM 알고리즘 평가의 사실상 표준 벤치마크**

본 프로젝트가 쓰는 부분:
- `image_00`, `image_01` — 흑백 스테레오 (왼/오)
- `oxts/data/*.txt` — 30개 값 한 줄 (위경도/고도/자세/속도/가속도/각속도/...)
  - **values[14..16]** → 가속도 (body frame, m/s²)
  - **values[20..22]** → 각속도 (body frame, rad/s)
  - lat/lon/alt → Mercator 투영 → GT 위치
- LiDAR (`velodyne_points/`) 와 tracklets는 안 씀

### F. 한 프레임에 일어나는 일을 한 그림으로

```
[흑백 좌/우 png]   ┐                                  ┌→ EKF가 보정한 위치
                  │                                   │   (매 0.1초)
   ① ORB로 좌↔우 매칭                                 │
   ② 삼각측량 → 미터 단위 3D 점 (cam0 frame)           │
   ③ KLT로 다음 프레임에서 같은 점 추적                │
   ④ PnP → cam0의 6-DoF pose (미터)                  ├→ trajectory_tum.txt
                  │                                   │
[OXTS IMU]        │                                   └→ trajectory_plot.png
   ⑤ 각/가속도 → RK4 적분으로 IMU 상태 전파            │
                  │                                   │
                  ↓                                   │
              ⑥ EKF가 ④와 ⑤ 융합 ────────────────────┘
                (Kalman gain K로 오차 가중)
                                                       
[GT (OXTS GPS)]
   별도 vector 에 저장                                ┐
                                                      ├→ ATE RMSE (단위 m)
   매 프레임 trajectory_est 와 비교 (Umeyama 정합)     ┘
```

(구체 수식과 행렬은 §5 코드 워크스루)

### G. 처음부터 끝까지 무엇을 하나 — 5단계 1시간 30분

내가 **백지 상태**에서 이 프로젝트를 굴리려면 정확히 다음:

| 단계 | 내용 | 시간 | 가이드 절 |
|------|------|------|-----------|
| 1 | WSL Ubuntu + apt 의존성 설치 (gcc, cmake, OpenCV, Eigen 등) | 30분 | §1 |
| 2 | KITTI 계정 → calib.zip + 한 개 drive sync.zip 다운 | 30분 | §2 |
| 3 | calib 파일 3종 → `T_cam_imu` 4×4 추출 → yaml 채우기 | 15분 | §3, §4 |
| 4 | cmake + make → `build/run_vio` 바이너리 | 5분 | §6 |
| 5 | 단일 시퀀스 실행 + 시각화 (trajectory_plot.png) | 1~5분 | §7, §8 |

여기까지 **약 1시간 30분**이면 첫 결과가 나온다. 이게 바닥선.

그 후 확장:
- 더 긴 시퀀스 받아 누적 드리프트 측정 (§2 + §9)
- 파라미터 튜닝 실험 (§11 시나리오 5선)
- 여러 drive 일괄 비교 → CSV 요약 (§9.5)

### H. 첫 결과 — "정상" 이라는 게 무엇인가

`drive_0001` (11초, 108프레임) 으로 처음 돌리면 stdout 끝에:

```
========================================
Frames processed : 108
ATE RMSE         : 156 cm
========================================
```

**ATE RMSE 1~3 m** 가 LC-EKF 정상 범위. 같은 시퀀스를 Tight MSCKF (OpenVINS 등) 로
돌리면 5~30 cm 까지 줄어든다 (5~30배 차이). 이게 **LC-EKF의 본질적 천장**.

`trajectory_plot.png` 의 왼쪽 패널 (XY top-down) 에서 추정(파란)과 GT(초록)이
**같은 모양**으로 겹쳐 보이면 정상. 약간의 평행이동/회전 오차는 LC-EKF의 알려진
한계 (글로벌 yaw 관측이 없어 절대 방향이 점점 어긋남).

### I. 알아둘 단어 (용어집)

| 단어 | 의미 |
|------|------|
| **VIO** | Visual-Inertial Odometry. 카메라+IMU 위치 추적 |
| **VO** | Visual Odometry. 이미지만으로 카메라 위치 추적 |
| **SLAM** | Simultaneous Localization And Mapping. 위치 + 지도 동시 추정 (VIO 의 상위) |
| **EKF** | Extended Kalman Filter. 비선형 상태 추정의 표준 도구 |
| **LC-EKF** | Loosely-Coupled EKF. VO의 출력(위치)을 측정값으로 받음 (이 프로젝트) |
| **Tight MSCKF** | 특징점 픽셀을 직접 측정값으로 받음 (OpenVINS, MSCKF) |
| **Stereo** | 좌/우 두 카메라로 깊이 추정 |
| **Baseline** | 두 카메라 사이 물리 거리 (KITTI: 53.7 cm) |
| **Disparity** | 같은 점이 좌↔우 이미지에서 픽셀 차이 |
| **KLT** | Kanade-Lucas-Tomasi 광류. 프레임간 특징점 추적 (작은 이동) |
| **ORB** | Oriented FAST + Rotated BRIEF. 회전·스케일 강건 디스크립터 (스테레오 매칭용) |
| **FAST** | Features from Accelerated Segment Test. 빠른 코너 검출기 |
| **PnP** | Perspective-n-Point. 2D-3D 대응에서 카메라 pose 계산 |
| **RANSAC** | RANdom SAmple Consensus. outlier 강건 추정 |
| **IMU** | Inertial Measurement Unit. 3축 가속도 + 3축 자이로 |
| **OXTS** | KITTI에 장착된 GPS+IMU 통합 유닛 (RT3003 모델) |
| **GT** | Ground Truth. 정답값 (KITTI는 OXTS GPS+RTK로 계산) |
| **ATE** | Absolute Trajectory Error. 정합 후 잔차 RMSE |
| **RMSE** | Root Mean Square Error. 잔차 제곱평균 제곱근 |
| **TUM 포맷** | timestamp + xyz + 쿼터니언 한 줄씩의 trajectory 저장 형식 |
| **Rectification** | 스테레오 좌/우 이미지의 epipolar line을 수평 스캔라인으로 만드는 변환 |
| **Lever arm** | IMU 위치와 카메라 위치 사이의 물리 벡터 (`p_IC`) |
| **Calibration** | 카메라 내부/외부 파라미터를 측정하는 절차. 결과물이 `T_cam_imu`, `K` 등 |
| **6-DoF pose** | 3D 위치 (x,y,z) + 3D 회전 (3개 자유도) = 6개 자유도의 자세 |
| **RK4** | Runge-Kutta 4차. 미분방정식 수치 적분기 |
| **Umeyama** | 두 점군을 강체 정합하는 SVD 기반 알고리즘 (ATE 계산 시 사용) |

### J. 어디서 막히면 어디를 보면 되는가

| 막히는 지점 | 보러 갈 곳 |
|-------------|-----------|
| 빌드 에러 | §6.3 (의존성 점검 표) |
| KITTI 페이지에서 다운로드 안 됨 | §2.1 (계정 인증) |
| `T_cam_imu` 행렬 어떻게 채울지 모름 | §3.3 (Python 변환 스크립트) |
| 실행은 됐는데 ATE 가 너무 큼 (10 m+) | §10.1 (실패 모드 표) |
| 코드 내부가 궁금함 | §5 (코드 워크스루) |
| 어디 만지면 뭐가 바뀌나 | §11 (시나리오) + §12 (knob 표) |
| Trajectory plot이 이상함 | §8.3 (발산 패턴 진단표) |

이제 **§0 사전 요구사항** 부터 진행하면 된다. §1~§8 까지 차례로 따라가면 손이
멈추지 않게 흐름이 짜여 있다.

---

## 목차

도입. [처음 읽는 사람을 위한 30분 개관](#처음-읽는-사람을-위한-30분-개관) ← 이미 읽었으면 건너뛰기

0. [사전 요구사항](#0-사전-요구사항)
1. [의존성 설치](#1-의존성-설치)
2. [KITTI raw 데이터셋 다운로드](#2-kitti-raw-데이터셋-다운로드)
3. [캘리브레이션 행렬 `T_cam_imu` 추출](#3-캘리브레이션-행렬-t_cam_imu-추출)
4. [새 시퀀스용 YAML config 작성](#4-새-시퀀스용-yaml-config-작성)
5. [코드가 한 프레임에 무엇을 하나 (코드 워크스루)](#5-코드가-한-프레임에-무엇을-하나-코드-워크스루)
6. [빌드](#6-빌드)
7. [단일 시퀀스 실행 & 1차 검증](#7-단일-시퀀스-실행--1차-검증)
8. [결과 시각화](#8-결과-시각화)
9. [다중 시퀀스 일괄 테스트 (단일 날짜 → 전체 데이터셋)](#9-다중-시퀀스-일괄-테스트-단일-날짜--전체-데이터셋)
10. [결과 해석 & 트러블슈팅](#10-결과-해석--트러블슈팅)
11. [수정 시나리오 5선 — 어디 만지면 무엇이 바뀌는가](#11-수정-시나리오-5선--어디-만지면-무엇이-바뀌는가)
12. [Knob 카탈로그 (튜닝 가능 파라미터 전체 목록)](#12-knob-카탈로그-튜닝-가능-파라미터-전체-목록)
13. [부록 A. 디렉터리 레이아웃 권장형](#부록-a-디렉터리-레이아웃-권장형)
14. [부록 B. 명령어 치트시트](#부록-b-명령어-치트시트)

---

## 0. 사전 요구사항

| 항목 | 권장 사양 | 비고 |
|------|----------|------|
| OS | WSL2 Ubuntu 22.04 또는 네이티브 Linux | macOS도 가능하나 미검증 |
| 컴파일러 | gcc 10 이상 (C++20) | `gcc --version` |
| CMake | 3.16 이상 | `cmake --version` |
| 디스크 | 단일 drive 0.5~5 GB · 전체 KITTI raw ~180 GB | sync zip만 받으면 작아짐 |
| 메모리 | 8 GB 이상 | OpenCV 빌드시 `-j` 줄이기 |
| Python | 3.8 이상 + numpy/matplotlib | 결과 시각화용 |

본 문서의 절대경로는 모두 **WSL2 Ubuntu**를 기준으로 `/mnt/d/...` 형식을 사용한다.
네이티브 Linux 사용자는 `/mnt/d/02_research/...` 부분을 본인 경로(예: `~/work/...`)로
바꿔 읽으면 된다.

---

## 1. 의존성 설치

### 1.1 시스템 패키지

```bash
sudo apt update
sudo apt install -y \
    build-essential cmake git pkg-config \
    libeigen3-dev libopencv-dev libyaml-cpp-dev \
    python3 python3-pip python3-numpy python3-matplotlib
```

### 1.2 버전 점검

```bash
gcc --version              # 10.x 이상
cmake --version            # 3.16 이상
pkg-config --modversion eigen3       # 3.3 이상
pkg-config --modversion opencv4      # 4.x
pkg-config --modversion yaml-cpp     # 0.6 이상
python3 -c "import numpy, matplotlib; print(numpy.__version__, matplotlib.__version__)"
```

OpenCV가 `pkg-config opencv4`로 안 잡히면 `dpkg -l | grep libopencv` 로 패키지가
실제 설치됐는지 확인. Ubuntu 20.04 환경은 OpenCV가 4.2 라서 본 프로젝트가 쓰는
`cv::calcOpticalFlowPyrLK` / `cv::solvePnPRansac` 모두 정상 동작한다.

### 1.3 Eigen / yaml-cpp 만 별도 설치한 경우

`find_package` 가 실패하면 CMake에 힌트를 주면 된다.

```bash
cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DEigen3_DIR=/usr/share/eigen3/cmake \
  -Dyaml-cpp_DIR=/usr/lib/x86_64-linux-gnu/cmake/yaml-cpp
```

---

## 2. KITTI raw 데이터셋 다운로드

### 2.1 계정 등록

1. https://www.cvlibs.net/datasets/kitti/raw_data.php 접속.
2. 우측 상단 **"login/register"** 메뉴에서 이메일을 등록 (확인 메일 수신 후 활성화).
3. 로그인 상태에서만 다운로드 링크가 노출된다.

### 2.2 받아야 하는 파일 — 시퀀스 한 개당 2종

본 프로젝트는 **rectified, synced 스테레오** 와 **OXTS IMU/GPS**만 사용한다.
LiDAR(Velodyne), tracklets는 필요 없다.

| 파일 | 크기 | 본 프로젝트 사용 여부 |
|------|------|----------------------|
| `2011_09_26_calib.zip` | ~1 MB | ✅ **반드시** (시퀀스 묶음당 1회) |
| `2011_09_26_drive_XXXX_sync.zip` | 0.3~3 GB | ✅ 메인 데이터 |
| `2011_09_26_drive_XXXX_extract.zip` | 더 큼 | ❌ unsync 버전, 안 씀 |
| `2011_09_26_drive_XXXX_tracklets.zip` | 작음 | ❌ |
| `2011_09_26_drive_XXXX_velodyne.zip` | 큼 | ❌ |

> **주의**: `calib.zip`은 **날짜 단위**로 한 번만 받으면 된다. 같은 `2011_09_26`
> 폴더 아래의 모든 drive는 동일한 카메라/IMU 캘리브레이션을 공유한다.
> 다른 날짜(`2011_09_28`, `2011_09_29`, `2011_09_30`, `2011_10_03`)는 각각 따로 받아야 한다.

### 2.3 추천 시퀀스 (난이도 순)

| drive | 카테고리 | 길이 | 특이점 | 권장 용도 |
|-------|---------|------|--------|----------|
| 0001 | residential | 11 s | 짧고 단순, 출발 전 정지구간 있음 | 디버깅 (현재 default) |
| 0009 | residential | 47 s | 직진 + 가벼운 좌/우회전 | 기본 검증 |
| 0011 | residential | 23 s | 짧은 회전 시퀀스 | 회전 응답 확인 |
| 0014 | city | 31 s | 도심, 정지·출발 빈번 | 정지 구간 처리 |
| 0017 | city | 11 s | 매우 짧은 도심 | 빠른 회귀 테스트 |
| 0027 | residential | 1 m 28 s | 가속·감속 풍부 | EKF 발산 테스트 |
| 0028 | residential | 4 m 30 s | 긴 시퀀스, 여러 회전 | 누적 드리프트 측정 |
| 0036 | road | 1 m 35 s | 고속 직진 | 속도 추정 검증 |
| 0046 | residential | 27 s | 회전 풍부 | yaw drift 측정 |
| 0086 | city | 7 m 5 s | 도심 장기 | 종합 테스트 |

### 2.4 압축 해제 후 디렉터리 구조

```
data/kitti_raw/
└── 2011_09_26/
    ├── calib_cam_to_cam.txt         ← 2.2의 calib.zip 압축 해제 결과
    ├── calib_imu_to_velo.txt
    ├── calib_velo_to_cam.txt
    ├── 2011_09_26_drive_0001_sync/
    │   ├── image_00/
    │   │   ├── data/0000000000.png ... 0000000107.png
    │   │   └── timestamps.txt
    │   ├── image_01/
    │   ├── image_02/
    │   ├── image_03/
    │   └── oxts/
    │       ├── data/0000000000.txt ... 0000000107.txt
    │       ├── timestamps.txt
    │       └── dataformat.txt
    ├── 2011_09_26_drive_0009_sync/
    └── ...
```

검증 한 줄:

```bash
tree -L 4 /mnt/d/02_research/04_cpp_seg_msckf_vio/data/kitti_raw/2011_09_26/2011_09_26_drive_0001_sync
```

본 프로젝트의 KITTI 리더(`src/io/kitti_raw_reader.cpp`)는 **`image_00`/`image_01`
(grayscale)을 우선** 사용하고, 둘 다 없으면 `image_02`/`image_03`(color)로 fallback
한다. 일반적으로 grayscale이 노출/콘트라스트가 더 안정적이다.

### 2.5 다운로드 자동화는 의도적으로 하지 않는다

KITTI 라이선스(CC BY-NC-SA)는 학술 용도 비상업이며, robots.txt 와 약관상 **자동
다운로드 스크립트 배포는 권장되지 않는다**. 직접 브라우저로 받거나, `wget`을
쓰더라도 본인 책임 하에 사용한다.

---

## 3. 캘리브레이션 행렬 `T_cam_imu` 추출

본 프로젝트의 YAML config는 `cam0`/`cam1` 각각에 대해 13개 값
(width, height, fx, fy, cx, cy, k1, k2, p1, p2, T_cam_imu 4×4)을 요구한다.
이 절은 KITTI 원본 캘리브레이션 파일 3종으로부터 그 값들을 **손으로** 뽑는 절차다.

### 3.1 KITTI 캘리브레이션 파일 구조

#### `calib_cam_to_cam.txt`

```
calib_time: ...
corner_dist: ...
S_00: 1392 512                 ← unrectified 해상도
K_00: fx 0 cx 0 fy cy 0 0 1    ← unrectified intrinsics (3x3, 9개 숫자)
D_00: k1 k2 p1 p2 k3           ← 왜곡 계수
R_00: 3x3 회전
T_00: 3x1 평행이동
S_rect_00: 1242 375            ← rectified 해상도 ← 우리가 쓰는 값
R_rect_00: 3x3 stereo rect 회전
P_rect_00: 3x4 사영 행렬       ← 우리가 쓰는 fx/fy/cx/cy
S_01: ... K_01: ... (cam1 동일 구조)
S_rect_01: ... R_rect_01: ... P_rect_01: ...
S_02 ... P_rect_02 (color cam2)
S_03 ... P_rect_03 (color cam3)
```

`P_rect_00`은 다음 형태의 3×4 행렬이다.

```
P_rect_00 =  [ fx   0   cx   0  ]
             [  0   fy  cy   0  ]
             [  0    0   1   0  ]
```

`P_rect_01`은 stereo right cam이라 마지막 열에 baseline 항이 들어간다.

```
P_rect_01 =  [ fx   0   cx   -fx*b ]
             [  0   fy  cy     0   ]
             [  0    0   1     0   ]
```

여기서 `b`는 cam0→cam1 베이스라인(미터). cam0/cam1 모두 `fx, fy, cx, cy`는
동일하다 (rectified 후 같은 가상 카메라). 우리 YAML에서는 둘 다 **같은 fx/fy/cx/cy**
를 적고, 베이스라인 차이는 `T_cam_imu`의 `tx` 항으로 표현된다.

#### `calib_velo_to_cam.txt`

```
R: 3x3 (Velodyne 좌표계 → cam0 좌표계 회전)
T: 3x1 (평행이동)
```

→ 4×4 동차행렬 `T_cam0_velo`.

#### `calib_imu_to_velo.txt`

```
R: 3x3 (IMU → Velodyne 회전)
T: 3x1 (평행이동)
```

→ 4×4 동차행렬 `T_velo_imu`.

### 3.2 chain rule

본 프로젝트가 요구하는 cam0 좌표계 → IMU 좌표계 변환은 다음 곱셈이다.

```
T_cam0_imu = R_rect_00 ⊗ T_cam0_velo · T_velo_imu
```

여기서 `R_rect_00`은 4×4로 확장한 동차 회전 (rect 좌표계로의 보정)이다.
cam1의 경우 cam0→cam1 베이스라인 평행이동(`T_cam1_cam0`)을 추가로 곱한다.

```
T_cam1_imu = T_cam1_cam0 · T_cam0_imu
T_cam1_cam0 = [ I  | (-baseline_x, 0, 0) ; 0 0 0 1 ]
```

> 현재 `config/kitti_raw_2011_09_26_drive_0001.yaml`을 보면 cam1의 `T_cam_imu`가
> cam0과 거의 동일하지만 마지막 열의 x 성분만 약 0.537 m 차이 난다. 이것이
> KITTI grayscale stereo 베이스라인이다.

### 3.3 Python 변환 스크립트 예제

이 스크립트는 본 프로젝트에 포함시키지 않는다 (KITTI 약관 회피 + 단발성 사용).
필요할 때 임시 파일에 저장해서 실행한 뒤 출력값을 YAML에 복붙하면 된다.

```python
# extract_calib.py — KITTI raw → YAML T_cam_imu 추출 (임시 사용 예)
from pathlib import Path
import numpy as np

CALIB_DIR = Path("/mnt/d/02_research/04_cpp_seg_msckf_vio/data/kitti_raw/2011_09_26")

def parse_kv(path):
    out = {}
    for line in path.read_text().splitlines():
        if ":" not in line:
            continue
        k, v = line.split(":", 1)
        out[k.strip()] = np.fromstring(v, sep=" ")
    return out

cam = parse_kv(CALIB_DIR / "calib_cam_to_cam.txt")
v2c = parse_kv(CALIB_DIR / "calib_velo_to_cam.txt")
i2v = parse_kv(CALIB_DIR / "calib_imu_to_velo.txt")

def homog(R, t):
    T = np.eye(4)
    T[:3, :3] = R.reshape(3, 3)
    T[:3, 3]  = t.reshape(3)
    return T

T_cam_velo = homog(v2c["R"], v2c["T"])      # Velo → cam0 unrect
T_velo_imu = homog(i2v["R"], i2v["T"])      # IMU → Velo

R_rect_00 = np.eye(4)
R_rect_00[:3, :3] = cam["R_rect_00"].reshape(3, 3)

T_cam0_imu = R_rect_00 @ T_cam_velo @ T_velo_imu

# Stereo right baseline: cam1 - cam0 (rectified)
P_rect_00 = cam["P_rect_00"].reshape(3, 4)
P_rect_01 = cam["P_rect_01"].reshape(3, 4)
fx = P_rect_00[0, 0]
fy = P_rect_00[1, 1]
cx = P_rect_00[0, 2]
cy = P_rect_00[1, 2]
baseline_x = -P_rect_01[0, 3] / fx          # m, positive
T_cam1_cam0 = np.eye(4)
T_cam1_cam0[0, 3] = -baseline_x

T_cam1_imu = T_cam1_cam0 @ T_cam0_imu

w, h = cam["S_rect_00"].astype(int)

print(f"# image size : {w} x {h}")
print(f"# fx fy cx cy: {fx:.4f} {fy:.4f} {cx:.4f} {cy:.4f}")
print(f"# baseline_x : {baseline_x:.6f}")

def yaml_block(name, T):
    print(f"{name}_T_cam_imu:")
    for row in T:
        print(f"  - [{row[0]: .12f}, {row[1]: .12f}, {row[2]: .12f}, {row[3]: .12f}]")

yaml_block("cam0", T_cam0_imu)
yaml_block("cam1", T_cam1_imu)
```

### 3.4 검증

위 스크립트를 `2011_09_26` 캘리브레이션에 돌리면 출력이 기존
`config/kitti_raw_2011_09_26_drive_0001.yaml` 의 값과 ±1e-6 안쪽으로 일치해야 한다.
**일치하지 않으면 행렬 곱 순서나 R_rect 적용 위치를 의심하라.** 가장 흔한 실수:

- `T_cam_velo` 와 `T_velo_cam` 을 헷갈림 (KITTI는 velo→cam 방향).
- `R_rect_00` 적용을 빠뜨림 → cam0 unrect 좌표계가 되어 fx/fy 값과 안 맞음.
- baseline 부호 (P_rect_01의 (0,3) 항이 음수임을 잊음).

---

## 4. 새 시퀀스용 YAML config 작성

### 4.1 템플릿 복사

```bash
cd /mnt/d/02_research/04_cpp_seg_msckf_vio
cp config/kitti_raw_template.yaml config/kitti_raw_2011_09_26_drive_0009.yaml
```

### 4.2 채워야 할 필드 (전체)

```yaml
dataset_type: "kitti_raw"
dataset: "/mnt/d/02_research/04_cpp_seg_msckf_vio/data/kitti_raw/2011_09_26/2011_09_26_drive_0009_sync"
output:  "/mnt/d/02_research/04_cpp_seg_msckf_vio/results/kitti_raw_2011_09_26_drive_0009"
max_frames: 0           # 0 = 전체. 디버깅 시 50으로 시작 권장.

init:
  mode: "stationary_imu"           # 현재 지원되는 유일한 모드
  static_window_sec: 1.0           # 첫 N초의 IMU 평균으로 초기화
  gravity_norm: 9.81

imu_noise:
  gyro:       1.6968e-4            # KITTI OXTS RT3003 기준으로도 무난
  accel:      2.0000e-3
  gyro_walk:  1.9393e-5
  accel_walk: 3.0000e-3

ekf:
  sigma_vo: 0.10                   # VO 위치 측정 잡음 표준편차 [m]

cam0:
  width: 1242
  height: 375
  fx: 721.5377
  fy: 721.5377
  cx: 609.5593
  cy: 172.8540
  k1: 0.0
  k2: 0.0                          # KITTI rect 이미지는 왜곡 0
  p1: 0.0
  p2: 0.0
  T_cam_imu:                       # 3.3 스크립트로 뽑은 4x4
    - [ ..., ..., ..., ... ]
    - [ ..., ..., ..., ... ]
    - [ ..., ..., ..., ... ]
    - [ 0.0,  0.0,  0.0,  1.0 ]

cam1:
  width: 1242
  height: 375
  fx: 721.5377
  fy: 721.5377
  cx: 609.5593
  cy: 172.8540
  k1: 0.0
  k2: 0.0
  p1: 0.0
  p2: 0.0
  T_cam_imu:
    - [ ..., ..., ..., ... ]       # cam0 의 (0,3)에서 baseline_x 만큼 빠진 값
    - [ ..., ..., ..., ... ]
    - [ ..., ..., ..., ... ]
    - [ 0.0,  0.0,  0.0,  1.0 ]
```

### 4.3 같은 날짜의 다른 drive는 캘리브레이션 재사용

`2011_09_26_drive_0001_sync`, `..._0009_sync`, `..._0011_sync` 등은 모두 같은
`2011_09_26` 캘리브레이션을 공유한다. → cam0/cam1의 13개 값 (특히 `T_cam_imu`)을
**그대로 복사**하면 된다. 바뀌는 것은 `dataset` / `output` 경로뿐이다.

### 4.4 파라미터 튜닝 가이드 (간단)

| 파라미터 | 기본값 | 줄였을 때 | 키웠을 때 | 비고 |
|---------|--------|-----------|-----------|------|
| `ekf.sigma_vo` | 0.10 | VO 신뢰 ↑, IMU 영향 ↓ → 발산 위험 | VO 무시 → IMU 단독 적분 | 0.05~0.20 권장 |
| `init.static_window_sec` | 1.0 | 빠른 초기화 (정지 구간 짧을 때) | 더 정확한 gyro bias | 차량이 즉시 출발하는 시퀀스는 0.3~0.5 |
| `imu_noise.gyro` | 1.6968e-4 | EKF가 gyro를 더 신뢰 | gyro에 둔감 | 데이터시트 √Hz 단위 |
| `max_frames` | 0 | (N) 처음 N프레임만 | (0) 전체 | 디버깅 50, 검증 0 |

(전체 knob 목록은 §12 카탈로그 참조)

### 4.5 흔한 실수 체크리스트

- [ ] 경로의 `\` 와 `/` 혼재 (WSL은 항상 `/`)
- [ ] YAML 들여쓰기 — **탭 금지, 스페이스만**
- [ ] `T_cam_imu` 의 마지막 행이 `[0, 0, 0, 1]` 인가
- [ ] cam0과 cam1에 둘 다 `T_cam_imu` 가 있는가 (둘 다 필요)
- [ ] `dataset` 경로가 **drive 디렉터리** 까지 (그 아래 `image_00/` 가 보여야 함)
- [ ] `output` 디렉터리가 부모까지 미리 존재할 필요는 없음 (실행 시 자동 생성)
- [ ] grayscale 이미지가 `image_00/data/*.png` 로 풀려 있나 (color만 풀린 경우 자동 fallback 되지만 grayscale 권장)

---

## 5. 코드가 한 프레임에 무엇을 하나 (코드 워크스루)

> 이 절은 **C++ 입문자**가 코드를 직접 열지 않고도 시스템 동작 원리를 잡을 수 있도록
> 짧은 발췌 + 줄별 주석 위주로 작성한다. 모든 코드 위치는 `파일:라인` 형식.

### 5.1 30초 요약 + 데이터 흐름

```
[data/kitti_raw/...]
    │
    ├── image_00, image_01 ──→ StereoTracker.process()
    │                              │ KLT + ORB + 삼각측량 + PnP
    │                              ↓
    │                          pose.t (cam0 위치, VO local frame)
    │                              ↓
    └── oxts/data ─────────→ ImuPropagator.propagate()  (RK4)
                                   ↓                      ↓
                              imu_.p, imu_.v, imu_.R   LcEkf.update_vo(p_world_cam0)
                                   ↓                      ↓
                             매 프레임 traj_est ←── EKF 보정 후 imu_.p
                                   ↓
                          metrics.txt
                          (ATE = Umeyama align(traj_est, traj_gt))
```

핵심은 세 객체:

| 객체 | 정의 위치 | 역할 |
|------|----------|------|
| `KittiRawReader` | `src/io/kitti_raw_reader.cpp` | 디스크에서 IMU/CAM/GT 읽어 vector로 보유 |
| `ImuPropagator` | `include/ekf/imu_propagator.hpp` | 15-state 상태 + 15×15 공분산을 IMU로 전파 |
| `LcEkf` | `include/ekf/lc_ekf.hpp` | 위 propagator를 wrap → VO 측정값으로 보정 |
| `StereoTracker` | `include/frontend/stereo_tracker.hpp` | KLT + Stereo + PnP → cam0 6-DoF pose |

### 5.2 데이터 구조 — `include/core/types.hpp:1-37`

```cpp
struct ImuData {
    double timestamp;       // seconds
    Eigen::Vector3d gyro;   // rad/s, body (IMU) frame
    Eigen::Vector3d accel;  // m/s^2, body (IMU) frame
};

struct CamData {
    double timestamp;
    std::string img_l;      // 좌측 png 절대경로
    std::string img_r;      // 우측 png 절대경로
};

struct GtData {
    double timestamp;
    Eigen::Vector3d p;      // world frame 위치 [m]
    Eigen::Quaterniond q;   // world ← body 쿼터니언
    Eigen::Vector3d v;      // world frame 속도 [m/s]
};

struct CameraParams {
    double fx, fy, cx, cy;          // intrinsics [px]
    double k1, k2, p1, p2;          // radtan 왜곡
    Eigen::Matrix4d T_cam_imu;      // T_CI: IMU → cam (4×4)
    int width, height;
};
```

KITTI 리더는 OXTS .txt 한 줄(30개 값)에서:
- `values[14..16]` (forward/left/up accel) → `ImuData.accel`
- `values[20..22]` (forward/left/up gyro) → `ImuData.gyro`
- `lat/lon/alt` → Mercator 투영해 `GtData.p`
- `roll/pitch/yaw` → `GtData.q`

(`src/io/kitti_raw_reader.cpp:170-205`)

### 5.3 EKF가 추적하는 15차원 상태 — `imu_propagator.hpp:18-27`

```cpp
class ImuPropagator {
public:
    // Nominal (best-estimate) state
    Eigen::Vector3d p;   // world frame 위치 [m]
    Eigen::Vector3d v;   // world frame 속도 [m/s]
    Eigen::Matrix3d R;   // world ← body (IMU) 자세 (3×3 회전)
    Eigen::Vector3d bg;  // gyro bias  [rad/s]
    Eigen::Vector3d ba;  // accel bias [m/s^2]

    // Error-state covariance  (15×15)
    using Mat15 = Eigen::Matrix<double, 15, 15>;
    Mat15 P;
    ...
};
```

**왜 R은 9개 값(3×3)인데 error-state δθ는 3차원?**
SO(3)는 3차원 매니폴드라 접공간(tangent space)이 3차원이다. 회전 오차는
"작은 각도 벡터" 3개로 표현하고, nominal R 에는 `R ← R · Exp(δθ)` 합성으로 적용한다.
이래서 공분산 P는 (15 = 3+3+3+3+3) × 15 행렬이고, R 은 P 안에 직접 들어가지 않는다.

**P 초기값** — `imu_propagator.cpp:18-24`:

```cpp
P.setZero();
P.block<3,3>(0,0)  = 1e-4 * Matrix3d::Identity(); // position
P.block<3,3>(3,3)  = 1e-2 * Matrix3d::Identity(); // velocity
P.block<3,3>(6,6)  = 1e-4 * Matrix3d::Identity(); // attitude
P.block<3,3>(9,9)  = 1e-6 * Matrix3d::Identity(); // gyro bias
P.block<3,3>(12,12)= 1e-4 * Matrix3d::Identity(); // accel bias
```

(15×15 P를 5개의 3×3 대각 블록으로 나눠 초기 분산 설정. attitude에 1e-4 = 약 ±1° 표준편차)

### 5.4 메인 루프 줄별 해설 — `apps/run_euroc.cpp:275-314`

```cpp
for (const auto& cam : cam_data) {                                    // ← 매 cam 프레임
    if (max_frames > 0 && frame_count >= max_frames) break;

    // (1) IMU를 현재 cam timestamp까지 모두 전파
    while (imu_idx < imu_data.size() && imu_data[imu_idx].timestamp <= cam.timestamp) {
        const auto& imu = imu_data[imu_idx++];
        ekf.propagate(imu.timestamp, imu.gyro, imu.accel);            // ← (1a) RK4 + 공분산 전파
    }

    // (2) 좌/우 png 로드
    const cv::Mat img_l = cv::imread(cam.img_l, cv::IMREAD_COLOR);
    const cv::Mat img_r = cv::imread(cam.img_r, cv::IMREAD_COLOR);
    if (img_l.empty() || img_r.empty()) { ... continue; }

    // (3) Stereo VO: KLT 추적 + PnP → cam0 pose (VO local frame)
    const auto pose = tracker.process(img_l, img_r);

    // (4) VO local → IMU world 좌표계로 회전+평행이동
    const Eigen::Vector3d p_world_cam = R_WC0 * pose.t + t_WC0;

    // (5) 첫 프레임 제외하고 EKF 측정 업데이트
    if (pose.valid && frame_count > 0) {
        ekf.update_vo(p_world_cam);                                    // ← (5a) Kalman 업데이트
    }

    // (6) 현재 위치를 trajectory에 기록
    traj_est.push_back(ekf.position());
    traj_ts.push_back(cam.timestamp);

    // (7) 같은 timestamp의 GT를 nearest-neighbor로 뽑아 저장
    Eigen::Vector3d gt_p = Eigen::Vector3d::Zero();
    if (sample_gt_position(gt_data, cam.timestamp, gt_p)) {
        traj_gt.push_back(gt_p);
    }

    ++frame_count;
}
```

요약 표:

| 단계 | 한 줄 호출 | 어떤 객체 | 무엇이 바뀌나 |
|------|-----------|----------|---------------|
| (1a) IMU 전파 | `ekf.propagate(t, gyro, accel)` (l.280) | `imu_propagator` | `p, v, R, P` 갱신 |
| (2) 이미지 로드 | `cv::imread(...)` (l.283-284) | OpenCV | 메모리에 cv::Mat |
| (3) Stereo VO | `tracker.process(img_l, img_r)` (l.290) | `StereoTracker` | `pose.R, pose.t` 출력 |
| (4) world 정렬 | `R_WC0 * pose.t + t_WC0` (l.291) | (수식) | VO local → world |
| (5a) EKF 업데이트 | `ekf.update_vo(p_world_cam)` (l.293) | `lc_ekf` | `p, v, R, bg, ba, P` 보정 |
| (6) 기록 | `traj_est.push_back(ekf.position())` (l.296) | `std::vector` | trajectory 누적 |

### 5.5 행렬 계산 도감 (한 시스템에 등장하는 모든 행렬)

#### A. 캘리브레이션 행렬

**`T_cam_imu` (4×4) — `apps/run_euroc.cpp:54-62`**

```cpp
const auto rows = node["T_cam_imu"];
for (int r = 0; r < 4; ++r) {
    for (int cc = 0; cc < 4; ++cc) {
        c.T_cam_imu(r, cc) = rows[r][cc].as<double>();   // yaml에서 4×4 그대로 로드
    }
}
```

→ IMU body 좌표계의 점을 cam 좌표계로 옮기는 변환. yaml의 16개 값이 그대로 들어옴.

**`p_IC` (3) — `lc_ekf.cpp:16-17`**

```cpp
Eigen::Matrix4d T_imu_cam0 = T_cam0_imu.inverse();   // 역변환: cam0 → IMU
p_IC_ = T_imu_cam0.block<3,1>(0,3);                  // IMU 좌표계에서 본 cam0 원점
```

**lever arm**. EKF 측정 모델에서 "IMU 위치"와 "cam0 위치"를 잇는 벡터.

**`R_WC0`, `t_WC0` — `apps/run_euroc.cpp:255-257`**

```cpp
const Eigen::Matrix4d T_IC = cam0.T_cam_imu.inverse();           // cam0 → IMU
const Eigen::Matrix3d R_WC0 = init.R0 * T_IC.block<3,3>(0,0);    // world ← cam0 (첫 프레임)
const Eigen::Vector3d t_WC0 = init.R0 * T_IC.block<3,1>(0,3) + init.p0;
```

VO는 첫 cam0 프레임을 원점 (0,0,0) 으로 출력한다. 그 좌표를 IMU world와 정합하기
위한 회전 + 평행이동.

#### B. IMU 전파 관련 행렬

**`R` (3×3) — `imu_propagator.cpp:64-65` (RK4 자세 적분)**

```cpp
R = R * so3::Exp(dt * gyro_c);                              // SO(3) 합성
R = Eigen::Quaterniond(R).normalized().toRotationMatrix();  // 정규화 (수치오차 방지)
```

**`F` (15×15) — `imu_propagator.cpp:93-99`** — error-state 전이 행렬

```cpp
Mat15 F = Mat15::Zero();
F.block<3,3>(0,3)  =  Matrix3d::Identity();          // δṗ = δv
F.block<3,3>(3,6)  = -R * so3::hat(accel_c);         // δv̇ ← -R[a×]δθ
F.block<3,3>(3,12) = -R;                             // δv̇ ← -R·δba
F.block<3,3>(6,6)  = -so3::hat(gyro_c);              // δθ̇ ← -[ω×]δθ
F.block<3,3>(6,9)  = -R;                             // δθ̇ ← -R·δbg
```

**`G` (15×12) — `imu_propagator.cpp:106-110`** — 잡음 → 상태 매핑

```cpp
Eigen::Matrix<double, 15, 12> G = Eigen::Matrix<double, 15, 12>::Zero();
G.block<3,3>(3,3)  = R;                              // accel noise → δv
G.block<3,3>(6,0)  = R;                              // gyro  noise → δθ
G.block<3,3>(9,6)  = Matrix3d::Identity();           // gyro  walk  → δbg
G.block<3,3>(12,9) = Matrix3d::Identity();           // accel walk  → δba
```

**`Qc` (12×12) — `imu_propagator.cpp:115-119`** — 연속시간 잡음 밀도 (대각)

```cpp
Qc.block<3,3>(0,0) = sg*sg  * I;   // gyro noise
Qc.block<3,3>(3,3) = sa*sa  * I;   // accel noise
Qc.block<3,3>(6,6) = sbg*sbg* I;   // gyro walk
Qc.block<3,3>(9,9) = sba*sba* I;   // accel walk
```

**`Φ` (15×15) — `imu_propagator.cpp:102`** — 이산화된 상태 전이

```cpp
Mat15 Phi = Mat15::Identity() + F * dt;   // 1차 Euler
```

**`Qd` (15×15) — `imu_propagator.cpp:122`** — 이산화된 잡음 공분산

```cpp
Mat15 Qd = (G * Qc * G.transpose()) * dt;
```

**`P` 갱신 — `imu_propagator.cpp:124`**

```cpp
P = Phi * P * Phi.transpose() + Qd;       // 표준 EKF 공분산 전파
```

#### C. EKF 측정 업데이트 — `lc_ekf.cpp`

**측정 모델**: `h(x) = R_wI · p_IC + p_wI`  (IMU world의 cam0 위치)

**`innov` (3) — l.33**:

```cpp
Vector3d innov = p_world_cam0 - h;        // 관측값 - 예측값
```

**`H` (3×15) — l.41-43**:

```cpp
Eigen::Matrix<double, 3, 15> H = ...::Zero();
H.block<3,3>(0,0) = Matrix3d::Identity();          // ∂h/∂δp = I
H.block<3,3>(0,6) = so3::hat(R_wI * p_IC_);       // ∂h/∂δθ
```

(나머지 9개 열은 0 — VO는 속도/바이어스를 직접 보지 못함)

**`R_noise` (3×3) — l.46**:

```cpp
Eigen::Matrix3d R_noise = (sigma_vo_*sigma_vo_) * Matrix3d::Identity();
```

(yaml의 `sigma_vo` 값이 여기로)

**`S` (3×3) — l.47**: innovation covariance

```cpp
Eigen::Matrix3d S = H * imu_.P * H.transpose() + R_noise;
```

**`K` (15×3) — l.50**: Kalman gain

```cpp
Eigen::Matrix<double, 15, 3> K = imu_.P * H.transpose() * S.inverse();
```

**`dx` (15) — l.53**: error-state correction

```cpp
Vec15 dx = K * innov;
```

**상태 보정 적용 — l.71-76**:

```cpp
imu_.p  += dx.segment<3>(0);                              // δp
imu_.v  += dx.segment<3>(3);                              // δv
imu_.R   = imu_.R * so3::Exp(dx.segment<3>(6));           // δθ (지수 합성)
imu_.R   = Eigen::Quaterniond(imu_.R).normalized().toRotationMatrix();
imu_.bg += dx.segment<3>(9);                              // δbg
imu_.ba += dx.segment<3>(12);                             // δba
```

**Joseph form 공분산 업데이트 — l.59-61** (수치 안정성):

```cpp
Mat15 I_KH = Mat15::Identity() - K * H;
imu_.P = I_KH * imu_.P * I_KH.transpose() + K * R_noise * K.transpose();
```

#### D. ATE 정합 — `apps/run_euroc.cpp:180-199`

**`T_gt_est` (4×4) — Umeyama**:

```cpp
const Eigen::Matrix4d T_gt_est = Eigen::umeyama(est_mat, gt_mat, false);
//                                         (estimated, gt, with_scale=false)
double sum2 = 0.0;
for (size_t i = 0; i < est.size(); ++i) {
    Eigen::Vector4d p_est(est[i].x(), est[i].y(), est[i].z(), 1.0);
    const Eigen::Vector3d aligned = (T_gt_est * p_est).head<3>();
    sum2 += (aligned - gt[i]).squaredNorm();
}
return std::sqrt(sum2 / static_cast<double>(est.size()));
```

→ 추정 trajectory를 GT에 강체정합(scale 고정) 후 잔차 RMSE.

#### E. 스테레오 rectify 결과 — `stereo_tracker.cpp:57-72`

```cpp
cv::stereoRectify(K0_, dist0_, K1_, dist1_, img_size,
                  R10, t10,
                  R0_rect_, R1_rect_, P0_, P1_, Q,
                  cv::CALIB_ZERO_DISPARITY, -1);
fx_rect_       = P0_.at<double>(0, 0);
baseline_rect_ = -P1_.at<double>(0, 3) / fx_rect_;
K_rect_        = P0_(cv::Rect(0, 0, 3, 3)).clone();
```

KITTI는 이미 rect 이미지지만 OpenCV의 P0_/P1_/K_rect_/baseline_rect_ 변수에 값을
넣어 PnP에서 사용한다.

### 5.6 GT vs estimate 비교 — ATE RMSE

`apps/run_euroc.cpp:321-329` 가 메인 루프 끝에서 한 번 호출:

```cpp
const double ate = compute_ate(traj_est, traj_gt);
std::cout << "ATE RMSE         : " << ate * 100.0 << " cm\n";
```

그리고 `apps/run_euroc.cpp:332-341` 에서 `metrics.txt` 에 기록:

```cpp
std::ofstream metrics(output_dir + "/metrics.txt");
metrics << "dataset_type: " << dataset_type << "\n";
metrics << "frames: " << frame_count << "\n";
metrics << "ate_rmse_m: " << ate << "\n";
metrics << "ate_rmse_cm: " << ate * 100.0 << "\n";
```

**같은 알고리즘이 `tools/plot_trajectory.py`에도 구현**되어 있으므로
두 결과가 ±5% 안에서 일치해야 한다 (다르면 TUM 파일 형식 문제).

### 5.7 자주 헷갈리는 5가지 좌표계 노트

1. **`R_wI`**: world ← IMU body. 즉 `v_world = R_wI · v_body`. 코드의
   `imu_.R` 이 이것 (`imu_propagator.hpp:21` 코멘트).
2. **`T_cam_imu`**: yaml 키 이름은 "cam_imu" 인데 의미는 **IMU 좌표계의 점을 cam
   좌표계로** 옮기는 행렬 (T_C←I). 헷갈리면 `types.hpp:28` 의 코멘트 참조.
3. **error-state δθ vs nominal R**: δθ를 R에 직접 더하지 않는다. 무조건
   `R ← R · Exp(δθ)` 합성 (`lc_ekf.cpp:73`).
4. **world frame 정의**: 첫 IMU 샘플 시점을 anchor. `init.p0 = (0,0,0)` 이고
   `init.R0` 은 정지 시 가속도 평균이 (0,0,9.81)을 가리키도록 회전을 맞춘 것.
5. **VO local → IMU world**: VO는 첫 cam0 프레임을 원점으로 잡으므로 EKF world와
   다르다. `R_WC0 · pose.t + t_WC0` 변환을 거쳐야 비교 가능
   (`run_euroc.cpp:255-261, 291`).

---

## 6. 빌드

### 6.1 표준 시퀀스 (Release)

```bash
cd /mnt/d/02_research/04_cpp_seg_msckf_vio
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
```

성공하면 `build/run_vio` 와 `build/run_euroc` 두 바이너리가 생긴다 (둘 다
`apps/run_euroc.cpp` 를 같은 옵션으로 링크한 것).

### 6.2 디버깅 빌드 (필요할 때만)

```bash
cd build
cmake .. -DCMAKE_BUILD_TYPE=Debug
make -j$(nproc)
gdb --args ./run_vio ../config/kitti_raw_2011_09_26_drive_0001.yaml
```

| 빌드 모드 | 속도 | 디버그 정보 | 용도 |
|----------|------|-------------|------|
| Release  | 빠름 | 없음 | 시퀀스 실행 |
| Debug    | 5~10× 느림 | 풀 | 크래시·EKF 발산 추적 |

### 6.3 빌드 실패 시 점검

| 증상 | 원인 | 해결 |
|------|------|------|
| `Could NOT find Eigen3` | apt 패키지 미설치 | `sudo apt install libeigen3-dev` |
| `Could NOT find OpenCV` | 4.x 미설치 | `sudo apt install libopencv-dev` 또는 `-DOpenCV_DIR=...` |
| `Could NOT find yaml-cpp` | 미설치 | `sudo apt install libyaml-cpp-dev` |
| `error: 'std::filesystem' has not been declared` | gcc 7 이하 | gcc 10+ 또는 `update-alternatives` |
| `nullptr_t` 관련 컴파일 에러 | C++20 미지원 | gcc 10+ 사용, `-std=gnu++20` 강제 |
| 링크 실패 `undefined reference to cv::...` | OpenCV 라이브러리 분산 설치 | `pkg-config opencv4 --libs` 출력 확인 |

---

## 7. 단일 시퀀스 실행 & 1차 검증

### 7.1 실행

```bash
cd /mnt/d/02_research/04_cpp_seg_msckf_vio/build
./run_vio ../config/kitti_raw_2011_09_26_drive_0001.yaml
```

> 바이너리 이름은 `run_vio` 이지만 KITTI/EuRoC 양쪽 모두 받는다. `run_euroc` 도
> 동일한 동작이다.

### 7.2 정상 stdout 흐름 (drive 0001 기준)

```
[KITTI raw] IMU=108  CAM=108  GT=108
Init window samples=11  mean gyro=-1.07e-05  ...   mean accel= 0.0028  -0.080  9.804
Init gravity= 0  0  -9.81  |g|=9.81
Dataset=kitti_raw  GT=yes  first_cam_t=46945.96779
Init cam0_world p=  -0.314  0.719  -1.089
... (108 프레임이라 100단위 진행 로그는 1번만 출력)
Frame 100  t=46955.93...  imu_p= 36.2  -2.1  -0.8  tracked=98  [0.42s]

========================================
Frames processed : 108
ATE RMSE         : 156.3 cm
Output           : /mnt/d/.../results/kitti_raw_2011_09_26_drive_0001
========================================
```

### 7.3 확인 포인트

| 라인 | 정상값 (KITTI) | 비정상이면 의심 |
|------|---------------|-----------------|
| `[KITTI raw] IMU=N CAM=N GT=N` | 셋 다 같은 N | 디렉터리 구조 깨짐 |
| `Init window samples=` | 1초 동안 OXTS가 보통 10~11샘플 | 0 이면 timestamp 파싱 실패 |
| `mean accel=` | (≈0, ≈0, ≈9.8) | (9.8, 0, 0) 식으로 축이 다르면 IMU 좌표계 의심 |
| `Init gravity=` | (0, 0, -9.81) | 항상 같음. 다르면 코드 문제 |
| `tracked=` | 50 이상 | 10 미만으로 떨어지면 VO 곧 실패 |
| `ATE RMSE` | 50~500 cm | 단위가 m면 단위 표시 확인. 발산 시 1000+ |

### 7.4 첫 디버깅 산출물

`output` 디렉터리에 다음 파일들이 생긴다.

```
results/kitti_raw_2011_09_26_drive_0001/
├── trajectory_tum.txt   # 첫 줄: 헤더 / 둘째 줄부터 추정 trajectory
├── gt_tum.txt           # GT trajectory
├── metrics.txt          # ATE 요약
└── (있으면) raw_l.png, rect_l.png, rect_sidebyside.png   # 첫 스테레오 페어 검증용
```

`raw_*.png` 와 `rect_*.png` 가 있는 경우 (`StereoTracker` 내부에서 첫 프레임만
저장):

- `raw_l.png` / `raw_r.png`: 원본 입력 (KITTI는 이미 rectified라 raw==rect)
- `rect_sidebyside.png`: 좌/우를 가로로 붙인 페어. 스캔라인 맞추기 시각 확인.

**눈으로 볼 것**: 좌우 이미지의 같은 객체가 **같은 y 좌표** 위에 있어야 한다.
어긋나면 stereo rect 적용에 문제가 있다.

### 7.5 max_frames 로 빠른 회귀 테스트

YAML의 `max_frames: 50` 으로 두면 50프레임만 돌고 끝난다. config 변경 후 바로
크래시 여부만 확인할 때 사용.

---

## 8. 결과 시각화

### 8.1 명령

```bash
cd /mnt/d/02_research/04_cpp_seg_msckf_vio
python3 tools/plot_trajectory.py results/kitti_raw_2011_09_26_drive_0001/
```

### 8.2 출력

`results/.../trajectory_plot.png` 가 생성된다 (150 DPI). 두 패널 구성:

- **왼쪽 (XY top-down)**: 추정 trajectory(파란색)와 GT(초록색). 시작점(원),
  종점(사각형) 마커 포함.
- **오른쪽 (XYZ vs time)**: 시간축 위에 X/Y/Z 세 라인. 발산 진단용.

### 8.2.1 Rerun 인터랙티브 시각화

Rerun C++ SDK로 `.rrd` 기록을 만들고 Windows PowerShell에서 viewer로 여는 절차는
별도 문서에 정리했다:

```text
docs/guides/rerun_cpp_visualization_guide.md
```

핵심 실행 흐름은 WSL에서 `build_rerun/run_vio --rerun-save ...`로 기록 파일을 만들고,
Windows PowerShell에서 `python -m rerun_cli ...\vio_rerun.rrd`로 여는 방식이다.

### 8.3 발산 패턴 진단표

| 그래프 양상 | 가능성 |
|------------|--------|
| Z축이 단조증가/감소로 폭주 | 중력 방향 추정 실패 (`Init accel`이 비정상) |
| XY가 GT와 회전된 채 평행이동 | 초기 yaw 정합 실패 (현재 LC-EKF는 yaw 미보정 → 정상 한계) |
| 중간부터 갑자기 점프 | VO outlier가 EKF에 들어옴 → `sigma_vo` 키워보기 |
| 처음부터 발산 | static window 안에 차량이 출발한 것. `static_window_sec` 줄이거나 정지 시퀀스 사용 |
| GT와 거의 일치하다가 끝부분만 어긋남 | LC-EKF의 누적 드리프트 한계 (정상) |

---

## 9. 다중 시퀀스 일괄 테스트 (단일 날짜 → 전체 데이터셋)

### 9.1 시퀀스별 config 일괄 생성 (같은 날짜)

같은 `2011_09_26` 날짜의 drive를 여러 개 돌릴 때:

```bash
cd /mnt/d/02_research/04_cpp_seg_msckf_vio

BASE=config/kitti_raw_2011_09_26_drive_0001.yaml
for drv in 0009 0011 0014 0017 0027 0028 0036 0046 0086; do
    NEW=config/kitti_raw_2011_09_26_drive_${drv}.yaml
    sed \
      -e "s|drive_0001_sync|drive_${drv}_sync|g" \
      -e "s|kitti_raw_2011_09_26_drive_0001|kitti_raw_2011_09_26_drive_${drv}|g" \
      "$BASE" > "$NEW"
    echo "wrote $NEW"
done
```

> sed 치환은 `dataset:` 과 `output:` 경로가 동일한 패턴을 쓰고 있을 때만 안전하다.
> 작성 후 `diff config/kitti_raw_2011_09_26_drive_000{1,9}.yaml` 로 변경된 라인이
> 정확히 그 두 줄인지 확인하라.

### 9.2 일괄 실행 bash

```bash
mkdir -p logs

for cfg in config/kitti_raw_2011_09_26_drive_*.yaml; do
    name=$(basename "$cfg" .yaml)
    echo "=== running $name ==="
    ./build/run_vio "$cfg" 2>&1 | tee "logs/${name}.log"
done
```

각 시퀀스의 ATE는 `results/<name>/metrics.txt` 에 떨어진다. stdout 전체는
`logs/<name>.log` 에 저장된다.

### 9.3 결과 요약 — CSV 한 줄로 모으기

```bash
echo "drive,frames,ate_rmse_m,ate_rmse_cm" > summary.csv
for m in results/kitti_raw_2011_09_26_drive_*/metrics.txt; do
    drv=$(basename $(dirname "$m") | sed 's/.*drive_//')
    frames=$(awk -F': ' '/^frames/{print $2}' "$m")
    m_val=$(awk -F': ' '/^ate_rmse_m/{print $2}' "$m")
    cm_val=$(awk -F': ' '/^ate_rmse_cm/{print $2}' "$m")
    echo "${drv},${frames},${m_val},${cm_val}" >> summary.csv
done
cat summary.csv
```

### 9.4 drive별 LC-EKF 기대 RMSE

LC-EKF 아키텍처상 KITTI 시퀀스에서 **1~5 m 범위가 정상**이다 (Tight MSCKF는 같은
시퀀스에서 0.1~0.3 m 수준). 5 m 를 크게 벗어나면 발산을 의심하라.

| drive | 길이 | 기대 RMSE | 비고 |
|-------|------|-----------|------|
| 0001 | 11 s | 1~2 m | 검증된 baseline |
| 0009 | 47 s | 2~4 m | 누적 회전 |
| 0027 | 88 s | 3~6 m | 가속·감속 누적 |
| 0036 | 95 s | 5~10 m | 고속 → 누적 드리프트 큼 |
| 0086 | 425 s | 10~50 m | 장기 시퀀스, LC-EKF 한계 명확 |

---

### 9.5 KITTI raw **전체 데이터셋** 일괄 테스트

위 9.1~9.4는 같은 날짜(`2011_09_26`)의 drive만 다뤘다. KITTI raw 는
**5개 캘리브레이션 날짜**로 나뉘어 있고, 날짜마다 calib 파일이 다르므로
전체 데이터셋을 굴리려면 추가 절차가 필요하다.

#### 9.5.1 KITTI raw 전체 카탈로그 (날짜 기준)

| 날짜 (calib 키) | 카테고리 | 시퀀스 수(대략) | 누적 길이(대략) |
|-----------------|---------|------------------|----------------|
| `2011_09_26` | residential / city / road | 매우 많음 (~30 sync drive) | ~30분 |
| `2011_09_28` | campus | 적음 (~9 sync drive) | ~10분 |
| `2011_09_29` | residential | 적음 (~3 sync drive) | ~5분 |
| `2011_09_30` | residential / road | 중간 (~10 sync drive) | ~25분 |
| `2011_10_03` | residential / road / city | 적음 (~5 sync drive) | ~15분 |

**정확한 drive ID 목록은 KITTI raw 공식 페이지 표에서 직접 받는 게 안전하다**
(시즌/연도별로 KITTI 측에서 추가/삭제하기 때문에 본 가이드에 박제하지 않음).
공식 페이지 좌측의 카테고리 필터 (Residential / City / Road / Campus / Person /
Calibration)를 켜고, 우측 표의 "sync+rectified data" 컬럼만 받으면 된다.

#### 9.5.2 디스크 용량 추정

| 단위 | 용량(대략) |
|------|-----------|
| sync drive 1개 | 0.3 ~ 3 GB |
| 한 날짜(예: 2011_09_26) 전체 | 30 ~ 50 GB |
| 5개 날짜 전체 sync 데이터 | **~180 GB** |
| (선택) extract 버전까지 모두 | 400 GB 이상 |

디스크 부족 시 §9.5.5 "코어 평가 셋트"로 압축할 수 있다.

#### 9.5.3 디렉터리 구조 — 전체 데이터셋

```
data/kitti_raw/
├── 2011_09_26/
│   ├── calib_cam_to_cam.txt
│   ├── calib_imu_to_velo.txt
│   ├── calib_velo_to_cam.txt
│   ├── 2011_09_26_drive_0001_sync/
│   ├── 2011_09_26_drive_0009_sync/
│   └── ... (이 날짜의 모든 drive)
├── 2011_09_28/
│   ├── calib_*.txt           ← 위와 다른 calib
│   └── 2011_09_28_drive_0001_sync/
├── 2011_09_29/
│   ├── calib_*.txt
│   └── 2011_09_29_drive_0004_sync/
├── 2011_09_30/
│   ├── calib_*.txt
│   └── 2011_09_30_drive_0016_sync/
└── 2011_10_03/
    ├── calib_*.txt
    └── 2011_10_03_drive_0027_sync/
```

**중요**: 각 날짜 디렉터리 바로 아래에 그 날짜의 calib 3개 파일이 있어야 한다.
공식 zip을 그대로 풀면 자동으로 이렇게 배치된다.

#### 9.5.4 전체 자동화 — calib 자동 추출 + drive별 yaml 생성

§3.3 의 `extract_calib.py` 를 "날짜별로 재사용 가능한 함수" 로 약간 확장한다.

```python
# tools/auto_make_kitti_configs.py — 전체 데이터셋용 yaml 자동 생성 (임시 사용 예)
from pathlib import Path
import numpy as np
import sys

PROJECT = Path("/mnt/d/02_research/04_cpp_seg_msckf_vio")
DATA    = PROJECT / "data" / "kitti_raw"
CONFIG  = PROJECT / "config"
RESULTS = PROJECT / "results"

def parse_kv(path):
    out = {}
    for line in path.read_text().splitlines():
        if ":" not in line: continue
        k, v = line.split(":", 1)
        out[k.strip()] = np.fromstring(v, sep=" ")
    return out

def homog(R, t):
    T = np.eye(4); T[:3,:3] = R.reshape(3,3); T[:3,3] = t.reshape(3); return T

def calib_for_date(date_dir: Path):
    cam = parse_kv(date_dir / "calib_cam_to_cam.txt")
    v2c = parse_kv(date_dir / "calib_velo_to_cam.txt")
    i2v = parse_kv(date_dir / "calib_imu_to_velo.txt")

    T_cam_velo = homog(v2c["R"], v2c["T"])
    T_velo_imu = homog(i2v["R"], i2v["T"])
    R_rect_00  = np.eye(4); R_rect_00[:3,:3] = cam["R_rect_00"].reshape(3,3)

    T_cam0_imu = R_rect_00 @ T_cam_velo @ T_velo_imu
    P_rect_00  = cam["P_rect_00"].reshape(3,4)
    P_rect_01  = cam["P_rect_01"].reshape(3,4)
    fx, fy = P_rect_00[0,0], P_rect_00[1,1]
    cx, cy = P_rect_00[0,2], P_rect_00[1,2]
    baseline_x = -P_rect_01[0,3] / fx
    T_cam1_cam0 = np.eye(4); T_cam1_cam0[0,3] = -baseline_x
    T_cam1_imu = T_cam1_cam0 @ T_cam0_imu
    w, h = cam["S_rect_00"].astype(int)
    return dict(w=int(w), h=int(h), fx=fx, fy=fy, cx=cx, cy=cy,
                T0=T_cam0_imu, T1=T_cam1_imu)

YAML_TPL = """\
dataset_type: "kitti_raw"
dataset: "{ds}"
output:  "{out}"
max_frames: 0

init:
  mode: "stationary_imu"
  static_window_sec: 1.0
  gravity_norm: 9.81

imu_noise:
  gyro:       1.6968e-4
  accel:      2.0000e-3
  gyro_walk:  1.9393e-5
  accel_walk: 3.0000e-3

ekf:
  sigma_vo: 0.10

cam0:
  width: {w}
  height: {h}
  fx: {fx:.4f}
  fy: {fy:.4f}
  cx: {cx:.4f}
  cy: {cy:.4f}
  k1: 0.0
  k2: 0.0
  p1: 0.0
  p2: 0.0
  T_cam_imu:
{T0_block}

cam1:
  width: {w}
  height: {h}
  fx: {fx:.4f}
  fy: {fy:.4f}
  cx: {cx:.4f}
  cy: {cy:.4f}
  k1: 0.0
  k2: 0.0
  p1: 0.0
  p2: 0.0
  T_cam_imu:
{T1_block}
"""

def matrix_block(T):
    rows = []
    for r in T:
        rows.append("    - [ {:.12f}, {:.12f}, {:.12f}, {:.12f} ]".format(*r))
    return "\n".join(rows)

def main():
    n = 0
    for date_dir in sorted(DATA.glob("2011_*")):
        if not (date_dir / "calib_cam_to_cam.txt").exists():
            continue
        cal = calib_for_date(date_dir)
        for drive_dir in sorted(date_dir.glob("*_sync")):
            tag = drive_dir.name.replace("_sync", "")   # 예: 2011_09_26_drive_0009
            ds  = drive_dir
            out = RESULTS / f"kitti_raw_{tag}"
            cfg = CONFIG / f"kitti_raw_{tag}.yaml"
            cfg.write_text(YAML_TPL.format(
                ds=str(ds), out=str(out),
                w=cal["w"], h=cal["h"],
                fx=cal["fx"], fy=cal["fy"], cx=cal["cx"], cy=cal["cy"],
                T0_block=matrix_block(cal["T0"]),
                T1_block=matrix_block(cal["T1"])))
            print(f"wrote {cfg}")
            n += 1
    print(f"total: {n} configs")

if __name__ == "__main__":
    main()
```

실행:

```bash
cd /mnt/d/02_research/04_cpp_seg_msckf_vio
python3 /tmp/auto_make_kitti_configs.py    # 위 스크립트를 임시로 저장해서 실행
```

→ `data/kitti_raw/` 아래에 풀어 둔 모든 sync drive에 대해
`config/kitti_raw_<date>_drive_<id>.yaml` 가 자동 생성된다.

#### 9.5.5 코어 평가 셋트 (디스크/시간 절약 옵션)

전체 180 GB가 부담스러우면 다음 8개만 받아도 캘리브레이션 5종 × 카테고리 다양성을
모두 커버한다 (총 ~15 GB):

| drive | 카테고리 | 길이 | 비고 |
|-------|---------|------|------|
| `2011_09_26_drive_0001` | residential | 11 s | 디버그 |
| `2011_09_26_drive_0009` | residential | 47 s | 표준 검증 |
| `2011_09_26_drive_0036` | road | 95 s | 고속 |
| `2011_09_26_drive_0086` | city | 7 m 5 s | 장기 |
| `2011_09_28_drive_0001` | campus | 짧음 | 다른 calib 환경 |
| `2011_09_29_drive_0004` | residential | 짧음 | 다른 calib 환경 |
| `2011_09_30_drive_0028` | residential | 4 m 30 s | 장기 다른 calib |
| `2011_10_03_drive_0027` | residential | 7 m | 장기 다른 calib |

(정확한 drive ID는 KITTI raw 페이지에서 확인 후 본인 환경에 맞게 조정)

#### 9.5.6 전체 일괄 실행 + 결과 집계

`config/` 안에 모든 날짜의 yaml이 생성됐으면, §9.2 의 for-loop를 약간만 확장:

```bash
mkdir -p logs

for cfg in config/kitti_raw_2011_*_drive_*.yaml; do   # ← 모든 날짜
    name=$(basename "$cfg" .yaml)
    echo "=== running $name ==="
    ./build/run_vio "$cfg" 2>&1 | tee "logs/${name}.log"
done
```

집계도 와일드카드만 확장:

```bash
echo "date,drive,frames,ate_rmse_m,ate_rmse_cm" > summary.csv
for m in results/kitti_raw_2011_*_drive_*/metrics.txt; do
    name=$(basename $(dirname "$m"))
    date=$(echo "$name" | sed 's/kitti_raw_//;s/_drive_.*//')
    drv=$(echo  "$name" | sed 's/.*_drive_//')
    f=$(awk -F': ' '/^frames/{print $2}' "$m")
    a=$(awk -F': ' '/^ate_rmse_m/{print $2}' "$m")
    c=$(awk -F': ' '/^ate_rmse_cm/{print $2}' "$m")
    echo "${date},${drv},${f},${a},${c}" >> summary.csv
done
cat summary.csv
```

날짜별 평균 ATE 한 줄 awk:

```bash
awk -F, 'NR>1 {sum[$1]+=$5; cnt[$1]++} END {for (d in sum) printf "%s : %.1f cm (n=%d)\n", d, sum[d]/cnt[d], cnt[d]}' summary.csv
```

#### 9.5.7 시간 추정

drive 1개당 처리 시간 ≈ 시퀀스 길이의 1~2배 (Release 빌드, 단일 코어). 즉
전체 ~150~200분 시퀀스를 한 번 굴리면 **2~6시간**. 백그라운드로 돌려도 무방.

---

## 10. 결과 해석 & 트러블슈팅

### 10.1 실패 모드 → 원인 → 처방

| 증상 | 원인 후보 | 처방 |
|------|----------|------|
| `tracked=0` 이 자주 | 어두운 시퀀스 / 빠른 회전 / KLT 손실 | grayscale (`image_00`) 강제, drive 변경 |
| EKF 발산 (frame 100쯤 imu_p 폭주) | `sigma_vo` 너무 작아 VO 노이즈가 그대로 들어감 | 0.10 → 0.20 |
| Init 직후부터 어긋남 | static_window 동안 차량이 이미 움직임 | 0.3~0.5초로 축소 또는 정지 시퀀스로 |
| ATE 가 항상 `unavailable` | GT 로드 실패 | `oxts/timestamps.txt` 존재 확인 |
| GT와 회전된 채 평행이동 | LC-EKF는 글로벌 yaw 관측이 없음 (의도된 한계) | 정상. Tight MSCKF로 가야 해결 |
| 중간에 갑자기 점프 | VO outlier (특정 프레임 추적 실패 후 PnP 잘못된 답) | `sigma_vo` 상향 + StereoTracker 임계 조정 |
| Z축 폭주 | 초기 gravity 추정 실패 (가속도 평균이 정지가 아님) | `static_window_sec` 단축 또는 다른 시퀀스 |
| 모든 프레임이 같은 위치 | EKF propagate 누락 (IMU timestamp가 cam보다 항상 미래) | timestamp 단위 확인, KITTI는 OXTS와 cam 모두 같은 epoch |

### 10.2 빠른 sanity check 체크리스트

문제가 생기면 위에서부터 순서대로:

- [ ] `[KITTI raw] IMU=N CAM=N GT=N` 의 N이 모두 같은가
- [ ] `Init mean accel` 이 (0, 0, ~9.8) 부근인가
- [ ] `tracked=` 값이 첫 100프레임 동안 50 이상 유지되는가
- [ ] `trajectory_tum.txt` 의 첫 줄(t=t0)의 위치가 `(0, 0, 0)` 근처인가
- [ ] `gt_tum.txt` 의 첫 줄도 `(0, 0, 0)` 근처인가 (KITTI는 OXTS Mercator 원점이 첫 프레임)
- [ ] 시퀀스를 `max_frames: 50` 으로 줄여서 돌렸을 때도 같은 증상인가

### 10.3 더 깊은 조사 도구

- **`raw_l.png` / `rect_l.png` 비교**: 둘이 같으면 KITTI rect 이미지를 그대로 쓰고
  있는 것. 다르면 `StereoTracker`가 추가 rect를 적용한 것이라 캘리브레이션 모순
  의심.
- **Debug 빌드 + gdb**: EKF 공분산이 NaN이 되는 시점 추적 (`ekf.propagate` 내부에
  break point).
- **`tools/plot_trajectory.py` 의 ATE 출력**: 본 스크립트는 자체적으로 Umeyama
  align 을 다시 돌리므로, 본 main 의 ATE와 ±5% 안에서 일치해야 한다. 크게 다르면
  TUM 파일 형식 문제.

### 10.4 LC-EKF 의 본질적 한계

LC-EKF는 **VO가 이미 출력한 위치**만 측정값으로 쓰기 때문에:

1. VO 자체의 누적 드리프트가 EKF 안으로 그대로 들어온다.
2. yaw에 대한 절대 관측이 없어 글로벌 yaw drift는 보정되지 않는다.
3. feature-level 기하 제약(reprojection)을 활용 못 한다.

→ 이 한계를 넘으려면 Tight MSCKF (OpenVINS 류) 로 가야 한다. 자세한 로드맵은
[`README.md` §9](../README.md) 참조.

---

## 11. 수정 시나리오 5선 — 어디 만지면 무엇이 바뀌는가

각 시나리오는 (1) 동기, (2) 변경 위치 `file:line`, (3) 변경 전/후, (4) 기대 효과
순. 빌드는 그때그때 `cd build && make -j` 한 번.

### 11.1 "VO를 더 신뢰하고 싶다"

**동기**: drive 0009에서 VO가 안정적으로 추적되는데 EKF가 IMU 쪽으로 끌려가
trajectory가 과도하게 꺾인다.

**변경 위치**: 해당 시퀀스 yaml의 `ekf.sigma_vo`.

```yaml
# config/kitti_raw_2011_09_26_drive_0009.yaml (변경 전)
ekf:
  sigma_vo: 0.10
```

```yaml
# 변경 후
ekf:
  sigma_vo: 0.05    # 측정 잡음 표준편차 절반
```

**기대 효과**: `R_noise` (`lc_ekf.cpp:46`)가 1/4로 줄어 → S 작아짐 → K 커짐 →
VO 측정값이 EKF 상태를 더 강하게 끌어당김. ATE는 보통 줄지만 VO outlier가 있으면
오히려 발산.

### 11.2 "초기 자세 불확실성을 더 키우고 싶다"

**동기**: 시작 시 차량이 약간 기울어져 있어서 첫 가속도 평균만으로는 자세가
부정확. EKF가 초기 자세 오류를 빨리 보정하려면 P 초기값이 커야 함.

**변경 위치**: `src/ekf/imu_propagator.cpp:22`.

```cpp
// 변경 전
P.block<3,3>(6,6)  = 1e-4 * Matrix3d::Identity(); // attitude
```

```cpp
// 변경 후
P.block<3,3>(6,6)  = 1e-2 * Matrix3d::Identity(); // attitude (×100)
```

**기대 효과**: 초기 attitude 분산이 커져 첫 EKF 업데이트에서 회전 보정량이 늘어남.
대신 정지 구간에서 회전이 너무 흔들릴 수 있어 1e-3 정도가 보통 무난.

### 11.3 "특징점을 늘려 추적 안정화"

**동기**: drive 0036(고속도로) 같이 텍스처가 단조로운 시퀀스에서 `tracked=`
값이 30 미만으로 떨어지며 EKF가 측정값을 거의 받지 못함.

**변경 위치**: `include/frontend/stereo_tracker.hpp:57`.

```cpp
// 변경 전
int target_features_ = 250;
```

```cpp
// 변경 후
int target_features_ = 400;   // 50% 증가
```

**기대 효과**: 매 프레임 KLT 추적 후보가 늘어 PnP 인라이어 풀이 두꺼워짐. CPU
시간은 비례해서 증가 (정도는 ORB 검출 + 삼각측량 비중에 따라). drop이 너무
빈번하면 600까지도 시도 가능.

### 11.4 "큰 VO 점프를 EKF에서 거부 (chi-square gate)"

**동기**: VO가 outlier 점프(예: 5 m 갑작스런 위치 변화)를 출력했을 때 EKF가
그대로 받아들여 발산.

**변경 위치**: `src/ekf/lc_ekf.cpp` — innovation 계산 직후(l.33) 게이트 추가.

```cpp
// 변경 전
Vector3d innov = p_world_cam0 - h;

// (debug removed)

// Measurement Jacobian H (3×15)
```

```cpp
// 변경 후
Vector3d innov = p_world_cam0 - h;

// 잠정 H, S 계산 후 chi-square gate
Eigen::Matrix<double, 3, 15> H_tmp = ...::Zero();
H_tmp.block<3,3>(0,0) = Matrix3d::Identity();
H_tmp.block<3,3>(0,6) = so3::hat(R_wI * p_IC_);
Eigen::Matrix3d S_tmp = H_tmp * imu_.P * H_tmp.transpose()
                       + (sigma_vo_*sigma_vo_) * Matrix3d::Identity();
double chi2 = innov.transpose() * S_tmp.inverse() * innov;
const double GATE_3DOF_99 = 11.345;   // χ² (df=3, p=0.99)
if (chi2 > GATE_3DOF_99) {
    std::cerr << "[EKF] gated VO innov: chi2=" << chi2 << "\n";
    return false;     // 측정 폐기
}
// 이후 기존 H, S, K 계산은 그대로 두거나 H_tmp/S_tmp 재사용
```

**기대 효과**: 통계적으로 99% 이상 outlier로 판정된 VO 측정은 무시. 발산 방지에
효과적이지만 너무 빡빡하면 정상 측정도 떨궈서 IMU drift만 누적될 수 있다.

### 11.5 "GPS 위치도 측정값으로 추가"

**동기**: KITTI는 OXTS GPS 위치(`GtData.p`)가 거의 매 프레임 함께 오므로,
EKF에서 GT와 별개로 GPS도 noisy 측정값으로 쓰면 글로벌 drift 보정이 가능.

**변경 위치**: `include/ekf/lc_ekf.hpp` 에 새 함수 추가, `src/ekf/lc_ekf.cpp` 에 구현.

```cpp
// include/ekf/lc_ekf.hpp 에 추가
class LcEkf {
public:
    // ... 기존 메서드들 ...
    bool update_gps(const Eigen::Vector3d& p_gps, double sigma_gps);  // ← NEW
};
```

```cpp
// src/ekf/lc_ekf.cpp 에 추가 (update_vo와 매우 유사)
bool LcEkf::update_gps(const Vector3d& p_gps, double sigma_gps) {
    // GPS는 IMU 위치를 직접 잰다고 가정 (lever arm 무시 또는 별도 lever 입력)
    // h(x) = p_wI
    Vector3d h = imu_.p;
    Vector3d innov = p_gps - h;

    Eigen::Matrix<double, 3, 15> H = ...::Zero();
    H.block<3,3>(0,0) = Matrix3d::Identity();    // ∂h/∂δp = I, 나머지 0

    Eigen::Matrix3d R_noise = (sigma_gps*sigma_gps) * Matrix3d::Identity();
    Eigen::Matrix3d S = H * imu_.P * H.transpose() + R_noise;
    Eigen::Matrix<double, 15, 3> K = imu_.P * H.transpose() * S.inverse();
    Vec15 dx = K * innov;
    apply_correction(dx);
    Mat15 I_KH = Mat15::Identity() - K * H;
    imu_.P = I_KH * imu_.P * I_KH.transpose() + K * R_noise * K.transpose();
    return true;
}
```

`apps/run_euroc.cpp`의 메인 루프에서:

```cpp
if (pose.valid && frame_count > 0)  ekf.update_vo(p_world_cam);
ekf.update_gps(gt_p, /*sigma_gps=*/1.0);   // ← GPS도 같은 프레임에 한 번 더
```

**기대 효과**: GPS 1 m 표준편차 정도면 누적 drift를 강하게 잡아준다. 단 KITTI에선
GT를 측정값으로 다시 넣으면 ATE 평가가 무의미해지므로 **GPS 평가용 별도 yaml 분리** 권장.
실제 차량 응용에선 GT 대신 별도 GPS 수신기 데이터를 쓴다.

---

## 12. Knob 카탈로그 (튜닝 가능 파라미터 전체 목록)

### 12.1 YAML knobs (실험마다 yaml 수정만으로 바뀜)

| knob | yaml 위치 | 코드 위치 | 기본값 | 영향 |
|------|----------|----------|--------|------|
| `ekf.sigma_vo` | yaml | `apps/run_euroc.cpp:240` | 0.10 (KITTI) | VO 측정 신뢰도. 작을수록 VO 우세 |
| `imu_noise.gyro` | yaml | `apps/run_euroc.cpp:238` (load_imu_noise) | 1.6968e-4 | 자이로 잡음 가정 |
| `imu_noise.accel` | yaml | 동상 | 2.0000e-3 | 가속도 잡음 가정 |
| `imu_noise.gyro_walk` | yaml | 동상 | 1.9393e-5 | 자이로 바이어스 드리프트 |
| `imu_noise.accel_walk` | yaml | 동상 | 3.0000e-3 | 가속도 바이어스 드리프트 |
| `init.mode` | yaml | `apps/run_euroc.cpp:75-81` | `stationary_imu` | 초기화 방식 (현재 한 종류) |
| `init.static_window_sec` | yaml | `apps/run_euroc.cpp:122-127` | 1.0 | 초기 평균화 구간 (s) |
| `init.gravity_norm` | yaml | `apps/run_euroc.cpp:115` | 9.81 | 중력 크기 (m/s²) |
| `max_frames` | yaml | `apps/run_euroc.cpp:228` | 0 (전체) | 부분 실행 (디버그) |
| `dataset_type` | yaml | `apps/run_euroc.cpp:225` | "kitti_raw" | 리더 선택 |
| `dataset` | yaml | `apps/run_euroc.cpp:226` | (경로) | 입력 디렉터리 |
| `output` | yaml | `apps/run_euroc.cpp:227` | (경로) | 결과 저장 디렉터리 |
| `cam0/cam1.fx,fy,cx,cy` | yaml | `apps/run_euroc.cpp:43-46` | 캘리브레이션값 | rectified intrinsics |
| `cam0/cam1.k1,k2,p1,p2` | yaml | `apps/run_euroc.cpp:47-50` | 0 (KITTI) | radtan 왜곡 |
| `cam0/cam1.width,height` | yaml | `apps/run_euroc.cpp:51-52` | 1242×375 (KITTI) | 이미지 해상도 |
| `cam0/cam1.T_cam_imu` | yaml | `apps/run_euroc.cpp:54-62` | 캘리브레이션값 | IMU→cam 4×4 |

### 12.2 C++ 상수 knobs (소스 수정 + 재빌드 필요)

| knob | 위치 | 현재값 | 영향 |
|------|------|--------|------|
| P 초기 분산 (position) | `src/ekf/imu_propagator.cpp:20` | 1e-4 | 초기 위치 불확실성 |
| P 초기 분산 (velocity) | `src/ekf/imu_propagator.cpp:21` | 1e-2 | 초기 속도 불확실성 |
| P 초기 분산 (attitude) | `src/ekf/imu_propagator.cpp:22` | 1e-4 | 초기 자세 불확실성 |
| P 초기 분산 (gyro bias) | `src/ekf/imu_propagator.cpp:23` | 1e-6 | 초기 자이로 bias 불확실성 |
| P 초기 분산 (accel bias) | `src/ekf/imu_propagator.cpp:24` | 1e-4 | 초기 가속도 bias 불확실성 |
| dt sanity gate | `src/ekf/imu_propagator.cpp:137` | 0.5 s | IMU 갭 허용 한계 |
| `target_features_` | `include/frontend/stereo_tracker.hpp:57` | 250 | KLT 추적 점 개수 |
| `min_pnp_inliers_` | `include/frontend/stereo_tracker.hpp:58` | 10 | PnP 최소 인라이어 |
| `disparity_offset_px_` 초기값 | `include/frontend/stereo_tracker.hpp:74` | 6.0 (placeholder) | 가이드 KLT |
| `disparity_offset_px_` 실제값 | `src/frontend/stereo_tracker.cpp:80` | -fx·b/3 | 생성자에서 덮어씀 |
| ORB 파라미터 | `src/frontend/stereo_tracker.cpp:85` | nfeatures=1500 | 매칭 풍부도 |
| 깊이 게이트 (z 범위) | `src/frontend/stereo_tracker.cpp:118-119` | 0.1~50 m | 삼각측량 유효 범위 |
| 큰 VO innovation 경고 | `src/ekf/lc_ekf.cpp:64-65` | 5.0 m | 디버그 로그 임계 |

---

## 부록 A. 디렉터리 레이아웃 권장형

```
04_cpp_seg_msckf_vio/
├── apps/                  # 빌드 시 자동
├── build/                 # cmake 빌드 산출물 (gitignore 권장)
│   ├── run_vio
│   └── run_euroc
├── config/
│   ├── euroc_v101.yaml
│   ├── kitti_raw_template.yaml
│   ├── kitti_raw_2011_09_26_drive_0001.yaml
│   ├── kitti_raw_2011_09_26_drive_0009.yaml      # 추가
│   ├── kitti_raw_2011_09_28_drive_0001.yaml      # 다른 날짜 (전체 데이터셋)
│   └── kitti_raw_2011_10_03_drive_0027.yaml
├── data/
│   └── kitti_raw/
│       ├── 2011_09_26/{calib_*.txt, *_sync/}
│       ├── 2011_09_28/{calib_*.txt, *_sync/}
│       ├── 2011_09_29/{calib_*.txt, *_sync/}
│       ├── 2011_09_30/{calib_*.txt, *_sync/}
│       └── 2011_10_03/{calib_*.txt, *_sync/}
├── docs/
│   └── guides/
│       └── kitti_full_pipeline_guide.md   # 이 파일
├── include/, src/         # 코드
├── logs/                  # 일괄 실행 stdout
├── results/
│   └── kitti_raw_<date>_drive_<id>/
│       ├── trajectory_tum.txt
│       ├── gt_tum.txt
│       ├── metrics.txt
│       └── trajectory_plot.png
├── summary.csv            # 일괄 비교 결과
├── tools/
│   └── plot_trajectory.py
├── CMakeLists.txt
└── README.md              # 이론 노트
```

권장 명명 규칙:

- config: `kitti_raw_<date>_drive_<id>.yaml`
- output: `results/kitti_raw_<date>_drive_<id>/`
- log: `logs/kitti_raw_<date>_drive_<id>.log`

---

## 부록 B. 명령어 치트시트

```bash
# 의존성
sudo apt install -y build-essential cmake libeigen3-dev libopencv-dev libyaml-cpp-dev python3-numpy python3-matplotlib

# 빌드
cd /mnt/d/02_research/04_cpp_seg_msckf_vio
mkdir -p build && cd build && cmake .. -DCMAKE_BUILD_TYPE=Release && make -j$(nproc)

# 단일 실행
./run_vio ../config/kitti_raw_2011_09_26_drive_0001.yaml

# 시각화
cd .. && python3 tools/plot_trajectory.py results/kitti_raw_2011_09_26_drive_0001/

# 단일 날짜 일괄 실행
mkdir -p logs
for cfg in config/kitti_raw_2011_09_26_drive_*.yaml; do
    name=$(basename "$cfg" .yaml)
    ./build/run_vio "$cfg" 2>&1 | tee "logs/${name}.log"
done

# 전체 데이터셋 일괄 실행
for cfg in config/kitti_raw_2011_*_drive_*.yaml; do
    name=$(basename "$cfg" .yaml)
    ./build/run_vio "$cfg" 2>&1 | tee "logs/${name}.log"
done

# 결과 요약
echo "date,drive,frames,ate_rmse_m,ate_rmse_cm" > summary.csv
for m in results/kitti_raw_2011_*_drive_*/metrics.txt; do
    name=$(basename $(dirname "$m"))
    date=$(echo "$name" | sed 's/kitti_raw_//;s/_drive_.*//')
    drv=$(echo  "$name" | sed 's/.*_drive_//')
    f=$(awk -F': ' '/^frames/{print $2}' "$m")
    a=$(awk -F': ' '/^ate_rmse_m/{print $2}' "$m")
    c=$(awk -F': ' '/^ate_rmse_cm/{print $2}' "$m")
    echo "${date},${drv},${f},${a},${c}" >> summary.csv
done
cat summary.csv

# 날짜별 평균
awk -F, 'NR>1 {sum[$1]+=$5; cnt[$1]++} END {for (d in sum) printf "%s : %.1f cm (n=%d)\n", d, sum[d]/cnt[d], cnt[d]}' summary.csv
```

---

## 끝맺으며

여기까지가 **혼자였다면** 거쳐야 했을 작업 + 코드 내부 이해다. 한 호흡으로 정리:

1. KITTI 계정 → calib.zip + sync drive 다운 (필요한 날짜만)
2. `calib_*.txt` 3종 → `T_cam_imu` 4×4 두 개 (cam0, cam1) 추출 (§3)
3. yaml config 작성 (§4) — 같은 날짜 drive는 calib 재사용
4. **(선택) §5 워크스루로 코드가 무엇을 하는지 한 번 훑기**
5. cmake 빌드 (§6)
6. 단일 실행 → stdout 정상성 확인 (§7) → 시각화 (§8)
7. 같은 날짜 일괄 (§9.1~9.4) 또는 전체 데이터셋 자동화 (§9.5)
8. 발산/이상이 있으면 §10 표 + §11 시나리오 + §12 knob 으로 튜닝

LC-EKF는 어차피 1~5 m RMSE 수준에서 천장이 명확한 구조이므로, 그 이상의 정확도를
원하면 Tight MSCKF 로 넘어가야 한다 ([README §9](../README.md)).
