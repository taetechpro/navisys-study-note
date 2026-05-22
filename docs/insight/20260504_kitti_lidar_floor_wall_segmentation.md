---
title: KITTI LiDAR Floor/Wall Segmentation 정리
date: 2026-05-04
type: insight
tags:
  - lidar
  - segmentation
  - kitti
  - ransac
related:
  - "[[20260514_ransac_lidar_segmentation_math]]"
  - "[[LidarSegmenter session_progress]]"
status: reviewed
---

# KITTI LiDAR Floor/Wall Segmentation 정리 — 2026-05-04

## 1. 목표

KITTI raw `velodyne_points/data/*.bin` 포인트클라우드를 읽어서 한 프레임마다 다음 라벨로 분류한다.

| 라벨 | 의미 | Rerun 색상 |
|---|---|---|
| `floor` | 바닥 plane inlier | green |
| `left_wall` | 차량 좌측 수직 벽 후보 | blue |
| `right_wall` | 차량 우측 수직 벽 후보 | orange |
| `front_wall` | 차량 전방 수직 벽 후보 | pink |
| `other` | 위 라벨에 속하지 않는 점 | gray |

현재 구현은 semantic segmentation이 아니라 **LiDAR geometry 기반 plane segmentation**이다.  
즉, 딥러닝 모델 없이 Velodyne 3D 점의 위치와 plane fitting만으로 바닥/벽 후보를 찾는다.

---

## 2. 처리 흐름

### 데이터 로딩

`KittiRawReader`가 기존 camera/IMU/OXTS 외에 Velodyne `.bin` path도 읽는다.

- KITTI Velodyne 포맷: `float x, y, z, intensity`
- 좌표계:
  - `x`: 전방
  - `y`: 좌측
  - `z`: 상방

코드 위치:

- `include/io/dataset_reader.hpp`
- `include/io/kitti_raw_reader.hpp`
- `src/io/kitti_raw_reader.cpp`

### Segmentation

`LidarSegmenter::process()`가 다음 순서로 처리한다.

1. ROI 필터링: 너무 멀거나 높이 범위 밖인 점 제거
2. 바닥 후보 선택: 낮은 `z` 영역에서 RANSAC plane fitting
3. floor inlier 라벨링
4. 남은 점 중 좌측/우측/전방 후보를 따로 모음
5. 각 후보군에 대해 수직 plane RANSAC
6. plane normal 방향, inlier 수, 높이/폭 extent 조건을 통과하면 wall 라벨 부여

코드 위치:

- `include/lidar/lidar_segmenter.hpp`
- `src/lidar/lidar_segmenter.cpp`

핵심 함수:

- `load_kitti_velodyne_bin()` — `.bin` point cloud 로드
- `LidarSegmenter::process()` — floor/wall segmentation
- `write_segmented_ply()` — Rerun 없이 실행할 때 colored PLY 저장
- `segment_color_rgb()` — 라벨별 색상 정의

---

## 3. Rerun 출력

`run_vio`에 LiDAR segmentation 옵션을 추가했다.

```bash
--segment-lidar
--segment-every <N>
```

Rerun이 켜져 있으면 `/lidar/segments`에 colored `Points3D`로 기록된다.

Rerun entity:

```text
/lidar/segments
/metrics/lidar_floor_points
/metrics/lidar_left_wall_points
/metrics/lidar_right_wall_points
/metrics/lidar_front_wall_points
```

코드 위치:

- `apps/run_vio.cpp`
  - CLI 옵션 파싱: `--segment-lidar`, `--segment-every`
  - `RerunLogger::log_lidar_segments()`
  - main loop에서 Velodyne frame 로드 및 segmentation 호출

---

## 4. 실행 예시

Rerun-enabled build:

```powershell
wsl bash -lc 'cd /mnt/d/02_research/04_cpp_LC-EKF_VIO && cmake -S . -B build_verify_rerun -DCMAKE_BUILD_TYPE=Release -DENABLE_RERUN=ON && cmake --build build_verify_rerun -j$(nproc)'
```

전체 frame 실행:

```powershell
wsl bash -lc 'cd /mnt/d/02_research/04_cpp_LC-EKF_VIO && mkdir -p results/kitti_raw_2011_09_26_drive_0117_full && ./build_verify_rerun/run_vio config/kitti_raw_2011_09_26_drive_0117_local_max_frame.yaml --segment-lidar --segment-every 1 --rerun-save results/kitti_raw_2011_09_26_drive_0117_full/lidar_segments_full.rrd --rerun-image-every 10'
```

Rerun 열기:

```powershell
rerun D:\02_research\04_cpp_LC-EKF_VIO\results\kitti_raw_2011_09_26_drive_0117_full\lidar_segments_full.rrd
```

---

## 5. 확인된 결과

1-frame sanity check:

```text
floor=60707
left=1339
right=1425
front=761
other=20368
```

Rerun Viewer에서 `/lidar/segments`를 선택하면 3D point cloud가 라벨별 색상으로 표시된다.

---

## 6. 주의점

- `front_wall`은 실제 전방에 수직 구조물이 있을 때만 의미 있게 잡힌다.
- 일반 도로 장면에서는 벽이 아닌 가드레일, 차량, 수풀 일부가 wall 후보로 섞일 수 있다.
- 현재 LiDAR segmentation 결과는 VIO/EKF update에는 사용하지 않고, 시각화와 분석용이다.
- `--rerun-save`의 부모 폴더는 미리 만들어야 한다.

---

## 7. 직접 수집 데이터로 응용하는 방법

현재 구현은 `run_vio`의 KITTI raw reader에 붙어 있으므로, 직접 수집한 데이터는 **KITTI-like 구조**로 변환해서 쓰는 것이 가장 빠르다.

필요 데이터:

- stereo left/right image
- IMU 또는 KITTI OXTS-like packet
- LiDAR point cloud
- camera intrinsics/extrinsics가 들어간 YAML config

권장 폴더 구조:

```text
data/my_dataset/my_drive_sync/
  image_00/timestamps.txt
  image_00/data/0000000000.png
  image_01/timestamps.txt
  image_01/data/0000000000.png
  oxts/timestamps.txt
  oxts/data/0000000000.txt
  velodyne_points/data/0000000000.bin
```

LiDAR `.bin` 변환 규칙:

- point 하나는 little-endian `float32 x, y, z, intensity`
- 좌표계는 KITTI Velodyne 기준으로 맞춤
  - `x`: 전방
  - `y`: 좌측
  - `z`: 위쪽
  - 단위: meter

중요 제약:

- 현재 Velodyne frame은 camera frame과 **같은 index**로 매칭된다.
- frame drop이 있으면 파일명을 다시 정렬해서 image/LiDAR index를 맞춰야 한다.
- LiDAR-only 로그만 바로 실행하는 기능은 아직 없다. 그 경우 `LidarSegmenter`를 직접 호출하는 별도 executable을 추가하는 것이 좋다.

실행 예:

```powershell
wsl bash -lc 'cd /mnt/d/02_research/04_cpp_LC-EKF_VIO && mkdir -p results/my_drive && ./build_verify_rerun/run_vio config/my_drive.yaml --segment-lidar --segment-every 1 --rerun-save results/my_drive/lidar_segments.rrd --rerun-image-every 10'
```

Rerun 열기:

```powershell
rerun D:\02_research\04_cpp_LC-EKF_VIO\results\my_drive\lidar_segments.rrd
```
