# Stereo/RGB-D + IMU Floor/Wall Segmentation 계획 정리 - 2026-05-06

## 1. Problem

목표는 카메라 기반 depth와 IMU를 사용해서 장면의 점들을 다음 범주로 구분하는 것이다.

| 라벨 | 의미 |
|---|---|
| `floor` | 중력 방향 기준으로 수평에 가까운 바닥 plane |
| `wall` | 중력 방향 기준으로 수직에 가까운 벽 plane |
| `other` | 바닥/벽으로 안정적으로 분류되지 않는 점 |

이 문제에서 중요한 것은 단순히 픽셀을 분류하는 것이 아니라, VIO/SLAM 파이프라인 안에서 재사용 가능한 **기하 정보**를 얻는 것이다.

카메라만 사용할 때는 이미지 texture, 조명, 재질에 의존하게 된다. 반면 Stereo/RGB-D는 depth를 제공하고, IMU는 gravity direction을 제공한다. 따라서 바닥/벽 구분은 먼저 semantic segmentation이 아니라 **3D point cloud + gravity prior + plane fitting**으로 접근하는 것이 자연스럽다.

---

## 2. Current Repo Context

현재 저장소에는 이미 KITTI raw 기반의 stereo + IMU 파이프라인이 있다.

관련 구조:

| 구성 | 코드 위치 | 역할 |
|---|---|---|
| `StereoTracker` | `include/frontend/stereo_tracker.hpp`, `src/frontend/stereo_tracker.cpp` | rectified stereo에서 sparse feature depth와 VO pose 계산 |
| `ImuPropagator`, `LcEkf` | `include/ekf`, `src/ekf` | IMU propagation과 VO measurement update |
| `CameraParams::T_cam_imu` | `include/core/types.hpp` | IMU frame에서 camera frame으로 가는 extrinsic |
| `KittiRawReader` | `include/io/kitti_raw_reader.hpp`, `src/io/kitti_raw_reader.cpp` | stereo image, OXTS/IMU, Velodyne path 로딩 |
| `LidarSegmenter` | `include/lidar/lidar_segmenter.hpp`, `src/lidar/lidar_segmenter.cpp` | LiDAR point cloud의 floor/wall RANSAC segmentation |
| `run_vio` | `apps/run_vio.cpp` | 전체 실행 루프, Rerun logging, LiDAR segmentation 옵션 |

이미 구현된 LiDAR segmentation은 다음 판단을 해준다.

1. ROI 밖의 점 제거
2. 낮은 `z` 범위에서 floor plane RANSAC
3. floor inlier 라벨링
4. 남은 점에서 left/right/front wall 후보 plane RANSAC
5. normal 방향, inlier 수, extent 조건으로 wall 라벨 검증

즉 새 기능을 완전히 새로 만들기보다, 이 구조를 **LiDAR 전용에서 depth point cloud 공통 구조로 일반화**하는 것이 좋다.

---

## 3. Options Considered

### Option A: Semantic Segmentation

이미지의 각 픽셀을 `floor`, `wall`, `object`로 분류하는 방식이다.

장점:

- texture와 색상 단서까지 활용할 수 있다.
- depth가 불안정한 영역에서도 학습 데이터가 충분하면 동작할 수 있다.
- 실내 semantic parsing에는 강하다.

단점:

- 모델, 학습 데이터, runtime dependency가 필요하다.
- VIO 코드베이스 안에서 디버깅 가능한 기하 제약으로 설명하기 어렵다.
- 도메인이 바뀌면 재학습 또는 fine-tuning이 필요할 수 있다.

### Option B: Geometry + RANSAC

Stereo/RGB-D에서 point cloud를 만들고, IMU gravity direction을 기준으로 평면을 찾는 방식이다.

장점:

- 현재 코드의 `LidarSegmenter`와 잘 맞는다.
- 결과가 plane normal, inlier, distance threshold로 설명 가능하다.
- dataset-specific semantic label 없이도 시작할 수 있다.
- Rerun/PLY로 시각 검증하기 쉽다.

단점:

- depth noise, textureless stereo, 반사/유리 표면에 취약하다.
- clutter가 많은 장면에서는 wall plane에 물체가 섞일 수 있다.
- dense/semi-dense depth 생성 품질이 전체 성능을 좌우한다.

### Option C: VIO/SLAM Map 기반 Plane Extraction

시간에 따라 누적된 sparse/dense map에서 plane을 찾는 방식이다.

장점:

- 단일 프레임보다 안정적인 plane을 얻을 수 있다.
- 일시적인 depth noise를 누적 관측으로 줄일 수 있다.
- map-level structure로 확장 가능하다.

단점:

- v1 구현 범위가 커진다.
- pose drift와 map management 문제가 같이 들어온다.
- 현재 저장소의 sparse VO 상태만으로는 dense plane extraction에 필요한 점 수가 부족할 수 있다.

---

## 4. Decision

v1은 **Option B: Geometry + RANSAC**으로 진행한다.

선택한 접근:

```text
Stereo/RGB-D frame
  ↓
Depth image 또는 disparity 생성
  ↓
Camera frame point cloud 생성
  ↓
T_cam_imu와 EKF/IMU orientation을 사용해 gravity-aligned frame으로 변환
  ↓
RANSAC plane fitting
  ↓
normal과 gravity direction으로 floor/wall 분류
  ↓
Rerun/PLY로 시각화 및 count logging
```

핵심 판단:

- depth가 있으면 semantic보다 geometry를 먼저 시도한다.
- IMU의 gravity direction은 floor/wall 분류에서 가장 강한 prior다.
- 구현은 `floor`, `wall`, `other` 단일 구조로 시작하고, 필요할 때 `left_wall`, `right_wall`, `front_wall`로 확장한다.
- 기존 `LidarSegmenter`의 RANSAC, inlier count, extent check 아이디어를 재사용한다.

---

## 5. Execution Plan

### 5.1 공통 point 타입 정리

LiDAR 전용 `LidarPoint`를 바로 확장할지, 새 공통 타입을 만들지 결정한다.

권장 방향:

- `PointXYZI` 또는 `PointXYZ` 같은 공통 타입을 추가한다.
- LiDAR intensity, RGB-D color는 optional 성격으로 둔다.
- `SegmentedCloud`는 LiDAR뿐 아니라 depth cloud에도 사용할 수 있게 이름을 일반화한다.

초기에는 코드 변경 범위를 줄이기 위해 기존 `SegmentLabel`과 색상 정의는 재사용해도 된다.

### 5.2 Stereo depth cloud 생성

현재 `StereoTracker`는 sparse feature 기반 VO에 집중한다. floor/wall segmentation에는 sparse feature만으로 점 수가 부족할 수 있다.

따라서 별도 depth cloud 생성 경로가 필요하다.

처리 흐름:

1. left/right image를 grayscale 변환
2. 기존 rectification map 또는 같은 calibration 기준으로 rectification
3. dense 또는 semi-dense disparity 계산
4. 유효 disparity만 선택
5. `Z = fx * baseline / disparity`로 depth 복원
6. `X = (u - cx) * Z / fx`, `Y = (v - cy) * Z / fy`로 camera frame point 생성
7. depth range, image stride, ROI 필터로 점 수 조절

초기 구현은 성능보다 검증이 중요하므로, 전체 픽셀을 다 쓰지 않고 stride/downsample을 둔다.

### 5.3 RGB-D depth cloud 생성

RGB-D 입력은 depth image가 직접 들어온다는 점만 다르다.

처리 흐름:

1. depth image 로드
2. depth scale을 meter로 변환
3. invalid depth와 min/max depth 제거
4. intrinsics로 camera frame point 생성
5. 필요하면 RGB image에서 color를 가져와 시각화에 사용

Stereo와 RGB-D는 point cloud 생성 이후 같은 segmentation 경로를 사용한다.

### 5.4 Gravity-aligned frame 변환

floor/wall 분류는 camera frame의 임의 축에 의존하면 안 된다. 기준은 gravity direction이어야 한다.

현재 파이프라인에서는 `ekf.orientation()`과 `cam0.T_cam_imu`를 이용할 수 있다.

개념적으로 필요한 변환:

```text
point_camera
  -> point_imu
  -> point_world_or_gravity_aligned
```

이렇게 바꾼 뒤, gravity-aligned frame에서:

- floor normal은 vertical axis와 거의 평행
- wall normal은 vertical axis와 거의 수직

으로 판단한다.

주의할 점:

- 좌표계의 `up/down` 부호보다 중요한 것은 normal과 gravity axis의 절대 내적이다.
- floor 후보의 높이 범위는 sensor mounting height와 dataset에 따라 설정값으로 둔다.
- EKF orientation이 초기 몇 프레임에서 불안정하면 segmentation을 늦게 시작하거나 초기 IMU 평균 자세를 사용할 수 있다.

### 5.5 Plane segmentation

v1 분류 순서:

1. ROI 필터링
2. floor 후보 RANSAC
3. floor inlier 제거 또는 라벨링
4. 남은 점에서 wall 후보 RANSAC
5. wall normal, inlier 수, extent 조건 검증
6. `Floor`, `Wall`, `Other` count 출력

분류 기준:

| 조건 | Floor | Wall |
|---|---|---|
| normal vs gravity axis | 거의 평행 | 거의 수직 |
| 위치 | sensor 아래쪽 또는 지면 높이 근처 | floor 위쪽 영역 |
| extent | 일정 면적 이상 | 일정 높이/폭 이상 |
| inliers | 충분히 많아야 함 | 충분히 많아야 함 |

기존 `LidarSegmenter`의 fixed `x/y/z` threshold는 KITTI Velodyne 좌표계에 강하게 묶여 있으므로, depth cloud 쪽에서는 gravity-aligned 좌표 기준으로 다시 파라미터화한다.

### 5.6 CLI와 config

새 실행 옵션은 기존 `--segment-lidar`와 나란히 둔다.

예상 CLI:

```bash
--segment-depth
--depth-source stereo
--segment-every <N>
```

RGB-D reader가 추가되면:

```bash
--segment-depth
--depth-source rgbd
```

YAML에는 다음 설정을 둔다.

```yaml
depth_segmentation:
  min_depth_m: 0.3
  max_depth_m: 20.0
  image_stride: 4
  floor_distance_threshold_m: 0.08
  wall_distance_threshold_m: 0.12
  floor_normal_min_abs_dot_gravity: 0.90
  wall_normal_max_abs_dot_gravity: 0.20
  min_floor_inliers: 800
  min_wall_inliers: 400
```

파라미터는 처음부터 과하게 세분화하지 않는다. 실제 Rerun 결과를 보면서 필요한 것만 추가한다.

---

## 6. Validation Plan

### Unit-level sanity check

synthetic point cloud를 만들어 segmenter만 검증한다.

검증 장면:

- 수평 floor plane 1개
- 수직 wall plane 1개
- random outlier
- gravity axis를 바꾼 회전 케이스

기대 결과:

- floor point 대부분이 `Floor`
- wall point 대부분이 `Wall`
- outlier 대부분이 `Other`
- point cloud 전체를 회전해도 gravity-aligned 변환 후 라벨이 유지됨

### KITTI stereo check

KITTI raw stereo frame에서 disparity 기반 point cloud를 만든 뒤 segmentation한다.

확인할 것:

- disparity가 비어 있지 않은가
- point cloud depth scale이 meter 단위로 맞는가
- floor plane이 차량 아래/전방 도로에 잡히는가
- 벽 후보가 실제 구조물, 차량, 가드레일과 어떻게 섞이는가

### Rerun visual check

Rerun entity는 기존 LiDAR logging 스타일을 따른다.

예상 entity:

```text
/depth/segments
/metrics/depth_floor_points
/metrics/depth_wall_points
/metrics/depth_other_points
```

색상:

| 라벨 | 색상 |
|---|---|
| `Floor` | green |
| `Wall` | blue 또는 orange |
| `Other` | gray |

검증 기준:

- camera image와 point cloud timestamp가 맞는가
- floor/wall 색상이 명확히 구분되는가
- 카메라 움직임 중 segmentation 결과가 급격히 튀지 않는가

---

## 7. Insights

이번 판단에서 가져갈 일반화 가능한 기준은 다음과 같다.

1. depth가 있으면 먼저 3D geometry로 문제를 푼다.
2. floor/wall은 class name보다 plane normal과 gravity relation으로 정의하는 편이 견고하다.
3. IMU는 단순 pose 보정용이 아니라 scene understanding의 기준축을 제공한다.
4. semantic segmentation은 설명 가능한 geometry baseline이 실패한 뒤 붙이는 것이 디버깅에 유리하다.
5. 기존 코드에 이미 존재하는 LiDAR segmentation은 새 sensor modality를 설계할 때 좋은 reference implementation이다.
6. 좌표계 정의가 segmentation threshold보다 먼저다. `camera`, `imu`, `world`, `gravity-aligned` frame을 헷갈리면 결과 해석이 불가능해진다.
7. sparse VO feature와 dense/semi-dense segmentation point cloud는 목적이 다르다. 같은 stereo pair를 쓰더라도 별도 데이터 경로로 두는 것이 낫다.

---

## 8. Open Questions

다음 구현과 실험에서 확인할 질문:

- KITTI stereo에서 dense disparity 품질이 floor/wall plane fitting에 충분한가?
- OpenCV StereoSGBM만으로 시작할지, 다른 disparity estimator가 필요한가?
- wall을 v1부터 `left/right/front`로 나눌 필요가 있는가, 아니면 단일 `Wall`이면 충분한가?
- floor height prior를 sensor mounting height로 둘지, 첫 frame RANSAC 결과로 adaptive하게 잡을지?
- RGB-D dataset을 추가할 경우 reader를 새로 만들지, 기존 `DatasetReader`에 optional depth path를 넣을지?
- segmentation 결과를 당장 EKF update에 넣을지, 당분간 visualization/analysis output으로만 둘지?

---

## 9. 기록 방식

앞으로 이 기능을 구현하면서 이 문서를 계속 업데이트한다.

기록 원칙:

- 명령어와 결과 수치는 재현을 위해 남긴다.
- 코드 변경 목록보다, 왜 그 변경을 선택했는지를 우선 기록한다.
- 실패한 접근도 지우지 않고 이유를 남긴다.
- Rerun capture, PLY output, metrics count는 가능한 한 같은 frame index 기준으로 비교한다.

추가할 예정 섹션:

```text
10. Execution Notes
11. Results
12. Retrospective
```

---

## 10. Execution Notes

2026-05-06 v1 구현에서는 RGB-D reader까지 확장하지 않고, 먼저 **stereo depth source**만 연결했다.

추가된 구조:

| 구성 | 코드 위치 | 역할 |
|---|---|---|
| `StereoDepthSegmenter` | `include/depth/stereo_depth_segmenter.hpp`, `src/depth/stereo_depth_segmenter.cpp` | rectified stereo pair에서 SGBM disparity를 만들고 gravity-aligned point cloud를 floor/wall/other로 라벨링 |
| `StereoDepthGeometry` | `include/depth/stereo_depth_segmenter.hpp` | rectified intrinsics, baseline, rectified-to-cam0 rotation, `T_cam0_imu` 묶음 |
| `StereoTracker` getter | `include/frontend/stereo_tracker.hpp`, `src/frontend/stereo_tracker.cpp` | rectified right image와 rectified camera parameters를 외부 depth segmentation 경로에 제공 |
| `run_vio` depth path | `apps/run_vio.cpp` | `--segment-depth`, `--depth-source stereo`, Rerun/PLY output 연결 |

실행 옵션:

```bash
--segment-depth
--depth-source stereo
--segment-every <N>
```

Rerun entity:

```text
/depth/segments
/metrics/depth_floor_points
/metrics/depth_wall_points
/metrics/depth_other_points
```

Rerun 없이 실행하면 다음 위치에 PLY가 저장된다.

```text
<output>/depth_segments/0000000000.ply
```

권장 results layout:

```text
results/
  kitti_raw/
    2011_09_26_drive_0117/
      sanity_1frame/
        depth_segments/*.ply
        metrics.txt
      full_sequence/
        rerun/*.rrd
        metrics.txt
        trajectory_tum.txt
        trajectory_aligned_tum.txt
        state_log.txt
        imu_log.txt
        gt_tum.txt
```

원칙:

- dataset과 sequence를 먼저 나눈다.
- `sanity_1frame`, `full_sequence`, `lidar_segmentation`, `stereo_depth_every5`처럼 run 목적을 폴더명에 드러낸다.
- Rerun 파일은 `rerun/` 아래에 둔다.
- PLY frame dump는 `<run>/depth_segments/` 또는 `<run>/lidar_segments/` 아래에 둔다.
- `results/kitti_raw_2011_09_26_drive_0117_velo_point_max`처럼 sensor와 목적이 섞인 flat 이름은 새 실험에서는 쓰지 않는다.

구현상 중요한 선택:

- stereo depth는 기존 sparse VO feature와 분리된 별도 SGBM dense/semi-dense 경로로 만든다.
- point는 rectified cam0 frame에서 original cam0 frame으로 되돌린 뒤, `T_cam0_imu`와 EKF orientation으로 gravity-aligned local frame에 둔다.
- translation은 segmentation 기준에는 넣지 않는다. IMU 원점 기준 local frame에서 floor가 sensor 아래쪽에 오도록 유지한다.
- v1 wall은 `left/right/front`가 아니라 단일 `Wall` 라벨로 둔다.

---

## 11. Results

빌드 확인:

```powershell
wsl bash -lc 'cd /mnt/d/02_research/04_cpp_LC-EKF_VIO && cmake -S . -B build_depth_verify -DCMAKE_BUILD_TYPE=Release && cmake --build build_depth_verify -j$(nproc)'
```

결과:

```text
[100%] Built target run_vio
[100%] Built target run_euroc
```

KITTI raw 1-frame sanity check:

```powershell
wsl bash -lc 'cd /mnt/d/02_research/04_cpp_LC-EKF_VIO && ./build_depth_verify/run_vio config/kitti_raw_2011_09_26_drive_0117_local_1frame.yaml --segment-depth --segment-every 1'
```

출력:

```text
depth[0] floor=633 wall=1118 other=13898
Frames processed : 1
Depth segmented  : 1
```

생성 파일:

```text
results/kitti_raw/2011_09_26_drive_0117/sanity_1frame/depth_segments/0000000000.ply
```

PLY header 기준 point 수:

```text
element vertex 15649
```

---

## 12. Retrospective

이번 v1은 전체 기능의 완성이 아니라 **실행 가능한 geometry baseline**을 만든 것이다.

확인된 점:

- 기존 `StereoTracker`의 rectification 결과를 재사용하면 별도 calibration 중복을 줄일 수 있다.
- SGBM disparity만으로도 KITTI 1-frame에서 floor/wall 후보 point count가 생성된다.
- LiDAR segmentation과 같은 `SegmentedCloud`, `SegmentLabel`, PLY writer, Rerun color path를 재사용할 수 있다.

다음 개선 후보:

- Rerun-enabled build에서 `/depth/segments`를 실제 viewer로 확인한다.
- floor count가 낮은 편이므로 SGBM parameter, stride, floor threshold를 조정한다.
- wall은 단일 plane 하나만 잡는 구조이므로, 장면에 따라 multi-wall extraction으로 확장한다.
- RGB-D reader를 추가할 때도 point cloud 이후는 같은 `StereoDepthSegmenter` 계열 segmentation 경로를 재사용한다.

---

## 13. Sparse VO Feature와 Dense Depth Cloud의 차이

Rerun에서 `/depth/segments`를 보면 camera feature보다 훨씬 많은 point가 보인다. 이것은 정상이다.

현재 파이프라인에는 서로 다른 두 개의 point 흐름이 있다.

```text
VO feature 흐름
  left/right image
    -> ORB/KLT feature matching
    -> 약 200~250개 tracked feature 유지
    -> PnP pose estimation
    -> EKF VO update

Depth segmentation 흐름
  left/right rectified image
    -> StereoSGBM disparity
    -> image_stride 간격으로 dense/semi-dense depth 샘플링
    -> 수천~수만 개 3D point 생성
    -> gravity-aligned RANSAC floor/wall/other segmentation
```

따라서 다음 두 Rerun entity는 같은 의미가 아니다.

| Rerun entity | 의미 | 일반적인 point 수 |
|---|---|---|
| `/camera/left_rectified/tracked_features` | VO pose 추정에 쓰는 sparse tracked feature | 수백 개 |
| `/depth/segments` | plane segmentation에 쓰는 stereo depth cloud | 수천~수만 개 |

1-frame sanity check에서도 이 차이가 확인됐다.

```text
tracked features ~= 250
depth[0] floor=633 wall=1118 other=13898
depth cloud total=15649
```

즉 `/depth/segments`의 point가 feature보다 많은 이유는, feature를 복사하거나 부풀린 것이 아니라 **별도 SGBM depth estimation 경로에서 image grid를 샘플링해 만든 point cloud**이기 때문이다.

이 분리는 설계상 필요하다.

- VO는 오래 추적되고 PnP에 안정적인 sparse correspondence가 중요하다.
- floor/wall segmentation은 평면 fitting을 위해 넓은 면을 덮는 많은 3D point가 필요하다.
- 같은 stereo pair를 입력으로 쓰지만, 목적이 다르므로 feature state와 segmentation cloud를 같은 데이터로 취급하면 안 된다.

정리하면:

```text
feature point != depth segmentation point
```

Stereo camera는 하나지만, 그 이미지에서 두 종류의 geometry product가 나온다.

1. pose estimation용 sparse feature
2. scene structure segmentation용 dense/semi-dense depth cloud

---

## 14. Stereo Depth Point Cloud 원리

Stereo depth point cloud의 기본 원리는 **좌우 이미지에서 같은 3D 점이 보이는 x 좌표 차이**를 이용해 깊이를 계산하는 것이다.

### 14.1 Stereo Camera Geometry

스테레오 카메라는 두 카메라가 일정 거리만큼 떨어져 있다.

```text
left camera  <---- baseline B ---->  right camera
```

여기서 `B`는 baseline이다. 단위는 meter다.

같은 3D 점 `P`는 left image와 right image에서 서로 다른 x 좌표에 투영된다.

```text
left image:   u_l
right image:  u_r
```

두 x 좌표의 차이를 disparity라고 한다.

```text
d = u_l - u_r
```

카메라가 rectification되어 있으면 같은 3D 점은 좌우 이미지에서 거의 같은 y 좌표에 놓인다.

```text
v_l ~= v_r
```

그래서 correspondence search가 2D 영역 검색이 아니라 같은 scanline 위의 1D 검색으로 단순해진다.

```text
before rectification:
  correspondence search = 2D search

after rectification:
  correspondence search = horizontal 1D search
```

현재 코드에서도 `StereoTracker`가 left/right image를 rectification한 뒤, `StereoDepthSegmenter`가 rectified image를 받아 disparity를 계산한다.

---

### 14.2 Depth Formula

Rectified stereo에서 depth는 다음 식으로 계산된다.

```text
Z = fx * B / d
```

| 기호 | 의미 |
|---|---|
| `Z` | camera frame에서의 depth |
| `fx` | rectified camera focal length in pixels |
| `B` | stereo baseline in meters |
| `d` | disparity in pixels |

직관:

```text
disparity가 크다   -> 가까운 물체
disparity가 작다   -> 먼 물체
disparity가 0 근처 -> depth가 매우 크거나 불안정
```

따라서 먼 물체일수록 작은 disparity 오차도 큰 depth 오차로 증폭된다.

예를 들어:

```text
Z = fx * B / d
```

에서 `d`가 작아질수록 `Z`는 급격히 커진다. 그래서 depth segmentation에서는 다음 필터가 필요하다.

```text
min_depth_m <= Z <= max_depth_m
d > minimum valid disparity
```

현재 v1 기본값:

```yaml
depth_segmentation:
  min_depth_m: 0.5
  max_depth_m: 30.0
```

---

### 14.3 Pixel to 3D Projection

disparity에서 depth `Z`를 얻은 뒤, 픽셀 좌표 `(u, v)`를 3D 점으로 되돌린다.

```text
X = (u - cx) * Z / fx
Y = (v - cy) * Z / fy
Z = fx * B / d
```

| 기호 | 의미 |
|---|---|
| `u, v` | rectified left image pixel coordinate |
| `cx, cy` | rectified principal point |
| `fx, fy` | rectified focal length |
| `X, Y, Z` | rectified camera frame의 3D point |

즉 하나의 유효한 disparity pixel은 하나의 3D point가 된다.

```text
pixel (u, v) + disparity d
  -> depth Z
  -> point (X, Y, Z)
```

현재 코드 위치:

```text
src/depth/stereo_depth_segmenter.cpp
  make_gravity_aligned_cloud()
```

---

### 14.4 Dense와 Semi-dense의 차이

Stereo depth는 얼마나 많은 픽셀에서 depth를 만들 것인지에 따라 dense 또는 semi-dense로 부를 수 있다.

| 방식 | 의미 | 장점 | 단점 |
|---|---|---|---|
| Dense | 거의 모든 픽셀에서 disparity/depth 계산 | 표면을 촘촘히 복원 | 계산량 큼, noise도 많음 |
| Semi-dense | 일부 픽셀만 사용 | 계산량 감소, segmentation에는 충분할 수 있음 | 세밀한 표면은 덜 촘촘함 |
| Sparse | feature point만 depth 계산 | VO에 적합, 빠름 | plane segmentation에는 점 수 부족 |

현재 구현은 StereoSGBM으로 dense disparity map을 계산한 뒤, `image_stride`로 샘플링한다.

```yaml
depth_segmentation:
  image_stride: 4
```

즉 전체 픽셀을 모두 point cloud로 만들지 않는다.

예를 들어 KITTI image 크기가 약 `1242 x 375`라고 하면:

```text
전체 픽셀 수 ~= 465,750
stride=4 샘플 후보 수 ~= 465,750 / 16 ~= 29,109
유효 disparity/depth 필터 후 point 수 ~= 10,000~20,000 수준
```

그래서 현재 결과는 dense라기보다 **semi-dense stereo depth cloud**에 가깝다.

---

### 14.5 StereoSGBM의 역할

현재 v1에서는 OpenCV `StereoSGBM`으로 disparity map을 만든다.

```text
rectified left image
rectified right image
  -> StereoSGBM
  -> disparity map
```

SGBM은 각 픽셀에 대해 같은 scanline 위에서 matching cost가 가장 좋은 right image 위치를 찾고, 주변 픽셀과의 smoothness를 고려해 disparity를 추정한다.

개념적으로는 다음 문제를 푸는 것이다.

```text
각 pixel p에 대해 disparity d를 찾는다.

좋은 d:
  left(p)와 right(p - d)의 appearance가 비슷함
  주변 pixel의 disparity와 너무 급격히 다르지 않음
```

그래서 SGBM에는 다음 성격의 파라미터가 있다.

| 파라미터 | 의미 |
|---|---|
| `sgbm_num_disparities` | 탐색할 disparity 범위 |
| `sgbm_block_size` | matching cost를 계산할 local window 크기 |
| `sgbm_uniqueness_ratio` | 애매한 match를 거르는 정도 |
| `sgbm_speckle_window_size` | 작은 isolated disparity blob 제거 |
| `sgbm_speckle_range` | speckle filtering 민감도 |

현재 기본값:

```yaml
depth_segmentation:
  sgbm_min_disparity: 0
  sgbm_num_disparities: 192
  sgbm_block_size: 5
  sgbm_uniqueness_ratio: 10
  sgbm_speckle_window_size: 100
  sgbm_speckle_range: 2
```

---

### 14.6 Gravity-aligned Frame으로 변환하는 이유

Stereo depth에서 얻은 point는 처음에는 rectified camera frame에 있다.

```text
P_rect = [X, Y, Z]
```

하지만 camera frame의 축은 카메라 장착 자세에 따라 달라진다. 바닥/벽을 안정적으로 구분하려면 카메라 축이 아니라 **중력 방향**을 기준으로 해야 한다.

현재 변환 흐름:

```text
P_rect
  -> P_cam0
  -> P_imu
  -> P_gravity
```

사용하는 정보:

| 정보 | 역할 |
|---|---|
| rectified-to-cam0 rotation | rectified frame을 original cam0 frame으로 복원 |
| `T_cam0_imu` | cam0 frame과 IMU frame 사이 변환 |
| `R_world_imu` | EKF가 추정한 IMU orientation |

gravity-aligned frame에서는 `z`축을 중력 기준 vertical axis처럼 쓸 수 있다.

그러면 floor/wall 판정이 단순해진다.

```text
floor normal: gravity axis와 거의 평행
wall normal:  gravity axis와 거의 수직
```

수식으로는 plane normal `n`과 gravity axis `g`의 내적을 본다.

```text
floor: |n dot g| >= threshold_floor
wall:  |n dot g| <= threshold_wall
```

현재 기본값:

```yaml
floor_normal_min_abs_dot_gravity: 0.90
wall_normal_max_abs_dot_gravity: 0.25
```

---

### 14.7 Plane Fitting과 Floor/Wall 분류

Point cloud가 만들어지면 RANSAC으로 plane을 찾는다.

평면 방정식:

```text
n^T p + d = 0
```

| 기호 | 의미 |
|---|---|
| `n` | plane normal |
| `p` | 3D point |
| `d` | plane offset |

한 점이 plane에 얼마나 가까운지는 다음 거리로 본다.

```text
distance = |n^T p + d|
```

RANSAC 흐름:

```text
1. 후보 point 중 3개를 랜덤 선택
2. plane hypothesis 생성
3. 모든 후보 point와 plane distance 계산
4. threshold 안에 들어오는 point를 inlier로 계산
5. 가장 inlier가 많은 plane 선택
6. normal 방향, inlier 수, extent 조건으로 최종 accept/reject
```

Floor:

```text
candidate height range:
  floor_search_min_z <= p.z <= floor_search_max_z

normal condition:
  |n dot gravity_axis| >= floor_normal_min_abs_dot_gravity
```

Wall:

```text
candidate height range:
  wall_min_z <= p.z <= wall_max_z

normal condition:
  |n dot gravity_axis| <= wall_normal_max_abs_dot_gravity
```

현재 v1은 먼저 floor plane을 찾고, floor inlier를 라벨링한 뒤, 남은 point에서 wall plane을 찾는다.

```text
depth cloud
  -> floor RANSAC
  -> mark Floor
  -> remaining points
  -> wall RANSAC
  -> mark Wall
  -> rest = Other
```

---

### 14.8 현재 구현의 한계

현재 구현은 baseline이다. 다음 한계를 가진다.

- SGBM disparity quality에 segmentation 품질이 강하게 의존한다.
- textureless floor, 반사면, 먼 벽에서는 depth noise가 커진다.
- wall은 현재 단일 plane만 찾는다.
- image mask 형태의 semantic segmentation이 아니라 3D point cloud label이다.
- point cloud를 누적 map으로 관리하지 않고 frame 단위로 처리한다.

그래도 이 방식은 중요한 baseline이다.

이유:

- 딥러닝 없이 설명 가능한 geometry만으로 동작한다.
- IMU gravity prior를 직접 활용한다.
- LiDAR 없이 stereo만으로 floor/wall 후보를 얻을 수 있다.
- Rerun에서 point-level 결과를 즉시 검증할 수 있다.
