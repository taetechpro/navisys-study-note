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
