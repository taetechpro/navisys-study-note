# Rerun C++ 시각화 연동 기록

이 문서는 `run_vio` 실행 결과를 Rerun `.rrd` 파일로 저장하고 Windows PowerShell에서 여는 과정을 정리한다.

대상 실행:

```text
D:\02_research\06_cpp_LC-EKF_VIO_kitti_raw\config\kitti_raw_2011_09_26_drive_0117.yaml
```

생성된 Rerun 기록:

```text
D:\02_research\06_cpp_LC-EKF_VIO_kitti_raw\results\kitti_raw_2011_09_26_drive_0117\vio_rerun.rrd
```

## 1. 전체 구조

Rerun 연동은 기존 VIO 실행 코드를 완전히 바꾸지 않고, 선택 기능으로 추가했다.

기존 실행:

```bash
./build/run_vio config/kitti_raw_2011_09_26_drive_0117.yaml
```

Rerun 기록 생성 실행:

```bash
./build_rerun/run_vio \
  config/kitti_raw_2011_09_26_drive_0117.yaml \
  --rerun-save results/kitti_raw_2011_09_26_drive_0117/vio_rerun.rrd \
  --rerun-image-every 10
```

핵심은 다음 두 단계를 분리한 것이다.

1. WSL/Linux 쪽에서 C++ VIO를 실행하면서 `.rrd` 파일을 만든다.
2. Windows PowerShell 쪽에서 Rerun viewer로 `.rrd` 파일을 연다.

이렇게 분리한 이유는 C++ 프로젝트가 WSL 기준으로 빌드되어 있고, Rerun viewer는 Windows Python 환경에 이미 설치되어 있었기 때문이다.

## 2. CMake에 Rerun SDK를 선택 기능으로 추가

수정 파일:

```text
CMakeLists.txt
```

추가한 옵션:

```cmake
option(ENABLE_RERUN "Enable optional Rerun logging support" OFF)
set(RERUN_SDK_URL
    "https://github.com/rerun-io/rerun/releases/download/0.30.2/rerun_cpp_sdk.zip"
    CACHE STRING "URL for the Rerun C++ SDK bundle")
```

Rerun SDK는 `FetchContent`로 내려받는다.

```cmake
if(ENABLE_RERUN)
    include(FetchContent)
    FetchContent_Declare(rerun_sdk URL ${RERUN_SDK_URL})
    FetchContent_MakeAvailable(rerun_sdk)
endif()
```

그리고 `run_vio`, `run_euroc` 타깃에만 조건부로 연결한다.

```cmake
if(ENABLE_RERUN)
    target_link_libraries(run_vio PRIVATE rerun_sdk)
    target_compile_definitions(run_vio PRIVATE LC_VIO_WITH_RERUN=1)
endif()
```

이 구조의 의미:

- `ENABLE_RERUN=OFF`: 기존 빌드와 동일하게 동작한다.
- `ENABLE_RERUN=ON`: Rerun SDK를 링크하고 `LC_VIO_WITH_RERUN` 매크로가 켜진다.
- Rerun SDK가 없어도 기존 `build/` 빌드는 깨지지 않는다.
- Rerun 전용 빌드는 `build_rerun/`에 따로 만든다.

## 3. run_vio에 추가한 명령행 옵션

수정 파일:

```text
apps/run_vio.cpp
```

추가 옵션:

```text
--rerun-save <file.rrd>       Rerun 기록을 파일로 저장
--rerun-spawn                 C++ 실행 중 Rerun viewer를 직접 spawn
--rerun-connect               이미 실행 중인 Rerun viewer에 gRPC 연결
--rerun-image-every <N>       rectified left image를 N프레임마다 기록
```

이번 실행에서는 가장 안정적인 파일 저장 방식을 썼다.

```bash
--rerun-save results/kitti_raw_2011_09_26_drive_0117/vio_rerun.rrd
--rerun-image-every 10
```

`--rerun-image-every 10`을 둔 이유:

- 궤적, GT, feature count, speed는 매 프레임 저장한다.
- 이미지는 용량이 크므로 10프레임마다 저장한다.
- KITTI 660프레임 전체를 기록해도 `.rrd`가 약 32 MB 정도로 유지된다.

## 4. StereoTracker에서 Rerun용 데이터 접근자 추가

수정 파일:

```text
include/frontend/stereo_tracker.hpp
```

추가한 함수:

```cpp
const std::vector<cv::Point2f>& tracked_points() const { return prev_pts_l_; }
const cv::Mat& rectified_left_image() const { return prev_img_l_; }
```

이유:

- `prev_pts_l_`는 현재 left image에서 추적 중인 2D feature 좌표다.
- `prev_img_l_`는 stereo rectification 이후의 left grayscale image다.
- Rerun에서 image 위에 feature point를 같이 보기 위해 필요하다.

기존 tracker 내부 상태를 복사하지 않고 const reference로만 읽게 했다.

## 5. RerunLogger가 저장하는 데이터

`apps/run_vio.cpp`에 `RerunLogger` 클래스를 추가했다.

프레임마다 기록하는 항목:

| Entity path | 내용 |
|---|---|
| `metrics/tracked_features` | 현재 추적 feature 개수 |
| `metrics/speed_mps` | EKF velocity norm |
| `imu/gyro_x_rad_s` | IMU gyro x [rad/s] |
| `imu/gyro_y_rad_s` | IMU gyro y [rad/s] |
| `imu/gyro_z_rad_s` | IMU gyro z [rad/s] |
| `imu/gyro_norm_rad_s` | IMU gyro norm [rad/s] |
| `imu/accel_x_mps2` | IMU accel x [m/s^2] |
| `imu/accel_y_mps2` | IMU accel y [m/s^2] |
| `imu/accel_z_mps2` | IMU accel z [m/s^2] |
| `imu/accel_norm_mps2` | IMU accel norm [m/s^2] |
| `camera/left_rectified` | rectified left grayscale image |
| `camera/left_rectified/tracked_features` | image 위 feature point |

실행 종료 후 전체 궤적에 Umeyama SE(3) 정렬을 적용한 뒤 기록하는 항목:

| Entity path | 내용 |
|---|---|
| `world/current_est_aligned` | GT 좌표계로 정렬된 현재 EKF 추정 위치 |
| `world/trajectory_est_aligned` | GT 좌표계로 정렬된 EKF 추정 궤적 |
| `world/current_gt` | 현재 GT 위치 |
| `world/trajectory_gt` | GT 궤적 |

Rerun 화면에서 GT와 est를 바로 비교하려면 `world/trajectory_est_aligned`와
`world/trajectory_gt`를 보면 된다. 원래 EKF raw 좌표는 `trajectory_tum.txt`에 남고,
정렬된 좌표는 `trajectory_aligned_tum.txt`에 저장된다.

타임라인:

```cpp
rec_->set_time_sequence("frame", frame_idx);
rec_->set_time_duration_secs("sensor_time", timestamp);
```

`frame`은 0부터 증가하는 정수 프레임 번호이고, `sensor_time`은 KITTI timestamp다.

Rerun 기록 저장은 다음 방식이다.

```cpp
rec_ = std::make_unique<rerun::RecordingStream>("lc_ekf_vio_kitti_raw");
rec_->save(options_.save_path).exit_on_failure();
```

실행이 끝나면 기록을 확실히 flush한다.

```cpp
rec_->flush_blocking(10.0f).exit_on_failure();
```

### 5.1 왜 aligned 궤적을 따로 기록하는가

KITTI GT는 OXTS `lat/lon/alt`를 Mercator 좌표로 바꾼 East-North-Up 계열 좌표다.
반면 EKF 추정 궤적은 첫 cam0/IMU 초기화 기준의 로컬 좌표다. 초기화는 중력 방향으로
roll/pitch는 맞출 수 있지만, yaw는 중력만으로 관측되지 않는다.

따라서 raw `trajectory_tum.txt`와 `gt_tum.txt`를 Rerun에 그대로 겹치면 두 궤적이
크게 벌어져 보일 수 있다. 이 차이는 실제 VIO 오차뿐 아니라 좌표계 yaw/translation
차이를 포함한다.

ATE RMSE는 다음처럼 Umeyama 정렬 후 계산한다.

```cpp
const Eigen::Matrix4d T_gt_est = Eigen::umeyama(est_mat, gt_mat, false);
```

Rerun에서도 같은 기준으로 보기 위해 실행 종료 후 `trajectory_aligned_tum.txt`를 만들고,
그 정렬된 궤적을 `world/trajectory_est_aligned`로 기록한다.

## 6. WSL에서 Rerun 포함 빌드

프로젝트 루트:

```bash
cd /mnt/d/02_research/06_cpp_LC-EKF_VIO_kitti_raw
```

Rerun 빌드 디렉터리를 별도로 만든다.

```bash
cmake -S . -B build_rerun -DENABLE_RERUN=ON -DCMAKE_BUILD_TYPE=Release
cmake --build build_rerun -j4
```

처음 빌드할 때는 Rerun C++ SDK도 같이 내려받고 컴파일하므로 시간이 오래 걸릴 수 있다. 한 번 완료되면 다음 빌드는 훨씬 빠르다.

확인된 configure 출력:

```text
-- Rerun SDK install version: 0.30.2
-- Build files have been written to: /mnt/d/02_research/06_cpp_LC-EKF_VIO_kitti_raw/build_rerun
```

## 7. WSL에서 .rrd 생성

실행 명령:

```bash
cd /mnt/d/02_research/06_cpp_LC-EKF_VIO_kitti_raw

./build_rerun/run_vio \
  config/kitti_raw_2011_09_26_drive_0117.yaml \
  --rerun-save results/kitti_raw_2011_09_26_drive_0117/vio_rerun.rrd \
  --rerun-image-every 10
```

이번 실행 결과:

```text
Frames processed : 660
ATE RMSE         : 152.955 cm
Output           : /mnt/d/02_research/06_cpp_LC-EKF_VIO_kitti_raw/results/kitti_raw_2011_09_26_drive_0117
```

생성 파일:

```text
results/kitti_raw_2011_09_26_drive_0117/vio_rerun.rrd
results/kitti_raw_2011_09_26_drive_0117/trajectory_aligned_tum.txt
results/kitti_raw_2011_09_26_drive_0117/imu_log.txt
```

파일 크기:

```text
32,332,328 bytes
```

## 8. PowerShell에서 viewer 열기

Windows PowerShell에서 실행:

```powershell
cd D:\02_research\06_cpp_LC-EKF_VIO_kitti_raw
python -m rerun_cli results\kitti_raw_2011_09_26_drive_0117\vio_rerun.rrd
```

또는 절대경로:

```powershell
python -m rerun_cli D:\02_research\06_cpp_LC-EKF_VIO_kitti_raw\results\kitti_raw_2011_09_26_drive_0117\vio_rerun.rrd
```

왜 `python -m rerun_cli`를 쓰는가:

- Windows Python에는 `rerun-sdk 0.30.2`가 설치되어 있다.
- 그런데 `rerun`이라는 다른 PyPI 패키지도 설치되어 있어서 `rerun.exe` 명령 이름이 충돌했다.
- `rerun --version`은 잘못된 패키지를 실행할 수 있다.
- `python -m rerun_cli`는 Rerun SDK의 CLI 모듈을 직접 실행하므로 충돌을 피한다.

확인된 버전:

```powershell
python -m rerun_cli --version
```

출력:

```text
rerun-cli 0.30.2
```

## 9. .rrd 파일 검증

viewer를 열기 전에 `.rrd`가 정상인지 확인할 수 있다.

```powershell
python -m rerun_cli rrd verify results\kitti_raw_2011_09_26_drive_0117\vio_rerun.rrd
```

정상 출력:

```text
1 file verified without error.
```

통계 확인:

```powershell
python -m rerun_cli rrd stats results\kitti_raw_2011_09_26_drive_0117\vio_rerun.rrd
```

이번 기록의 주요 통계:

```text
num_entity_paths = 9
num_rows = 4091
/camera/left_rectified: 66
/camera/left_rectified/tracked_features: 66
/imu/accel_x_mps2
/imu/accel_y_mps2
/imu/accel_z_mps2
/imu/accel_norm_mps2
/imu/gyro_x_rad_s
/imu/gyro_y_rad_s
/imu/gyro_z_rad_s
/imu/gyro_norm_rad_s
/world/current_est_aligned
/world/current_gt
/world/trajectory_est_aligned
/world/trajectory_gt
```

이미지를 10프레임마다 저장했기 때문에 660프레임 중 image row가 66개다.
`world/*_aligned`는 전체 실행이 끝난 뒤 한 번 더 타임라인에 기록되므로, viewer에서
GT 좌표계로 정렬된 궤적을 볼 수 있다.

## 10. WSL에서 실패한 이유

다음 명령은 WSL에서 실패했다.

```bash
python3 -m rerun_cli D:\02_research\06_cpp_LC-EKF_VIO_kitti_raw\results\kitti_raw_2011_09_26_drive_0117\vio_rerun.rrd
```

에러:

```text
/usr/bin/python3: No module named rerun_cli
```

원인:

1. `python3`는 WSL 안의 Python이다.
2. `rerun_cli`는 Windows Python에 설치되어 있고 WSL Python에는 설치되어 있지 않았다.
3. WSL에서는 `D:\...` Windows 경로를 그대로 쓰지 않는다.
4. WSL 경로는 `/mnt/d/...` 형식이어야 한다.

WSL에서 굳이 viewer를 열려면 다음처럼 해야 한다.

```bash
python3 -m pip install rerun-sdk
python3 -m rerun_cli /mnt/d/02_research/06_cpp_LC-EKF_VIO_kitti_raw/results/kitti_raw_2011_09_26_drive_0117/vio_rerun.rrd
```

다만 GUI viewer가 뜨려면 WSLg가 정상 동작해야 한다. 현재 환경에서는 Windows PowerShell에서 여는 방식이 더 확실하다.

## 11. 자주 쓰는 명령 요약

WSL에서 기록 생성:

```bash
cd /mnt/d/02_research/06_cpp_LC-EKF_VIO_kitti_raw
cmake -S . -B build_rerun -DENABLE_RERUN=ON -DCMAKE_BUILD_TYPE=Release
cmake --build build_rerun -j4
./build_rerun/run_vio \
  config/kitti_raw_2011_09_26_drive_0117.yaml \
  --rerun-save results/kitti_raw_2011_09_26_drive_0117/vio_rerun.rrd \
  --rerun-image-every 10
```

PowerShell에서 열기:

```powershell
cd D:\02_research\06_cpp_LC-EKF_VIO_kitti_raw
python -m rerun_cli results\kitti_raw_2011_09_26_drive_0117\vio_rerun.rrd
```

PowerShell에서 검증:

```powershell
python -m rerun_cli rrd verify results\kitti_raw_2011_09_26_drive_0117\vio_rerun.rrd
```

## 12. 트러블슈팅

### `rerun` 명령이 이상하게 동작함

증상:

```text
AttributeError: module 'signal' has no attribute 'SIGTTOU'
```

원인:

- `rerun-sdk`가 아니라 다른 PyPI 패키지인 `rerun`이 실행된 것이다.

해결:

```powershell
python -m rerun_cli --version
python -m rerun_cli results\kitti_raw_2011_09_26_drive_0117\vio_rerun.rrd
```

### WSL에서 `No module named rerun_cli`

원인:

- WSL Python에 `rerun-sdk`가 설치되어 있지 않다.

해결 선택지:

```powershell
# 권장: Windows PowerShell에서 실행
python -m rerun_cli D:\02_research\06_cpp_LC-EKF_VIO_kitti_raw\results\kitti_raw_2011_09_26_drive_0117\vio_rerun.rrd
```

또는:

```bash
# WSL에 따로 설치
python3 -m pip install rerun-sdk
python3 -m rerun_cli /mnt/d/02_research/06_cpp_LC-EKF_VIO_kitti_raw/results/kitti_raw_2011_09_26_drive_0117/vio_rerun.rrd
```

### `.rrd` 파일이 열리지 않음

먼저 파일 검증:

```powershell
python -m rerun_cli rrd verify D:\02_research\06_cpp_LC-EKF_VIO_kitti_raw\results\kitti_raw_2011_09_26_drive_0117\vio_rerun.rrd
```

정상이면:

```text
1 file verified without error.
```

검증은 통과하지만 viewer가 안 뜨면 Windows GUI 권한, 백신, 그래픽 드라이버, Rerun viewer 상태를 확인한다.
