# Insight Notes

날짜별 판단 기록, 실험 회고, 구현하면서 얻은 인사이트를 둔다.

이 폴더의 목적은 “무엇을 했는가”보다 **왜 그렇게 판단했는가**를 남기는 것이다.

## Current Reading Path

| File | Focus |
|---|---|
| _(예정)_ `20260515_tc_msckf_pivot_decision.md` | LC → TC MSCKF 전환 결정과 ov_plane 한계 분석. 차주 ov_plane `UpdaterMSCKF.cpp` 분석 후 작성 |
| `20260514_lc_ekf_cam_imu_fusion.md` | Cam+IMU 가 LC-EKF 안에서 어떻게 융합되어 pose 가 되는가 (수식·코드·직관 통합) |
| `20260514_ransac_lidar_segmentation_math.md` | LiDAR floor/wall segmentation 의 RANSAC 수학적 정식화 (코드와 1:1 매핑) |
| `20260506_stereo_rgbd_imu_floor_wall_segmentation_plan.md` | Stereo/RGB-D + IMU 기반 floor/wall segmentation 계획, 구현, 이론 |
| `20260504_kitti_lidar_floor_wall_segmentation.md` | KITTI LiDAR floor/wall segmentation 구현 기록 |
| `20260501_stereo_rectification_left_feature_state.md` | Stereo rectification과 left feature state 구조 |
| `260430_lc_ekf_vio_insights.md` | LC-EKF VIO 초기 구현 인사이트 |
| `260430_ppt_guide.md` | 발표/정리 방향 |

## Note Rule

- 결정 이유를 남긴다.
- 버린 대안과 실패 이유도 남긴다.
- 실행 명령, 결과 수치, Rerun/PLY 위치를 함께 남긴다.
- 나중에 재사용할 판단 기준은 `Insights` 섹션으로 분리한다.
