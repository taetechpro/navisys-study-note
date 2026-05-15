# Docs Index

이 폴더는 LC-EKF VIO 구현, 실험, 인사이트를 모아 두는 작업 노트다.  
루트에는 이 인덱스만 두고, 문서는 목적별 폴더에서 찾는다.

> **현재 연구 단계 (2026-05-15~):** LC EKF-VIO baseline 보존 + Seg-aided **TC MSCKF** 전환 준비 중. `include/msckf/` 와 `include/plane_constraint/` 가 다음 구현 대상. 자세한 결정 배경은 메모리 `[[tc-msckf-pivot]]`, `[[ov-plane-benchmark]]` 참조.

## 먼저 볼 것

| 목적 | 문서 |
|---|---|
| 오늘 한 판단/실험을 복습 | `insight/` |
| Stereo/RGB-D + IMU 바닥/벽 분류 이해 | `insight/20260506_stereo_rgbd_imu_floor_wall_segmentation_plan.md` |
| KITTI raw 전체 파이프라인 실행 | `guides/kitti_full_pipeline_guide.md` |
| Rerun 기록/시각화 | `guides/rerun_cpp_visualization_guide.md` |
| 제로베이스 빌드 절차 | `manuals/zero_base_build_manual.pdf` |

## Structure

| Path | Purpose |
|---|---|
| `guides/` | 실행 방법, 파이프라인 사용법, Rerun 사용법 |
| `insight/` | 날짜별 판단 기록, 실험 노트, 구현 인사이트 |
| `theory/` | 논문, 방법론, 수식 배경 자료 |
| `manuals/` | 긴 매뉴얼, LaTeX 원본, 생성 PDF |
| `assets/` | 문서 이미지, 리뷰 이미지, 이슈 캡처 |
| `build/` | LaTeX 중간 산출물. 추적 대상이 아니라 재생성 가능 |

## Current Reading Path

1. `insight/20260506_stereo_rgbd_imu_floor_wall_segmentation_plan.md`
2. `guides/rerun_cpp_visualization_guide.md`
3. `guides/kitti_full_pipeline_guide.md`
4. `manuals/zero_base_build_manual.pdf`

## Notes

- 새 실험에서 배운 판단 기준은 먼저 `insight/`에 남긴다.
- 실행 명령과 재현 절차는 `guides/`에 둔다.
- 논문 PDF와 수식 배경은 `theory/`에 둔다.
- 발표/문서 이미지와 이슈 캡처는 모두 `assets/` 아래로 모은다.
