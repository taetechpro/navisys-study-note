# Documentation Layout

이 폴더는 LC-EKF VIO 구현/실험 문서를 모아 둔다.

## Structure

| Path | Purpose |
|---|---|
| `insight/` | 날짜별 분석 메모와 구현 인사이트 |
| `latex/` | 편집 가능한 LaTeX 원본과 주요 PDF |
| `assets/diagram_reviews/` | 문서/발표용 리뷰 이미지 |
| `assets/images/` | 공통 이미지 자료 |
| `issue/` | 이슈 재현/설명용 이미지 |
| `build/` | LaTeX `.aux`, `.log`, `.out`, `.toc` 등 중간 산출물 |

## Current Key Docs

| File | Description |
|---|---|
| `KITTI_FULL_PIPELINE_GUIDE.md` | KITTI raw pipeline 실행 가이드 |
| `RERUN_CPP_VISUALIZATION_GUIDE.md` | Rerun C++ logging/visualization 가이드 |
| `zero_base_build_manual.tex/.pdf` | 제로베이스 빌드 매뉴얼 |
| `latex/kitti_lidar_segmentation_principle.tex/.pdf` | KITTI LiDAR floor/wall segmentation 원리 정리 |
| `insight/20260504_kitti_lidar_floor_wall_segmentation.md` | 오늘 구현한 LiDAR segmentation 요약 |

## Notes

- Root에는 사람이 바로 찾는 핵심 가이드와 매뉴얼만 둔다.
- LaTeX 중간 산출물은 `docs/build/` 아래로 정리한다.
- 이미지 파일은 가능하면 `docs/assets/` 아래에 둔다.
