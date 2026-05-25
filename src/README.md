# `src/` — 구현 모듈

`include/` 의 헤더와 짝지어진 구현 파일. 모두 `libvio_core.a` 한 라이브러리로 빌드됨 (`CMakeLists.txt` 참조).

## 모듈 자산 카테고리

자산 분류 기준은 [`docs/PLAN/README.md`](../docs/PLAN/README.md) 및 작업 이력. 각 모듈의 **검증 상태와 결정 배경**은 `docs/insight/` 의 해당 날짜 노트에서 확인.

| 모듈 | 카테고리 | 역할 | 검증 |
|---|---|---|---|
| `io/` | A. 검증 본체 | EuRoC / KITTI raw 데이터 reader | EuRoC + KITTI 첫 프레임 OK |
| `ekf/` | A. 검증 본체 | LC EKF VIO (IMU propagator + LC update) | EuRoC ATE ~59 cm, KITTI ATE ~153 cm |
| `frontend/` | A. 검증 본체 | Stereo KLT tracker | LC 파이프라인 통합 |
| `lidar/` | A. 검증 본체 | RANSAC 기반 floor/wall plane segmenter | KITTI floor 법선 z=0.9995 / d=1.70 m |
| `depth/` | A. 검증 본체 | Stereo depth segmenter (RGB-D 호환) | 통합 빌드 |
| `msckf/` | **B. 포트 (미검증)** | OpenVINS types/utils/cam/state 포트 (P1+P2) | 빌드 ✅, 런타임 ❌ — P5 에서 검증 예정 |
| `plane_constraint/` | 스캐폴딩 | TC MSCKF plane pseudo-measurement (예정) | 빈 디렉토리 |

## 빌드

표준 환경: WSL Ubuntu (`/mnt/d/02_research/04_cpp_seg_msckf_vio`).

```bash
wsl -- bash -c "cd /mnt/d/02_research/04_cpp_seg_msckf_vio && cmake -S . -B build && cmake --build build --target vio_core -j"
```

자세한 단계는 [`docs/guides/kitti_full_pipeline_guide.md`](../docs/guides/kitti_full_pipeline_guide.md).

## 진행 중인 작업

- TC MSCKF 포트 (P1+P2 완료, P3~P6 예정) — 진행 상황: [`docs/Practice/tc_msckf_port_progress.md`](../docs/Practice/tc_msckf_port_progress.md)
- 전체 로드맵 (9주): [`docs/PLAN/README.md`](../docs/PLAN/README.md)
