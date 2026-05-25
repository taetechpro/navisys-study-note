---
title: "Docs Vault — 마스터 MOC"
date: 2026-05-22
type: moc
tags: [moc, vault-root]
related:
  - "[[OBSIDIAN]]"
status: reviewed
---

# Docs Vault — 마스터 MOC

LC-EKF VIO 구현, 실험, 인사이트, 클린룸 학습을 모은 작업 vault.
**Obsidian 으로 열 때:** [[OBSIDIAN]] 가이드 먼저 읽기 (Vault 설정 + LLM 플러그인 + 컨벤션).

> **현재 연구 단계 (2026-05-22 기준)**
> - ✅ LC-EKF VIO baseline 구현 + 실험 완료 (EuRoC ATE ~59cm, KITTI ATE ~153cm)
> - ✅ Lidar floor/wall segmentation 클린룸 재구현 학습 완료 (Stage A~H, [[LidarSegmenter session_progress]])
> - 🚧 **Seg-aided TC MSCKF** 전환 준비 — `include/msckf/` + `include/plane_constraint/` 다음 구현 대상
> - 핵심 결정 배경: [[insight/README|insight 노트들]], 메모리의 `tc-msckf-pivot` / `ov-plane-benchmark`

---

## 영역별 진입점 (MOC)

| 영역 | MOC 노트 | 주제 |
|---|---|---|
| 🧠 **판단/실험 기록** | [[insight/README]] | 날짜별 결정, 실험 회고, 구현 인사이트 |
| 🎓 **학습 노트 MOC** | [[Practice/README]] | 트랙별 학습 진행 추적 (lidar 완료, msckf 진행) |
| 🚧 **TC MSCKF 포트 진행** | [[Practice/tc_msckf_port_progress]] | OpenVINS 핵심 포트 P1~P6 (현재 2/6 완료) |
| 🗓 **9주 연구 로드맵** | [[PLAN/README]] | Phase 0 (W1) → Phase 3 (W7~W9) 일자별 일정 |
| 🛠 **실행 가이드** | [[guides/README]] | 빌드, 파이프라인, Rerun 시각화 |
| 📘 **매뉴얼** | [[manuals/README]] | LaTeX 원본, 긴 절차서, PDF |
| 📚 **이론 자료** | [[theory/README]] | 외부 논문, 수식 배경 |
| 🖼 **이미지 자산** | [[assets/README]] | 다이어그램, 캡처, 발표 이미지 |

---

## 빠른 진입 — "지금 뭘 하려는데?"

| 목적 | 진입 노트 |
|---|---|
| TC MSCKF 전환 배경 이해 | [[insight/20260514_lc_ekf_cam_imu_fusion]] (LC-EKF 융합 기준), 메모리 `tc-msckf-pivot` |
| Stereo/RGB-D + IMU 바닥/벽 분류 이론 | [[insight/20260506_stereo_rgbd_imu_floor_wall_segmentation_plan]] |
| RANSAC LiDAR segmentation 수식↔코드 매핑 | [[insight/20260514_ransac_lidar_segmentation_math]] |
| VIO/LVIO/MSCKF 사고 치트시트 (시각적) | [[insight/20260520_vio_lvio_msckf_thinking_cheatsheet]] |
| **TC MSCKF VIO 학습 spine — 개념·수식 전반** | [[insight/20260522_tc_msckf_overview]] |
| **OpenVINS → src/msckf/ 포트 실행 플랜** | [[insight/20260522_tc_msckf_port_plan]] |
| **TC MSCKF 포트 진행 추적 (성장 가시화)** | [[Practice/tc_msckf_port_progress]] |
| Semantic plane normal + gravity prior | [[insight/20260509_semantic_plane_normal_gravity_prior]] |
| 클린룸 학습 결과 보기 | [[LidarSegmenter session_progress]] |
| RANSAC C++ 학습 노트 (45 가지 문법) | [[Practice/stage_d_ransac]], [[Practice/stage_a_header_basics]] |
| KITTI raw 파이프라인 실행 | [[guides/kitti_full_pipeline_guide]] |
| Rerun 시각화 사용법 | [[guides/rerun_cpp_visualization_guide]] |
| 제로베이스 빌드 절차 | [[manuals/zero_base_build_manual.pdf]] (또는 `.tex`) |

---

## Structure

| Path | Purpose | MOC |
|---|---|---|
| `insight/` | 날짜별 판단/실험 노트, 구현 인사이트 | [[insight/README]] |
| `Practice/` | 학습 진행 추적 + Stage 노트 (lidar, msckf) | [[Practice/README]] |
| `PLAN/` | 9주 연구 로드맵 (Phase 0~3) | [[PLAN/README]] |
| `guides/` | 실행 방법 / 파이프라인 / Rerun | [[guides/README]] |
| `manuals/` | 긴 매뉴얼, LaTeX 원본, 생성 PDF | [[manuals/README]] |
| `theory/` | 논문, 방법론, 수식 배경 | [[theory/README]] |
| `assets/` | 이미지, 다이어그램, 이슈 캡처 | [[assets/README]] |
| `build/` | LaTeX 중간 산출물 (.gitignore 처리, 재생성 가능) | — |

---

## 워크플로우 컨벤션

### 새 결정/실험을 했을 때
→ `insight/YYYYMMDD_<주제>.md` 작성
- frontmatter (type: insight, tags, related) 박기 — 템플릿: [[OBSIDIAN]] § 3
- "**왜 그렇게 판단했는가**" 중심 (무엇을 했는지보다)
- 버린 대안 + 실패 이유도 기록

### 실행 명령 / 재현 절차를 정리할 때
→ `guides/<목적>_guide.md` 작성
- 바로 복붙 가능한 PowerShell / Bash 명령 위주
- 결과 수치 / 출력 위치 명시

### 긴 정식 매뉴얼을 만들 때
→ `manuals/<제목>.tex` (LaTeX) + PDF 빌드
- 발표 / 외부 공유용 분량 (>10페이지)

### 외부 논문 / 자료 정리
→ `theory/papers/` 에 PDF 보관
- 본인이 이해한 내용은 `insight/` 에 별도 노트

---

## 다음 작업할 영역 (planted seeds)

존재 안 하는 wiki-link 는 회색 표시 + Ctrl+클릭 시 자동 생성 (= 미래의 자리 미리 표시):

- [[insight/20260522_tc_msckf_overview]] ✅ — TC MSCKF 학습 spine (개념·수식 전반, 2026-05-22 작성)
- Stage 1~3 deep dive (예정, [[project_tc_msckf_curriculum]] 참조):
  - `tc_msckf_clone_augment.md` — Stage 1a
  - `tc_msckf_marginalize.md` — Stage 1b
  - `tc_msckf_feature_jacobian.md` — Stage 2a
  - `tc_msckf_nullspace.md` — Stage 2b
  - `tc_msckf_compress_update.md` — Stage 2c
  - `tc_msckf_orchestration.md` — Stage 3
- [[insight/20260615_plane_constraint_msckf_update]] — 평면 제약 MSCKF update 수식 (후속 사이클)
- [[insight/ov_plane_updater_msckf_analysis]] — ov_plane 비교 분석 (후속 사이클)
- [[guides/tc_msckf_pipeline_guide]] — TC MSCKF 실행 절차 (구현 후 작성)

---

## 관련
- [[OBSIDIAN]] — Vault 사용 가이드 (플러그인, frontmatter, 태그 컨벤션)
- 메모리 `project_main_research` — 졸업 연구 메인 컨텍스트
- 메모리 `tc-msckf-pivot` — LC→TC 전환 결정 기록
