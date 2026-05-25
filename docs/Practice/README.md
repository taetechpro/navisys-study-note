---
title: "Practice — 학습 진행 추적"
date: 2026-05-25
type: moc
tags: [moc, practice, learning]
related:
  - "[[README]]"
  - "[[OBSIDIAN]]"
status: active
---

# `docs/Practice/` — 학습 진행 추적

각 트랙의 **stage 진행 상황 + 사고 변화 + 누적 산출물** 을 시계열로 보존. 미래의 본인이 "내가 뭐 했는지 감이 안 잡힘" 상태가 되지 않도록.

## 트랙

| 트랙 | 진행 추적 노트 | 상태 | Stage |
|---|---|---|---|
| Lidar 클린룸 재구현 | [[LidarSegmenter session_progress]] | ✅ 완료 (2026-05-22) | A~H, 8/8 |
| TC MSCKF OpenVINS 포트 | [[tc_msckf_port_progress]] | 🚧 진행 중 | P1+P2 / P1~P6 |

## Stage 노트 (트랙 내 단계별)

- [[stage_a_header_basics]] — C++ 헤더 기본 (Lidar Stage A)
- [[stage_d_ransac]] — RANSAC 코어 3함수 (Lidar Stage D)
- (이외 Stage B/C/E/F/G/H 는 `LidarSegmenter session_progress` 본문에 흡수됨)

## 컨벤션

새 트랙을 시작할 때:

1. `<track>_progress.md` 파일 1개 — 진행률 박스 + 사고 변화 + 산출물 인덱스 + 다음 액션 + 메타 학습 관찰
2. 필요 시 stage 별 깊이 노트 (`stage_X_<topic>.md`)
3. 본 README 표에 한 줄 추가
4. `docs/README.md` MOC 의 "학습 노트" 항목에도 등록

기존 lidar 진행 노트가 효과적이었던 패턴을 mirror 하면 됨 (LidarSegmenter session_progress.md 가 참조 표준).

## 관련

- [[README]] — Docs Vault 마스터 MOC
- [[OBSIDIAN]] — Vault 사용 가이드
