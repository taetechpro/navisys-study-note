# 연구 로드맵 — Seg-aided TC MSCKF

**Start:** 2026-05-18 (월) | **Target:** 2026-07-18 (토) | **9주 / 45 평일**

> 평일 = 작업, 주말 = buffer. 당일 작업 일찍 끝나면 다음날 첫 항목을 당겨서 진행해도 됨.
> 각 주 금요일 오후는 **catch-up / 정리 시간**으로 여유 둠.

---

## Phase 키워드 (요약)

- **Phase 0** (W1)         : ov_plane 분석 + fork/직접 결정 + practice 마무리
- **Phase 1** (W2~W4, 3주) : TC MSCKF 본체 (sliding window / track / triangulation / nullspace)
- **Phase 2** (W5~W6, 2주) : Plane constraint 결합 (floor + wall pseudo-measurement)
- **Phase 3** (W7~W9, 3주) : EuRoC 실험 + ov_plane 비교 + 논문 draft

---

## 일자별 일정

### W1 — Phase 0 결정 + practice 마무리

| 날짜 | 작업 |
|---|---|
| 5/18 월 | ov_plane 디렉토리 트리 + `UpdaterMSCKF.cpp` 전체 1회 통독 (이해 안 가도 OK) |
| 5/19 화 | practice Stage B (`.bin` 파싱) · ov_plane Updater 30% 라인 매핑 |
| 5/20 수 | practice Stage C (ROI + finite 필터) · 매핑 60% |
| 5/21 목 | practice Stage D (RANSAC 코어 3함수) · fork vs 직접 결정 초안 |
| 5/22 금 | practice Stage E + F (Floor + Wall) · 결정 `.md` 확정 (`docs/insight/20260522_tc_pivot_decision.md`) |
| 5/23~24 | **buffer** |

**Gate:** Phase 0 결정 .md 1개 산출. practice Stage F 까지 완료.

---

### W2 — Phase 1.1 sliding_window

| 날짜 | 작업 |
|---|---|
| 5/25 월 | `include/msckf/sliding_window.hpp` 헤더 설계 (clone struct, state 확장 layout) |
| 5/26 화 | clone 추가 함수 (`add_clone`) 구현 + smoke test |
| 5/27 수 | clone marginalize (oldest 제거, covariance Schur complement) |
| 5/28 목 | 기존 LC state 와 통합 (msckf_state 확장 형태) |
| 5/29 금 | 빌드 + 1프레임 sanity (state size 변화 print) · 정리 |
| 5/30~31 | **buffer** |

**Gate:** state size 가 frame 마다 늘어나고 max 도달 시 oldest 가 빠져나가는 것 확인.

---

### W3 — Phase 1.2 feature_track

| 날짜 | 작업 |
|---|---|
| 6/01 월 | `include/msckf/feature_track.hpp` 설계 (Track struct, TrackDB) |
| 6/02 화 | track add / extend (KLT 결과 받기) |
| 6/03 수 | track close (lost 처리, max age 정책) |
| 6/04 목 | `frontend/stereo_tracker` 와 wiring |
| 6/05 금 | 빌드 + multi-frame track lifetime 통계 print · 정리 |
| 6/06~07 | **buffer** |

**Gate:** track 의 평균 lifetime > 5 frame, total active track count 안정.

---

### W4 — Phase 1.3-1.4 triangulation + msckf_update

| 날짜 | 작업 |
|---|---|
| 6/08 월 | `include/msckf/triangulation.hpp` (multi-view, Gauss-Newton 수렴) |
| 6/09 화 | triangulation 검증 (KITTI gt 와 reprojection 비교) |
| 6/10 수 | `include/msckf/msckf_update.hpp` (residual + Jacobian 조립) |
| 6/11 목 | nullspace projection (`H_f` QR) + chi² gate |
| 6/12 금 | KITTI drive_0117 1 sequence ATE 측정 · `apps/run_msckf.cpp` 정리 |
| 6/13~14 | **buffer (ATE 미달 시 디버깅용)** |

**🚦 GATE:** KITTI drive_0117 ATE < 153 cm (LC baseline). 미달 시 W5 진입 보류 → buffer 사용해 디버깅.

---

### W5 — Phase 2.1 plane pseudo-measurement

| 날짜 | 작업 |
|---|---|
| 6/15 월 | `include/plane_constraint/plane_pseudo_measurement.hpp` 설계 |
| 6/16 화 | floor pseudo-measurement (`z = n_f^T·p + b_f`, R 계산) |
| 6/17 수 | wall pseudo-measurement |
| 6/18 목 | `SegmentedCloud` → measurement 변환 wrapper |
| 6/19 금 | 빌드 + 1프레임 plane normal / b print · 정리 |
| 6/20~21 | **buffer** |

**Gate:** floor / wall 각각 measurement vector + noise 가 정상값으로 출력.

---

### W6 — Phase 2.2-2.3 Jacobian + update stack

| 날짜 | 작업 |
|---|---|
| 6/22 월 | `include/plane_constraint/plane_jacobian.hpp` (∂z/∂state 유도) |
| 6/23 화 | numerical Jacobian 검증 (analytic vs numeric finite-diff) |
| 6/24 수 | `msckf_update` 에 plane measurement stack 추가 |
| 6/25 목 | KITTI 전체 sequence ATE 측정 (TC vs TC+plane) |
| 6/26 금 | degenerate 구간 (직선 주행) 별도 측정 · 정리 |
| 6/27~28 | **buffer** |

**🚦 GATE:** TC+plane ATE < TC only ATE. degenerate 구간 차이가 더 커야 contribution 성립.

---

### W7 — Phase 3.1 EuRoC 실험

| 날짜 | 작업 |
|---|---|
| 6/29 월 | EuRoC V1_01, V2_03 config 추가 / 점검 |
| 6/30 화 | V1_01 ATE 측정 (LC / TC / TC+plane 3 모드) |
| 7/01 수 | V2_02 ATE 측정 |
| 7/02 목 | V2_03 ATE 측정 + plane active count 통계 |
| 7/03 금 | 결과 표 정리 (ATE × 3 dataset × 3 모드) · 정리 |
| 7/04~05 | **buffer** |

**Gate:** EuRoC 3 dataset 모두에서 결과 표 완성.

---

### W8 — Phase 3.2 ov_plane 직접 비교

| 날짜 | 작업 |
|---|---|
| 7/06 월 | ov_plane 빌드 + 동일 EuRoC sequence 돌리기 |
| 7/07 화 | 비교 표 (ours vs ov_plane vs LC baseline) |
| 7/08 수 | V2_03 PL active 통계 비교 (ov_plane=0.0 vs ours>0) ← **핵심 evidence** |
| 7/09 목 | 그래프 / plot 생성 (`tools/plot_trajectory.py` 확장) |
| 7/10 금 | 결과 sanity check · 정리 |
| 7/11~12 | **buffer** |

**🚦 GATE:** V2_03 에서 ours 의 plane active count > 0 인 figure / table 확보.

---

### W9 — Phase 3.3 논문 draft

| 날짜 | 작업 |
|---|---|
| 7/13 월 | outline + abstract + figure 목록 |
| 7/14 화 | introduction + related work |
| 7/15 수 | methodology (수식 정리, plane pseudo-measurement 유도) |
| 7/16 목 | experiments section + 결과 표 / figure |
| 7/17 금 | discussion + conclusion |
| 7/18 토 | 최종 검토 + 교수님 회람 준비 |

**🚦 FINAL GATE:** 논문 초안 1개 (학회 / IROS / 저널 tier 결정 후 길이 조정).

---

## 가드레일

- 🚦 GATE 통과 못 하면 다음 Phase 진입 금지 — buffer 로 디버깅 우선
- LC 코드 (`include/ekf/`, `src/ekf/`) 절대 수정 금지 — baseline 보존
- Phase 1 안정화 전 plane constraint 건드리지 않기 — 변수 동시 변화 디버깅 지옥
- sliding window 크기 N 은 **첫 hyperparameter** (5 → 10 → 15 순으로 sweep)

---

## 당기기 / 미루기 정책

- **당김:** 당일 끝났으면 다음날 첫 항목 미리 시작 OK. 단 GATE 가 있는 금요일까지 끌지 말 것.
- **미룸:** 막히면 그날 안에 끝장 보지 말고 다음날로 넘김. 주말 buffer 또는 다음주 금요일 catch-up 시간으로 흡수.
- **catch-up 시간:** 매주 금요일 오후 (정리 + 미진행분 흡수).
- **Phase gate buffer:** W4, W6, W8 의 주말은 GATE 디버깅 전용으로 우선 배정.

---

## 병행 학습 슬롯

- W1: `practice/lidar_clean_room/` Stage B → F (Stage G, H 스킵)
- W2~ : 필요 시 `practice/tc_msckf_clean_room/` 신설 (nullspace projection 직접 손코딩용)

---

## 기보유 자산

- LC EKF-VIO (KITTI ATE ~153 cm) — baseline
- KITTI LiDAR floor/wall seg
- Stereo + IMU gravity seg v1
- TC 빈 스캐폴딩: `include/msckf/`, `include/plane_constraint/`, `include/semantic/segment_types.hpp`

---

## 관련

- 메모리: `tc-msckf-pivot`, `ov-plane-benchmark`
- ov_plane: https://github.com/rpng/ov_plane
- 각 Phase 종료 시 본 파일에 실제 진행 / 변경 사항 갱신
