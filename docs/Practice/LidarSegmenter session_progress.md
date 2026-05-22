---
title: "LidarSegmenter 클린룸 재구현 — 진행 상태"
date: 2026-05-22
type: practice
tags: [lidar, segmentation, cpp-learning, ransac, moc]
related:
  - "[[stage_a_header_basics]]"
  - "[[stage_d_ransac]]"
  - "[[../insight/20260514_ransac_lidar_segmentation_math]]"
status: completed
---

# LidarSegmenter 클린룸 재구현 — 🎉 학습 완료 (2026-05-22)

> **완료일:** 2026-05-22 (시작 2026-05-13, 약 10 일)
> **상태:** Stage A~H 모두 통과, 빌드/실행/PLY 검증 OK
> **다음:** 본 학습은 종료. 다음 졸업 연구 주력 = Seg-aided TC MSCKF (별도 메모 참조)

---

## 진행률 한눈에

```
[A] 헤더 6 type + 자유함수 2 선언 ........ ✅ 완료
[B] load_kitti_velodyne_bin (.bin 파싱) .. ✅ 완료
[C] ROI + finite 필터 .................... ✅ 완료
[D] RANSAC 코어 4 함수 ................... ✅ 완료
[E] Floor 분리 ........................... ✅ 완료
[F] Wall 3종 그리디 피링 ................. ✅ 완료
[G] write_segmented_ply (PLY 저장) ....... ✅ 완료
[H] main_test.cpp + 빌드 + 시각 검증 ..... ✅ 완료
```

진행률: **8/8 stage** 완료 (100%).

## 최종 실행 검증
KITTI drive 0001 첫 프레임으로 학습판 segmenter 실행:
- Loaded 121015 → ROI 86738 → Floor 62933 + Right wall 8694
- **Floor 평면 법선 (-0.009, 0.030, 0.9995)**, d=1.703m
- KITTI Velodyne 실제 마운트 높이 1.73m 와 일치 ✓ (학습판 정확도 검증)

---

## Stage A 결과 (완료)

### 산출물
- `practice/lidar_clean_room/my_lidar_segmenter.hpp` — 84 줄
- `docs/Practice/stage_a_header_basics.md` — 학습 노트 (학습 #1~#9 정리)

### Stage A 에서 익힌 C++ 문법 9 가지
| # | 주제 | 어디 등장 |
|---|---|---|
| 1 | `struct` 와 default member init | `LidarPoint` |
| 2 | `#pragma once` / `#include` | 헤더 시작 |
| 3 | `std::vector<T>` 크기 시스템 | `SegmentedCloud::points` |
| 4 | `std::cout <<` 출력 | (디버그용) |
| 5 | `enum class : underlying_type` | `SegmentLabel` |
| 6 | 옵션 묶음 struct + 단위 컨벤션 | `LidarSegmentationOptions` |
| 7 | 멤버 함수 선언 vs 정의 + `const` 멤버 함수 | `SegmentedCloud::counts()` |
| 8 | `class`, `public:`/`private:`, 생성자, `explicit`, `const T&` | `LidarSegmenter` |
| 9 | 자유 함수 선언 + `void` 반환 | `load_kitti_velodyne_bin`, `write_segmented_ply` |

---

## Stage B 결과 (완료)

### 산출물
- `practice/lidar_clean_room/my_lidar_segmenter.cpp` — 약 40 줄 (`load_kitti_velodyne_bin` 완성)

### Stage B 에서 익힌 C++ 문법 7 가지
| # | 주제 | 어디 등장 |
|---|---|---|
| 10 | `<fstream>`, `std::ifstream` | 파일 읽기 스트림 |
| 11 | `std::ios::binary \| std::ios::ate` | 비트 플래그 OR |
| 12 | `tellg() / seekg()` | 파일 커서 측정/이동 |
| 13 | `constexpr`, `sizeof(T)` | 컴파일 타임 상수, 타입 크기 |
| 14 | `static_cast<T>(...)` | 안전한 타입 변환 (signed↔unsigned) |
| 15 | `reinterpret_cast<char*>(...)` | 메모리를 byte 자루로 재해석 (디스크 ↔ 구조체) |
| 16 | `throw std::runtime_error`, `if (!f)` | 예외 throw + stream 상태 검사 |

### Stage B 의 교훈 (사용자 패턴)
- **같은 scope 실수 두 번 발생**: 시범 코드를 함수 `}` 바깥에 붙임. 다음부턴 복붙 직전 VS Code 하단 상태바의 "함수명" 표시 확인 권장.
- 들여쓰기 / 공백 묻어들어오는 패턴은 Stage B 에서 거의 사라짐.

---

## Stage C 결과 (완료)

### 산출물
- `practice/lidar_clean_room/my_lidar_segmenter.hpp` — `filter_roi` 선언 1줄 추가 (85~86)
- `practice/lidar_clean_room/my_lidar_segmenter.cpp` — `filter_roi` 함수 약 20줄 추가
- 두 함수 (`load_kitti_velodyne_bin`, `filter_roi`) 가 나란히 자유 함수로 공존

### Stage C 에서 익힌 C++ 문법 4 가지
| # | 주제 | 어디 등장 |
|---|---|---|
| 17 | `<cmath>`, `std::isfinite(x)` | NaN/Inf 검사 |
| 18 | range-based for (`for (const auto& p : v)`) | 컨테이너 순회 관용구 |
| 19 | `auto` + `const T&` 조합 | `const auto& p` 표준 패턴 |
| 20 | `.hpp` 와 `.cpp` 동시 수정 (자유 함수 추가) | 선언 vs 정의 분리의 실전 |

### Stage C 에서 배운 디자인 패턴
- **early continue 패턴**: 박스 밖 / NaN → 즉시 `continue` → 들여쓰기 평평하게 유지
- **6면 박스 검사의 부정형**: de Morgan 으로 "박스 안" 을 "박스 밖이면 continue" 로 뒤집어 가독성 ↑
- **NaN/Inf 박멸 디자인**: 입력 단계에서 거르면 후속 RANSAC/EKF 가 안전. 본 repo 의 sanity check 와 같은 철학

### Stage C 의 교훈 (사용자 패턴)
- 시범 코드에서 8칸 들여쓰기로 옮기는 경향이 계속됨 (시범엔 4칸인데). 복붙 후 들여쓰기 한 단계 줄이는 셀프 체크 권장.
- `for` 시작했지만 `}` 빼먹는 brace mismatch 한 번 발생. **중괄호 카운팅 (+1 / −1) 셀프 체크** 가이드 제공.
- 저장(Ctrl+S) 빼먹는 패턴 여전히 1회 발생. VS Code 탭의 `●` 점 확인 습관화 필요.

---

## Stage D 결과 (완료, 2026-05-21)

### 산출물
- `practice/lidar_clean_room/my_lidar_segmenter.hpp` — `Plane` struct + 자유함수 4 선언 (총 111 줄)
- `practice/lidar_clean_room/my_lidar_segmenter.cpp` — RANSAC 코어 4 함수 (총 189 줄, +128 줄)
- `docs/Practice/stage_d_ransac.md` — Stage D 학습 노트 (#21~#29 정리 + 알고리즘 직관)

### Stage D 의 4 sub-stage 완성
| sub | 함수 | 분량 | 핵심 |
|---|---|---|---|
| D-1 | `point_to_plane_distance` + `Plane` struct | ~10 줄 | `\|a·x+b·y+c·z+d\|` (단위벡터 약속) |
| D-2 | `fit_plane_3pt` | ~40 줄 | 외적 → 정규화 → d 계산, degenerate 가드 |
| D-3 | `count_inliers` | ~15 줄 | for 루프 + 거리 ≤ threshold 카운팅 (빈칸 첫 성공!) |
| D-4 | `ransac_plane` | ~55 줄 | mt19937 + uniform_int + 메인 루프 + 베스트 갱신 |

### Stage D 에서 익힌 C++ 문법 9 가지 (#21~#29)
| # | 주제 | sub-stage |
|---|---|---|
| 21 | `std::abs(double)` (cmath 버전) | D-1 |
| 22 | `std::sqrt` | D-2 |
| 23 | `constexpr` + `1e-6` (지수 표기) | D-2 |
| 24 | **out parameter 패턴** (`T& out`) | D-2 |
| 25 | 함수 합성 (호출 결과로 비교) | D-3 |
| 26 | `<random>`, `std::mt19937` | D-4 |
| 27 | `std::uniform_int_distribution<T>` | D-4 |
| 28 | seed (`rng(7)`) — 재현성 | D-4 |
| 29 | `vector::operator[]` 인덱스 접근 | D-4 |

상세는 `docs/Practice/stage_d_ransac.md` 참조.

### Stage D 의 교훈 (사용자 패턴)
- **빈칸 채우기 첫 시도 성공** (D-3) — `if (point_to_plane_distance(plane, p) <= threshold) { ++count; }` 정확히 조립.
- **헤더에 자유함수 추가 시 한 단계 더 들여쓰기** 패턴 2 회 발생 (count_inliers, ransac_plane). cpp 에선 사라짐. 다음 stage 부터 헤더 작업 시 "커서가 컬럼 1 인지" 한 번 더 확인 권장.
- 저장 빼먹은 사례 1 회 (D-3 직후) — 디스크에 안 보여서 재확인 필요했음.
- 외적, mt19937, 함수 합성 등 새 개념을 큰 어려움 없이 흡수.

---

## Stage E 미리보기 (다음 세션에서 진행할 내용)

### 목표
Stage D 의 `ransac_plane` 을 **실전 사용** 해서 LiDAR 점 클라우드에서 **바닥 평면** 을 찾고, 그 평면에 가까운 점들을 `SegmentLabel::Floor` 로 라벨링.

### 핵심 아이디어
```
1. 사전 필터: ROI 안 → z 가 floor_search_min/max_z 범위인 점만
2. RANSAC: 그 부분 집합에서 ransac_plane 호출
3. 법선 방향 검증: |normal.z| >= floor_normal_min_z (바닥은 거의 수평)
4. 최소 inlier 검증: count >= min_floor_inliers
5. 라벨링: 평면에 가까운 점 → SegmentLabel::Floor
```

### 새로 만날 C++ 문법 (예상)
| # | 문법 | 의미 |
|---|---|---|
| 30~ | `std::vector<std::size_t>` 인덱스 리스트 | 어떤 점이 바닥인지 표시 |
| 30~ | 다중 조건 분기 (법선 방향 + 최소 inlier) | RANSAC 결과 검증 |
| 30~ | `SegmentedCloud` 결과 구조체 채우기 | 라벨링 |
| 30~ | range-based for 의 인덱스 추적 (`for (std::size_t i = 0; i < N; ++i)`) | 점-라벨 동시 채움 |

### Stage E 의 sub-stage 쪼개기 (예정)
- **E-1**: `extract_floor_search_points` — z 범위 사전 필터
- **E-2**: `segment_floor` — RANSAC 호출 + 검증 + 인덱스 추출
- **E-3**: `SegmentedCloud` 결과에 라벨 채우기

Stage E 는 Stage D 보다 작은 한 입 (~70~100 줄). 새 알고리즘 적고, 옵션 활용 + RANSAC 호출 + 라벨 채우기가 주.

---

## 학습 모드 (다음 세션부터 점진 전환)

**"시범 → 옮기기 → 의미 풀이 → 의문 받기"** 기본 모드 유지하되, Stage D-3 의 빈칸 성공을 발판으로 **Stage E 부터 빈칸 비중 점진 확대**:
- 새 개념 (E-1 의 z 범위 필터 — 기존 filter_roi 와 유사) → **반-빈칸**
- 새 함수 합성 (E-2 의 ransac 호출 + 조건 검증) → **시범** (out parameter 2개 처리가 첫 사례)
- 라벨링 루프 (E-3) → **빈칸 채우기** 시도

사용자 페이스 보고 결정.

---

## 산출물 위치 모음

| 무엇 | 경로 |
|---|---|
| 학습 코드 (헤더) | `practice/lidar_clean_room/my_lidar_segmenter.hpp` (111 줄) |
| 학습 코드 (구현) | `practice/lidar_clean_room/my_lidar_segmenter.cpp` (189 줄) |
| Stage A 학습 노트 | `docs/Practice/stage_a_header_basics.md` |
| Stage D 학습 노트 | `docs/Practice/stage_d_ransac.md` |
| 본 진행 가이드 | `docs/Practice/session_progress.md` |
| 원본 파이프라인 다이어그램 (reference) | `docs/Practice/kitti_segmentation_rerun_pipeline.html` |
| Reference 정답 코드 (보지 말 것 - 클린룸용) | `include/lidar/lidar_segmenter.hpp` |
| Reference 정답 코드 (보지 말 것 - 클린룸용) | `src/lidar/lidar_segmenter.cpp` |
| IntelliSense compile_commands | `build_ic/compile_commands.json` |

---

## 학습 종료

본 클린룸 재구현 학습은 **완료** 되었습니다. 다시 열어볼 일이 있다면:

- 학습판 코드: `practice/lidar_clean_room/my_lidar_segmenter.hpp` + `.cpp` + `main_test.cpp`
- 학습 노트: `docs/Practice/stage_a_header_basics.md`, `docs/Practice/stage_d_ransac.md`
- Reference (정답): `include/lidar/lidar_segmenter.hpp`, `src/lidar/lidar_segmenter.cpp`

**WSL g++ 빌드 명령** (재실행 시):
```powershell
wsl bash -lc "cd /mnt/d/02_research/04_cpp_seg_msckf_vio/practice/lidar_clean_room && g++ -std=c++17 -O2 main_test.cpp my_lidar_segmenter.cpp -o lidar_test"
```

**실행 명령**:
```powershell
wsl bash -lc "cd /mnt/d/02_research/04_cpp_seg_msckf_vio/practice/lidar_clean_room && ./lidar_test /mnt/d/02_research/04_cpp_seg_msckf_vio/data/kitti_raw/2011_09_26/2011_09_26_drive_0001_sync/velodyne_points/data/0000000000.bin segmented.ply"
```
