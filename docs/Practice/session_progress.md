# LidarSegmenter 클린룸 재구현 — 진행 상태 & 재개 가이드

> **마지막 작업일:** 2026-05-13
> **다음 세션 시작 트리거:** 채팅에 그대로 던지세요:
>
> ```
> practice stage B 시작하자
> ```
>
> Claude 가 자동으로 메모리에서 어디까지 했는지 복원 후 Stage B-1 부터 진행합니다.

---

## 진행률 한눈에

```
[A] 헤더 6 type + 자유함수 2 선언 ........ ✓ 완료 (84 줄)
[B] load_kitti_velodyne_bin (.bin 파싱) ... ← 다음
[C] ROI + finite 필터 .................... · pending
[D] RANSAC 코어 3 함수 ................... · pending
[E] Floor 분리 ........................... · pending
[F] Wall 3종 그리디 피링 ................. · pending
[G] write_segmented_ply (PLY 저장) ....... · pending
[H] main_test.cpp + 빌드 + 시각 검증 ..... · pending
```

진행률: **1/8 stage** 완료.

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

## Stage B 미리보기 (다음 세션에서 진행할 내용)

### 목표
`practice/lidar_clean_room/my_lidar_segmenter.cpp` 를 새로 만들고, **`load_kitti_velodyne_bin(path)`** 함수 구현 (약 30 줄).

### 핵심 아이디어
KITTI `.bin` 파일은 점이 **연속된 바이너리 (float[4] × N)** 로 저장:

```
파일 0000000000.bin
┌──────────────────────────────────────────────────────────────┐
│ float float float float │ float float float float │ ... 12만점 │
│  x    y    z   intens   │  x    y    z   intens   │           │
│   (점 1번, 16 byte)     │   (점 2번, 16 byte)     │           │
└──────────────────────────────────────────────────────────────┘
```

`LidarPoint` 도 정확히 16 byte 라 → 디스크의 바이트를 메모리에 그대로 부어 넣을 수 있음.

### 새로 만날 C++ 문법 7 개
| # | 문법 | 의미 |
|---|---|---|
| 1 | `<fstream>`, `std::ifstream` | 파일 읽기 스트림 |
| 2 | `std::ios::binary \| std::ios::ate` | 비트 플래그 OR로 옵션 합치기 |
| 3 | `f.tellg()`, `f.seekg(...)` | 파일 위치 측정/이동 |
| 4 | `reinterpret_cast<char*>(...)` | "이 메모리를 바이트 배열로 해석해줘" |
| 5 | `throw std::runtime_error(...)` | 예외 던지기 |
| 6 | `constexpr` | 컴파일 타임 상수 |
| 7 | `static_cast<T>(...)` | 안전한 타입 변환 |

### Stage B 의 sub-stage 쪼개기
- **B-1**: `.cpp` 파일 시작 — include 들 + 함수 시그니처
- **B-2**: 파일 열기 (`std::ifstream`, binary | ate) + 예외 1
- **B-3**: 파일 크기 측정 (`tellg`) + 점 개수 계산
- **B-4**: 점별 읽기 루프 (`reinterpret_cast`) + 예외 2
- **B-5**: return + 마무리

각 sub-stage 는 5~10 줄 분량, 새 문법은 1~2 개씩.

---

## 학습 모드 (다음 세션에도 동일)

**"시범 → 옮기기 → 의미 풀이 → 의문 받기"** 모드:
1. Claude 가 시범 코드를 보여줌
2. 사용자가 그대로 옮기고 저장
3. 각 줄 의미를 표/비유로 풀이
4. 사용자가 의문 던지면 그 부분 깊게
5. 다음 sub-stage 로 이동

이 모드를 Stage D (RANSAC 코어) 까지 유지 예정. 그 이후 사용자가 적응했으면 점진적으로 "빈칸 채우기" 로 전환.

---

## 산출물 위치 모음

| 무엇 | 경로 |
|---|---|
| 학습 코드 (헤더) | `practice/lidar_clean_room/my_lidar_segmenter.hpp` |
| 학습 코드 (구현, 다음 세션에 생성) | `practice/lidar_clean_room/my_lidar_segmenter.cpp` |
| Stage A 학습 노트 | `docs/Practice/stage_a_header_basics.md` |
| 본 진행 가이드 | `docs/Practice/session_progress.md` |
| 원본 파이프라인 다이어그램 (reference) | `docs/Practice/kitti_segmentation_rerun_pipeline.html` |
| Reference 정답 코드 (보지 말 것 - 클린룸용) | `include/lidar/lidar_segmenter.hpp` |
| Reference 정답 코드 (보지 말 것 - 클린룸용) | `src/lidar/lidar_segmenter.cpp` |
| IntelliSense compile_commands | `build_ic/compile_commands.json` |

---

## 다음 세션 트리거 — 그대로 채팅에 던지기

```
practice stage B 시작하자
```

또는

```
session_progress.md 참고해서 stage B 부터 이어가자
```

→ Claude 가 메모리 + 본 파일 + 현재 코드 상태 확인 후 **Stage B-1 시범 코드** 부터 시작합니다.
