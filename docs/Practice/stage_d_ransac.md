---
title: Stage D 학습 노트 — RANSAC 평면 fitting 코어
date: 2026-05-21
type: practice
tags:
  - cpp-learning
  - ransac
  - random
  - vector
related:
  - "[[LidarSegmenter session_progress]]"
  - "[[stage_a_header_basics]]"
  - "[[../insight/20260514_ransac_lidar_segmentation_math]]"
status: completed
---

# Stage D 학습 노트 — RANSAC 평면 fitting 코어

> **본 노트의 범위**
> 클린룸 재구현 (`practice/lidar_clean_room/my_lidar_segmenter.cpp`) 의 Stage D 를 짜는 동안 마주친 새 C++ 문법 9 개 (#21~#29) 와 RANSAC 알고리즘의 직관·수식·코드를 정리.
>
> **연결되는 파일**
> - 학습 산출물 (헤더): `practice/lidar_clean_room/my_lidar_segmenter.hpp` (Plane struct + 자유함수 4 선언)
> - 학습 산출물 (구현): `practice/lidar_clean_room/my_lidar_segmenter.cpp` (D-1 ~ D-4 약 128 줄)
> - 원본 reference: `src/lidar/lidar_segmenter.cpp:17-150` (Eigen 사용, namespace anonymous)
> - Stage A 학습 노트: `docs/Practice/stage_a_header_basics.md` (헤더 기초 9 가지)

---

## Stage D 한 페이지 요약 — RANSAC 의 큰 그림

```
입력:  점 N 개, threshold τ, iterations T
─────────────────────────────────────────
반복 T 회 {
    1. 무작위 3 점 뽑기                      ← D-4 (rng)
    2. 3 점으로 평면 (a,b,c,d) 정의         ← D-2 (외적)
    3. 모든 점의 평면까지 거리 측정          ← D-1 (signed dist)
    4. |거리| ≤ τ 인 점 (inlier) 개수 카운트 ← D-3 (조건 카운팅)
    5. 베스트 갱신
}
출력: 가장 많이 지지받은 평면 + inlier 개수  ← D-4 의 마무리
```

**RANSAC = RANdom SAmple Consensus** = "랜덤 표본 합의".

- 핵심 발상: 정답 평면을 미리 모르니까 **무작위로 던져보고 가장 많은 점이 지지하는 가설 채택** ("투표").
- 강점: outlier (잡음, 사람, 차) 에 매우 강함. 통계 모델이 못 푸는 환경에서도 작동.
- 약점: 무작위라 결과가 매번 달라질 수 있음 → seed 고정으로 결정성 회복 (D-4).

본 repo 의 segmentation 파이프라인 (`docs/Practice/kitti_segmentation_rerun_pipeline.html`) 의 Stage 3 (Floor) 와 Stage 4 (Walls) 가 이 RANSAC 을 부품으로 사용.

---

## 학습 #21 — `std::abs` (cmath 버전) [D-1]

### [요약]
부호 있는 거리 → 절댓값. `<cmath>` 의 `std::abs` 는 `double` / `float` 까지 다 처리한다. `<cstdlib>` 의 `abs` 는 **정수 전용**.

### [코드]
```cpp
return std::abs(signed_dist);   // signed_dist 는 double
```

### [직관]
RANSAC 의 inlier 판정은 "평면에 가깝기만 하면 OK". 평면 위/아래 어느 쪽인지는 안 따짐. 그래서 절댓값.

### [C++ 노트]
- `<cmath>` include 했으면 `std::abs(double)` / `std::abs(float)` 둘 다 OK.
- `#include <cstdlib>` 만 한 채로 `std::abs(2.5)` 를 쓰면 → `int` 로 변환되어 `2` 가 됨. 무서운 함정.

---

## 학습 #22 — `std::sqrt` 와 길이 계산 [D-2]

### [요약]
3D 벡터의 길이 = `sqrt(x²+y²+z²)`. `std::sqrt` 는 `<cmath>` 의 제곱근.

### [코드]
```cpp
const double length = std::sqrt(nx * nx + ny * ny + nz * nz);
```

### [수식]
```
|n| = √(n_x² + n_y² + n_z²)
```

### [직관]
"법선벡터 길이" = 평면이 얼마나 "선명하게" 정의됐는지의 척도.
- 3 점이 일직선 → 외적 = 0 벡터 → 길이 ≈ 0 → 평면 정의 불가
- 3 점이 큰 삼각형 → 외적 크기 큼 → 길이 ↑ → 평면 잘 정의

### [C++ 노트]
- `nx * nx` 가 `std::pow(nx, 2)` 보다 빠르고 명확. 작은 정수 거듭제곱은 손곱셈이 관례.
- `std::sqrt(double)` 반환은 `double`.

---

## 학습 #23 — `constexpr` + 지수 표기 `1e-6` [D-2]

### [요약]
`constexpr` = **컴파일 타임 상수**. `1e-6` = 1 × 10⁻⁶ = 0.000001.

### [코드]
```cpp
constexpr double kMinNormalLength = 1e-6;
if (length < kMinNormalLength) {
    return false; // 세 점이 거의 일직선
}
```

### [직관 — degenerate 가드]
"외적 길이가 너무 작다 = 3 점이 거의 일직선 = 평면 정의 불가". 임계값은 충분히 작은 수면 OK. `1e-6` 은 보수적 선택 (LiDAR 점 단위 m, 노이즈 cm 급이라 cm² 단위에서도 안전).

### [C++ 노트]
- `constexpr` vs `const`: 둘 다 변경 불가지만 `constexpr` 은 컴파일 타임에 값이 확정. 컴파일러가 더 적극 최적화.
- 지수 표기: `1e-6`, `1.5e3` 처럼 과학 표기법 그대로. `1e6 == 1000000.0`.
- Stage B 의 `kBytesPerPoint` 와 같은 패턴 (둘 다 `constexpr`).

---

## 학습 #24 — out parameter 패턴 [D-2]

### [요약]
**함수가 결과를 인자로 돌려준다.** `T& out` 으로 받으면 함수 안에서 `out.멤버 = 값` 가능.

### [코드]
```cpp
// 선언 (헤더)
bool fit_plane_3pt(const LidarPoint& p1,
                   const LidarPoint& p2,
                   const LidarPoint& p3,
                   Plane& out_plane);     // ← const 없음 = 수정 가능

// 정의 (cpp)
bool fit_plane_3pt(..., Plane& out_plane) {
    if (length < kMinNormalLength) return false;
    out_plane.a = a;     // ← 함수가 채워 넣음
    out_plane.b = b;
    out_plane.c = c;
    out_plane.d = d;
    return true;
}

// 호출 (호출자가 미리 빈 그릇 준비)
Plane plane;
if (fit_plane_3pt(a, b, c, plane)) {
    // plane 사용
}
```

### [직관 — 왜 이 패턴?]
**반환 채널이 두 개 필요할 때**:
- 채널 ① `bool` 반환 → 성공/실패
- 채널 ② `Plane&` 인자 → 실제 결과

C++ 에는 `std::optional<Plane>` (C++17) 같은 대안도 있지만, out parameter 는 C 시절부터의 고전 관용구라 익히면 평생 씀.

### [관련 표준 변형]
- 본 학습: bool + out parameter
- modern alternative: `std::optional<Plane>` (C++17), 또는 `std::expected<Plane, Error>` (C++23)
- 본 repo reference: `bool make_plane(..., PlaneModel& plane)` — 동일 패턴

### [C++ 노트]
- `Plane& out_plane` 의 `&` 가 핵심. **참조** 라서 호출자가 보유한 진짜 변수에 직접 쓴다.
- `const` 가 없으면 = "함수가 이 인자를 변경할 수 있음" 의 시그널.
- 관습: out parameter 는 **인자 목록의 끝** 에 둠 (코드 읽을 때 "결과는 마지막 인자에" 라는 신호).

---

## 학습 #25 — 함수 합성 [D-3]

### [요약]
한 함수의 호출 결과를 즉시 다른 비교나 함수에 투입. **임시 변수 안 만듦**.

### [코드]
```cpp
if (point_to_plane_distance(plane, p) <= threshold) {
    ++count;
}
```

→ 다음과 의미적으로 동일:
```cpp
const double dist = point_to_plane_distance(plane, p);
if (dist <= threshold) {
    ++count;
}
```

### [직관]
- 결과를 한 번만 쓰면 → **합성** (한 줄)
- 결과를 여러 번 쓰면 → **임시 변수** 에 담음 (가독성/효율)

D-3 는 결과 한 번만 쓰니까 합성. D-2 의 `length` 는 두 번 (가드 + 정규화) 쓰니까 변수.

### [C++ 노트]
- 비교 연산자 `<=`, `<`, `==` 등은 양쪽에 임의 표현식 OK.
- "한 줄이 너무 길어진다" 싶으면 임시 변수로 빼는 게 가독성 ↑.

---

## 학습 #26 — `<random>` 의 `std::mt19937` [D-4]

### [요약]
**Mersenne Twister** 알고리즘의 32-bit 의사난수 엔진. C 의 `rand()` 보다 통계적으로 압도적으로 우수.

### [코드]
```cpp
#include <random>

std::mt19937 rng(7);  // seed = 7 로 고정
```

### [직관]
"진짜 난수" 는 비싸니까, 결정적이지만 통계적으로 난수처럼 보이는 수열 (= 의사난수) 을 만든다. Mersenne Twister 는:
- 주기 ≈ 2^19937 (사실상 무한)
- 통계 검정 모두 통과 (균등성, 독립성)
- 표준 라이브러리에 들어있어 어디서나 쓸 수 있음

### [엔진 vs 분포 — modern C++ 난수의 2 단 구조]
```
       엔진 (engine)             분포 (distribution)
   ┌──────────────────┐        ┌──────────────────┐
   │  std::mt19937    │ ──→    │ uniform_int_dist │ ──→  결과
   │  (원시 수열)     │        │ (원하는 모양)    │
   └──────────────────┘        └──────────────────┘
```

- 엔진은 "균등하게 0 ~ 2^32-1 정수" 만 뱉음
- 거기에 분포를 씌워서 "0 ~ N-1 균등" / "평균 μ, 분산 σ² 가우시안" 등을 만듦

### [C++ 노트]
- `std::mt19937` 외에도 `std::ranlux24`, `std::minstd_rand` 등 있지만 99% mt19937 로 OK.
- 64-bit 가 필요하면 `std::mt19937_64`.

---

## 학습 #27 — `std::uniform_int_distribution<T>` [D-4]

### [요약]
"정해진 범위에서 정수 균등 추출" 의 분포 객체.

### [코드]
```cpp
std::uniform_int_distribution<std::size_t> pick(0, points.size() - 1);

// 사용
const std::size_t i1 = pick(rng);   // 0 ~ N-1 중 하나
```

### [직관]
인덱스 추첨용 슬롯머신. 만들 때 범위를 정하고 (`0, N-1`), `pick(rng)` 호출할 때마다 한 번 돌림.

### [구체 예]
points.size() == 100 이면 `pick(rng)` 는 0 ~ 99 중 하나를 균등 확률로 반환.

### [C++ 노트]
- 범위는 **양쪽 포함** (`[lo, hi]`). C++ STL 의 다른 알고리즘 (`[begin, end)`) 과 관습이 다름 — 주의.
- 템플릿 인자 `<std::size_t>` 는 결과 타입. `vector::size()` 와 호환되도록 size_t 선택.
- 호환 함수 `std::uniform_real_distribution<double>` 은 실수용.

---

## 학습 #28 — seed 와 재현성 [D-4]

### [요약]
`rng(7)` 의 `7` 은 **seed (씨앗)**. 같은 seed → 같은 수열. 디버깅에서 결정성 보장.

### [코드]
```cpp
std::mt19937 rng(7);   // 항상 같은 결과
// vs
std::random_device rd;
std::mt19937 rng(rd());  // 매번 다른 결과 (운영 환경)
```

### [직관]
의사난수의 본질: **수열의 시작점만 다르면 전부 다른 수열**. seed = 시작점.

| 상황 | 권장 seed |
|---|---|
| 알고리즘 개발 / 디버깅 | **고정 seed** (`7`, `42` 등) — 같은 입력 = 같은 출력 |
| 운영 환경 / 매번 다른 trial 원함 | `std::random_device` 로 진짜 난수 seed |
| 통계 실험 N 회 평균 | 매 trial 마다 다른 seed |

### [본 학습에서 7 을 쓴 이유]
- 디버깅 친화 — 출력이 매번 같음
- 본 repo reference 도 같은 패턴 (`std::mt19937 rng(7);` at `src/lidar/lidar_segmenter.cpp:84`)

---

## 학습 #29 — `vector::operator[]` 인덱스 접근 [D-4]

### [요약]
`points[i]` = vector 의 i 번째 원소. 0-based.

### [코드]
```cpp
const std::size_t i1 = pick(rng);
const std::size_t i2 = pick(rng);
const std::size_t i3 = pick(rng);

fit_plane_3pt(points[i1], points[i2], points[i3], candidate);
```

### [직관]
배열 첨자와 동일. 0 ≤ i < size() 범위여야 안전 (D-4 에선 `uniform_int_distribution(0, N-1)` 로 보장됨).

### [`[]` vs `at()` — C++ 의 두 인덱스 방식]
| 방식 | 범위 체크 | 속도 |
|---|---|---|
| `points[i]` | **없음** (잘못된 i = UB) | 빠름 |
| `points.at(i)` | **있음** (잘못된 i = `std::out_of_range` throw) | 살짝 느림 |

성능 critical 코드 (RANSAC 처럼 N 회 반복) 는 `[]` 가 관례. 단, 인덱스 안전성이 보장되어야 함.

### [C++ 노트]
- `vector` 외에 `array`, `string`, `unordered_map` 등도 `[]` 지원.
- `[]` 는 reference 반환 → `points[i].x = 10;` 도 가능 (mutable 인 경우).

---

## RANSAC 4 함수의 상호 의존성

```
ransac_plane(D-4)              ← 외부에서 호출
   │
   ├─ fit_plane_3pt(D-2) ─────→ Plane
   │      └─ (수학) 외적, 정규화
   │
   ├─ count_inliers(D-3) ─────→ size_t
   │      └─ for 각 점:
   │           point_to_plane_distance(D-1) ──→ double
   │
   └─ 베스트 갱신 + 출력
```

- **D-1 (가장 안쪽)**: 점 ↔ 평면 거리. 가장 자주 호출됨 (`N × iterations` 회).
- **D-2**: 3 점 → 평면. iteration 당 1 회.
- **D-3 (D-1 사용)**: 후보 평면의 inlier 카운트. iteration 당 1 회 (내부에서 N 회 D-1 호출).
- **D-4 (D-2, D-3 사용)**: 메인 루프 + 베스트 추적.

### 복잡도
- 시간: O(T × N) — T = iterations, N = 점 개수
- 공간: O(1) (best_plane 만 보관)
- 본 repo reference 는 inlier 인덱스 리스트도 모음 → O(N)

---

## Stage D 의 새 C++ 문법 9 가지 — 한 페이지 요약

| # | 문법 | 카테고리 | 어디 |
|---|---|---|---|
| 21 | `std::abs(double)` (`<cmath>`) | 표준 라이브러리 | D-1 |
| 22 | `std::sqrt` | 표준 라이브러리 | D-2 |
| 23 | `constexpr` + `1e-6` 지수 표기 | 키워드 + 리터럴 | D-2 |
| 24 | out parameter (`T& out`) | 관용구 | D-2 |
| 25 | 함수 합성 (호출 결과로 비교) | 표현식 패턴 | D-3 |
| 26 | `<random>` + `std::mt19937` | 표준 라이브러리 | D-4 |
| 27 | `std::uniform_int_distribution<T>` | 표준 라이브러리 | D-4 |
| 28 | seed 의 의미 (`rng(7)`) | 개념 | D-4 |
| 29 | `vector::operator[]` | 표준 라이브러리 | D-4 |

---

## 본 repo reference 와의 차이 (참고)

| 항목 | 학습 코드 (clean room) | reference 코드 |
|---|---|---|
| 평면 표현 | `Plane { double a,b,c,d; }` | `PlaneModel { Eigen::Vector3d n; double d; ... }` |
| 외적 | 직접 손으로 | `(b-a).cross(c-a)` (Eigen) |
| 정규화 | `inv_length` 곱셈 | `n /= n.norm()` |
| inlier 추적 | 개수만 | 인덱스 vector |
| 평면 방향 조정 | 없음 (D 학습 범위 밖) | `n.z() < 0` 이면 부호 뒤집기 |
| accept_plane callback | 없음 (D 학습 범위 밖) | 있음 (Floor 의 법선 z 조건) |
| 네임스페이스 | global | anonymous namespace (TU-local) |

**왜 학습판은 단순?** — Eigen 의존 없이 raw C++ 만으로 RANSAC 의 본질 (랜덤 + 거리 + 투표) 을 익히는 게 목적이라. Stage E 에서 floor 방향 조정 (학습 #30~ 쯤) 등을 추가할 예정.

---

## Stage D 의 교훈 (사용자 패턴)

### 잘된 점
- 빈칸 채우기 (D-3) 첫 시도 통과 — `if (point_to_plane_distance(plane, p) <= threshold) { ++count; }` 깔끔하게 조립.
- 들여쓰기 묻어들어옴 패턴은 **헤더에서만 1~2 회** 재발. cpp 에선 사라짐.

### 반복 패턴
- **헤더에 자유함수 추가할 때** 한 단계 더 들여쓰는 경향 (count_inliers, ransac_plane 둘 다 컬럼 1 이 아니라 컬럼 5 / 23 에서 시작). 사용자 무의식 패턴인 듯. 다음 stage 부터 헤더 작업 시 "커서가 컬럼 1 인지" 한 번 더 확인.
- 저장 빼먹은 사례 1 회 (D-3) — VS Code 탭의 `●` 점 확인 습관 권장.

### 다음 stage 부터 시도해볼 모드
Stage D 의 D-3 가 빈칸 채우기 성공이므로, **Stage E 부터 빈칸 비중 점진 확대**:
- D-1, D-2 같은 새 개념 → 시범
- D-3 같은 패턴 합성 → 빈칸 채우기
- D-4 같은 큰 한 입 → 의사코드 보여주고 사용자가 슬롯 채우기 시도

---

## 다음 Stage E 미리보기 — Floor 분리

### 목표
Stage D 에서 만든 `ransac_plane` 을 **실전 사용** 해서 LiDAR 점 클라우드에서 **바닥 평면** 을 찾고, 그 평면에 가까운 점들을 `SegmentLabel::Floor` 로 라벨링.

### 새 요소 (예상)
- **사전 필터링**: ROI 박스 안에서 다시 `floor_search_min_z ~ max_z` 범위로 좁힘 (바닥은 z 가 낮은 영역에만 있음)
- **법선 방향 조건**: 바닥의 법선은 **+z 방향** 이어야 함 (또는 abs(c) > 0.85 등의 옵션 활용)
- **최소 inlier 수**: 너무 작으면 거부 (`min_floor_inliers = 500`)
- **라벨링**: 평면에 가까운 점들에 `SegmentLabel::Floor` 부여

### 새로 만날 C++ 문법 (예상)
- `std::vector<std::size_t>` 인덱스 리스트
- `if (abs(normal.z) > opt.floor_normal_min_z)` 의 조건 분기
- `for (std::size_t i = 0; i < N; ++i)` 로 라벨 할당
- 어쩌면 `SegmentedCloud` 결과 구조체 채우기

### 분량 예상
Stage E 는 D 보다 작은 한 입 (~70 ~100 줄). 새 알고리즘이 적고, 옵션 활용 + RANSAC 호출 + 라벨 채우기가 주.

---

## 다음 세션 트리거

```
practice stage E 시작하자
```

→ Claude 가 메모리 + session_progress.md + 본 노트 + 현재 코드 상태 확인 후 Stage E-1 부터 진행.
