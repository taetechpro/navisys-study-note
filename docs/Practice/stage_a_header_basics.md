---
title: Stage A 학습 노트 — C++ 헤더의 기초
date: 2026-05-13
type: practice
tags:
  - cpp-learning
  - header
  - struct
  - enum
  - vector
related:
  - "[[LidarSegmenter session_progress]]"
  - "[[stage_d_ransac]]"
status: completed
---

# Stage A 학습 노트 — C++ 헤더의 기초

> **본 노트의 범위**
> 클린룸 재구현(`practice/lidar_clean_room/my_lidar_segmenter.hpp`) 의 첫 단계인 Stage A 를 짜는 동안 마주친 C++ 기초 개념 4 개를 정리. 본 노트는 `kitti_segmentation_rerun_pipeline.html` 의 동반 자료이며, 그 문서의 Stage 1 (데이터 정의) 에 대응.
>
> **연결되는 파일**
> - 원본 reference 코드: `include/lidar/lidar_segmenter.hpp:10-15`
> - 학습 산출물: `practice/lidar_clean_room/my_lidar_segmenter.hpp`

---

## 학습 #1 — `struct` 와 `LidarPoint`

### [요약]
`struct LidarPoint { ... };` 는 **새 타입(type) 을 정의** 한 것. 변수 하나가 아니라 "이런 모양의 변수를 만들 수 있는 청사진" 이다.

### [코드]
```cpp
struct LidarPoint {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float intensity = 0.0f;
};
```

### [줄 단위 의미]
| 요소 | 의미 |
|---|---|
| `struct` | "묶음 타입 정의 시작" 키워드 |
| `LidarPoint` | 사용자가 붙이는 **타입 이름** (변수가 아님) |
| `{ ... }` | 이 타입이 들고 있을 **멤버 변수**들의 정의 |
| `float x = 0.0f;` | 'x' 라는 float 칸, 기본값 0 (default member initializer, C++11+) |
| `};` | struct 정의 종료 — **세미콜론 필수** (가장 흔한 초보 함정) |

### [직관 — "양식(form)"]
타입 정의 = 빈 견본 양식 인쇄. 인스턴스 생성 = 견본 복사해서 종이 한 장 뽑기.

```
┌─────────────────────────────┐
│       LidarPoint            │
├─────────────────────────────┤
│ x         : [ 0.0f ]        │
│ y         : [ 0.0f ]        │
│ z         : [ 0.0f ]        │
│ intensity : [ 0.0f ]        │
└─────────────────────────────┘
```

### [사용 예시]
```cpp
LidarPoint p;                      // (1) 견본 한 장 → 모든 칸 0
LidarPoint q{1, 2, 3, 0.5f};       // (2) aggregate init — 한꺼번에 값 채움
p.x = 5.0f;                        // (3) p 의 x 칸 수정
std::cout << p.x;                  // (4) "5" 출력
```

### [메모리 관점 — 왜 결정적인가]
한 `LidarPoint` 의 메모리 레이아웃:
```
오프셋: 0      4      8      12      16
       ┌──────┬──────┬──────┬─────────┐
       │  x   │  y   │  z   │intensity│
       └──────┴──────┴──────┴─────────┘
        4B     4B     4B      4B       총 16 B
```
KITTI `.bin` 파일은 한 점 = `float[4]` = 16 B 로 저장 → **메모리 레이아웃이 디스크 바이트와 정확히 일치**. Stage B 에서 `.bin` 바이트를 `LidarPoint` 배열에 그대로 부어 넣을 수 있는 이유.

### [C++ 노트]
- `struct` vs `class` 차이는 **기본 접근 권한**뿐 (`struct`=public, `class`=private). 데이터 묶음은 `struct` 가 관례.
- `= 0.0f` 의 `f` 접미사 = float 리터럴. 안 붙이면 double 인 `0.0` 이 되어 (보통은 문제 없지만) 컴파일러 경고날 수 있음.
- `};` 의 세미콜론 빼먹으면 알 수 없는 에러 폭주.

---

## 학습 #2 — `#pragma once` 와 `#include`

### [요약]
헤더 맨 위 6 줄은 **두 가지 다른 일**을 함:
- `#pragma once` → 같은 헤더 중복 포함 방지
- `#include <...>` 4 줄 → 다른 헤더의 내용을 텍스트로 끌어옴

둘 다 `#` 시작 → **전처리기(preprocessor) 명령** (컴파일러가 코드를 보기 *전*에 처리).

### [코드]
```cpp
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>
```

### [`#pragma once` 가 하는 일]
한 파일이 여러 곳에서 간접적으로 두 번 include 되면 같은 타입을 두 번 정의 → 컴파일 에러. `#pragma once` 는 "두 번째부터는 통째로 스킵" 을 컴파일러에게 요청.

전통적 방식(`#ifndef`/`#define`/`#endif`) 의 한 줄 대체. MSVC/GCC/Clang 모두 지원하므로 현대 C++ 에서 거의 표준 관행.

### [`#include` 의 동작 — 진짜로 텍스트 복붙]
`#include <vector>` = 전처리기가 그 자리에 `<vector>` 헤더 파일 내용을 통째로 텍스트로 붙여 넣음. 결과적으로 컴파일러는 마치 그 모든 코드가 내 파일 안에 있었던 것처럼 봄.

### [`<...>` vs `"..."` 차이]
| 형식 | 검색 경로 |
|---|---|
| `#include <vector>` | **시스템 헤더 경로** 부터 검색 (표준/외부 라이브러리용) |
| `#include "my_lidar_segmenter.hpp"` | **현재 파일 디렉토리** 부터 검색 (내 프로젝트 헤더용) |

### [4 개 헤더 각각의 역할]
| Include | 가져오는 타입 | 어디서 등장 |
|---|---|---|
| `<cstddef>` | `std::size_t` (부호없는 정수) | `min_floor_inliers`, `SegmentCounts` |
| `<cstdint>` | `std::uint8_t` (정확히 1 byte) | `enum class SegmentLabel : std::uint8_t` |
| `<string>` | `std::string` | `load_kitti_velodyne_bin(const std::string&)` |
| `<vector>` | `std::vector<T>` (가변 길이 배열) | `SegmentedCloud::points/labels` |

**원칙: 쓰는 타입이 정의된 헤더만 정확히.** 더 적으면 컴파일 안 됨, 더 많으면 컴파일 시간만 늘어남.

### [비유 — 원고에 다른 책 붙여넣기]
내 헤더가 원고 한 권이라면:
- `#include <vector>` = 출판사 표준 책장의 "vector" 책을 통째로 내 원고 첫 페이지에 풀로 붙임.
- `#pragma once` = "이 원고가 다른 원고에 두 번 인용되면, 두 번째는 빈 종이로 처리" 라는 부탁.

### [C++ 노트]
- 헤더에는 **꼭 필요한 include 만** 둠. `<iostream>`, `<cmath>` 등은 헤더에서 안 쓰면 빼야 함 — 헤더가 전이 의존성을 키워서 빌드 시간 폭주의 주범.

---

## 학습 #3 — `std::vector` 의 크기 시스템

### [요약]
`std::vector<T>` 는 **동적 컨테이너** — 크기를 미리 정하지 않음. `push_back` 할 때마다 자동으로 늘어남. 코드에 보이는 "12 만" 같은 숫자는 **주석/설명** 일 뿐, 코드 약속이 아님.

### [기본 동작]
```cpp
std::vector<LidarPoint> cloud;   // 시작 크기 0
cloud.size();                    // → 0
cloud.push_back({1,2,3,0});      // 크기 1
cloud.push_back({4,5,6,1});      // 크기 2
```

### [미리 알려주는 3 가지 방법 — 성능 최적화]
| 방법 | 코드 | size() | capacity() |
|---|---|---|---|
| A: 메모리만 미리 확보 | `v.reserve(N);` | 0 (변화 없음) | ≥ N |
| B: 처음부터 N 개 만들기 | `vector<T> v(N);` | N | ≥ N |
| C: 그냥 둠 (느림) | (아무것도 안 함, push_back 만) | 늘어남 | 자동 재할당 |

vector 는 메모리 모자라면 더 큰 메모리 새로 잡고 기존 데이터 복사 → 재할당 비용 큼. 미리 알면 `reserve` 로 한 번에 잡는 게 관용구.

### [reserve vs resize 함정]
```cpp
std::vector<LidarPoint> v;
v.reserve(120000);     // size=0, capacity=120000
v[0].x = 1.0f;         // ❌ undefined behavior! [0] 위치가 "있지" 않음

std::vector<LidarPoint> w(120000);   // size=120000, 모두 기본값
w[0].x = 1.0f;         // ✅ OK
```

### [Stage B 미리보기 — KITTI 에서는 정확한 크기 계산 가능]
```cpp
const std::size_t count = bytes / 16;        // 파일크기 ÷ 점크기
std::vector<LidarPoint> points(count);       // 정확히 그만큼 한 번에
```
→ "12 만 추정" 안 해도 됨. 파일이 정확한 답을 알려줌.

### [C++ 노트]
- `vector::size()` = 현재 들어있는 요소 수.
- `vector::capacity()` = 재할당 없이 더 들어갈 수 있는 한도.
- `[]` 인덱스 접근은 `size()` 미만에서만 안전. capacity 안이라도 size 밖이면 UB.

---

## 학습 #4 — `std::cout` 으로 출력

### [요약]
`std::cout << p.x;` = "**`p.x` 값을 콘솔에 출력**". Python 의 `print()`, C 의 `printf()` 와 같은 역할.

### [코드 + 분해]
```cpp
std::cout  <<  p.x  ;
   ↑       ↑    ↑   ↑
   (a)    (b)  (c) (d)
```
| 기호 | 의미 |
|---|---|
| (a) `std::cout` | "**c**onsole **out**put" — 표준 출력 스트림 객체. `<iostream>` 헤더에서 제공. |
| (b) `<<` | **stream insertion operator** — 왼쪽 스트림에 오른쪽 값을 흘려보냄. 화살표 모양이 데이터 방향. |
| (c) `p.x` | 출력할 값 |
| (d) `;` | 문장 끝 |

### [체이닝]
```cpp
std::cout << "p.x = " << p.x << ", p.y = " << p.y << "\n";
// 화면: p.x = 5, p.y = 0
```
`<<` 는 이어 붙일 수 있음 — 텍스트/숫자 섞어 출력 쉬움.

### [필요한 헤더]
```cpp
#include <iostream>
```
**우리 `my_lidar_segmenter.hpp` 에는 안 넣음** — 출력은 디버깅용이지 라이브러리의 일부가 아님. Stage H 의 `main_test.cpp` 에서야 등장 예정.

### [줄바꿈 2 가지 방법]
```cpp
std::cout << p.x << "\n";        // 빠름 (그냥 줄바꿈 문자)
std::cout << p.x << std::endl;   // 줄바꿈 + 강제 flush (느림)
```
일반 출력은 `"\n"` 추천.

### [다른 언어와 비교]
| 언어 | 출력 |
|---|---|
| C++ | `std::cout << p.x;` |
| C | `printf("%f", p.x);` |
| Python | `print(p.x)` |
| MATLAB | `disp(p.x)` |

C++ 방식의 장점: **타입 안전** — `printf` 의 `%f`/`%d` 헷갈리는 사고가 없음.

### [C++ 노트]
- `std::cout` 의 `std::` 빼면 에러 — 표준 라이브러리는 모두 `std::` namespace.
- `using namespace std;` 는 cpp 파일 안에서만 (헤더에는 절대 금지 — 모든 include 자에 namespace 오염).
- 디버그용으론 `std::cerr` (error stream, 무버퍼) 가 충돌 시에도 출력이 남아 유용.

---

## 학습 #5 — `enum class` 와 `SegmentLabel`

### [요약]
한 점에 붙일 라벨을 **6 가지 값 중 하나로 제한**하는 새 타입. C++ 의 `enum class` 문법으로 만들면 `int` 와 자동으로 안 섞여서 안전.

### [코드]
```cpp
enum class SegmentLabel : std::uint8_t {
    Other     = 0,
    Floor     = 1,
    LeftWall  = 2,
    RightWall = 3,
    FrontWall = 4,
    Wall      = 5,
};
```

### [줄 단위 의미]
| 위치 | 의미 |
|---|---|
| `enum class` | "스코프 있는 enum" — 새 타입 정의 |
| `SegmentLabel` | 새 타입 이름 |
| `: std::uint8_t` | 바닥 타입(underlying type) = 1 byte 부호없는 정수 |
| `Other = 0, ...` | 이름에 정수값 부여 |
| `};` | 정의 끝, 세미콜론 필수 |

### [`enum class` vs 옛 `enum` 차이]
- **옛 `enum`**: `int` 와 자동 변환됨, 같은 스코프에 이름 오염, 비교 사고 잦음
- **`enum class`**: `int` 와 안 섞임, `SegmentLabel::Floor` 처럼 스코프로 접근, 컴파일러가 실수 차단

```cpp
SegmentLabel l = SegmentLabel::Floor;        // ✓ 명확
SegmentLabel l = 1;                           // ❌ 컴파일 에러 (int 와 안 섞임)
int x = static_cast<int>(l);                  // ✓ 명시적 캐스트로만 변환 가능
```

### [왜 `std::uint8_t` 인가 — 메모리]
- `int` (4 B) × 100,000 점 = **400 KB 라벨 배열**
- `uint8_t` (1 B) × 100,000 점 = **100 KB**
- 값이 0~5 뿐이라 1 byte 면 충분 → **4 배 메모리 절약**

### [비유 — 라디오 버튼 6 개]
```
○ Other      (값 0)
● Floor      (값 1)   ← 현재 선택된 라벨
○ LeftWall   (값 2)
○ RightWall  (값 3)
○ FrontWall  (값 4)
○ Wall       (값 5)
```
한 점에 라벨 하나만 — 6 가지 중 하나만 켜질 수 있는 라디오 버튼.

### [C++ 노트]
- `class` 키워드 빼먹지 말 것 (옛 enum 으로 회귀)
- `::` 는 "스코프 접근 연산자". `SegmentLabel::Floor`, `std::cout` 의 `::` 와 같은 의미
- 트레일링 콤마(`Wall = 5,` 끝 콤마) 는 합법, 가독성/diff 깔끔

---

## 학습 #6 — `LidarSegmentationOptions` 와 단위 표기

### [요약]
알고리즘이 쓰는 튜닝 값 22 개를 한 struct 에 모은 옵션 묶음. 새 C++ 문법은 없음 (struct + default value 패턴 반복). 다만 **C++ 은 단위 시스템이 없다**는 결정적 사실 학습.

### [코드 (요약)]
```cpp
struct LidarSegmentationOptions {
    // ROI
    double min_x = -5.0;    double max_x = 50.0;
    double min_y = -25.0;   double max_y = 25.0;
    double min_z = -3.0;    double max_z = 3.0;
    // Floor (5 개)
    // Wall  (10 개)
    int    ransac_iterations = 180;
};
```

### [그룹 구조 4 개]
| 그룹 | 멤버 수 | 의미 |
|---|---|---|
| ROI | 6 | 차량 주위 어디까지 점을 볼지 |
| Floor | 5 | 바닥 평면 RANSAC 의 조건 |
| Wall | 10 | 좌/우/전방 벽 RANSAC 의 조건 |
| RANSAC | 1 | RANSAC 반복 횟수 |

### [KITTI 좌표계 (모든 길이 단위 m)]
- 원점 = Velodyne LiDAR 센서 위치
- **+x = 전방, +y = 좌측, +z = 상방** (오른손 좌표계)
- `min_x = -5.0` = 차량 후방 5m, `max_x = 50.0` = 전방 50m

ROI 박스 위에서 본 모습:
```
   y (left)
   +25 ┌──────────────────────────────┐
       │                              │
     0 ●──────────────────────────────►  +x
       │                              │
   -25 └──────────────────────────────┘
       -5                            +50
```

### [C++ 의 단위 시스템 부재]
C++ 변수는 **숫자만 들고 다님**. `double x = 5.0;` 만으론 미터/센티/인치 모름.
→ 컴파일러는 단위 실수를 못 잡음. **변수명/주석/문서**로만 약속.

사람들이 쓰는 컨벤션:
- 이름에 단위 붙이기: `length_m`, `time_ms`, `angle_deg`
- 본 코드 일부: `min_wall_extent_m`, `min_wall_height_m` (`_m` 접미사)
- 단위 라이브러리(`mp-units`)는 학습/실험 코드엔 오버킬

### [타입 선택의 이유]
| 타입 | 어디 쓰임 |
|---|---|
| `double` | 거리/임계값/좌표 한계 (음수 가능, 정밀도 필요) |
| `std::size_t` | inlier 개수 (음수 불가) |
| `int` | 반복 횟수 (음수 → 스킵 신호값 여지) |

### [왜 옵션을 한 struct 에 묶나]
인자를 22 개 따로 받으면 호출 사고 폭주. struct 로 묶으면:
```cpp
LidarSegmentationOptions opt;
opt.floor_distance_threshold = 0.25;        // 하나만 수정
LidarSegmenter seg(opt);                    // 깔끔
```
→ Strategy 패턴의 경량 버전. YAML 옵션 로딩 쉬워짐.

### [모든 옵션이 m 인가? — 아님]
| 멤버 | 단위 |
|---|---|
| `min_x` ... `wall_distance_threshold` | **m** |
| `floor_normal_min_z` (0.85), `wall_normal_max_abs_z` (0.25) | **단위 없음** (단위 벡터 성분) |
| `min_floor_inliers`, `min_wall_inliers` | **개 (count)** |
| `ransac_iterations` | **회** |

---

## 학습 #7 — `SegmentCounts` (작은 카운터 struct)

### [요약]
각 라벨이 몇 점인지 세는 용도의 작은 struct. `LidarPoint` 와 같은 패턴, 새 문법 0.

### [코드]
```cpp
struct SegmentCounts {
    std::size_t other = 0;
    std::size_t floor = 0;
    std::size_t wall = 0;
    std::size_t left_wall = 0;
    std::size_t right_wall = 0;
    std::size_t front_wall = 0;
};
```

### [어디 쓰이나]
```cpp
SegmentedCloud cloud = segmenter.process(points);
SegmentCounts c = cloud.counts();
std::cout << "floor: " << c.floor << "\n";    // "floor: 12345"
```

### [C++ 노트]
- 6 개 멤버는 `SegmentLabel` 의 6 값에 정확히 대응
- 모두 `std::size_t = 0` — 카운터에 자연스러운 타입 (음수 없는 정수)
- 멤버 이름은 소문자 + 언더스코어 (이 repo 스타일). 타입 멤버(`Floor`)는 카멜케이스, 카운터 변수는 snake_case — 의도된 차이

---

## 학습 #8 — `SegmentedCloud` 와 **멤버 함수 선언**

### [요약]
점 배열 + 라벨 배열 묶음 struct. 새 문법 2 개: **`std::vector<T>` 사용**, **멤버 함수 "선언만"** (`const` 키워드 포함).

### [코드]
```cpp
struct SegmentedCloud {
    std::vector<LidarPoint>   points;
    std::vector<SegmentLabel> labels;

    SegmentCounts counts() const;
};
```

### [멤버 변수 두 줄]
- `points` 와 `labels` 는 **항상 같은 길이를 유지해야 한다는 약속** — 한 점 ↔ 한 라벨 대응
- default value 안 적어도 vector 는 자동으로 빈 vector 로 초기화

### [멤버 함수 선언 — 새 문법]
```cpp
SegmentCounts counts() const;
   ↑          ↑      ↑     ↑
   반환       이름   인자  멤버변수 안바꿈
                    없음
```
- 끝의 `;` = **선언만이라는 신호**. 본체 `{...}` 없음.
- `const` 멤버 함수 = "이 함수는 멤버 변수를 안 바꿈" 컴파일러가 차단.

### [선언(declaration) vs 정의(definition)]
C++ 함수는 두 단계로 쪼개짐:

```cpp
// 선언 (.hpp 에) — "이런 함수가 있다"
SegmentCounts counts() const;

// 정의 (.cpp 에) — "그 함수는 이렇게 동작한다"
SegmentCounts SegmentedCloud::counts() const {
    SegmentCounts result;
    // ... 라벨 세는 로직 ...
    return result;
}
```

**왜 분리:**
- 헤더는 인터페이스만 → 컴파일 빨라짐
- 구현 수정해도 헤더를 include 한 파일들 영향 없음
- 본 학습은 Stage F 끝에 `.cpp` 에 정의 추가 예정

### [`const` 멤버 함수의 가치]
1. 코드 읽는 사람에게 "이 함수는 안전" 신호
2. `const SegmentedCloud&` 같은 매개변수에서도 호출 가능

```cpp
struct Foo {
    int x;
    void modify()      { x = 5; }    // 일반 멤버 함수
    int read() const   { return x; } // const 멤버 함수 — x 못 바꿈
};
```

### [C++ 노트]
- 멤버 함수 선언 끝의 `;` 꼭
- `const` 위치: 함수명 뒤 + `;` 앞 (다른 자리는 다른 의미)
- `std::vector<LidarPoint>` 는 `LidarPoint` 가 위에서 이미 정의돼야 가능 → type 정의 순서 중요

---

## 학습 #9 — `LidarSegmenter` 클래스 (`class`, 생성자, public/private, explicit)

### [요약]
지금까지의 `struct` 와 달리 **알고리즘을 들고 있는 클래스**. 새 문법 4 개가 한 번에: `class`, 접근 지정자, 생성자, `explicit`.

### [코드]
```cpp
class LidarSegmenter {
public:
    explicit LidarSegmenter(LidarSegmentationOptions options = {});

    SegmentedCloud process(const std::vector<LidarPoint>& points) const;

private:
    LidarSegmentationOptions options_;
};
```

### [`class` vs `struct`]
**유일한 차이는 기본 접근권한**:
| 키워드 | 기본 권한 |
|---|---|
| `struct` | `public` |
| `class` | `private` |

→ 데이터 묶음은 `struct`, 동작(메서드) 가진 추상화는 `class` 가 관례.

### [`public:` / `private:` 접근 지정자]
```cpp
public:        ← 아래 멤버들 외부 접근 허용
private:       ← 아래 멤버들 외부 접근 차단
```
- 외부 인터페이스(생성/실행) → public
- 내부 디테일(옵션 저장) → private (함부로 못 바꾸게)
- 끝의 **콜론 `:`** 빼먹지 말기 (세미콜론 아님)

### [생성자 (constructor) — 처음 등장]
```cpp
explicit LidarSegmenter(LidarSegmentationOptions options = {});
   (a)        (b)                  (c)                      (d)
```
| 위치 | 의미 |
|---|---|
| (a) `explicit` | 암묵 변환 차단 (아래 설명) |
| (b) **이름이 클래스명과 같음** | 생성자 신호. 반환 타입 적지 않음 |
| (c) 인자 1 개 | 옵션 묶음 |
| (d) `= {}` | 인자 기본값 — 옵션 생략 가능 |

객체 만들어질 때 자동 호출:
```cpp
LidarSegmenter seg1;                        // 옵션 생략 → 모든 default
LidarSegmenter seg2(my_options);            // 옵션 직접 전달
```

### [`explicit` 키워드 — 안전장치]
**없으면:**
```cpp
LidarSegmenter seg = my_options;            // ❌ "옵션→세그멘터" 암묵 변환 통과
```
**있으면:**
```cpp
LidarSegmenter seg(my_options);             // ✓ 명시적 — OK
LidarSegmenter seg = my_options;            // ❌ 컴파일 에러
```
→ 단일 인자 생성자는 거의 무조건 `explicit` 가 안전 관례.

### [`process(...) const` — `const` 멤버 함수 + const reference 인자]
```cpp
SegmentedCloud process(const std::vector<LidarPoint>& points) const;
   ↑              ↑      ↑                              ↑       ↑
  반환            이름   const reference 인자                  멤버 const
```

**`const T&` 인자 — 처음 만나는 문법:**
- `&` = 참조(reference). 복사 안 하고 원본을 가리킴
- 12만 점 복사하면 느림 → 참조면 0 비용
- `const` = 안 바꿀 거라는 약속

### [멤버 변수 끝 `_` 컨벤션]
```cpp
private:
    LidarSegmentationOptions options_;        // ← `_` 접미사
```
컴파일러는 신경 안 쓰지만 사람이 "아 이게 멤버 변수구나" 즉시 식별. 이 repo 스타일.

### [전체 사용 패턴 — Stage H 미리보기]
```cpp
LidarSegmentationOptions opt;
opt.floor_distance_threshold = 0.25;

LidarSegmenter seg(opt);                                      // 생성자 호출
std::vector<LidarPoint> points = load_kitti_velodyne_bin("0000.bin");
SegmentedCloud result = seg.process(points);                  // 참조 전달
SegmentCounts c = result.counts();
```

### [C++ 노트 — 새 문법 총정리]
| 문법 | 의미 | 빠뜨리면? |
|---|---|---|
| `class` | 기본 private 묶음 | struct 와 차이는 기본 권한만 |
| `public:` / `private:` | 멤버 접근 권한 | class 안 모든 멤버가 private (안 보임) |
| `Name(...);` | 생성자 (반환 없음, 이름=클래스명) | 객체 초기화 못 함 |
| `explicit` | 암묵 변환 차단 | `Foo f = something;` 사고 |
| `= {}` (인자) | 함수 인자 기본값 | 호출자가 매번 인자 적어야 |
| `const T&` (인자) | 복사 없는 참조, 수정 못 함 | 큰 객체 복사로 느려짐 |
| `() const;` (함수 뒤) | 멤버 변수 수정 안 함 | const 인스턴스에서 호출 불가 |
| 멤버 `_` 접미사 | 컨벤션 (강제 아님) | 이름 충돌 가능 (인자 vs 멤버) |

---

## 다음 단계 예고

| 다음 학습 | 새로 만나는 문법 |
|---|---|
| ⑦ 자유 함수 선언 2 줄 | 클래스 밖 함수 선언 (`load_kitti_velodyne_bin`, `write_segmented_ply`) |
| Stage B — `.bin` 읽기 | `std::ifstream` binary, `reinterpret_cast`, `tellg/seekg`, `throw` 예외 |
| Stage C — ROI 필터 | range-for, `std::isfinite`, lambda 캡처 |
| Stage D — RANSAC 코어 | Eigen `Vector3d` cross/dot, `std::mt19937` 난수, `std::function` 콜백, `std::move` |
| Stage E — Floor 분리 | 람다 `[this](...){...}`, `accept_plane` 콜백 패턴 |
| Stage F — Wall 3 종 그리디 피링 | 람다 재사용, bounding box 계산 |
| Stage G — PLY 저장 | `std::ofstream`, `std::filesystem::create_directories` |
| Stage H — main 빌드 + 검증 | `argv` 처리, `std::chrono` 타이밍, clang++ 한 줄 빌드 |

→ Stage A 의 9 개 학습(타입 정의 + C++ 기초)이 끝나면 **Stage B 부터가 알고리즘 본 게임**입니다.

---

## 참고 자료
- 원본 파이프라인 다이어그램: `docs/Practice/kitti_segmentation_rerun_pipeline.html`
- Reference 코드 (정답): `include/lidar/lidar_segmenter.hpp`, `src/lidar/lidar_segmenter.cpp`
- 본 학습 산출물 위치: `practice/lidar_clean_room/my_lidar_segmenter.hpp`
