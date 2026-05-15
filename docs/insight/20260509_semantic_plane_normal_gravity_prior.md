# Semantic Plane Label과 Normal Vector - 2026-05-09

## Insight

Semantic label은 plane을 직접 정의하는 것이 아니라, 먼저 추정된 plane normal vector와 gravity direction의 관계를 보고 붙이는 해석이다.

즉 floor/wall segmentation에서 핵심은 다음 순서다.

1. Depth point cloud에서 plane을 추정한다.
2. 각 plane의 normal vector를 계산한다.
3. IMU에서 얻은 gravity direction과 normal vector의 각도를 비교한다.
4. 그 결과로 `floor`, `wall`, `other` semantic label을 붙인다.

짧게 쓰면 다음과 같다.

```text
semantic plane label = geometry plane normal + gravity prior의 해석 결과
```

## Floor와 Wall의 차이

Plane은 보통 다음 형태로 표현된다.

```text
n^T x + d = 0
```

여기서 `n`은 plane normal vector이고, `g`는 gravity direction이다.

Floor는 normal vector가 gravity direction과 거의 평행하다.

```text
floor if |n dot g| ~= 1
```

Wall은 normal vector가 gravity direction과 거의 수직이다.

```text
wall if |n dot g| ~= 0
```

따라서 semantic label이 다르면 일반적으로 plane normal vector도 다르다. 특히 `floor`와 `wall`은 gravity 기준으로 명확히 다른 방향성을 가진다.

## 중요한 점

같은 `wall` semantic label 안에서도 left wall, right wall, front wall은 서로 다른 normal vector를 가질 수 있다.

예를 들어 gravity-aligned frame에서:

| Plane | Normal 예시 | Gravity와 관계 |
|---|---:|---|
| floor | `[0, 0, 1]` | 평행 |
| left wall | `[1, 0, 0]` | 수직 |
| right wall | `[-1, 0, 0]` | 수직 |
| front wall | `[0, 1, 0]` | 수직 |

즉 `wall`이라는 semantic label은 하나의 고정된 벡터를 의미하지 않는다. 여러 wall plane은 모두 gravity와 수직이라는 공통 조건을 만족하지만, 각 wall의 normal vector는 서로 다를 수 있다.

## SLAM 관점에서의 의미

이 인사이트는 floor/wall segmentation을 단순한 시각화 결과가 아니라 SLAM/VIO에서 재사용 가능한 geometric feature로 확장할 때 중요하다.

Semantic label만 저장하면 정보가 부족하다. Plane normal, distance, inlier set, gravity 기준 각도까지 함께 저장해야 map constraint나 pose correction에 사용할 수 있다.

