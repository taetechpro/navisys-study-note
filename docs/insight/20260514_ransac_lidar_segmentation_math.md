---
title: "RANSAC LiDAR Floor/Wall Segmentation 수식·코드·직관"
date: 2026-05-14
type: insight
tags: [ransac, lidar, segmentation]
related:
  - "[[20260504_kitti_lidar_floor_wall_segmentation]]"
  - "[[Practice/stage_d_ransac]]"
status: reviewed
---

# RANSAC 으로 LiDAR Floor/Wall Segmentation — 수식·코드·직관 정리

> **2026-05-14** · `src/lidar/lidar_segmenter.cpp` 의 plane-RANSAC 이 *수학적으로 무엇을 풀고 있는가* 를 코드와 1:1 매핑해서 정리.

---

## 0. 한 줄 요약

LiDAR point cloud 에 **RANSAC (Random Sample Consensus)** 을 써서 *세 점만 골라 평면을 세우고, 그 평면을 충분히 많은 점이 지지하면 채택* 하는 과정을 4회 반복(바닥 / 좌벽 / 우벽 / 전벽). 모델은 **3-parameter plane**, 데이터는 **3D point**, 평가지표는 **inlier count**, 그리고 도메인 prior 로 **normal 방향과 위치 박스 제약** 을 끼워 정확도를 높임.

---

## 1. RANSAC 일반 형식 — 무엇을 푸는가

### 📌 요약

RANSAC 은 **outlier 가 많은 데이터에서 모델 파라미터를 강건하게 추정** 하는 알고리즘. *최소 자유도 만큼만* 데이터를 뽑아 모델 후보를 만들고, 그 후보를 *전체 데이터로 검증* 하는 hypothesize-and-verify 패턴.

### 🧮 수식 — RANSAC 의 추상적 정의

데이터 $\mathcal{D}=\{x_i\}_{i=1}^N$, 모델 클래스 $\mathcal{M}=\{f(\cdot;\boldsymbol{\theta})\}$, residual 함수 $r(x_i;\boldsymbol{\theta})$, 임계값 $\tau$ 가 주어졌을 때 — **inlier consensus** 를 최대화하는 모델 파라미터:

$$
\hat{\boldsymbol{\theta}} \;=\; \arg\max_{\boldsymbol{\theta}\in\Theta}\;
\Bigl|\,\{\,i : r(x_i;\boldsymbol{\theta}) \le \tau\,\}\,\Bigr|
$$

이 문제는 **non-convex 이고 조합론적** 이라 직접 풀 수 없다. RANSAC 은 다음과 같이 **확률적으로 근사**:

```
반복 T 회:
  1. 𝒟 에서 무작위로 minimal sample S (|S| = s) 추출
  2. S 로 모델 θ_t 를 정확히 결정 (closed-form)
  3. 𝒟 전체에서 r(x_i; θ_t) ≤ τ 인 점 = 인라이어 집합 I_t
  4. |I_t| 가 지금까지 최대면 best ← (θ_t, I_t)
출력: best
```

여기서 $s$ = 모델의 minimum sample size (평면은 3, 선은 2, fundamental matrix 는 7 또는 8 등).

### 🎯 왜 이게 작동하나 — 반복 횟수 $T$ 의 유도

inlier 비율을 $w = |\mathcal{I}_\text{true}|/N$ 라 하자. 한 번의 minimal sample $s$ 점이 모두 inlier 일 확률:

$$
p_\text{all-inlier} = w^s
$$

$T$ 번 시도 중 **한 번도 all-inlier sample 을 못 뽑을 확률**:

$$
p_\text{fail} = (1 - w^s)^T
$$

이걸 원하는 실패 확률 $\epsilon$ (예: 0.01) 이하로 만들려면:

$$
\boxed{\;T \;\ge\; \frac{\log\epsilon}{\log(1-w^s)}\;}
$$

**예시 (이 코드의 floor RANSAC):** $s=3$, $w\approx 0.4$ (ROI 안에서 바닥 점 비율), $\epsilon=0.01$
$$T \ge \log(0.01)/\log(1-0.064) \approx 70$$
→ 코드의 `ransac_iterations = 180` 은 안전 마진 2~3x.

### 💡 직관

- RANSAC 은 *"모든 가능한 부분집합을 보지 않고 확률적으로만 본다"* 는 **Las Vegas 알고리즘**. 시간을 줄인 대가로 *전역최적해 보장이 사라진다*.
- $w$ 가 작으면 (outlier 가 많을수록) $T$ 가 *지수적으로* 증가. 바닥처럼 $w$ 가 큰 시나리오에서 매우 효율적, 1% 짜리 미세 구조 찾기에는 부적합.
- 대안: MSAC (M-estimator SAC), LO-RANSAC (local optimization), PROSAC (progressive). 현재 코드는 **vanilla RANSAC + cardinality score** 만 사용.

---

## 2. 평면 모델 — Minimal Sample s=3 의 정확한 결정

### 📌 요약

3D 평면은 3 자유도 (normal 2 + offset 1, 단위노름 제약 1 차감 → 사실상 3 DOF). **임의의 (collinear 하지 않은) 3점 → 평면 유일 결정**. 그래서 RANSAC 의 minimal sample size = 3.

### 🧮 수식 — 평면 표현과 3점 결정

평면을 **Hesse normal form** 으로:

$$
\boldsymbol{\pi} : \mathbf{n}^\top \mathbf{x} + d = 0,\quad \|\mathbf{n}\|=1
$$

- $\mathbf{n}\in\mathbb{R}^3$ : 단위 normal
- $d\in\mathbb{R}$ : 원점에서 평면까지 부호 있는 거리 ($-\mathbf{n}^\top\mathbf{p}$, 평면 위의 임의 점 $\mathbf{p}$)

세 점 $\mathbf{p}_a, \mathbf{p}_b, \mathbf{p}_c$ 가 주어지면:

$$
\mathbf{n}' \;=\; (\mathbf{p}_b - \mathbf{p}_a) \times (\mathbf{p}_c - \mathbf{p}_a)
\;\Rightarrow\;
\mathbf{n} = \mathbf{n}'/\|\mathbf{n}'\|,\quad
d = -\mathbf{n}^\top \mathbf{p}_a
$$

**collinear 가드**: $\|\mathbf{n}'\| < \epsilon$ 이면 reject (세 점이 한 직선 위라는 뜻 → 평면 불정).

### 🧮 수식 — Point-to-plane 거리 (residual)

임의 점 $\mathbf{q}$ 에서 평면까지의 *수직 거리*:

$$
r(\mathbf{q};\boldsymbol{\pi}) \;=\; |\mathbf{n}^\top \mathbf{q} + d|
$$

inlier 판정: $r(\mathbf{q};\boldsymbol{\pi}) \le \tau$.

### 🔧 코드 (`src/lidar/lidar_segmenter.cpp:41-70`)

```cpp
bool make_plane(const LidarPoint& a,
                const LidarPoint& b,
                const LidarPoint& c,
                PlaneModel& plane) {
    const Eigen::Vector3d pa = to_vec(a);
    const Eigen::Vector3d pb = to_vec(b);
    const Eigen::Vector3d pc = to_vec(c);
    Eigen::Vector3d n = (pb - pa).cross(pc - pa);     // n' = (b-a) × (c-a)
    const double norm = n.norm();
    if (norm < 1e-6) return false;                    // collinear 가드
    n /= norm;                                        // 단위 normal
    double d = -n.dot(pa);                            // d = -n·a
    if (n.z() < 0.0) { n = -n; d = -d; }              // normal 부호 통일
    plane.n = n;
    plane.d = d;
    plane.valid = true;
    return true;
}

double plane_distance(const PlaneModel& plane, const LidarPoint& p) {
    return std::abs(plane.n.dot(to_vec(p)) + plane.d);  // |n·q + d|
}
```

### 💡 직관

- $\mathbf{n}$ 의 **부호 반전** (`if (n.z() < 0.0)`): 외적은 점의 순서에 의존해 방향이 두 가지로 나옴. "위 방향 normal" 로 통일해두면 뒤의 *normal 방향 게이트* 가 절댓값 없이 깔끔해진다.
- $d$ 의 기하적 의미: 평면이 원점에서 $-d$ (부호 포함) 만큼 떨어져 있다 ($\mathbf{n}\cdot\mathbf{0}+d=d$). 바닥면 RANSAC 에서는 $d\approx |\text{LiDAR 높이}|$ 가 되어야 정상.

### 📚 C++ 노트

- `(pb - pa).cross(pc - pa)` — Eigen 의 `cross` 는 정확히 3-vector 에서만 정의. 4D 이상은 컴파일 에러.
- `std::abs(plane.n.dot(...) + plane.d)` — normal 이 단위벡터라 분모 정규화 생략 가능. 만약 비정규화 normal 이라면 `/ n.norm()` 필요.

---

## 3. Hypothesize-and-Verify Loop — Plane RANSAC 구현

### 📌 요약

`fit_plane_ransac` 가 위 알고리즘의 직접 구현. 표준 RANSAC 위에 **두 가지 도메인 제약** (`accept_plane` 함수와 `min_inliers` 게이트) 을 끼워 *후보 단계에서* 부적합 평면을 빠르게 버린다.

### 🔧 코드 (`src/lidar/lidar_segmenter.cpp:72-125`)

```cpp
PlaneModel fit_plane_ransac(
    const std::vector<LidarPoint>& points,
    const std::vector<std::size_t>& candidates,         // 검색 대상 인덱스
    const std::function<bool(const PlaneModel&)>& accept_plane,
    double distance_threshold,                          // τ
    int iterations,                                     // T
    std::size_t min_inliers) {                          // |𝓘| 하한
    PlaneModel best;
    if (candidates.size() < 3 || iterations <= 0) return best;

    std::mt19937 rng(7);                                // 결정적 시드
    std::uniform_int_distribution<std::size_t> pick(0, candidates.size() - 1);

    for (int iter = 0; iter < iterations; ++iter) {
        const std::size_t ia = pick(rng);
        std::size_t ib = pick(rng);
        std::size_t ic = pick(rng);
        if (ia == ib || ia == ic || ib == ic) continue; // 중복 방지

        PlaneModel plane;
        if (!make_plane(points[candidates[ia]],
                        points[candidates[ib]],
                        points[candidates[ic]], plane))
            continue;                                   // collinear → 다음

        if (!accept_plane(plane)) continue;             // ← 도메인 prior 게이트

        std::vector<std::size_t> inliers;
        for (const std::size_t idx : candidates) {
            if (plane_distance(plane, points[idx]) <= distance_threshold)
                inliers.push_back(idx);
        }
        if (inliers.size() > best.inliers.size()) {     // cardinality score
            plane.inliers = std::move(inliers);
            best = std::move(plane);
        }
    }

    if (best.inliers.size() < min_inliers) {            // 최종 게이트
        best.valid = false;
        best.inliers.clear();
    }
    return best;
}
```

### 🧮 수식 — 실제 구현 대응

| 수식 기호 | 코드 |
|---|---|
| $T$ | `iterations` (=180) |
| $\tau$ | `distance_threshold` (floor: 0.18 m, wall: 0.25 m) |
| minimal sample $S$ | `(ia, ib, ic)` 세 인덱스 |
| 후보 모델 $\boldsymbol{\theta}_t$ | `plane` |
| inlier set $\mathcal{I}_t$ | `inliers` |
| score $|\mathcal{I}_t|$ | `inliers.size()` |
| 사전 제약 $\mathcal{A}(\boldsymbol{\theta})$ | `accept_plane(plane)` lambda |
| 최종 채택 조건 | `best.inliers.size() >= min_inliers` |

### 💡 직관

- `accept_plane` 가 *없다면* 천장이나 트럭 옆면이 "바닥" 으로 선정될 수 있다. 도메인 prior 가 RANSAC 의 ambiguity 를 깬다.
- `min_inliers` 가 *없다면* 잡음 무리 위에 작은 평면이 거짓 채택될 수 있다. 카디널리티 하한이 spurious detection 방지.
- `mt19937 rng(7)` — **고정 시드**. 같은 frame 에 대해 항상 같은 결과 → 디버깅 가능. 다만 *frame 간 독립성 가정* 이 약해지므로, 시간순 분석에서 작은 편향 가능성.

### 📚 C++ 노트

- `std::function<bool(const PlaneModel&)>` — accept criterion 을 lambda 로 받아 다형성 확보. 바닥/좌벽/우벽/전벽 각각 다른 람다 주입.
- `std::mt19937 rng(7)` vs `std::random_device{}()`: 전자는 재현 가능, 후자는 매 실행 다른 결과. 연구 코드는 **재현 가능성 우선** 으로 전자 선택이 맞다.
- `std::uniform_int_distribution<std::size_t>` — 정확한 균등분포 보장 (modulo 편향 없음).
- `std::move(inliers)` / `std::move(plane)` — 큰 vector copy 회피. RANSAC 핫루프에서 무시 못 할 차이.

---

## 4. 도메인 Prior — 바닥/벽 각각의 `accept_plane` 게이트

### 📌 요약

순수 RANSAC 은 "어떤 평면" 이든 받지만, 자율주행에서 *어떤 평면이 바닥인지* 는 *물리적으로 정해져 있다*. 이 사전 정보(자세 prior + 위치 prior)를 평면 후보 단계에서 적용해 가짜 평면을 일찍 버린다.

### 4-1. Floor — 수평하고 LiDAR 아래에 있어야

### 🧮 수식

$$
\mathcal{A}_\text{floor}(\mathbf{n}, d) = \Bigl[|n_z| \ge n_z^{\min}\Bigr]\;\land\;\Bigl[z_\text{origin}(\mathbf{n},d)\in[z^{\min}_f, z^{\max}_f]\Bigr]
$$

여기서:
- $n_z^{\min}=0.85$ ⇒ 평면의 normal 이 +Z 와 *32° 이내* (수평 가정).
- $z_\text{origin} = -d/n_z$ : 평면이 $x=y=0$ 위에서 가지는 $z$ 값. LiDAR 가 차량 지붕 위 1.73 m 에 있으므로 바닥은 $z\approx -1.73$ 부근.
- $[z^{\min}_f, z^{\max}_f] = [-2.6,\,-0.4]\,\text{m}$ ⇒ KITTI 차량 기준 합리적 범위.

기하적 의미: $\mathbf{n}$ 가 거의 수직이고 그 평면을 $x=y=0$ 으로 평행이동했을 때 $z$ 값이 차량 아래에 있어야 한다.

### 🔧 코드 (`src/lidar/lidar_segmenter.cpp:217-224`)

```cpp
[this](const PlaneModel& plane) {
    if (std::abs(plane.n.z()) < options_.floor_normal_min_z) return false;
    const double z_at_origin = -plane.d / plane.n.z();
    return z_at_origin >= options_.floor_search_min_z &&
           z_at_origin <= options_.floor_search_max_z;
}
```

### 4-2. Side wall (Left / Right) — 거의 수직, normal 이 $\pm Y$ 방향

### 🧮 수식

$$
\mathcal{A}_\text{side}(\mathbf{n}) = \Bigl[|n_z| \le n_z^{\max,w}\Bigr]\;\land\;\Bigl[|n_y| \ge n_\text{axis}^{\min}\Bigr]
$$

- $n_z^{\max,w}=0.25$ ⇒ normal 이 *수평에서 14° 이내* (벽이 거의 수직).
- $n_\text{axis}^{\min}=0.70$ ⇒ normal 이 *Y 축에 정렬* (좌/우 벽).

좌/우 구분은 게이트가 아니라 *후보 풀* 단계에서 `p.y >= side_min_abs_y` 또는 `<= -side_min_abs_y` 로 처리.

### 4-3. Front wall — 거의 수직, normal 이 $\pm X$ 방향

### 🧮 수식

$$
\mathcal{A}_\text{front}(\mathbf{n}) = \Bigl[|n_z| \le n_z^{\max,w}\Bigr]\;\land\;\Bigl[|n_x| \ge n_\text{axis}^{\min}\Bigr]
$$

전방 차량/건물 외벽이 차량을 *마주보는* 평면. 후보 풀: `p.x >= front_min_x && |p.y| <= front_max_abs_y`.

### 💡 직관 (4-1~4-3 통합)

- 세 게이트가 사실상 **"평면의 normal 이 어느 축에 정렬되는가"** 에 대한 prior. 자연환경에는 Manhattan 가정 (서로 수직인 3축 평면) 이 자주 맞는다.
- normal 게이트 + 후보 풀 ROI 조합으로 RANSAC 이 *각 평면을 독립적으로* 찾을 수 있게 만든다. 한 번에 모든 평면을 동시 추정 ($\to$ EM, multi-model RANSAC) 하지 않고 *분할 정복* 으로 처리.

### 🔧 코드 매핑 (`src/lidar/lidar_segmenter.cpp:246-287`)

```cpp
// Left wall
[this](const PlaneModel& plane) {
    return std::abs(plane.n.z()) <= options_.wall_normal_max_abs_z &&
           std::abs(plane.n.y()) >= options_.wall_axis_min_abs;
}
// Right wall — 동일 lambda (좌/우는 후보 ROI 로 구분)
// Front wall
[this](const PlaneModel& plane) {
    return std::abs(plane.n.z()) <= options_.wall_normal_max_abs_z &&
           std::abs(plane.n.x()) >= options_.wall_axis_min_abs;
}
```

---

## 5. 사후 검증 — Extent 게이트

### 📌 요약

inlier 개수만으로는 "한 평면 위에 우연히 모인 동전 더미" 도 통과시킨다. 그래서 **공간적으로 충분히 펼쳐졌는가** 를 한 번 더 확인. 벽이면 *가로 폭 + 세로 높이*, 바닥이면 게이트 없음.

### 🧮 수식

inlier 집합 $\mathcal{I}$ 의 점들의 bounding-box 길이를 $\Delta x, \Delta y, \Delta z$ 라 하면:

- 좌/우 벽: $\Delta x \ge \Delta x_\min^w$ (차량 진행 방향 폭) **AND** $\Delta z \ge \Delta z_\min^w$ (높이)
- 전벽: $\Delta y \ge \Delta y_\min^w$ (좌우 폭) **AND** $\Delta z \ge \Delta z_\min^w$

본 코드 기본값: `min_wall_extent_m = 3.0`, `min_wall_height_m = 1.0`.

### 🔧 코드 (`src/lidar/lidar_segmenter.cpp:151-181`)

```cpp
bool has_wall_extent(const std::vector<LidarPoint>& points,
                     const std::vector<std::size_t>& inliers,
                     SegmentLabel label,
                     const LidarSegmentationOptions& opt) {
    // inlier 들의 AABB 계산
    double min_x = ∞, max_x = -∞, min_y = ∞, max_y = -∞, min_z = ∞, max_z = -∞;
    for (idx : inliers) { /* 갱신 */ }

    const double horizontal_extent =
        (label == SegmentLabel::FrontWall) ? (max_y - min_y) : (max_x - min_x);
    const double vertical_extent = max_z - min_z;
    return horizontal_extent >= opt.min_wall_extent_m &&
           vertical_extent >= opt.min_wall_height_m;
}
```

### 💡 직관

- 점 N 개가 한 평면에 *우연히 가까이* 있어도, 그 점들이 *국소적으로 뭉쳐* 있다면 벽이라 부를 수 없다 (그냥 표면 한 조각). extent 게이트가 이 모호함을 해소.
- 바닥은 ROI 자체가 넓어서 extent 게이트 없이도 안정적 — 그래서 코드도 floor 에는 적용 안 함.

---

## 6. 전체 흐름 — `LidarSegmenter::process`

### 📌 요약

1. ROI clip + finite 검사 → 깨끗한 `cloud`
2. 모든 라벨 `Other` 로 초기화
3. **Floor RANSAC** → 바닥 inlier 를 `Floor` 라벨
4. **Left wall RANSAC** (후보풀: $y\ge +y_\text{side}$, 미라벨)
5. **Right wall RANSAC** (후보풀: $y\le -y_\text{side}$, 미라벨)
6. **Front wall RANSAC** (후보풀: $x\ge x_\text{front}$ ∧ $|y|\le y_\text{front}^{\max}$, 미라벨)

핵심: **각 단계가 끝나면 inlier 가 `Floor`/`*Wall` 로 라벨링되어, 다음 RANSAC 의 후보풀에서 자동 제외**. 이것이 *순차적 multi-plane segmentation* 의 표준 전략.

### 🔧 코드 골격 (`src/lidar/lidar_segmenter.cpp:195-289` 요약)

```cpp
SegmentedCloud LidarSegmenter::process(const std::vector<LidarPoint>& points) const {
    // 1. ROI clip
    for (const auto& p : points)
        if (is_finite(p) && in_roi(p, options_)) cloud.points.push_back(p);
    cloud.labels.assign(cloud.points.size(), SegmentLabel::Other);

    // 2. Floor
    auto floor_candidates = collect_candidates(cloud, /* Other ∧ z ∈ floor_search 범위 */);
    const auto floor = fit_plane_ransac(cloud.points, floor_candidates,
                                         /* accept: |n_z|≥0.85 ∧ z_origin ∈ floor 범위 */,
                                         options_.floor_distance_threshold,    // τ_f
                                         options_.ransac_iterations,            // T
                                         options_.min_floor_inliers);
    apply_label(cloud, floor, SegmentLabel::Floor);

    // 3~5. Left/Right/Front wall (동일 패턴, 후보 ROI 와 accept lambda 만 다름)
    // 단, fit_wall 안에서 has_wall_extent 추가 검사
}
```

### 🧮 수식 — 순차적 라벨링의 의미

$k$ 번째 평면 RANSAC 이 풀려는 문제:

$$
\hat{\boldsymbol{\pi}}_k \;=\; \arg\max_{\boldsymbol{\pi}\in\mathcal{P}_k}\;\bigl|\{\,\mathbf{x}_i\in\mathcal{C}_k \,:\, r(\mathbf{x}_i;\boldsymbol{\pi})\le\tau_k\,\}\bigr|
$$

여기서 후보풀은 *이전 라벨링이 반영된 집합*:

$$
\mathcal{C}_k = \{\,\mathbf{x}_i : \text{label}_i = \text{Other} \;\land\; \mathbf{x}_i\in\mathcal{R}_k\,\}
$$

$\mathcal{R}_k$ 는 평면별 ROI (좌벽: $y\ge 2$, 우벽: $y\le -2$, 전벽: $x\ge 5$ etc.). 이 *greedy* 전략은 전역최적해를 보장하지 않지만, 평면들이 *충분히 분리* 되어 있는 환경에서는 잘 작동.

### 💡 직관 — 왜 동시추정 안 하나?

- 멀티 평면 동시추정 (예: J-Linkage, Multi-RANSAC) 은 모델 결합 explosion. $T^4$ 이상.
- 자율주행 환경에서 normal 방향이 *물리적으로 분리* 되어 있어 greedy 가 거의 항상 최적해와 일치.
- 코드 단순성: 평면 4개에 대해 같은 함수 반복.

---

## 6½. 데이터 흐름 트레이스 — 한 `.bin` 파일이 segmented cloud 가 되기까지

### 📌 요약

LiDAR raw 데이터가 *어떤 메모리 구조* 에서 출발해 *어떤 정수 인덱스 indirection* 을 거쳐 RANSAC 안으로 들어가고, 어떻게 라벨이 되어 나오는지의 *런타임 트레이스*. 수식 위가 아닌 *데이터 그릇* 의 모양을 본다.

### Stage 0 — Raw `.bin` 파일

KITTI Velodyne HDL-64E 한 frame:

| 항목 | 값 |
|---|---|
| 포인트 수 | ~120,000 |
| 파일 크기 | ~1.9 MB |
| 형식 | float32 × 4 interleaved: `(x, y, z, intensity)` 반복 |

```
[x0|y0|z0|i0][x1|y1|z1|i1] ... [x_{N-1}|y_{N-1}|z_{N-1}|i_{N-1}]
```

### Stage 1 — `load_kitti_velodyne_bin`

```cpp
// src/lidar/lidar_segmenter.cpp:320-348
std::vector<LidarPoint> points(count);   // count ≈ 120,000
for (auto& point : points) {
    std::array<float, 4> values{};
    f.read(reinterpret_cast<char*>(values.data()), 16);
    point.x = values[0]; point.y = values[1];
    point.z = values[2]; point.intensity = values[3];
}
```

**결과**: 힙에 16 byte × ~120k ≈ 1.9 MB 의 연속 배열. **C++ struct 메모리 레이아웃이 .bin 파일과 1:1 동형**이라 read 한 번에 바로 들어감.

### Stage 2 — ROI clip + finite 검사

```cpp
// lines 199-203
for (const auto& p : points) {
    if (is_finite(p) && in_roi(p, options_)) {
        cloud.points.push_back(p);    // 복사 (값 의미)
    }
}
```

ROI 박스: $x\in[-5,50],\;y\in[-25,25],\;z\in[-3,3]$ → 뒤·옆 멀리·하늘 제거.

| | Before | After |
|---|---|---|
| 점 개수 | 120,000 | **30,000–50,000** |

이 단계만 한 번 *점 자체를 복사*. 이후 모든 단계는 인덱스만 옮김.

### Stage 3 — 라벨 배열 초기화

```cpp
cloud.labels.assign(cloud.points.size(), SegmentLabel::Other);
```

`labels` 는 `vector<uint8_t>`, **points 와 동일 길이·동일 순서**. 점 1개당 1 byte.

```
cloud.points   [P0][P1][P2][P3][P4][P5] ...   ← 16 byte/each
cloud.labels   [O ][O ][O ][O ][O ][O ] ...   ←  1 byte/each
                ↑ 같은 인덱스 i 가 (point, label) 짝
```

### Stage 4 — Floor candidate 수집

```cpp
auto floor_candidates = collect_candidates(cloud, [this](auto& p, auto label) {
    return label == SegmentLabel::Other &&
           p.z >= options_.floor_search_min_z &&   // -2.6
           p.z <= options_.floor_search_max_z;     // -0.4
});
```

**중요**: `floor_candidates` 는 *점이 아니라 인덱스* 의 vector (`std::vector<size_t>`).

```
cloud.points   [P0][P1][P2][P3][P4][P5][P6][P7][P8][P9] ...
                          ↑                       ↑
floor_candidates   = [3, 7, ...]   ← cloud.points 의 인덱스 모음
```

**메모리 절약**: 점 16 byte 복사 대신 인덱스 8 byte 저장.  
**Stage 4 결과**: 보통 **10,000–20,000 인덱스**.

### Stage 5 — RANSAC iteration 한 바퀴의 데이터 트레이스

루프 한 iteration 안에서 데이터가 어떻게 움직이는지 (이중 indirection 이 핵심):

```
[5-A] rng() 세 번 → ia=4231, ib=89, ic=15772   ← 0 ≤ * < candidates.size()

[5-B] 인덱스 한 번 더 옮기기:
       candidates[ia]=88293  → cloud.points[88293] = (12.3, 8.1, -1.69)
       candidates[ib]=1024   → cloud.points[1024]  = (4.2, -3.0, -1.71)
       candidates[ic]=44012  → cloud.points[44012] = (28.7, 14.2, -1.66)

[5-C] make_plane(pa, pb, pc):
       v1 = pb - pa = (-8.1, -11.1, -0.02)
       v2 = pc - pa = (16.4,  6.1,  0.03)
       n' = v1 × v2 ≈ (-0.45, 0.57, 132.7)
       ‖n'‖ ≈ 132.7
       n  = n'/‖n'‖ ≈ (-0.003, 0.004, 0.9999)   ← 거의 +Z!
       d  = -n·pa  ≈ 1.694

[5-D] accept_plane(plane):
       |n_z| = 0.9999 ≥ 0.85    ✓
       z_origin = -d/n_z = -1.694 ∈ [-2.6, -0.4]   ✓
       → 통과

[5-E] inlier scan: ★ 비싼 단계, candidates 전체 순회
       for idx in floor_candidates:           // ~15,000 회
           q = cloud.points[idx]
           r = |n·q + d|
           if r ≤ 0.18: inliers.push_back(idx)
       → inliers.size() = 8432 (예시)

[5-F] if 8432 > best.inliers.size():
          best.plane   = (n, d)
          best.inliers = inliers              // 인덱스 vector move
```

iteration 한 번의 *지배적 비용* 은 **5-E** 의 $N$ 번 거리 계산.

### Stage 5 — 전체 비용

| 항목 | 양 |
|---|---|
| iterations $T$ | 180 |
| candidate pool 크기 $N$ | ~15,000 |
| 총 거리 계산 | $T\cdot N \approx 2.7\text{M}$ |
| 거리 1회 비용 | $\approx 6$ flop |
| 총 flop | $\approx 16\text{M}$ |
| 1 core ~5 GHz 환경 wall time | **~5–8 ms** |

### Stage 6 — 라벨 적용

```cpp
for (const std::size_t idx : plane.inliers) {
    cloud.labels[idx] = label;   // Other → Floor
}
```

best.inliers (예: 8,432 개) 의 인덱스가 cloud.labels 의 해당 칸을 덮어쓴다.

```
labels (before)   [O ][O ][O ][O ][O ][O ][O ][O ] ...
                                     ↑     ↑
                                 idx=4, 6 이 inlier
labels (after)    [O ][O ][O ][O ][F ][O ][F ][O ] ...
```

### Stages 7-9 — Left / Right / Front wall

같은 RANSAC 함수가 세 번 더 호출되지만 **두 가지만 달라진다**:

| | predicate (위치 게이트) | accept (normal 게이트) | 예상 inlier |
|---|---|---|---|
| Left wall | `label==Other ∧ z∈[-1.4,2.5] ∧ y≥+2` | $\|n_z\|\le 0.25 \land \|n_y\|\ge 0.7$ | 100–2,000 |
| Right wall | `label==Other ∧ z∈[-1.4,2.5] ∧ y≤-2` | 동일 | 100–2,000 |
| Front wall | `label==Other ∧ z∈[-1.4,2.5] ∧ x≥5 ∧ \|y\|≤12` | $\|n_z\|\le 0.25 \land \|n_x\|\ge 0.7$ | 100–3,000 |

**핵심**: `collect_candidates` 가 매번 `label==Other` 인 점만 모으므로 **이전 RANSAC 에서 라벨된 점은 자동 제외**. greedy multi-plane 의 본질.

### Stage 10 — 출력

```
SegmentedCloud {
    points:  vector<LidarPoint>  (~40,000)
    labels:  vector<SegmentLabel> (~40,000)   // 1:1 짝
}
```

→ PLY 파일 (`write_segmented_ply`) 또는 Rerun (`log_lidar_segments`) 로 시각화. RANSAC 결과의 *해석* 은 외부 모듈에서.

### 데이터의 *세 좌표공간*

이 코드 안에서 데이터가 동시에 세 가지 인덱스 공간을 산다 — 헷갈리지 않으려면 분명히 구분:

```
[1] 3D 물리 공간
     LidarPoint(x, y, z): 미터 단위, KITTI Velodyne 본체 좌표.

[2] cloud 의 정수 인덱스 i  (0 ≤ i < cloud.points.size())
     cloud.points[i] 와 cloud.labels[i] 가 짝.
     ROI clip 이후 ~40,000 개.

[3] candidate 의 정수 인덱스 j  (0 ≤ j < candidates.size())
     candidates[j] = i (위 공간 [2] 의 정수)
     RANSAC 의 pick(rng) 가 만드는 정수는 j.
     candidates[ pick(rng) ] → 한 번 더 옮겨야 *진짜 점* 도착.
```

→ 이중 indirection $j\to i\to \mathbf{p}$ 가 *점 자체는 한 번도 복사·정렬되지 않게* 하는 효율 트릭.

### 시간 budget — KITTI 10 Hz 환경

| 단계 | 1 core wall time |
|---|---|
| load `.bin` | ~3 ms |
| ROI clip + finite | ~1 ms |
| Floor RANSAC | ~5–8 ms |
| Wall RANSAC ×3 | ~3–5 ms each |
| 라벨 적용 | <1 ms |
| **합계** | **~20–30 ms** |

frame 간 간격 100 ms 중 ~25 ms 사용 → 여유 75 ms 가 VIO 와 시각화에 할당. *동시에 돌릴 수 있는 이유*.

### 💡 직관 — 자주 묻는 함정

- **"RANSAC 안에서 점을 정렬하나?"** → 아니. 입력 순서 그대로. candidates 도 점 좌표가 아니라 *원본 인덱스 그대로* 의 부분집합.
- **"매 iteration 마다 candidates 전체를 보나?"** → 그렇다. 이게 RANSAC 의 지배적 비용. 일부 변형 (PROSAC, NAPSAC) 은 *uniform 이 아닌* sampling 으로 줄이지만, 본 코드는 vanilla.
- **"inlier 가 겹치면?"** → 라벨이 *덮어쓰기* 라 마지막 RANSAC 이 이긴다. 하지만 *후보풀에서 이미 라벨된 점을 빼므로* 사실상 겹침이 없음 — 이게 순차 라벨링의 보호장치.
- **"먼 곳의 점도 똑같이 본다?"** → 그렇다. distance threshold 가 *절대값* $\tau$ 라 멀리 있는 노이즈 큰 점이 동일 가중. range-adaptive threshold 가 자연스러운 확장.

### 📚 C++ 노트 — 이 트레이스에서 새로 배운 패턴

- **`std::vector<std::size_t> candidates`** 패턴: 점 vector 의 *인덱스만* 들고 다니는 zero-copy 부분집합. SLAM / point cloud 코드의 표준 관용구.
- **`for (const std::size_t idx : candidates)`** range-for: 인덱스 vector 위를 깔끔하게 순회. raw `for (size_t j=0; j<n; ++j)` 보다 의도 명확.
- **`std::move(inliers)` / `std::move(plane)`**: best 갱신 시 vector 복사 0. 본 코드 핫루프의 가장 중요한 최적화 — *최선 갱신 빈도 × 인라이어 vector 길이* 만큼 시간 절약.
- **`reinterpret_cast<char*>(values.data())`**: float 배열을 그대로 byte stream 으로. KITTI .bin 같은 *정의된 바이너리 포맷* 에서만 안전. 일반 객체에는 절대 금지.
- **struct LidarPoint 의 padding**: `float x,y,z,intensity` 4개 = 16 byte 정확히 일치. 만약 멤버 순서를 바꾸거나 다른 타입을 끼우면 alignment padding 으로 reinterpret_cast 가 깨진다. 외부 바이너리 호환을 의식한 설계.

---

## 7. RANSAC 의 통계적 해석 — 왜 "consensus" 가 좋은 score 인가

### 🧮 수식 — Maximum Likelihood 관점

inlier 가 평면 주변 가우시안, outlier 가 균등분포에서 나왔다고 모델하면:

$$
p(r_i\mid \boldsymbol{\theta}) \;=\; \pi\,\mathcal{N}(r_i;0,\sigma^2) + (1-\pi)\,\mathcal{U}(r_i;[-L,L])
$$

이 mixture 의 log-likelihood 는 inlier 비율 $\pi$ 와 $\boldsymbol{\theta}$ 의 함수. **임계값 $\tau \approx 2\sigma$ 로 hard-threshold** 하면 likelihood maximization 이 *cardinality maximization* 과 근사적으로 동치 → RANSAC 의 "count inliers" 가 ML 의 근사라는 사실.

### 💡 직관

- **M-estimator (MSAC)** 은 hard-threshold 대신 soft loss (Huber, Geman-McClure) 를 써 정밀도를 약간 높임. 본 코드는 hard threshold (구현 단순성 우선).
- $\tau$ 의 의미: *measurement noise + plane curvature* 합산의 표준편차 2~3배 정도가 합리적. 바닥의 0.18 m, 벽의 0.25 m 는 KITTI LiDAR 노이즈 (~5 cm) 보다 훨씬 크다 — *지면 미세 굴곡, 차량 진동, 도로 캠버* 를 흡수하기 위한 설계 마진.

---

## 8. 한계와 확장 — 다음 학습 방향

### 현재 구현의 한계

| 한계 | 증상 | 해결방향 |
|---|---|---|
| Cardinality score | 같은 inlier 수면 평면이 *얇은지/두꺼운지* 구분 못 함 | MSAC: $\sum_i \min(r_i^2, \tau^2)$ |
| Hard threshold | $\tau$ 근처에서 binary cliff | MLESAC / soft RANSAC |
| Greedy multi-plane | 평면들이 겹치면 잘못된 분할 | J-Linkage, multi-model RANSAC |
| Fixed iterations | $T=180$ 고정, 실제 $w$ 가 크면 낭비 | Adaptive RANSAC: 매 inlier 갱신마다 $T$ 재계산 |
| Random seed 고정 | frame 간 편향 가능 | 시드를 timestamp 로, 또는 분석 시 여러 seed 평균 |
| 도메인 prior 가 hand-coded | 새 환경 (실내, 터널) 마다 튜닝 | learned prior, ground-projected histogram |

### 적용 가능한 EKF/VIO 의미

- 바닥 평면이 *연속 frame 에서 일관* 되게 검출되면 **gravity 방향 measurement** 로 EKF 에 넣을 수 있다 (yaw 외 attitude 추가 관측). 이미 `docs/insight/20260509_semantic_plane_normal_gravity_prior.md` 의 주제.
- 벽이 검출되면 *방향 prior* 가 되어 yaw drift 까지 부분적으로 묶는다 (Manhattan 가정).
- 단, RANSAC 자체는 결정론적 추정이 아니므로 EKF 에 넣을 때는 **inlier count 와 covariance 의 적절한 매핑** 이 필수.

---

## 9. 부록 — 기호·코드 사전

| 기호 | 의미 | 코드 |
|---|---|---|
| $\mathbf{n}\in\mathbb{R}^3$ | 평면 단위 normal | `plane.n` |
| $d\in\mathbb{R}$ | 평면 offset, $\mathbf{n}^\top\mathbf{x}+d=0$ | `plane.d` |
| $\tau$ | inlier 거리 임계값 | `*_distance_threshold` |
| $T$ | RANSAC 반복 수 | `ransac_iterations` |
| $s$ | minimal sample size (=3) | `make_plane` 의 인자 3개 |
| $\mathcal{I}_t$ | 시도 $t$ 의 inlier 집합 | `inliers` |
| $w$ | true inlier 비율 (사전 불명) | (이론적 가정만) |
| $\mathcal{A}(\boldsymbol{\theta})$ | accept criterion | `accept_plane` lambda |
| $\mathcal{C}_k$ | $k$ 번째 평면의 candidate pool | `collect_candidates` 결과 |
| $\mathcal{R}_k$ | $k$ 번째 평면의 공간 ROI | 람다 안의 좌표 조건들 |

---

## 10. C++ 학습 누적

- **`std::function<bool(const PlaneModel&)>`** — 타입 소거를 통한 다형성. virtual 함수 없이 평면별 accept 로직을 외부 주입.
- **`[this]` lambda capture** — 클래스 멤버 함수 안에서 옵션 (`options_.floor_normal_min_z` 등) 접근. 캡처 안 하면 빌드 에러.
- **`std::mt19937 rng(7)`** — Mersenne Twister, 시드 7. 결정적 재현용. 학술 코드에서는 *재현성이 정답이 아닌 종교* 같은 부분.
- **`std::uniform_int_distribution`** — `rng() % N` 보다 modulo-bias 가 없는 정통적 방법.
- **`std::move(plane)` / `std::move(inliers)`** — RANSAC 핫루프에서 vector 복사 비용을 0 으로. C++11 rvalue ref 가 가장 효과적인 자리.
- **default member initializer** (`double floor_distance_threshold = 0.18;`) — YAML 에 해당 키가 없어도 안전한 기본값 보장.
- **`enum class SegmentLabel : std::uint8_t`** — 강한 타입 + 명시적 바이트 크기. label vector 가 메모리 1바이트/점 → 큰 cloud 에서 RAM 절약.
- **`std::filesystem::create_directories`** — PLY 출력 시 폴더 자동 생성. C++17 부터 표준.
- **`std::ifstream f(path, std::ios::binary | std::ios::ate)`** — KITTI .bin 읽기 패턴. `ate` (at-the-end) 로 파일 크기 즉시 확인 후 seekg 으로 처음으로.

---

관련: [[20260504_kitti_lidar_floor_wall_segmentation]] (실제 segmentation 구현 기록, 파라미터 튜닝), [[20260509_semantic_plane_normal_gravity_prior]] (검출된 평면을 EKF prior 로 활용), [[20260514_lc_ekf_cam_imu_fusion]] (이 segmentation 결과를 EKF 측정으로 넣을 때의 흐름).
