# LiDAR Geometry-Based Floor/Wall Segmentation / LiDAR 기하 기반 바닥·벽 분할

Target Rerun file:

```text
results/kitti_raw_2011_09_26_drive_0117_full/lidar_segments_full.rrd
```

Rerun entity:

```text
/lidar/segments
```

## 1. Input Point Cloud / 입력 포인트 클라우드

KITTI Velodyne point는 3D point로 표현한다.

```math
\mathbf{p}_i =
\begin{bmatrix}
x_i \\
y_i \\
z_i
\end{bmatrix}
```

KITTI LiDAR coordinate frame:

```math
x: \mathrm{forward},
\qquad
y: \mathrm{left},
\qquad
z: \mathrm{up}
```

Vertical axis / 수직 기준축:

```math
\mathbf{e}_z =
\begin{bmatrix}
0 \\
0 \\
1
\end{bmatrix}
```

## 2. Plane Model / 평면 모델

Floor와 wall은 모두 3D plane으로 모델링한다.

```math
\pi: \mathbf{n}^{T}\mathbf{p} + b = 0,
\qquad
\|\mathbf{n}\| = 1
```

Point-to-plane distance / 점-평면 거리:

```math
d(\mathbf{p}_i, \pi)
=
\left|\mathbf{n}^{T}\mathbf{p}_i + b\right|
```

## 3. RANSAC Plane Fitting / RANSAC 평면 추정

RANSAC은 inlier point 수가 가장 많은 plane을 선택한다.

```math
(\mathbf{n}^{*}, b^{*})
=
\arg\max_{\mathbf{n}, b}
\sum_i
\mathbf{1}
\left(
\left|\mathbf{n}^{T}\mathbf{p}_i + b\right| < \tau
\right)
```

Here, `tau` is the inlier distance threshold.  
여기서 `tau`는 plane inlier로 인정할 거리 threshold다.

## 4. Floor Condition / 바닥 판정 조건

Floor plane normal은 LiDAR vertical axis와 거의 평행해야 한다.

```math
\mathrm{floor}
\quad \text{if} \quad
\left|\mathbf{n}_f^{T}\mathbf{e}_z\right| > \cos\theta_f
```

그리고 point가 floor plane에 충분히 가까워야 한다.

```math
\left|\mathbf{n}_f^{T}\mathbf{p}_i + b_f\right| < \tau_f
```

Intuition:

```math
\mathrm{floor\ normal} \parallel \mathbf{e}_z
```

## 5. Wall Condition / 벽 판정 조건

Wall plane normal은 vertical axis와 거의 수직이어야 한다.

```math
\mathrm{wall}
\quad \text{if} \quad
\left|\mathbf{n}_w^{T}\mathbf{e}_z\right| < \sin\theta_w
```

그리고 point가 wall plane에 충분히 가까워야 한다.

```math
\left|\mathbf{n}_w^{T}\mathbf{p}_i + b_w\right| < \tau_w
```

Intuition:

```math
\mathrm{wall\ normal} \perp \mathbf{e}_z
```

## 6. Wall Direction Split / 벽 방향 분리

KITTI LiDAR coordinate를 이용해 wall candidate를 방향별로 나눈다.

```math
\mathrm{left\_wall}: y_i > 0
```

```math
\mathrm{right\_wall}: y_i < 0
```

```math
\mathrm{front\_wall}: x_i > 0
```

## 7. Final Label / 최종 라벨

```math
\operatorname{label}(\mathbf{p}_i)
=
\begin{cases}
\mathrm{floor}, & \mathbf{p}_i \in I_f \\
\mathrm{left\_wall}, & \mathbf{p}_i \in I_w,\ y_i > 0 \\
\mathrm{right\_wall}, & \mathbf{p}_i \in I_w,\ y_i < 0 \\
\mathrm{front\_wall}, & \mathbf{p}_i \in I_w,\ x_i > 0 \\
\mathrm{other}, & \text{otherwise}
\end{cases}
```

## PPT Caption / 슬라이드 캡션

Velodyne point cloud에서 RANSAC으로 dominant plane을 찾고, plane normal과 LiDAR vertical axis의 관계를 이용해 floor와 wall을 분류한다.

```math
\mathrm{floor\ normal} \parallel \mathbf{e}_z,
\qquad
\mathrm{wall\ normal} \perp \mathbf{e}_z
```

## Minimal PPT Blocks / 한 장에 넣을 핵심 블록

1. Plane model / 평면 모델
2. RANSAC plane fitting / RANSAC 평면 추정
3. Floor-wall condition / 바닥·벽 판정 조건
4. Final label / 최종 라벨
