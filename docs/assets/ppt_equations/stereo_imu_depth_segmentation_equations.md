# Stereo-IMU Geometry-Based Floor/Wall Segmentation / Stereo-IMU 기하 기반 바닥·벽 분할

Source PDF:

```text
docs/assets/ppt_equations/stereo_imu_depth_segmentation_equations.pdf
```

Target Rerun file:

```text
results/kitti_raw/2011_09_26_drive_0117/full_sequence/rerun/depth_segments_every5.rrd
```

Rerun entity:

```text
/depth/segments
```

## 1. Stereo Disparity / 스테레오 disparity

Rectified stereo pair에서는 같은 3D point가 좌우 이미지에서 같은 scanline에 놓인다.
좌우 x-coordinate 차이를 disparity로 둔다.

```math
d(u,v) = u_L - u_R
```

Depth is recovered from disparity:

```math
Z = \frac{f_x B}{d}
```

where:

```text
fx: focal length
B : stereo baseline
d : disparity
Z : depth
```

## 2. Pixel to 3D Point / 픽셀에서 3D point 복원

Depth `Z`와 camera intrinsics를 이용해 rectified camera frame의 3D point를 만든다.

```math
X = \frac{(u-c_x)Z}{f_x},
\qquad
Y = \frac{(v-c_y)Z}{f_y},
\qquad
Z = \frac{f_x B}{d}
```

```math
\mathbf{p}_{\mathrm{rect}} =
\begin{bmatrix}
X \\
Y \\
Z
\end{bmatrix}
```

즉, SGBM disparity map에서 semi-dense depth cloud를 생성한다.

## 3. Gravity-Aligned Frame / 중력 정렬 좌표계

Stereo depth point는 먼저 camera frame에 있다. Floor-wall classification은 camera axis가 아니라 gravity axis 기준으로 해야 한다.

```math
\mathbf{p}_g =
R_{GI} R_{IC} R_{CR}
\mathbf{p}_{\mathrm{rect}}
```

Transformation chain:

```text
rectified camera
  -> original cam0
  -> IMU frame
  -> gravity-aligned frame
```

Gravity axis:

```math
\mathbf{g} =
\begin{bmatrix}
0 \\
0 \\
1
\end{bmatrix}
```

IMU/EKF orientation provides the vertical reference.  
IMU/EKF 자세가 floor-wall 판정의 기준축을 제공한다.

## 4. Plane Model / 평면 모델

Gravity-aligned point cloud에서 plane을 찾는다.

```math
\pi: \mathbf{n}^{T}\mathbf{p}_g + b = 0,
\qquad
\|\mathbf{n}\| = 1
```

Point-to-plane distance:

```math
d(\mathbf{p}_i, \pi)
=
\left|\mathbf{n}^{T}\mathbf{p}_i + b\right|
```

## 5. RANSAC Plane Fitting / RANSAC 평면 추정

RANSAC은 inlier support가 가장 큰 plane을 선택한다.

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
여기서 `tau`는 plane inlier 거리 threshold다.

## 6. Floor / Wall Decision / 바닥·벽 판정

Floor normal is parallel to gravity:

```math
\mathrm{floor}
\quad \text{if} \quad
\left|\mathbf{n}_f^{T}\mathbf{g}\right| > \cos\theta_f
```

and the point is close to the floor plane:

```math
\left|\mathbf{n}_f^{T}\mathbf{p}_i + b_f\right| < \tau_f
```

Wall normal is perpendicular to gravity:

```math
\mathrm{wall}
\quad \text{if} \quad
\left|\mathbf{n}_w^{T}\mathbf{g}\right| < \sin\theta_w
```

and the point is close to the wall plane:

```math
\left|\mathbf{n}_w^{T}\mathbf{p}_i + b_w\right| < \tau_w
```

Core intuition:

```math
\mathrm{floor\ normal} \parallel \mathbf{g},
\qquad
\mathrm{wall\ normal} \perp \mathbf{g}
```

## 7. Final Label / 최종 라벨

```math
\operatorname{label}(\mathbf{p}_i)
=
\begin{cases}
\mathrm{floor}, & \mathbf{p}_i \in I_f \\
\mathrm{wall}, & \mathbf{p}_i \in I_w \\
\mathrm{other}, & \text{otherwise}
\end{cases}
```

## 8. Pipeline Summary / 파이프라인 요약

```math
\mathrm{rectified\ stereo}
\xrightarrow{\mathrm{SGBM}}
d(u,v)
\xrightarrow{Z=f_xB/d}
\mathbf{p}_{\mathrm{rect}}
\xrightarrow{R_{GI}R_{IC}R_{CR}}
\mathbf{p}_g
\xrightarrow{\mathrm{RANSAC}}
\{\mathrm{floor}, \mathrm{wall}, \mathrm{other}\}
```

## PPT Caption / 슬라이드 캡션

Rectified stereo에서 SGBM disparity로 semi-dense 3D point cloud를 만들고, IMU/EKF orientation으로 gravity-aligned frame에 정렬한 뒤, RANSAC plane normal과 gravity axis의 관계로 floor와 wall을 분류한다.

## Minimal PPT Blocks / 한 장에 넣을 핵심 블록

1. Stereo disparity to depth / disparity에서 depth 복원
2. Gravity alignment / 중력 정렬
3. RANSAC plane fitting / RANSAC 평면 추정
4. Floor-wall decision / 바닥·벽 판정
