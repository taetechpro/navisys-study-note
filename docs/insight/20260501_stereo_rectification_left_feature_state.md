# Stereo Rectification과 Left 기준 Feature State — 2026-05-01

## 1. 결론

현재 VIO 구현은 **left/right 이미지를 모두 rectification**한 뒤, VO의 추적 상태는
**left image feature 기준**으로만 유지한다.

즉 right image를 안 쓰는 것이 아니다. right image는 stereo depth를 만들 때 쓰이고,
시간축 추적과 PnP는 left image 좌표를 기준으로 진행된다.

핵심 구조:

```text
raw left/right
  ↓
stereo rectification
  ↓
gray_l, gray_r
  ↓
left-right stereo matching
  ↓
left feature 좌표 + triangulated 3D point 생성
  ↓
다음 프레임부터 left image에서 temporal KLT tracking
  ↓
left 2D feature + 3D map point로 PnP
  ↓
camera pose → EKF VO measurement
```

---

## 2. 코드상 상태 정의

`StereoTracker`의 핵심 상태는 다음 두 벡터다.

```cpp
std::vector<cv::Point2f> prev_pts_l_;         // left image feature 좌표
std::vector<Eigen::Vector3d> map_pts_world_;  // 대응되는 3D map point
```

불변식:

```text
prev_pts_l_[i] ↔ map_pts_world_[i]
```

즉 각 left image feature 하나가 world 3D point 하나와 1:1 대응한다.

right feature 좌표는 장기 상태로 저장하지 않는다. right image는 매번 stereo matching /
triangulation 단계에서만 사용된다.

---

## 3. Rectification은 left/right 둘 다 수행

`StereoTracker::process()` 초반에서 raw BGR image를 grayscale로 바꾸고, 좌우 모두
rectification한다.

```cpp
cv::Mat raw_l, raw_r;
cv::cvtColor(img_l, raw_l, cv::COLOR_BGR2GRAY);
cv::cvtColor(img_r, raw_r, cv::COLOR_BGR2GRAY);

cv::Mat gray_l, gray_r;
cv::remap(raw_l, gray_l, map0x_, map0y_, cv::INTER_LINEAR);
cv::remap(raw_r, gray_r, map1x_, map1y_, cv::INTER_LINEAR);
```

여기서:

| 변수 | 의미 |
|---|---|
| `gray_l` | rectified left grayscale image |
| `gray_r` | rectified right grayscale image |
| `map0x_/map0y_` | left camera undistort + rectify map |
| `map1x_/map1y_` | right camera undistort + rectify map |

Rectification 이후에는 epipolar line이 수평 scanline이 된다. 따라서 stereo matching은
주로 같은 y 근처에서 대응점을 찾으면 된다.

---

## 4. Bootstrap: left-right matching으로 3D map 생성

첫 프레임에서는 아직 temporal tracking할 이전 image가 없다. 그래서 left/right stereo
matching으로 초기 3D map을 만든다.

```cpp
auto match = orb_stereo_match(gray_l, gray_r, {}, target_features_);
auto pts3d = stereo_triangulate(match.pts_l, match.pts_r);
```

이때 `match.pts_l`과 `match.pts_r`는 rectified stereo pair에서 얻은 대응점이다.

그 다음에는 depth가 정상인 point만 남긴다.

```cpp
for (size_t i = 0; i < pts3d.size(); ++i) {
    double z = pts3d[i][2];
    if (z > 0.1 && z < 50.0) {
        prev_pts_l_.push_back(kept_l[i]);
        map_pts_world_.push_back(pts3d[i]); // world = cam0_0 frame
    }
}
```

중요한 점:

- `pts3d`를 만들 때는 left/right 둘 다 사용한다.
- 하지만 state에 저장하는 2D 좌표는 `kept_l[i]`, 즉 left feature다.
- 첫 프레임에서는 world frame을 cam0 첫 프레임 기준으로 둔다.

---

## 5. 일반 프레임: temporal tracking은 left image만 사용

두 번째 프레임부터는 이전 left image의 feature를 현재 left image로 KLT tracking한다.

```cpp
auto curr_pts = klt_track(prev_img_l_, gray_l, prev_pts_l_, temp_ok);
```

이 결과로 현재 left image 좌표 `curr_pts`가 얻어진다.

그 뒤 valid tracking만 남긴다.

```cpp
std::vector<cv::Point2f> tracked_pts;
std::vector<Vector3d> tracked_map;

for (size_t i = 0; i < temp_ok.size(); ++i) {
    if (temp_ok[i]) {
        tracked_pts.push_back(curr_pts[i]);
        tracked_map.push_back(map_pts_world_[i]);
    }
}
```

여기서도 불변식은 유지된다.

```text
tracked_pts[i] ↔ tracked_map[i]
```

---

## 6. PnP는 left 2D + world 3D 대응으로 수행

현재 카메라 pose는 PnP RANSAC으로 계산한다.

```cpp
if (solve_pnp(tracked_map, tracked_pts, R_cw, t_cw, inliers)) {
    pose.R = R_cw.transpose();
    pose.t = -R_cw.transpose() * t_cw;
    pose.valid = true;
}
```

PnP 입력:

| 입력 | 의미 |
|---|---|
| `tracked_map` | world 3D points |
| `tracked_pts` | current rectified left image 2D points |

즉 PnP는 right image를 직접 쓰지 않는다. right image는 이미 3D point의 depth를 만들 때
기여했다.

---

## 7. 새 feature가 부족할 때 right image가 다시 쓰임

기존 tracked feature가 줄어들면 새 feature를 추가한다.

```cpp
int need = target_features_ - static_cast<int>(tracked_pts.size());
if (need > 0) {
    auto match = orb_stereo_match(gray_l, gray_r, tracked_pts, need);
    if (!match.pts_l.empty()) {
        auto pts3d_cam = stereo_triangulate(match.pts_l, match.pts_r);
        ...
    }
}
```

여기서 다시 right image가 사용된다.

역할:

1. left/right ORB matching
2. stereo triangulation
3. 새 3D point 생성
4. 새 left feature 좌표를 tracking state에 추가

즉 right image는 **새 landmark의 depth를 초기화하는 용도**다.

---

## 8. 왜 right feature를 계속 상태로 들고 가지 않는가

현재 구현은 단순한 sparse stereo VO 구조다.

상태를 left 기준으로만 유지하면:

- temporal tracking이 단순해진다.
- PnP 입력이 `left 2D ↔ world 3D`로 깔끔해진다.
- right tracking 실패가 pose estimation을 직접 흔들지 않는다.
- 구현 난이도가 낮다.

반면 한계도 있다.

- 기존 feature의 depth를 매 프레임 stereo로 재검증하지 않는다.
- 오래된 3D point가 stale해질 수 있다.
- stereo reprojection residual을 직접 최적화하지 않는다.
- right image의 정보가 pose update에 직접 들어가는 tight 구조는 아니다.

코드에도 이 선택이 주석으로 남아 있다.

```cpp
// Re-triangulating tracked features via KLT stereo fails on repeated textures.
// Keep 3D positions from the original bootstrap / step-4 addition; let PnP RANSAC
// weed out stale points.
```

즉 이 코드는 right image를 매 프레임 억지로 KLT 추적해 재삼각측량하려다 생기는 반복
텍스처 문제를 피하고, stale correspondence는 PnP RANSAC이 제거하도록 둔다.

---

## 9. 이 구조는 Loosely-Coupled VIO에 맞다

현재 전체 시스템은 LC-EKF다.

```text
StereoTracker → VO pose / camera position
IMU           → propagation
LC-EKF        → VO position measurement로 update
```

EKF는 raw feature residual을 직접 보지 않는다. EKF가 받는 것은 StereoTracker가 만든
카메라 위치 측정값이다.

따라서 StereoTracker 내부가:

```text
left 기준 temporal tracking
right 기준 depth initialization
PnP로 camera pose 계산
```

형태여도 LC-EKF 입장에서는 문제가 없다.

---

## 10. 더 타이트하게 만들려면

더 고급 구조로 가려면 다음 중 하나가 필요하다.

### A. right feature도 상태로 저장

```cpp
std::vector<cv::Point2f> prev_pts_l_;
std::vector<cv::Point2f> prev_pts_r_;
std::vector<Eigen::Vector3d> map_pts_world_;
```

장점:

- left/right feature visualization 가능
- 매 프레임 stereo consistency 확인 가능

단점:

- left/right/3D correspondence 동기화가 더 어려워진다.
- right tracking failure 처리 로직이 추가된다.

### B. 매 프레임 stereo 재삼각측량

기존 feature를 현재 left/right에서 다시 matching해서 depth를 갱신한다.

장점:

- landmark depth가 최신화된다.

단점:

- 반복 텍스처에서 잘못된 right match가 들어오기 쉽다.
- depth noise가 pose estimation을 흔들 수 있다.

### C. Tight MSCKF / BA 구조

feature pixel residual을 직접 필터/최적화에 넣는다.

```text
residual = observed_pixel - projected_pixel(state, landmark)
```

장점:

- 이론적으로 더 정확하다.
- feature-level uncertainty를 직접 다룰 수 있다.

단점:

- Jacobian, null-space projection, sliding window 관리가 필요하다.
- 현재 LC-EKF보다 구현 난이도가 크게 올라간다.

---

## 11. Rerun에서 right image도 보고 싶을 때

현재 Rerun에는 `camera/left_rectified`만 기록한다. right image도 보기만 하려면
`StereoTracker`가 rectified right image를 저장하고 accessor로 노출하면 된다.

필요한 변경:

```cpp
cv::Mat prev_img_r_;
const cv::Mat& rectified_right_image() const { return prev_img_r_; }
```

`process()`에서:

```cpp
prev_img_l_ = gray_l.clone();
prev_img_r_ = gray_r.clone();
```

Rerun logging에서:

```cpp
rec_->log("camera/right_rectified",
          rerun::Image::from_grayscale8(...));
```

단, right image 위에 feature point까지 찍고 싶다면 별도 right feature state가 필요하다.
현재 `tracked_points()`는 left image 좌표만 반환한다.

---

## 12. 한 줄 요약

이 구현은 **stereo로 depth를 만들고, monocular-left temporal tracking으로 pose를
이어가는 sparse stereo VO**다. right image는 depth initialization에 쓰이고, left image는
tracking/PnP의 기준 image로 쓰인다.

