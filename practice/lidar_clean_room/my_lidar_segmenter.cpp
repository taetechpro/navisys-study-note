#include "my_lidar_segmenter.hpp"

#include<array>
#include<fstream>
#include<stdexcept>
#include<cmath>
#include<random>
#include<limits>

std::vector<LidarPoint> load_kitti_velodyne_bin(const std::string& path) {
    // B-2: 파일 열기 + 예외
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (!f.is_open()) {
        throw std::runtime_error("Cannot open KITTI Velodyne file: " + path);
    }

    // B-3: 파일 크기 측정 + 점 개수 계산
    const std::streamsize file_size = f.tellg();
    f.seekg(0, std::ios::beg);

    constexpr std::size_t kBytesPerPoint = sizeof(LidarPoint);  // = 16
    if (file_size < 0 || static_cast<std::size_t>(file_size) % kBytesPerPoint != 0) {
        throw std::runtime_error("KITTI .bin size is not a multiple of 16 bytes: " + path);
    }
    const std::size_t num_points = static_cast<std::size_t>(file_size) / kBytesPerPoint;



    // TODO B-4: 점별 읽기 루프
    std::vector<LidarPoint> points;
    points.reserve(num_points);

    for (std::size_t i = 0; i < num_points; ++i) {
        LidarPoint p;
        f.read(reinterpret_cast<char*>(&p), kBytesPerPoint);
        if (!f) {
            throw std::runtime_error(
                "KITTI .bin read failed at point #" + std::to_string(i) + ": " + path);
        }
        points.push_back(p);
    }
    return points; // TODO B-5: return
}

std::vector<LidarPoint> filter_roi(const std::vector<LidarPoint>& points,
                                    const LidarSegmentationOptions& options) {
    std::vector<LidarPoint> filtered;
    filtered.reserve(points.size());

    // TODO C-2: ROI 박스 6 면 검사
        for (const auto& p : points) {
            if (p.x < options.min_x || p.x > options.max_x ||
                p.y < options.min_y || p.y > options.max_y ||
                p.z < options.min_z || p.z > options.max_z) {
                continue; // ROI 밖
            }
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
                continue; // 유한하지 않은 점
            }
            filtered.push_back(p);
        }// TODO C-3: finite 검사 + push_back + return
    return filtered;
}

double point_to_plane_distance(const Plane& plane, const LidarPoint& p) {
    const double signed_dist =
        plane.a * static_cast<double>(p.x) +
        plane.b * static_cast<double>(p.y) +
        plane.c * static_cast<double>(p.z) +
        plane.d;
    return std::abs(signed_dist);
}

bool fit_plane_3pt(const LidarPoint& p1,
                    const LidarPoint& p2,
                    const LidarPoint& p3,
                    Plane& out_plane) {
    // ── 1. 평면 위의 두 방향 벡터 (double 로 계산) ──
    const double v1x = static_cast<double>(p2.x) - static_cast<double>(p1.x);
    const double v1y = static_cast<double>(p2.y) - static_cast<double>(p1.y);
    const double v1z = static_cast<double>(p2.z) - static_cast<double>(p1.z);

    const double v2x = static_cast<double>(p3.x) - static_cast<double>(p1.x);
    const double v2y = static_cast<double>(p3.y) - static_cast<double>(p1.y);
    const double v2z = static_cast<double>(p3.z) - static_cast<double>(p1.z);

    // ── 2. 외적 n = v1 × v2  (법선 방향, 아직 정규화 전) ──
    const double nx = v1y * v2z - v1z * v2y;
    const double ny = v1z * v2x - v1x * v2z;
    const double nz = v1x * v2y - v1y * v2x;

    // ── 3. 길이 계산 + degenerate 가드 (세 점이 일직선) ──
    const double length = std::sqrt(nx * nx + ny * ny + nz * nz);
    constexpr double kMinNormalLength = 1e-6;
    if (length < kMinNormalLength) {
        return false; // 세 점이 거의 일직선 → 평면 못 만듦
    }

    // ── 4. 단위벡터로 정규화 ──
    const double inv_length = 1.0 / length;
    const double a = nx * inv_length;
    const double b = ny * inv_length;
    const double c = nz * inv_length;

    // ── 5. d = -(a·p1.x + b·p1.y + c·p1.z) ──
    const double d = -(a * static_cast<double>(p1.x) +
                        b * static_cast<double>(p1.y) +
                        c * static_cast<double>(p1.z));

    // ── 6. 출력 ──
    out_plane.a = a;
    out_plane.b = b;
    out_plane.c = c;
    out_plane.d = d;
    return true;
}

std::size_t count_inliers(const Plane& plane,
                        const std::vector<LidarPoint>& points,
                        double threshold) {
    std::size_t count = 0;

    for (const auto& p : points) {
        // TODO D-3a: p 와 plane 의 거리가 threshold 이하이면 count 증가
        //   힌트 ① 거리 함수:  point_to_plane_distance(plane, p)
        //   힌트 ② 비교 연산자:  <=
        //   힌트 ③ 카운터 증가:  ++count;
        //   → if 조건문 한 개 (2~3 줄) 가 들어가면 됨
        if (point_to_plane_distance(plane, p) <= threshold) {
            ++count;
        }
    }

    return count;
}

bool ransac_plane(const std::vector<LidarPoint>& points,
                int iterations,
                double distance_threshold,
                Plane& out_plane,
                std::size_t& out_inlier_count) {
    // ── 0. 사전 가드: 점이 3개 미만이거나 iter ≤ 0 이면 평면 못 만듦 ──
    if (points.size() < 3 || iterations <= 0) {
        return false;
    }

    // ── 1. 난수 생성기 + 분포 ──
    std::mt19937 rng(7);  // seed 고정 → 같은 입력 = 같은 결과 (디버깅 친화)
    std::uniform_int_distribution<std::size_t> pick(0, points.size() - 1);

    // ── 2. 베스트 추적 변수 ──
    Plane best_plane;
    std::size_t best_count = 0;
    bool found = false;

    // ── 3. RANSAC 메인 루프 ──
    for (int iter = 0; iter < iterations; ++iter) {
        // 3.1 무작위 3 점 인덱스 (서로 다른지 확인)
        const std::size_t i1 = pick(rng);
        const std::size_t i2 = pick(rng);
        const std::size_t i3 = pick(rng);
        if (i1 == i2 || i1 == i3 || i2 == i3) {
            continue; // 중복 인덱스 → 평면 못 만듦 → 다음 시도
        }

        // 3.2 후보 평면 fit (세 점이 일직선이면 skip)
        Plane candidate;
        if (!fit_plane_3pt(points[i1], points[i2], points[i3], candidate)) {
            continue;
        }

        // 3.3 후보 평면의 inlier 개수
        const std::size_t count = count_inliers(candidate, points, distance_threshold);

        // 3.4 베스트 갱신
        if (count > best_count) {
            best_plane = candidate;
            best_count = count;
            found = true;
        }
    }

    // ── 4. 출력 ──
    if (!found) {
        return false;
    }
    out_plane = best_plane;
    out_inlier_count = best_count;
    return true;
}

bool segment_floor(const std::vector<LidarPoint>& points,
                    const LidarSegmentationOptions& opt,
                    Plane& out_plane,
                    std::vector<std::size_t>& out_inlier_indices) {
    // ── 1. z 범위 사전 필터: floor 후보만 추출 ──
    std::vector<LidarPoint> search_points;
    search_points.reserve(points.size());
    for (const auto& p : points) {
        if (p.z >= opt.floor_search_min_z && p.z <= opt.floor_search_max_z) {
            search_points.push_back(p);
        }
    }
    if (search_points.size() < 3) {
        return false; // 후보 점 부족
    }

    // ── 2. RANSAC 호출 ──
    Plane plane;
    std::size_t inlier_count_in_search = 0;
    if (!ransac_plane(search_points,
                    opt.ransac_iterations,
                    opt.floor_distance_threshold,
                    plane,
                    inlier_count_in_search)) {
        return false;
    }

    // ── 3. 법선이 위쪽 (+z) 향하도록 보정 ──
    if (plane.c < 0.0) {
        plane.a = -plane.a;
        plane.b = -plane.b;
        plane.c = -plane.c;
        plane.d = -plane.d;
    }

    // ── 4. 평면 수평성 검증: |c| ≥ floor_normal_min_z ──
    if (plane.c < opt.floor_normal_min_z) {
        return false;
    }

    // ── 5. 원본 모든 점에서 floor inlier 식별 ──
    out_inlier_indices.clear();
    out_inlier_indices.reserve(points.size());
    for (std::size_t i = 0; i < points.size(); ++i) {
        if (point_to_plane_distance(plane, points[i]) <= opt.floor_distance_threshold) {
            out_inlier_indices.push_back(i);
        }
    }

    // ── 6. 최소 inlier 수 검증 ──
    if (out_inlier_indices.size() < opt.min_floor_inliers) {
        return false;
    }

    // ── 7. 출력 ──
    out_plane = plane;
    return true;
}

// cpp 내부 헬퍼: wall extent + height 검증
static bool check_wall_extent(const std::vector<LidarPoint>& points,
                              const std::vector<std::size_t>& inliers,
                              SegmentLabel label,
                              const LidarSegmentationOptions& opt) {
    if (inliers.empty()) {
        return false;
    }

    double min_x = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    double min_y = std::numeric_limits<double>::max();
    double max_y = std::numeric_limits<double>::lowest();
    double min_z = std::numeric_limits<double>::max();
    double max_z = std::numeric_limits<double>::lowest();

    for (const std::size_t idx : inliers) {
        const auto& p = points[idx];
        if (p.x < min_x) min_x = p.x;
        if (p.x > max_x) max_x = p.x;
        if (p.y < min_y) min_y = p.y;
        if (p.y > max_y) max_y = p.y;
        if (p.z < min_z) min_z = p.z;
        if (p.z > max_z) max_z = p.z;
    }

    const double horizontal_extent =
        (label == SegmentLabel::FrontWall) ? (max_y - min_y) : (max_x - min_x);
    const double vertical_extent = max_z - min_z;

    return horizontal_extent >= opt.min_wall_extent_m &&
           vertical_extent >= opt.min_wall_height_m;
}

bool segment_wall(const std::vector<LidarPoint>& points,
                  const std::vector<bool>& used_mask,
                  const LidarSegmentationOptions& opt,
                  SegmentLabel target_wall,
                  Plane& out_plane,
                  std::vector<std::size_t>& out_inlier_indices) {
    // ── 1. 후보 추출: 미라벨링 + z 범위 + wall 별 영역 ──
    std::vector<LidarPoint> search_points;
    search_points.reserve(points.size());

    for (std::size_t i = 0; i < points.size(); ++i) {
        if (used_mask[i]) {
            continue; // 이미 floor 또는 이전 wall 로 라벨됨
        }
        const auto& p = points[i];
        if (p.z < opt.wall_min_z || p.z > opt.wall_max_z) {
            continue;
        }

        bool in_region = false;
        switch (target_wall) {
        case SegmentLabel::LeftWall:
            in_region = (p.y >= opt.side_min_abs_y);
            break;
        case SegmentLabel::RightWall:
            in_region = (p.y <= -opt.side_min_abs_y);
            break;
        case SegmentLabel::FrontWall:
            in_region = (p.x >= opt.front_min_x &&
                         std::abs(p.y) <= opt.front_max_abs_y);
            break;
        default:
            return false; // 다른 라벨은 wall 아님
        }
        if (in_region) {
            search_points.push_back(p);
        }
    }
    if (search_points.size() < 3) {
        return false;
    }

    // ── 2. RANSAC ──
    Plane plane;
    std::size_t inlier_count_in_search = 0;
    if (!ransac_plane(search_points,
                      opt.ransac_iterations,
                      opt.wall_distance_threshold,
                      plane,
                      inlier_count_in_search)) {
        return false;
    }

    // ── 3. 법선 방향 검증: 수직성 + 축 방향 ──
    if (std::abs(plane.c) > opt.wall_normal_max_abs_z) {
        return false; // 너무 수평 (= 바닥/천장 같음)
    }
    if (target_wall == SegmentLabel::FrontWall) {
        if (std::abs(plane.a) < opt.wall_axis_min_abs) {
            return false; // 정면 벽은 x 축 법선 (a 가 커야)
        }
    } else { // LeftWall / RightWall
        if (std::abs(plane.b) < opt.wall_axis_min_abs) {
            return false; // 옆 벽은 y 축 법선 (b 가 커야)
        }
    }

    // ── 4. 원본 점에서 inlier 식별 (마스킹된 점은 제외) ──
    out_inlier_indices.clear();
    out_inlier_indices.reserve(points.size());
    for (std::size_t i = 0; i < points.size(); ++i) {
        if (used_mask[i]) {
            continue;
        }
        if (point_to_plane_distance(plane, points[i]) <= opt.wall_distance_threshold) {
            out_inlier_indices.push_back(i);
        }
    }

    // ── 5. 최소 inlier 검증 ──
    if (out_inlier_indices.size() < opt.min_wall_inliers) {
        return false;
    }

    // ── 6. extent + height 검증 ──
    if (!check_wall_extent(points, out_inlier_indices, target_wall, opt)) {
        return false;
    }

    // ── 7. 출력 ──
    out_plane = plane;
    return true;
}

void segment_all_walls(const std::vector<LidarPoint>& points,
                       const LidarSegmentationOptions& opt,
                       const std::vector<std::size_t>& floor_inliers,
                       std::vector<std::size_t>& out_left_inliers,
                       std::vector<std::size_t>& out_right_inliers,
                       std::vector<std::size_t>& out_front_inliers) {
    // 그리디 피링용 마스크: 한 번 라벨된 점은 다음 단계에서 제외
    std::vector<bool> used_mask(points.size(), false);
    for (const std::size_t idx : floor_inliers) {
        used_mask[idx] = true;
    }

    // ── Left wall ──
    Plane left_plane;
    out_left_inliers.clear();
    if (segment_wall(points, used_mask, opt, SegmentLabel::LeftWall,
                     left_plane, out_left_inliers)) {
        for (const std::size_t idx : out_left_inliers) {
            used_mask[idx] = true;
        }
    }

    // ── Right wall ──
    Plane right_plane;
    out_right_inliers.clear();
    if (segment_wall(points, used_mask, opt, SegmentLabel::RightWall,
                     right_plane, out_right_inliers)) {
        for (const std::size_t idx : out_right_inliers) {
            used_mask[idx] = true;
        }
    }

    // ── Front wall ──
    Plane front_plane;
    out_front_inliers.clear();
    if (segment_wall(points, used_mask, opt, SegmentLabel::FrontWall,
                     front_plane, out_front_inliers)) {
        for (const std::size_t idx : out_front_inliers) {
            used_mask[idx] = true;
        }
    }
}
  SegmentCounts SegmentedCloud::counts() const {
      SegmentCounts result;
      for (const SegmentLabel label : labels) {
          switch (label) {
          case SegmentLabel::Floor:     ++result.floor;      break;
          case SegmentLabel::LeftWall:  ++result.left_wall;  break;
          case SegmentLabel::RightWall: ++result.right_wall; break;
          case SegmentLabel::FrontWall: ++result.front_wall; break;
          case SegmentLabel::Wall:      ++result.wall;       break;
          case SegmentLabel::Other:
          default:                      ++result.other;      break;
          }
      }
      return result;
  }

void write_segmented_ply(const std::string& path, const SegmentedCloud& cloud) {
    if (cloud.points.size() != cloud.labels.size()) {
        throw std::runtime_error("SegmentedCloud size mismatch: points vs labels");
    }
    std::ofstream f(path);
    if (!f.is_open()) {
        throw std::runtime_error("Cannot open PLY file for writing: " + path);
    }

    const std::size_t N = cloud.points.size();

    // PLY 헤더
    f << "ply\n"
    << "format ascii 1.0\n"
    << "element vertex " << N << "\n"
    << "property float x\n"
    << "property float y\n"
    << "property float z\n"
    << "property uchar red\n"
    << "property uchar green\n"
    << "property uchar blue\n"
    << "end_header\n";

    // 본문: 점 + 라벨별 RGB
    for (std::size_t i = 0; i < N; ++i) {
        const auto& p = cloud.points[i];
        int r = 128, g = 128, b = 128;  // 기본: Other = 회색
        switch (cloud.labels[i]) {
        case SegmentLabel::Floor:     r =   0; g = 200; b =   0; break;
        case SegmentLabel::LeftWall:  r =   0; g =   0; b = 255; break;
        case SegmentLabel::RightWall: r = 255; g =   0; b =   0; break;
        case SegmentLabel::FrontWall: r = 255; g = 255; b =   0; break;
        case SegmentLabel::Wall:      r = 200; g =   0; b = 200; break;
        case SegmentLabel::Other:
        default:                      break;
        }
        f << p.x << ' ' << p.y << ' ' << p.z << ' '
        << r << ' ' << g << ' ' << b << '\n';
    }
}
