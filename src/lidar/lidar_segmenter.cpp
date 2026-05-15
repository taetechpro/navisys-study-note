#include "lidar/lidar_segmenter.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <random>
#include <stdexcept>

#include <Eigen/Geometry>

namespace fs = std::filesystem;

namespace {

struct PlaneModel {
    Eigen::Vector3d n = Eigen::Vector3d::Zero();
    double d = 0.0;
    std::vector<std::size_t> inliers;
    bool valid = false;
};

Eigen::Vector3d to_vec(const LidarPoint& p) {
    return Eigen::Vector3d(p.x, p.y, p.z);
}

bool is_finite(const LidarPoint& p) {
    return std::isfinite(p.x) && std::isfinite(p.y) &&
           std::isfinite(p.z) && std::isfinite(p.intensity);
}

bool in_roi(const LidarPoint& p, const LidarSegmentationOptions& opt) {
    return p.x >= opt.min_x && p.x <= opt.max_x &&
           p.y >= opt.min_y && p.y <= opt.max_y &&
           p.z >= opt.min_z && p.z <= opt.max_z;
}

bool make_plane(const LidarPoint& a,
                const LidarPoint& b,
                const LidarPoint& c,
                PlaneModel& plane) {
    const Eigen::Vector3d pa = to_vec(a);
    const Eigen::Vector3d pb = to_vec(b);
    const Eigen::Vector3d pc = to_vec(c);
    Eigen::Vector3d n = (pb - pa).cross(pc - pa);
    const double norm = n.norm();
    if (norm < 1e-6) {
        return false;
    }

    n /= norm;
    double d = -n.dot(pa);
    if (n.z() < 0.0) {
        n = -n;
        d = -d;
    }

    plane.n = n;
    plane.d = d;
    plane.inliers.clear();
    plane.valid = true;
    return true;
}

double plane_distance(const PlaneModel& plane, const LidarPoint& p) {
    return std::abs(plane.n.dot(to_vec(p)) + plane.d);
}

PlaneModel fit_plane_ransac(
    const std::vector<LidarPoint>& points,
    const std::vector<std::size_t>& candidates,
    const std::function<bool(const PlaneModel&)>& accept_plane,
    double distance_threshold,
    int iterations,
    std::size_t min_inliers) {
    PlaneModel best;
    if (candidates.size() < 3 || iterations <= 0) {
        return best;
    }

    std::mt19937 rng(7);
    std::uniform_int_distribution<std::size_t> pick(0, candidates.size() - 1);

    for (int iter = 0; iter < iterations; ++iter) {
        const std::size_t ia = pick(rng);
        std::size_t ib = pick(rng);
        std::size_t ic = pick(rng);
        if (ia == ib || ia == ic || ib == ic) {
            continue;
        }

        PlaneModel plane;
        if (!make_plane(points[candidates[ia]],
                        points[candidates[ib]],
                        points[candidates[ic]],
                        plane)) {
            continue;
        }
        if (!accept_plane(plane)) {
            continue;
        }

        std::vector<std::size_t> inliers;
        inliers.reserve(candidates.size());
        for (const std::size_t idx : candidates) {
            if (plane_distance(plane, points[idx]) <= distance_threshold) {
                inliers.push_back(idx);
            }
        }

        if (inliers.size() > best.inliers.size()) {
            plane.inliers = std::move(inliers);
            best = std::move(plane);
        }
    }

    if (best.inliers.size() < min_inliers) {
        best.valid = false;
        best.inliers.clear();
    }
    return best;
}

template <typename Predicate>
std::vector<std::size_t> collect_candidates(const SegmentedCloud& cloud,
                                            Predicate predicate) {
    std::vector<std::size_t> candidates;
    candidates.reserve(cloud.points.size());
    for (std::size_t i = 0; i < cloud.points.size(); ++i) {
        if (predicate(cloud.points[i], cloud.labels[i])) {
            candidates.push_back(i);
        }
    }
    return candidates;
}

void apply_label(SegmentedCloud& cloud,
                 const PlaneModel& plane,
                 SegmentLabel label) {
    if (!plane.valid) {
        return;
    }
    for (const std::size_t idx : plane.inliers) {
        cloud.labels[idx] = label;
    }
}

bool has_wall_extent(const std::vector<LidarPoint>& points,
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
        min_x = std::min(min_x, static_cast<double>(p.x));
        max_x = std::max(max_x, static_cast<double>(p.x));
        min_y = std::min(min_y, static_cast<double>(p.y));
        max_y = std::max(max_y, static_cast<double>(p.y));
        min_z = std::min(min_z, static_cast<double>(p.z));
        max_z = std::max(max_z, static_cast<double>(p.z));
    }

    const double horizontal_extent =
        (label == SegmentLabel::FrontWall) ? (max_y - min_y) : (max_x - min_x);
    const double vertical_extent = max_z - min_z;
    return horizontal_extent >= opt.min_wall_extent_m &&
           vertical_extent >= opt.min_wall_height_m;
}

bool is_unlabeled_wall_candidate(const LidarPoint& p,
                                 SegmentLabel label,
                                 const LidarSegmentationOptions& opt) {
    return label == SegmentLabel::Other &&
           p.z >= opt.wall_min_z && p.z <= opt.wall_max_z;
}

} // namespace

LidarSegmenter::LidarSegmenter(LidarSegmentationOptions options)
    : options_(options) {}

SegmentedCloud LidarSegmenter::process(const std::vector<LidarPoint>& points) const {
    SegmentedCloud cloud;
    cloud.points.reserve(points.size());

    for (const auto& p : points) {
        if (is_finite(p) && in_roi(p, options_)) {
            cloud.points.push_back(p);
        }
    }
    cloud.labels.assign(cloud.points.size(), SegmentLabel::Other);

    auto floor_candidates = collect_candidates(
        cloud,
        [this](const LidarPoint& p, SegmentLabel label) {
            return label == SegmentLabel::Other &&
                   p.z >= options_.floor_search_min_z &&
                   p.z <= options_.floor_search_max_z;
        });

    const auto floor = fit_plane_ransac(
        cloud.points,
        floor_candidates,
        [this](const PlaneModel& plane) {
            if (std::abs(plane.n.z()) < options_.floor_normal_min_z) {
                return false;
            }
            const double z_at_origin = -plane.d / plane.n.z();
            return z_at_origin >= options_.floor_search_min_z &&
                   z_at_origin <= options_.floor_search_max_z;
        },
        options_.floor_distance_threshold,
        options_.ransac_iterations,
        options_.min_floor_inliers);
    apply_label(cloud, floor, SegmentLabel::Floor);

    auto fit_wall = [this, &cloud](SegmentLabel label,
                                   const std::vector<std::size_t>& candidates,
                                   const std::function<bool(const PlaneModel&)>& accept_plane) {
        PlaneModel wall = fit_plane_ransac(
            cloud.points,
            candidates,
            accept_plane,
            options_.wall_distance_threshold,
            options_.ransac_iterations,
            options_.min_wall_inliers);
        if (!wall.valid || !has_wall_extent(cloud.points, wall.inliers, label, options_)) {
            return;
        }
        apply_label(cloud, wall, label);
    };

    const auto left_candidates = collect_candidates(
        cloud,
        [this](const LidarPoint& p, SegmentLabel label) {
            return is_unlabeled_wall_candidate(p, label, options_) &&
                   p.y >= options_.side_min_abs_y;
        });
    fit_wall(
        SegmentLabel::LeftWall,
        left_candidates,
        [this](const PlaneModel& plane) {
            return std::abs(plane.n.z()) <= options_.wall_normal_max_abs_z &&
                   std::abs(plane.n.y()) >= options_.wall_axis_min_abs;
        });

    const auto right_candidates = collect_candidates(
        cloud,
        [this](const LidarPoint& p, SegmentLabel label) {
            return is_unlabeled_wall_candidate(p, label, options_) &&
                   p.y <= -options_.side_min_abs_y;
        });
    fit_wall(
        SegmentLabel::RightWall,
        right_candidates,
        [this](const PlaneModel& plane) {
            return std::abs(plane.n.z()) <= options_.wall_normal_max_abs_z &&
                   std::abs(plane.n.y()) >= options_.wall_axis_min_abs;
        });

    const auto front_candidates = collect_candidates(
        cloud,
        [this](const LidarPoint& p, SegmentLabel label) {
            return is_unlabeled_wall_candidate(p, label, options_) &&
                   p.x >= options_.front_min_x &&
                   std::abs(p.y) <= options_.front_max_abs_y;
        });
    fit_wall(
        SegmentLabel::FrontWall,
        front_candidates,
        [this](const PlaneModel& plane) {
            return std::abs(plane.n.z()) <= options_.wall_normal_max_abs_z &&
                   std::abs(plane.n.x()) >= options_.wall_axis_min_abs;
        });

    return cloud;
}

SegmentCounts SegmentedCloud::counts() const {
    SegmentCounts result;
    for (const SegmentLabel label : labels) {
        switch (label) {
        case SegmentLabel::Floor:
            ++result.floor;
            break;
        case SegmentLabel::Wall:
            ++result.wall;
            break;
        case SegmentLabel::LeftWall:
            ++result.left_wall;
            break;
        case SegmentLabel::RightWall:
            ++result.right_wall;
            break;
        case SegmentLabel::FrontWall:
            ++result.front_wall;
            break;
        case SegmentLabel::Other:
        default:
            ++result.other;
            break;
        }
    }
    return result;
}

std::vector<LidarPoint> load_kitti_velodyne_bin(const std::string& path) {
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (!f.is_open()) {
        throw std::runtime_error("Cannot open KITTI Velodyne file: " + path);
    }

    const std::streamoff bytes = f.tellg();
    constexpr std::streamoff kPointBytes = 4 * sizeof(float);
    if (bytes < 0 || bytes % kPointBytes != 0) {
        throw std::runtime_error("Invalid KITTI Velodyne file size: " + path);
    }

    const std::size_t count = static_cast<std::size_t>(bytes / kPointBytes);
    std::vector<LidarPoint> points(count);

    f.seekg(0, std::ios::beg);
    for (auto& point : points) {
        std::array<float, 4> values{};
        f.read(reinterpret_cast<char*>(values.data()), kPointBytes);
        if (!f) {
            throw std::runtime_error("Failed while reading KITTI Velodyne file: " + path);
        }
        point.x = values[0];
        point.y = values[1];
        point.z = values[2];
        point.intensity = values[3];
    }
    return points;
}

void write_segmented_ply(const std::string& path, const SegmentedCloud& cloud) {
    const fs::path output_path(path);
    if (output_path.has_parent_path()) {
        fs::create_directories(output_path.parent_path());
    }

    std::ofstream f(output_path);
    if (!f.is_open()) {
        throw std::runtime_error("Cannot write segmented PLY: " + output_path.string());
    }

    f << "ply\n";
    f << "format ascii 1.0\n";
    f << "element vertex " << cloud.points.size() << "\n";
    f << "property float x\n";
    f << "property float y\n";
    f << "property float z\n";
    f << "property uchar red\n";
    f << "property uchar green\n";
    f << "property uchar blue\n";
    f << "property uchar label\n";
    f << "end_header\n";

    for (std::size_t i = 0; i < cloud.points.size(); ++i) {
        const auto& p = cloud.points[i];
        const auto color = segment_color_rgb(cloud.labels[i]);
        f << p.x << " " << p.y << " " << p.z << " "
          << color.x() << " " << color.y() << " " << color.z() << " "
          << static_cast<int>(cloud.labels[i]) << "\n";
    }
}

const char* segment_label_name(SegmentLabel label) {
    switch (label) {
    case SegmentLabel::Floor:
        return "floor";
    case SegmentLabel::Wall:
        return "wall";
    case SegmentLabel::LeftWall:
        return "left_wall";
    case SegmentLabel::RightWall:
        return "right_wall";
    case SegmentLabel::FrontWall:
        return "front_wall";
    case SegmentLabel::Other:
    default:
        return "other";
    }
}

Eigen::Vector3i segment_color_rgb(SegmentLabel label) {
    switch (label) {
    case SegmentLabel::Floor:
        return Eigen::Vector3i(70, 180, 90);
    case SegmentLabel::Wall:
        return Eigen::Vector3i(60, 150, 255);
    case SegmentLabel::LeftWall:
        return Eigen::Vector3i(60, 150, 255);
    case SegmentLabel::RightWall:
        return Eigen::Vector3i(255, 160, 60);
    case SegmentLabel::FrontWall:
        return Eigen::Vector3i(230, 80, 160);
    case SegmentLabel::Other:
    default:
        return Eigen::Vector3i(150, 150, 150);
    }
}
