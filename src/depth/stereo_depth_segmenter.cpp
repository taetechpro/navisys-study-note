#include "depth/stereo_depth_segmenter.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <random>
#include <stdexcept>
#include <vector>

#include <Eigen/Geometry>
#include <opencv2/calib3d.hpp>

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

bool in_roi(const LidarPoint& p, const StereoDepthSegmentationOptions& opt) {
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

    std::mt19937 rng(11);
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
                     const StereoDepthSegmentationOptions& opt) {
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

    const double horizontal_extent = std::max(max_x - min_x, max_y - min_y);
    const double vertical_extent = max_z - min_z;
    return horizontal_extent >= opt.min_wall_extent_m &&
           vertical_extent >= opt.min_wall_height_m;
}

int normalize_num_disparities(int value) {
    if (value <= 0) {
        return 16;
    }
    return ((value + 15) / 16) * 16;
}

int normalize_block_size(int value) {
    int result = std::max(3, value);
    if (result % 2 == 0) {
        ++result;
    }
    return result;
}

cv::Mat compute_disparity_sgbm(const cv::Mat& left,
                               const cv::Mat& right,
                               const StereoDepthSegmentationOptions& opt) {
    const int block_size = normalize_block_size(opt.sgbm_block_size);
    const int num_disparities = normalize_num_disparities(opt.sgbm_num_disparities);

    auto sgbm = cv::StereoSGBM::create(
        opt.sgbm_min_disparity,
        num_disparities,
        block_size);
    const int channels = 1;
    sgbm->setP1(8 * channels * block_size * block_size);
    sgbm->setP2(32 * channels * block_size * block_size);
    sgbm->setMode(cv::StereoSGBM::MODE_SGBM_3WAY);
    sgbm->setUniquenessRatio(opt.sgbm_uniqueness_ratio);
    sgbm->setSpeckleWindowSize(opt.sgbm_speckle_window_size);
    sgbm->setSpeckleRange(opt.sgbm_speckle_range);
    sgbm->setDisp12MaxDiff(1);

    cv::Mat disparity16;
    sgbm->compute(left, right, disparity16);
    return disparity16;
}

SegmentedCloud make_gravity_aligned_cloud(
    const cv::Mat& rectified_left_gray,
    const cv::Mat& disparity16,
    const StereoDepthGeometry& geometry,
    const Eigen::Matrix3d& R_world_imu,
    const StereoDepthSegmentationOptions& opt) {
    SegmentedCloud cloud;
    cloud.points.reserve(
        static_cast<std::size_t>(rectified_left_gray.rows / std::max(1, opt.image_stride)) *
        static_cast<std::size_t>(rectified_left_gray.cols / std::max(1, opt.image_stride)));

    const int stride = std::max(1, opt.image_stride);
    const double fx = geometry.fx;
    const double fy = geometry.fy;
    const double cx = geometry.cx;
    const double cy = geometry.cy;
    const double baseline = std::abs(geometry.baseline_m);
    if (fx <= 0.0 || fy <= 0.0 || baseline <= 0.0) {
        return cloud;
    }

    const Eigen::Matrix3d R_cam0_imu =
        geometry.T_cam0_imu.block<3, 3>(0, 0);
    const Eigen::Vector3d t_cam0_imu =
        geometry.T_cam0_imu.block<3, 1>(0, 3);

    for (int v = 0; v < disparity16.rows; v += stride) {
        for (int u = 0; u < disparity16.cols; u += stride) {
            const float disp = static_cast<float>(disparity16.at<short>(v, u)) / 16.0f;
            if (!std::isfinite(disp) || disp <= 0.5f) {
                continue;
            }

            const double z = fx * baseline / static_cast<double>(disp);
            if (z < opt.min_depth_m || z > opt.max_depth_m) {
                continue;
            }

            const Eigen::Vector3d p_rect(
                (static_cast<double>(u) - cx) * z / fx,
                (static_cast<double>(v) - cy) * z / fy,
                z);
            const Eigen::Vector3d p_cam0 = geometry.R_cam0_rect * p_rect;
            const Eigen::Vector3d p_imu = R_cam0_imu.transpose() * (p_cam0 - t_cam0_imu);
            const Eigen::Vector3d p_gravity = R_world_imu * p_imu;

            LidarPoint point;
            point.x = static_cast<float>(p_gravity.x());
            point.y = static_cast<float>(p_gravity.y());
            point.z = static_cast<float>(p_gravity.z());
            point.intensity = static_cast<float>(rectified_left_gray.at<uchar>(v, u)) / 255.0f;

            if (is_finite(point) && in_roi(point, opt)) {
                cloud.points.push_back(point);
            }
        }
    }

    cloud.labels.assign(cloud.points.size(), SegmentLabel::Other);
    return cloud;
}

SegmentedCloud segment_planes(SegmentedCloud cloud,
                              const StereoDepthSegmentationOptions& opt) {
    const Eigen::Vector3d gravity_axis = Eigen::Vector3d::UnitZ();

    auto floor_candidates = collect_candidates(
        cloud,
        [&opt](const LidarPoint& p, SegmentLabel label) {
            return label == SegmentLabel::Other &&
                   p.z >= opt.floor_search_min_z &&
                   p.z <= opt.floor_search_max_z;
        });

    const auto floor = fit_plane_ransac(
        cloud.points,
        floor_candidates,
        [&opt, &gravity_axis](const PlaneModel& plane) {
            if (std::abs(plane.n.dot(gravity_axis)) <
                opt.floor_normal_min_abs_dot_gravity) {
                return false;
            }
            if (std::abs(plane.n.z()) < 1e-6) {
                return false;
            }
            const double z_at_origin = -plane.d / plane.n.z();
            return z_at_origin >= opt.floor_search_min_z &&
                   z_at_origin <= opt.floor_search_max_z;
        },
        opt.floor_distance_threshold,
        opt.ransac_iterations,
        opt.min_floor_inliers);
    apply_label(cloud, floor, SegmentLabel::Floor);

    auto wall_candidates = collect_candidates(
        cloud,
        [&opt](const LidarPoint& p, SegmentLabel label) {
            return label == SegmentLabel::Other &&
                   p.z >= opt.wall_min_z &&
                   p.z <= opt.wall_max_z;
        });

    const auto wall = fit_plane_ransac(
        cloud.points,
        wall_candidates,
        [&opt, &gravity_axis](const PlaneModel& plane) {
            return std::abs(plane.n.dot(gravity_axis)) <=
                   opt.wall_normal_max_abs_dot_gravity;
        },
        opt.wall_distance_threshold,
        opt.ransac_iterations,
        opt.min_wall_inliers);
    if (wall.valid && has_wall_extent(cloud.points, wall.inliers, opt)) {
        apply_label(cloud, wall, SegmentLabel::Wall);
    }

    return cloud;
}

} // namespace

StereoDepthSegmenter::StereoDepthSegmenter(StereoDepthSegmentationOptions options)
    : options_(options) {}

SegmentedCloud StereoDepthSegmenter::process(
    const cv::Mat& rectified_left_gray,
    const cv::Mat& rectified_right_gray,
    const StereoDepthGeometry& geometry,
    const Eigen::Matrix3d& R_world_imu) const {
    if (rectified_left_gray.empty() || rectified_right_gray.empty()) {
        return {};
    }
    if (rectified_left_gray.size() != rectified_right_gray.size()) {
        throw std::runtime_error("Stereo depth segmentation requires equal-size rectified images");
    }
    if (rectified_left_gray.type() != CV_8UC1 || rectified_right_gray.type() != CV_8UC1) {
        throw std::runtime_error("Stereo depth segmentation expects CV_8UC1 rectified images");
    }

    const cv::Mat disparity16 =
        compute_disparity_sgbm(rectified_left_gray, rectified_right_gray, options_);
    auto cloud = make_gravity_aligned_cloud(
        rectified_left_gray,
        disparity16,
        geometry,
        R_world_imu,
        options_);
    return segment_planes(std::move(cloud), options_);
}
