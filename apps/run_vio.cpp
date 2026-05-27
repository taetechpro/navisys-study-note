#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/imgcodecs.hpp>
#include <yaml-cpp/yaml.h>

#ifdef LC_VIO_WITH_RERUN
#include <rerun.hpp>
#endif

#include "depth/stereo_depth_segmenter.hpp"
#include "ekf/imu_propagator.hpp"
#include "ekf/lc_ekf.hpp"
#include "frontend/stereo_tracker.hpp"
#include "io/dataset_reader.hpp"
#include "io/euroc_reader.hpp"
#include "io/kitti_raw_reader.hpp"
#include "lidar/lidar_segmenter.hpp"
#include "msckf_pipeline/msckf_pipeline.hpp"

namespace fs = std::filesystem;

namespace {

struct InitParams {
    double static_window_sec = 1.0;
    double gravity_norm = 9.81;
};

struct RerunOptions {
    bool requested = false;
    bool spawn = false;
    bool connect = false;
    std::string save_path;
    int image_every = 10;
};

struct CliOptions {
    std::string config_path;
    RerunOptions rerun;
    bool segment_lidar = false;
    bool segment_depth = false;
    std::string depth_source = "stereo";
    int segment_every = 1;
    bool segment_every_set = false;
    std::string engine = "lc";  // "lc" (default, baseline) | "msckf" (TC port)
};

struct InitialState {
    double t0 = 0.0;
    Eigen::Vector3d p0 = Eigen::Vector3d::Zero();
    Eigen::Vector3d v0 = Eigen::Vector3d::Zero();
    Eigen::Matrix3d R0 = Eigen::Matrix3d::Identity();
    Eigen::Vector3d bg0 = Eigen::Vector3d::Zero();
    Eigen::Vector3d ba0 = Eigen::Vector3d::Zero();
    Eigen::Vector3d g_world = Eigen::Vector3d(0.0, 0.0, -9.81);
};

void print_usage(const char* exe) {
    std::cerr
        << "Usage: " << exe << " <config.yaml> [options]\n"
        << "Options:\n"
        << "  --rerun-save <file.rrd>       Save a Rerun recording to disk\n"
        << "  --rerun-spawn                 Spawn a Rerun viewer from this process\n"
        << "  --rerun-connect               Connect to an already running Rerun viewer\n"
        << "  --rerun-image-every <N>       Log rectified camera image every N frames (default: 10, 0=off)\n"
        << "  --segment-lidar               Segment KITTI Velodyne frames into floor/wall classes\n"
        << "  --segment-depth               Segment stereo depth into floor/wall classes\n"
        << "  --depth-source <stereo>       Depth source for --segment-depth (default: stereo)\n"
        << "  --segment-every <N>           Segment every Nth frame (default: 1)\n"
        << "  --engine <lc|msckf>           VIO engine (default: lc). msckf uses the ported OpenVINS TC MSCKF.\n";
}

int parse_nonnegative_int(const std::string& value, const std::string& option_name) {
    size_t pos = 0;
    const int parsed = std::stoi(value, &pos);
    if (pos != value.size() || parsed < 0) {
        throw std::runtime_error(option_name + " expects a non-negative integer");
    }
    return parsed;
}

int parse_positive_int(const std::string& value, const std::string& option_name) {
    const int parsed = parse_nonnegative_int(value, option_name);
    if (parsed <= 0) {
        throw std::runtime_error(option_name + " expects a positive integer");
    }
    return parsed;
}

CliOptions parse_cli(int argc, char** argv) {
    CliOptions options;
    if (argc < 2) {
        return options;
    }

    const std::string first_arg = argv[1];
    if (first_arg == "-h" || first_arg == "--help") {
        return options;
    }

    options.config_path = argv[1];
    for (int i = 2; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--rerun-save") {
            if (i + 1 >= argc) {
                throw std::runtime_error("--rerun-save requires a file path");
            }
            options.rerun.requested = true;
            options.rerun.save_path = argv[++i];
        } else if (arg == "--rerun-spawn") {
            options.rerun.requested = true;
            options.rerun.spawn = true;
        } else if (arg == "--rerun-connect") {
            options.rerun.requested = true;
            options.rerun.connect = true;
        } else if (arg == "--rerun-image-every") {
            if (i + 1 >= argc) {
                throw std::runtime_error("--rerun-image-every requires a frame interval");
            }
            options.rerun.requested = true;
            options.rerun.image_every = parse_nonnegative_int(argv[++i], arg);
        } else if (arg == "--segment-lidar") {
            options.segment_lidar = true;
        } else if (arg == "--segment-depth") {
            options.segment_depth = true;
        } else if (arg == "--depth-source") {
            if (i + 1 >= argc) {
                throw std::runtime_error("--depth-source requires a source name");
            }
            options.depth_source = argv[++i];
            if (options.depth_source != "stereo") {
                throw std::runtime_error("--depth-source currently supports stereo only");
            }
            options.segment_depth = true;
        } else if (arg == "--segment-every") {
            if (i + 1 >= argc) {
                throw std::runtime_error("--segment-every requires a frame interval");
            }
            options.segment_every = parse_positive_int(argv[++i], arg);
            options.segment_every_set = true;
        } else if (arg == "--engine") {
            if (i + 1 >= argc) {
                throw std::runtime_error("--engine requires lc or msckf");
            }
            options.engine = argv[++i];
            if (options.engine != "lc" && options.engine != "msckf") {
                throw std::runtime_error("--engine must be 'lc' or 'msckf'");
            }
        } else if (arg == "-h" || arg == "--help") {
            options.config_path.clear();
            return options;
        } else {
            throw std::runtime_error("Unknown option: " + arg);
        }
    }

    if (options.segment_every_set && !options.segment_lidar && !options.segment_depth) {
        options.segment_lidar = true;
    }

    return options;
}

CameraParams load_cam(const YAML::Node& node) {
    CameraParams c;
    c.fx = node["fx"].as<double>();
    c.fy = node["fy"].as<double>();
    c.cx = node["cx"].as<double>();
    c.cy = node["cy"].as<double>();
    c.k1 = node["k1"].as<double>(0.0);
    c.k2 = node["k2"].as<double>(0.0);
    c.p1 = node["p1"].as<double>(0.0);
    c.p2 = node["p2"].as<double>(0.0);
    c.width = node["width"].as<int>(c.width);
    c.height = node["height"].as<int>(c.height);

    const auto rows = node["T_cam_imu"];
    if (!rows || rows.size() != 4) {
        throw std::runtime_error("Camera config is missing a 4x4 T_cam_imu matrix");
    }
    for (int r = 0; r < 4; ++r) {
        for (int cc = 0; cc < 4; ++cc) {
            c.T_cam_imu(r, cc) = rows[r][cc].as<double>();
        }
    }
    return c;
}

ImuNoiseParams load_imu_noise(const YAML::Node& node) {
    ImuNoiseParams n;
    n.gyro_noise = node["gyro"].as<double>();
    n.accel_noise = node["accel"].as<double>();
    n.gyro_walk = node["gyro_walk"].as<double>();
    n.accel_walk = node["accel_walk"].as<double>();
    return n;
}

InitParams load_init_params(const YAML::Node& node) {
    InitParams params;
    if (!node) return params;
    params.static_window_sec = node["static_window_sec"].as<double>(params.static_window_sec);
    params.gravity_norm = node["gravity_norm"].as<double>(params.gravity_norm);
    return params;
}

StereoDepthSegmentationOptions load_depth_segmentation_options(const YAML::Node& node) {
    StereoDepthSegmentationOptions options;
    if (!node) return options;

    options.min_depth_m = node["min_depth_m"].as<double>(options.min_depth_m);
    options.max_depth_m = node["max_depth_m"].as<double>(options.max_depth_m);
    options.image_stride = node["image_stride"].as<int>(options.image_stride);

    options.min_x = node["min_x"].as<double>(options.min_x);
    options.max_x = node["max_x"].as<double>(options.max_x);
    options.min_y = node["min_y"].as<double>(options.min_y);
    options.max_y = node["max_y"].as<double>(options.max_y);
    options.min_z = node["min_z"].as<double>(options.min_z);
    options.max_z = node["max_z"].as<double>(options.max_z);

    options.floor_search_min_z =
        node["floor_search_min_z"].as<double>(options.floor_search_min_z);
    options.floor_search_max_z =
        node["floor_search_max_z"].as<double>(options.floor_search_max_z);
    options.floor_distance_threshold =
        node["floor_distance_threshold_m"].as<double>(options.floor_distance_threshold);
    options.floor_normal_min_abs_dot_gravity =
        node["floor_normal_min_abs_dot_gravity"].as<double>(
            options.floor_normal_min_abs_dot_gravity);
    options.min_floor_inliers =
        node["min_floor_inliers"].as<std::size_t>(options.min_floor_inliers);

    options.wall_min_z = node["wall_min_z"].as<double>(options.wall_min_z);
    options.wall_max_z = node["wall_max_z"].as<double>(options.wall_max_z);
    options.wall_distance_threshold =
        node["wall_distance_threshold_m"].as<double>(options.wall_distance_threshold);
    options.wall_normal_max_abs_dot_gravity =
        node["wall_normal_max_abs_dot_gravity"].as<double>(
            options.wall_normal_max_abs_dot_gravity);
    options.min_wall_extent_m =
        node["min_wall_extent_m"].as<double>(options.min_wall_extent_m);
    options.min_wall_height_m =
        node["min_wall_height_m"].as<double>(options.min_wall_height_m);
    options.min_wall_inliers =
        node["min_wall_inliers"].as<std::size_t>(options.min_wall_inliers);

    options.ransac_iterations =
        node["ransac_iterations"].as<int>(options.ransac_iterations);
    options.sgbm_min_disparity =
        node["sgbm_min_disparity"].as<int>(options.sgbm_min_disparity);
    options.sgbm_num_disparities =
        node["sgbm_num_disparities"].as<int>(options.sgbm_num_disparities);
    options.sgbm_block_size =
        node["sgbm_block_size"].as<int>(options.sgbm_block_size);
    options.sgbm_uniqueness_ratio =
        node["sgbm_uniqueness_ratio"].as<int>(options.sgbm_uniqueness_ratio);
    options.sgbm_speckle_window_size =
        node["sgbm_speckle_window_size"].as<int>(options.sgbm_speckle_window_size);
    options.sgbm_speckle_range =
        node["sgbm_speckle_range"].as<int>(options.sgbm_speckle_range);
    return options;
}

std::unique_ptr<DatasetReader> create_reader(const std::string& dataset_type,
                                             const std::string& dataset_root) {
    if (dataset_type == "euroc") {
        return std::make_unique<EuRoCReader>(dataset_root);
    }
    if (dataset_type == "kitti_raw") {
        return std::make_unique<KittiRawReader>(dataset_root);
    }
    throw std::runtime_error("Unsupported dataset_type: " + dataset_type);
}

Eigen::Matrix3d estimate_initial_orientation(const Eigen::Vector3d& accel_mean) {
    if (accel_mean.norm() < 1e-6) {
        return Eigen::Matrix3d::Identity();
    }

    Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(
        accel_mean.normalized(),
        Eigen::Vector3d::UnitZ());
    q.normalize();
    return q.toRotationMatrix();
}

InitialState initialize_from_imu(const std::vector<ImuData>& imu_data,
                                 double t0,
                                 const InitParams& init_params) {
    if (imu_data.empty()) {
        throw std::runtime_error("No IMU data available for initialization");
    }

    InitialState init;
    init.t0 = t0;
    init.g_world = Eigen::Vector3d(0.0, 0.0, -init_params.gravity_norm);

    Eigen::Vector3d gyro_sum = Eigen::Vector3d::Zero();
    Eigen::Vector3d accel_sum = Eigen::Vector3d::Zero();
    int sample_count = 0;

    for (const auto& imu : imu_data) {
        if (imu.timestamp < t0) continue;
        if (imu.timestamp > t0 + init_params.static_window_sec) break;
        gyro_sum += imu.gyro;
        accel_sum += imu.accel;
        ++sample_count;
    }

    if (sample_count == 0) {
        const double fallback_end = imu_data.front().timestamp + init_params.static_window_sec;
        for (const auto& imu : imu_data) {
            if (imu.timestamp > fallback_end) break;
            gyro_sum += imu.gyro;
            accel_sum += imu.accel;
            ++sample_count;
        }
    }

    if (sample_count == 0) {
        throw std::runtime_error("Failed to collect IMU samples for initialization");
    }

    init.bg0 = gyro_sum / static_cast<double>(sample_count);
    const Eigen::Vector3d accel_mean = accel_sum / static_cast<double>(sample_count);
    init.R0 = estimate_initial_orientation(accel_mean);

    std::cout << "Init window samples=" << sample_count
              << "  mean gyro=" << init.bg0.transpose()
              << "  mean accel=" << accel_mean.transpose() << "\n";
    std::cout << "Init gravity=" << init.g_world.transpose()
              << "  |g|=" << init.g_world.norm() << "\n";
    return init;
}

bool sample_gt_position(const std::vector<GtData>& gt,
                        double timestamp,
                        Eigen::Vector3d& out) {
    if (gt.empty()) return false;

    auto it = std::lower_bound(gt.begin(), gt.end(), timestamp,
                               [](const GtData& g, double t) {
                                   return g.timestamp < t;
                               });
    if (it == gt.begin()) {
        out = it->p;
        return true;
    }
    if (it == gt.end()) {
        out = gt.back().p;
        return true;
    }

    const auto prev = std::prev(it);
    out = (std::abs(prev->timestamp - timestamp) <= std::abs(it->timestamp - timestamp))
              ? prev->p
              : it->p;
    return true;
}

std::optional<Eigen::Matrix4d> estimate_alignment_transform(
    const std::vector<Eigen::Vector3d>& est,
    const std::vector<Eigen::Vector3d>& gt) {
    if (est.size() != gt.size() || est.empty()) return std::nullopt;

    Eigen::MatrixXd est_mat(3, est.size());
    Eigen::MatrixXd gt_mat(3, gt.size());
    for (size_t i = 0; i < est.size(); ++i) {
        est_mat.col(static_cast<Eigen::Index>(i)) = est[i];
        gt_mat.col(static_cast<Eigen::Index>(i)) = gt[i];
    }

    return Eigen::umeyama(est_mat, gt_mat, false);
}

std::vector<Eigen::Vector3d> apply_alignment(
    const std::vector<Eigen::Vector3d>& points,
    const Eigen::Matrix4d& T_gt_est) {
    std::vector<Eigen::Vector3d> aligned;
    aligned.reserve(points.size());
    for (const auto& p : points) {
        const Eigen::Vector4d p_h(p.x(), p.y(), p.z(), 1.0);
        aligned.push_back((T_gt_est * p_h).head<3>());
    }
    return aligned;
}

double compute_ate(const std::vector<Eigen::Vector3d>& est,
                   const std::vector<Eigen::Vector3d>& gt) {
    const auto T_gt_est = estimate_alignment_transform(est, gt);
    if (!T_gt_est.has_value()) return -1.0;

    double sum2 = 0.0;
    for (size_t i = 0; i < est.size(); ++i) {
        Eigen::Vector4d p_est(est[i].x(), est[i].y(), est[i].z(), 1.0);
        const Eigen::Vector3d aligned = (*T_gt_est * p_est).head<3>();
        sum2 += (aligned - gt[i]).squaredNorm();
    }
    return std::sqrt(sum2 / static_cast<double>(est.size()));
}

// ZYX Euler angles in degrees from rotation matrix R_world_body
Eigen::Vector3d rot_to_euler_deg(const Eigen::Matrix3d& R) {
    double roll  = std::atan2(R(2,1), R(2,2));
    double pitch = std::asin(std::clamp(-R(2,0), -1.0, 1.0));
    double yaw   = std::atan2(R(1,0), R(0,0));
    constexpr double rad2deg = 180.0 / M_PI;
    return Eigen::Vector3d(roll, pitch, yaw) * rad2deg;
}

std::string frame_id(int frame_idx) {
    std::ostringstream oss;
    oss << std::setw(10) << std::setfill('0') << frame_idx;
    return oss.str();
}

void print_segment_counts(int frame_idx, const SegmentCounts& counts) {
    std::cout << "  lidar[" << frame_idx << "]"
              << " floor=" << counts.floor
              << " wall=" << counts.wall
              << " left=" << counts.left_wall
              << " right=" << counts.right_wall
              << " front=" << counts.front_wall
              << " other=" << counts.other << "\n";
}

void print_depth_segment_counts(int frame_idx, const SegmentCounts& counts) {
    std::cout << "  depth[" << frame_idx << "]"
              << " floor=" << counts.floor
              << " wall=" << counts.wall
              << " other=" << counts.other << "\n";
}

#ifdef LC_VIO_WITH_RERUN
rerun::Position3D to_rerun_position(const Eigen::Vector3d& p) {
    return rerun::Position3D(
        static_cast<float>(p.x()),
        static_cast<float>(p.y()),
        static_cast<float>(p.z()));
}

rerun::Vec3D to_rerun_vec3(const Eigen::Vector3d& p) {
    return rerun::Vec3D(
        static_cast<float>(p.x()),
        static_cast<float>(p.y()),
        static_cast<float>(p.z()));
}

std::vector<rerun::Vec3D> to_rerun_line(const std::vector<Eigen::Vector3d>& points) {
    std::vector<rerun::Vec3D> result;
    result.reserve(points.size());
    for (const auto& p : points) {
        result.push_back(to_rerun_vec3(p));
    }
    return result;
}

std::vector<rerun::Position2D> to_rerun_points2d(const std::vector<cv::Point2f>& points) {
    std::vector<rerun::Position2D> result;
    result.reserve(points.size());
    for (const auto& p : points) {
        result.emplace_back(p.x, p.y);
    }
    return result;
}

std::vector<rerun::Position3D> to_rerun_lidar_positions(const SegmentedCloud& cloud) {
    std::vector<rerun::Position3D> result;
    result.reserve(cloud.points.size());
    for (const auto& p : cloud.points) {
        result.emplace_back(p.x, p.y, p.z);
    }
    return result;
}

std::vector<rerun::Color> to_rerun_lidar_colors(const SegmentedCloud& cloud) {
    std::vector<rerun::Color> result;
    result.reserve(cloud.labels.size());
    for (const auto label : cloud.labels) {
        const auto rgb = segment_color_rgb(label);
        result.emplace_back(
            static_cast<uint8_t>(rgb.x()),
            static_cast<uint8_t>(rgb.y()),
            static_cast<uint8_t>(rgb.z()));
    }
    return result;
}

std::vector<uint8_t> mat_bytes_u8(const cv::Mat& image) {
    if (image.empty() || image.depth() != CV_8U) {
        return {};
    }

    const cv::Mat continuous = image.isContinuous() ? image : image.clone();
    const auto* begin = continuous.ptr<uint8_t>(0);
    const auto* end = begin + continuous.total() * continuous.elemSize();
    return std::vector<uint8_t>(begin, end);
}

class RerunLogger {
public:
    explicit RerunLogger(const RerunOptions& options, double time_origin_sec)
        : options_(options), time_origin_sec_(time_origin_sec) {
        if (!options_.requested) {
            return;
        }

        rec_ = std::make_unique<rerun::RecordingStream>("lc_ekf_vio_kitti_raw");
        if (!options_.save_path.empty()) {
            const fs::path save_path(options_.save_path);
            if (save_path.has_parent_path()) {
                fs::create_directories(save_path.parent_path());
            }
            rec_->save(options_.save_path).exit_on_failure();
            std::cout << "Rerun recording : " << options_.save_path << "\n";
        }
        if (options_.connect) {
            rec_->connect_grpc().exit_on_failure();
            std::cout << "Rerun sink      : grpc\n";
        }
        if (options_.spawn) {
            rec_->spawn().exit_on_failure();
            std::cout << "Rerun sink      : spawned viewer\n";
        }
    }

    double relative_time(double timestamp_sec) const {
        return timestamp_sec - time_origin_sec_;
    }

    void log_imu(int imu_idx,
                 double timestamp,
                 const Eigen::Vector3d& gyro,
                 const Eigen::Vector3d& accel) {
        if (!rec_) {
            return;
        }

        (void)imu_idx;
        rec_->set_time_duration_secs("time", relative_time(timestamp));
        rec_->log("imu/gyro_x_rad_s", rerun::Scalars(gyro.x()));
        rec_->log("imu/gyro_y_rad_s", rerun::Scalars(gyro.y()));
        rec_->log("imu/gyro_z_rad_s", rerun::Scalars(gyro.z()));
        rec_->log("imu/gyro_norm_rad_s", rerun::Scalars(gyro.norm()));
        rec_->log("imu/accel_x_mps2", rerun::Scalars(accel.x()));
        rec_->log("imu/accel_y_mps2", rerun::Scalars(accel.y()));
        rec_->log("imu/accel_z_mps2", rerun::Scalars(accel.z()));
        rec_->log("imu/accel_norm_mps2", rerun::Scalars(accel.norm()));
    }

    void log_frame(int frame_idx,
                   double timestamp,
                   const Eigen::Vector3d& velocity,
                   int feature_count,
                   const cv::Mat& rectified_left,
                   const std::vector<cv::Point2f>& tracked_points) {
        if (!rec_) {
            return;
        }

        rec_->set_time_duration_secs("time", relative_time(timestamp));

        rec_->log("metrics/tracked_features", rerun::Scalars(static_cast<double>(feature_count)));
        rec_->log("metrics/speed_mps", rerun::Scalars(velocity.norm()));

        if (options_.image_every > 0 && frame_idx % options_.image_every == 0 && !rectified_left.empty()) {
            auto bytes = mat_bytes_u8(rectified_left);
            if (!bytes.empty() && rectified_left.channels() == 1) {
                rec_->log("camera/left_rectified",
                          rerun::Image::from_grayscale8(std::move(bytes),
                                                         {static_cast<uint32_t>(rectified_left.cols),
                                                          static_cast<uint32_t>(rectified_left.rows)}));
            }

            auto points = to_rerun_points2d(tracked_points);
            if (!points.empty()) {
                rec_->log("camera/left_rectified/tracked_features",
                          rerun::Points2D(points)
                              .with_colors(rerun::Color(0, 255, 255))
                              .with_radii(rerun::Radius::ui_points(3.0f)));
            }
        }
    }

    void log_lidar_segments(int frame_idx,
                            double timestamp,
                            const SegmentedCloud& cloud) {
        if (!rec_ || cloud.points.empty()) {
            return;
        }

        rec_->set_time_duration_secs("time", relative_time(timestamp));
        const auto counts = cloud.counts();
        rec_->log("metrics/lidar_floor_points", rerun::Scalars(static_cast<double>(counts.floor)));
        rec_->log("metrics/lidar_left_wall_points", rerun::Scalars(static_cast<double>(counts.left_wall)));
        rec_->log("metrics/lidar_right_wall_points", rerun::Scalars(static_cast<double>(counts.right_wall)));
        rec_->log("metrics/lidar_front_wall_points", rerun::Scalars(static_cast<double>(counts.front_wall)));

        auto positions = to_rerun_lidar_positions(cloud);
        auto colors = to_rerun_lidar_colors(cloud);
        rec_->log("lidar/segments",
                  rerun::Points3D(std::move(positions))
                      .with_colors(std::move(colors))
                      .with_radii(0.035f));

        (void)frame_idx;
    }

    void log_depth_segments(int frame_idx,
                            double timestamp,
                            const SegmentedCloud& cloud) {
        if (!rec_ || cloud.points.empty()) {
            return;
        }

        rec_->set_time_duration_secs("time", relative_time(timestamp));
        const auto counts = cloud.counts();
        rec_->log("metrics/depth_floor_points", rerun::Scalars(static_cast<double>(counts.floor)));
        rec_->log("metrics/depth_wall_points", rerun::Scalars(static_cast<double>(counts.wall)));
        rec_->log("metrics/depth_other_points", rerun::Scalars(static_cast<double>(counts.other)));

        auto positions = to_rerun_lidar_positions(cloud);
        auto colors = to_rerun_lidar_colors(cloud);
        rec_->log("depth/segments",
                  rerun::Points3D(std::move(positions))
                      .with_colors(std::move(colors))
                      .with_radii(0.035f));

        (void)frame_idx;
    }

    void log_aligned_world(const std::vector<double>& timestamps,
                           const std::vector<Eigen::Vector3d>& traj_est_aligned,
                           const std::vector<Eigen::Vector3d>& traj_gt) {
        if (!rec_ || timestamps.empty() || traj_est_aligned.empty()) {
            return;
        }
        if (traj_est_aligned.size() != timestamps.size() || traj_gt.size() != timestamps.size()) {
            return;
        }

        std::vector<Eigen::Vector3d> est_prefix;
        std::vector<Eigen::Vector3d> gt_prefix;
        est_prefix.reserve(traj_est_aligned.size());
        gt_prefix.reserve(traj_gt.size());

        for (size_t i = 0; i < timestamps.size(); ++i) {
            est_prefix.push_back(traj_est_aligned[i]);
            gt_prefix.push_back(traj_gt[i]);

            rec_->set_time_duration_secs("time", relative_time(timestamps[i]));

            rec_->log("world/current_est_aligned",
                      rerun::Points3D({to_rerun_position(traj_est_aligned[i])})
                          .with_colors(rerun::Color(0, 180, 255))
                          .with_radii(0.35f));
            rec_->log("world/current_gt",
                      rerun::Points3D({to_rerun_position(traj_gt[i])})
                          .with_colors(rerun::Color(255, 90, 80))
                          .with_radii(0.35f));

            if (est_prefix.size() >= 2) {
                rec_->log("world/trajectory_est_aligned",
                          rerun::LineStrips3D(rerun::LineStrip3D(to_rerun_line(est_prefix)))
                              .with_colors(rerun::Color(0, 180, 255))
                              .with_radii(0.08f));
                rec_->log("world/trajectory_gt",
                          rerun::LineStrips3D(rerun::LineStrip3D(to_rerun_line(gt_prefix)))
                              .with_colors(rerun::Color(255, 90, 80))
                              .with_radii(0.06f));
            }
        }
    }

    void flush() {
        if (rec_) {
            rec_->flush_blocking(10.0f).exit_on_failure();
        }
    }

private:
    RerunOptions options_;
    double time_origin_sec_ = 0.0;
    std::unique_ptr<rerun::RecordingStream> rec_;
};
#else
class RerunLogger {
public:
    explicit RerunLogger(const RerunOptions& options, double) {
        if (options.requested) {
            throw std::runtime_error("Rerun options were passed, but this binary was built without ENABLE_RERUN=ON");
        }
    }

    void log_imu(int,
                 double,
                 const Eigen::Vector3d&,
                 const Eigen::Vector3d&) {}

    void log_frame(int,
                   double,
                   const Eigen::Vector3d&,
                   int,
                   const cv::Mat&,
                   const std::vector<cv::Point2f>&) {}

    void log_lidar_segments(int,
                            double,
                            const SegmentedCloud&) {}

    void log_depth_segments(int,
                            double,
                            const SegmentedCloud&) {}

    void log_aligned_world(const std::vector<double>&,
                           const std::vector<Eigen::Vector3d>&,
                           const std::vector<Eigen::Vector3d>&) {}

    void flush() {}
};
#endif

void write_state_log(const std::string& path,
                     const std::vector<double>& timestamps,
                     const std::vector<Eigen::Vector3d>& positions,
                     const std::vector<Eigen::Vector3d>& velocities,
                     const std::vector<Eigen::Vector3d>& eulers,
                     const std::vector<Eigen::Quaterniond>& quats,
                     const std::vector<int>& feat_counts) {
    std::ofstream f(path);
    f << "# timestamp px py pz vx vy vz roll_deg pitch_deg yaw_deg qx qy qz qw feat_count\n";
    for (size_t i = 0; i < timestamps.size(); ++i) {
        f << std::fixed << std::setprecision(9)
          << timestamps[i]     << " "
          << positions[i].x()  << " " << positions[i].y()  << " " << positions[i].z()  << " "
          << velocities[i].x() << " " << velocities[i].y() << " " << velocities[i].z() << " "
          << eulers[i].x()     << " " << eulers[i].y()     << " " << eulers[i].z()     << " "
          << quats[i].x()      << " " << quats[i].y()      << " " << quats[i].z()      << " "
          << quats[i].w()      << " " << feat_counts[i]    << "\n";
    }
}

void write_imu_log(const std::string& path,
                   const std::vector<double>& timestamps,
                   const std::vector<Eigen::Vector3d>& gyros,
                   const std::vector<Eigen::Vector3d>& accels) {
    std::ofstream f(path);
    f << "# timestamp gyro_x gyro_y gyro_z accel_x accel_y accel_z gyro_norm accel_norm\n";
    for (size_t i = 0; i < timestamps.size(); ++i) {
        f << std::fixed << std::setprecision(9)
          << timestamps[i] << " "
          << gyros[i].x() << " " << gyros[i].y() << " " << gyros[i].z() << " "
          << accels[i].x() << " " << accels[i].y() << " " << accels[i].z() << " "
          << gyros[i].norm() << " " << accels[i].norm() << "\n";
    }
}

void write_tum_positions(const std::string& path,
                         const std::vector<double>& timestamps,
                         const std::vector<Eigen::Vector3d>& positions) {
    std::ofstream f(path);
    f << "# timestamp tx ty tz qx qy qz qw\n";
    for (size_t i = 0; i < timestamps.size(); ++i) {
        f << std::fixed << std::setprecision(9)
          << timestamps[i] << " "
          << positions[i].x() << " "
          << positions[i].y() << " "
          << positions[i].z() << " 0 0 0 1\n";
    }
}

} // namespace

int main(int argc, char** argv) {
    try {
        const CliOptions cli = parse_cli(argc, argv);
        if (cli.config_path.empty()) {
            print_usage(argv[0]);
            return argc < 2 ? 1 : 0;
        }

        const YAML::Node cfg = YAML::LoadFile(cli.config_path);
        const std::string dataset_type = cfg["dataset_type"].as<std::string>("euroc");
        const std::string dataset_root = cfg["dataset"].as<std::string>();
        const std::string output_dir = cfg["output"].as<std::string>();
        const int max_frames = cfg["max_frames"].as<int>(0);

        fs::create_directories(output_dir);

        auto data = create_reader(dataset_type, dataset_root);
        if (data->cam().empty()) throw std::runtime_error("No camera data loaded");
        if (data->imu().empty()) throw std::runtime_error("No IMU data loaded");

        const CameraParams cam0 = load_cam(cfg["cam0"]);
        const CameraParams cam1 = load_cam(cfg["cam1"]);
        const ImuNoiseParams imu_noise = load_imu_noise(cfg["imu_noise"]);
        const InitParams init_params = load_init_params(cfg["init"]);
        const StereoDepthSegmentationOptions depth_options =
            load_depth_segmentation_options(cfg["depth_segmentation"]);
        const double sigma_vo = cfg["ekf"]["sigma_vo"].as<double>(0.5);

        const auto& cam_data = data->cam();
        const auto& imu_data = data->imu();
        const auto& gt_data = data->gt();
        const auto& lidar_data = data->lidar();
        const double t0 = cam_data.front().timestamp;
        RerunLogger rerun(cli.rerun, t0);

        if (cli.segment_lidar) {
            if (dataset_type != "kitti_raw") {
                throw std::runtime_error("--segment-lidar currently supports dataset_type=kitti_raw only");
            }
            if (lidar_data.empty()) {
                throw std::runtime_error("--segment-lidar was requested, but no Velodyne files were found");
            }
            std::cout << "LiDAR segmentation: every " << cli.segment_every
                      << " frame(s), Velodyne frames=" << lidar_data.size() << "\n";
        }
        if (cli.segment_depth) {
            if (cli.depth_source != "stereo") {
                throw std::runtime_error("--segment-depth currently supports --depth-source stereo only");
            }
            std::cout << "Stereo depth segmentation: every " << cli.segment_every
                      << " frame(s), stride=" << depth_options.image_stride
                      << " depth=[" << depth_options.min_depth_m
                      << ", " << depth_options.max_depth_m << "] m\n";
        }
        const bool write_lidar_ply = cli.segment_lidar && !cli.rerun.requested;
        const std::string lidar_output_dir = output_dir + "/lidar_segments";
        if (write_lidar_ply) {
            fs::create_directories(lidar_output_dir);
        }
        const bool write_depth_ply = cli.segment_depth && !cli.rerun.requested;
        const std::string depth_output_dir = output_dir + "/depth_segments";
        if (write_depth_ply) {
            fs::create_directories(depth_output_dir);
        }
        LidarSegmenter lidar_segmenter;
        StereoDepthSegmenter depth_segmenter(depth_options);

        const InitialState init = initialize_from_imu(imu_data, t0, init_params);

        ImuPropagator imu_prop(init.p0, init.v0, init.R0, init.bg0, init.ba0,
                               imu_noise, init.g_world);
        LcEkf ekf(imu_prop, cam0.T_cam_imu, sigma_vo);
        StereoTracker tracker(cam0, cam1);

        // ---- MSCKF engine (optional, parallel to LC) ----
        std::unique_ptr<MsckfPipeline> msckf;
        const bool use_msckf = (cli.engine == "msckf");
        if (use_msckf) {
            // Recompute mean accel/gyro over the same stationary window the LC
            // init used (post-t0, see initialize_from_imu lines 318-324).
            // MsckfPipeline rebuilds R_GtoI internally via OpenVINS' gram_schmidt,
            // so we feed it the raw means rather than LC's R0.
            Eigen::Vector3d mean_accel = Eigen::Vector3d::Zero();
            Eigen::Vector3d mean_gyro  = Eigen::Vector3d::Zero();
            int n_init = 0;
            const double win = init_params.static_window_sec;
            for (const auto& imu : imu_data) {
                if (imu.timestamp < t0) continue;
                if (imu.timestamp > t0 + win) break;
                mean_accel += imu.accel;
                mean_gyro  += imu.gyro;
                ++n_init;
            }
            if (n_init == 0) {
                throw std::runtime_error("MSCKF init: no IMU samples in stationary window");
            }
            mean_accel /= double(n_init);
            mean_gyro  /= double(n_init);
            std::cout << "[engine] MSCKF init from " << n_init
                      << " IMU samples, |a|=" << mean_accel.norm() << "\n";

            // MSCKF consumes rectified cam0 + cam1 pixels, so its camera model
            // uses the virtual rectified frames, not the raw cam frames. After
            // cv::stereoRectify with alpha=0, both rectified cams share K and
            // differ only by the baseline along the rectified x axis.
            //
            //   T_rectcam0_imu = R_rectcam0_cam0 * T_cam0_imu                  (rectify rotation)
            //   T_rectcam1_imu = T_rectcam1_rectcam0 * T_rectcam0_imu          (baseline shift)
            //   where T_rectcam1_rectcam0 = [I | (-baseline, 0, 0)] (cam1 origin sits
            //   at +baseline of cam0 in rectified frame, so the *cam1 frame's*
            //   view of a point in cam0 frame translates by -baseline in x).
            Eigen::Matrix4d T_rectcam0_imu = cam0.T_cam_imu;
            const Eigen::Matrix3d R_rectcam0_cam0 =
                tracker.rectified_to_cam0_rotation().transpose();
            T_rectcam0_imu.block<3, 3>(0, 0) =
                R_rectcam0_cam0 * cam0.T_cam_imu.block<3, 3>(0, 0);
            T_rectcam0_imu.block<3, 1>(0, 3) =
                R_rectcam0_cam0 * cam0.T_cam_imu.block<3, 1>(0, 3);

            Eigen::Matrix4d T_rectcam1_rectcam0 = Eigen::Matrix4d::Identity();
            T_rectcam1_rectcam0(0, 3) = -tracker.rectified_baseline();
            Eigen::Matrix4d T_rectcam1_imu = T_rectcam1_rectcam0 * T_rectcam0_imu;
            std::cout << "[engine] MSCKF stereo: rectified baseline="
                      << tracker.rectified_baseline()
                      << "m  T_rectcam1_imu.t.x=" << T_rectcam1_imu(0, 3) << "\n";

            msckf = std::make_unique<MsckfPipeline>(
                init.t0,
                init.R0,                                       // yaw-aligned with GT world
                mean_accel, mean_gyro,                          // OpenVINS-style bg/ba
                std::abs(init.g_world.z()),
                T_rectcam0_imu,
                T_rectcam1_imu,                                 // cycle 5: cam1
                tracker.rectified_fx(), tracker.rectified_fy(),
                tracker.rectified_cx(), tracker.rectified_cy(),
                cam0.width, cam0.height);
            std::cout << "[engine] MSCKF (TC, ported OpenVINS)\n";
        } else {
            std::cout << "[engine] LC EKF (baseline)\n";
        }
        StereoDepthGeometry depth_geometry;
        depth_geometry.fx = tracker.rectified_fx();
        depth_geometry.fy = tracker.rectified_fy();
        depth_geometry.cx = tracker.rectified_cx();
        depth_geometry.cy = tracker.rectified_cy();
        depth_geometry.baseline_m = tracker.rectified_baseline();
        depth_geometry.R_cam0_rect = tracker.rectified_to_cam0_rotation();
        depth_geometry.T_cam0_imu = cam0.T_cam_imu;

        // VO is local to the first cam0 frame. Anchor that frame to the initialized IMU world.
        const Eigen::Matrix4d T_IC = cam0.T_cam_imu.inverse(); // cam0 -> IMU
        const Eigen::Matrix3d R_WC0 = init.R0 * T_IC.block<3, 3>(0, 0);
        const Eigen::Vector3d t_WC0 = init.R0 * T_IC.block<3, 1>(0, 3) + init.p0;
        std::cout << "Dataset=" << dataset_type
                  << "  GT=" << (!gt_data.empty() ? "yes" : "no")
                  << "  first_cam_t=" << t0 << "\n";
        std::cout << "Init cam0_world p=" << t_WC0.transpose() << "\n";

        std::vector<Eigen::Vector3d>   traj_est;
        std::vector<Eigen::Vector3d>   traj_vel;
        std::vector<Eigen::Vector3d>   traj_euler;
        std::vector<Eigen::Quaterniond> traj_quat;
        std::vector<int>               traj_feat;
        std::vector<Eigen::Vector3d>   traj_gt;
        std::vector<double>            traj_ts;
        std::vector<double>            imu_ts_log;
        std::vector<Eigen::Vector3d>   imu_gyro_log;
        std::vector<Eigen::Vector3d>   imu_accel_log;

        size_t imu_idx = 0;
        while (imu_idx < imu_data.size() && imu_data[imu_idx].timestamp < t0) {
            ++imu_idx;
        }

        // MSCKF wants the full IMU stream buffered upfront so the propagator
        // can select bracketing samples (including one *before* time0) for any
        // cam.timestamp. We feed the entire IMU stream from index 0, not from
        // the post-init imu_idx, so pre-t0 samples are available for boundary
        // interpolation. LC EKF (else branch) keeps the incremental schedule.
        if (use_msckf) {
            int pre_count = 0;
            for (size_t k = 0; k < imu_data.size(); ++k) {
                const auto& imu = imu_data[k];
                msckf->feed_imu(imu.timestamp, imu.gyro, imu.accel);
                imu_ts_log.push_back(imu.timestamp);
                imu_gyro_log.push_back(imu.gyro);
                imu_accel_log.push_back(imu.accel);
                ++pre_count;
            }
            imu_idx = imu_data.size();
            std::cout << "[msckf] pre-loaded " << pre_count << " IMU samples\n";
        }

        int frame_count = 0;
        int imu_count = 0;
        int lidar_segmented_frames = 0;
        int depth_segmented_frames = 0;
        bool msckf_have_visual_imu = false;
        bool msckf_velocity_seeded = false;
        double msckf_prev_visual_t = -1.0;
        Eigen::Vector3d msckf_prev_visual_imu = Eigen::Vector3d::Zero();
        const auto wall_start = std::chrono::steady_clock::now();

        for (const auto& cam : cam_data) {
            if (max_frames > 0 && frame_count >= max_frames) break;

            while (imu_idx < imu_data.size() && imu_data[imu_idx].timestamp <= cam.timestamp) {
                const auto& imu = imu_data[imu_idx++];
                rerun.log_imu(imu_count, imu.timestamp, imu.gyro, imu.accel);
                imu_ts_log.push_back(imu.timestamp);
                imu_gyro_log.push_back(imu.gyro);
                imu_accel_log.push_back(imu.accel);
                ++imu_count;
                // MSCKF already received all IMU upfront; here we only feed LC.
                ekf.propagate(imu.timestamp, imu.gyro, imu.accel);
            }

            const cv::Mat img_l = cv::imread(cam.img_l, cv::IMREAD_COLOR);
            const cv::Mat img_r = cv::imread(cam.img_r, cv::IMREAD_COLOR);
            if (img_l.empty() || img_r.empty()) {
                std::cerr << "Failed to load images at t=" << cam.timestamp << "\n";
                continue;
            }

            const auto pose = tracker.process(img_l, img_r);
            const Eigen::Vector3d p_world_cam = R_WC0 * pose.t + t_WC0;
            if (use_msckf) {
                const Eigen::Matrix3d R_world_cam = R_WC0 * pose.R;
                const Eigen::Vector3d p_world_imu_visual =
                    p_world_cam + R_world_cam * cam0.T_cam_imu.block<3, 1>(0, 3);
                if (!msckf_have_visual_imu) {
                    msckf_prev_visual_t = cam.timestamp;
                    msckf_prev_visual_imu = p_world_imu_visual;
                    msckf_have_visual_imu = true;
                } else if (!msckf_velocity_seeded && msckf->msckf_update_count() == 0) {
                    const double dt = cam.timestamp - msckf_prev_visual_t;
                    Eigen::Vector3d v0 = Eigen::Vector3d::Zero();
                    if (dt > 1e-6) {
                        v0 = (p_world_imu_visual - msckf_prev_visual_imu) / dt;
                    }
                    if (std::isfinite(v0.norm()) && v0.norm() > 0.1 && v0.norm() < 50.0) {
                        msckf->seed_initial_velocity(v0);
                        msckf_velocity_seeded = true;
                    }
                    msckf_prev_visual_t = cam.timestamp;
                    msckf_prev_visual_imu = p_world_imu_visual;
                }

                // Feed all currently tracked features. cam0 measurements cover
                // every alive track; cam1 measurements cover only the subset
                // where the stereo match held (stereo_valid()[i] == true).
                const auto& ids = tracker.tracked_ids();
                const auto& pts_l = tracker.tracked_points();
                const auto& pts_r = tracker.tracked_points_right();
                const auto& s_ok  = tracker.stereo_valid();
                std::vector<MsckfPipeline::TrackedFeat> left;
                std::vector<MsckfPipeline::TrackedFeat> right;
                left.reserve(pts_l.size());
                right.reserve(pts_l.size());
                for (size_t i = 0; i < pts_l.size(); ++i) {
                    left.push_back({ids[i], pts_l[i].x, pts_l[i].y});
                    if (i < s_ok.size() && s_ok[i] && i < pts_r.size()) {
                        right.push_back({ids[i], pts_r[i].x, pts_r[i].y});
                    }
                }
                msckf->feed_camera(cam.timestamp, left, right);
            } else if (pose.valid && frame_count > 0) {
                ekf.update_vo(p_world_cam);
            }

            // Pull trajectory state from whichever engine is active
            Eigen::Vector3d cur_p, cur_v;
            Eigen::Matrix3d cur_R;
            if (use_msckf) {
                const auto mpose = msckf->latest_pose();
                cur_p = mpose.p; cur_v = mpose.v; cur_R = mpose.R;
            } else {
                cur_p = ekf.position(); cur_v = ekf.velocity(); cur_R = ekf.orientation();
            }
            traj_est.push_back(cur_p);
            traj_vel.push_back(cur_v);
            traj_euler.push_back(rot_to_euler_deg(cur_R));
            traj_quat.push_back(Eigen::Quaterniond(cur_R).normalized());
            traj_feat.push_back(tracker.tracked_count());
            traj_ts.push_back(cam.timestamp);

            Eigen::Vector3d gt_p = Eigen::Vector3d::Zero();
            if (sample_gt_position(gt_data, cam.timestamp, gt_p)) {
                traj_gt.push_back(gt_p);
            }

            rerun.log_frame(frame_count,
                            cam.timestamp,
                            traj_vel.back(),
                            tracker.tracked_count(),
                            tracker.rectified_left_image(),
                            tracker.tracked_points());

            if (cli.segment_lidar && frame_count % cli.segment_every == 0) {
                if (static_cast<size_t>(frame_count) >= lidar_data.size()) {
                    std::cerr << "Missing Velodyne frame for camera frame " << frame_count << "\n";
                } else {
                    const auto lidar_points = load_kitti_velodyne_bin(lidar_data[frame_count]);
                    const auto segmented = lidar_segmenter.process(lidar_points);
                    rerun.log_lidar_segments(frame_count, cam.timestamp, segmented);
                    if (write_lidar_ply) {
                        write_segmented_ply(
                            lidar_output_dir + "/" + frame_id(frame_count) + ".ply",
                            segmented);
                    }
                    ++lidar_segmented_frames;
                    if (frame_count == 0 || frame_count % 100 == 0) {
                        print_segment_counts(frame_count, segmented.counts());
                    }
                }
            }

            if (cli.segment_depth && frame_count % cli.segment_every == 0) {
                const auto segmented = depth_segmenter.process(
                    tracker.rectified_left_image(),
                    tracker.rectified_right_image(),
                    depth_geometry,
                    ekf.orientation());
                rerun.log_depth_segments(frame_count, cam.timestamp, segmented);
                if (write_depth_ply) {
                    write_segmented_ply(
                        depth_output_dir + "/" + frame_id(frame_count) + ".ply",
                        segmented);
                }
                ++depth_segmented_frames;
                if (frame_count == 0 || frame_count % 100 == 0) {
                    print_depth_segment_counts(frame_count, segmented.counts());
                }
            }

            ++frame_count;
            if (frame_count % 100 == 0) {
                const auto now = std::chrono::steady_clock::now();
                const double elapsed = std::chrono::duration<double>(now - wall_start).count();
                std::cout << "Frame " << frame_count
                          << "  t=" << cam.timestamp
                          << "  imu_p=" << traj_est.back().transpose()
                          << "  tracked=" << tracker.tracked_count()
                          << "  [" << elapsed << "s]\n";
            }
        }

        write_tum_positions(output_dir + "/trajectory_tum.txt", traj_ts, traj_est);
        write_state_log(output_dir + "/state_log.txt", traj_ts, traj_est, traj_vel, traj_euler, traj_quat, traj_feat);
        write_imu_log(output_dir + "/imu_log.txt", imu_ts_log, imu_gyro_log, imu_accel_log);
        if (!traj_gt.empty() && traj_gt.size() == traj_ts.size()) {
            write_tum_positions(output_dir + "/gt_tum.txt", traj_ts, traj_gt);
        }

        const double ate = compute_ate(traj_est, traj_gt);
        if (const auto T_gt_est = estimate_alignment_transform(traj_est, traj_gt)) {
            const auto traj_est_aligned = apply_alignment(traj_est, *T_gt_est);
            write_tum_positions(output_dir + "/trajectory_aligned_tum.txt", traj_ts, traj_est_aligned);
            rerun.log_aligned_world(traj_ts, traj_est_aligned, traj_gt);
        }
        rerun.flush();

        std::cout << "\n========================================\n";
        std::cout << "Frames processed : " << frame_count << "\n";
        if (cli.segment_lidar) {
            std::cout << "LiDAR segmented  : " << lidar_segmented_frames << "\n";
        }
        if (cli.segment_depth) {
            std::cout << "Depth segmented  : " << depth_segmented_frames << "\n";
        }
        if (ate >= 0.0) {
            std::cout << "ATE RMSE         : " << ate * 100.0 << " cm\n";
        } else {
            std::cout << "ATE RMSE         : unavailable\n";
        }
        std::cout << "Output           : " << output_dir << "\n";
        std::cout << "========================================\n";

        std::ofstream metrics(output_dir + "/metrics.txt");
        metrics << "dataset_type: " << dataset_type << "\n";
        metrics << "frames: " << frame_count << "\n";
        if (cli.segment_lidar) {
            metrics << "lidar_segmented_frames: " << lidar_segmented_frames << "\n";
        }
        if (cli.segment_depth) {
            metrics << "depth_segmented_frames: " << depth_segmented_frames << "\n";
        }
        if (ate >= 0.0) {
            metrics << "ate_rmse_m: " << ate << "\n";
            metrics << "ate_rmse_cm: " << ate * 100.0 << "\n";
        } else {
            metrics << "ate_rmse_m: unavailable\n";
            metrics << "ate_rmse_cm: unavailable\n";
        }
    } catch (const std::exception& e) {
        std::cerr << "Fatal: " << e.what() << "\n";
        return 1;
    }

    return 0;
}
