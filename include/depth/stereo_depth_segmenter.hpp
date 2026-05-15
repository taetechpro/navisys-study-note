#pragma once

#include <cstddef>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core.hpp>

#include "semantic/segment_types.hpp"

struct StereoDepthSegmentationOptions {
    double min_depth_m = 0.5;
    double max_depth_m = 30.0;
    int image_stride = 4;

    double min_x = -10.0;
    double max_x = 50.0;
    double min_y = -25.0;
    double max_y = 25.0;
    double min_z = -3.0;
    double max_z = 3.0;

    double floor_search_min_z = -2.6;
    double floor_search_max_z = -0.2;
    double floor_distance_threshold = 0.10;
    double floor_normal_min_abs_dot_gravity = 0.90;
    std::size_t min_floor_inliers = 500;

    double wall_min_z = -1.6;
    double wall_max_z = 2.5;
    double wall_distance_threshold = 0.15;
    double wall_normal_max_abs_dot_gravity = 0.25;
    double min_wall_extent_m = 1.5;
    double min_wall_height_m = 0.8;
    std::size_t min_wall_inliers = 250;

    int ransac_iterations = 160;

    int sgbm_min_disparity = 0;
    int sgbm_num_disparities = 192;
    int sgbm_block_size = 5;
    int sgbm_uniqueness_ratio = 10;
    int sgbm_speckle_window_size = 100;
    int sgbm_speckle_range = 2;
};

struct StereoDepthGeometry {
    double fx = 0.0;
    double fy = 0.0;
    double cx = 0.0;
    double cy = 0.0;
    double baseline_m = 0.0;
    Eigen::Matrix3d R_cam0_rect = Eigen::Matrix3d::Identity();
    Eigen::Matrix4d T_cam0_imu = Eigen::Matrix4d::Identity();
};

class StereoDepthSegmenter {
public:
    explicit StereoDepthSegmenter(StereoDepthSegmentationOptions options = {});

    SegmentedCloud process(const cv::Mat& rectified_left_gray,
                           const cv::Mat& rectified_right_gray,
                           const StereoDepthGeometry& geometry,
                           const Eigen::Matrix3d& R_world_imu) const;

private:
    StereoDepthSegmentationOptions options_;
};
