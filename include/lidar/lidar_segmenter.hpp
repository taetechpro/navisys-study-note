#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "semantic/segment_types.hpp"

struct LidarSegmentationOptions {
    double min_x = -5.0;
    double max_x = 50.0;
    double min_y = -25.0;
    double max_y = 25.0;
    double min_z = -3.0;
    double max_z = 3.0;

    double floor_search_min_z = -2.6;
    double floor_search_max_z = -0.4;
    double floor_distance_threshold = 0.18;
    double floor_normal_min_z = 0.85;
    std::size_t min_floor_inliers = 500;

    double wall_min_z = -1.4;
    double wall_max_z = 2.5;
    double side_min_abs_y = 2.0;
    double front_min_x = 5.0;
    double front_max_abs_y = 12.0;
    double wall_distance_threshold = 0.25;
    double wall_normal_max_abs_z = 0.25;
    double wall_axis_min_abs = 0.70;
    double min_wall_extent_m = 3.0;
    double min_wall_height_m = 1.0;
    std::size_t min_wall_inliers = 150;

    int ransac_iterations = 180;
};

class LidarSegmenter {
public:
    explicit LidarSegmenter(LidarSegmentationOptions options = {});

    SegmentedCloud process(const std::vector<LidarPoint>& points) const;

private:
    LidarSegmentationOptions options_;
};

std::vector<LidarPoint> load_kitti_velodyne_bin(const std::string& path);
void write_segmented_ply(const std::string& path, const SegmentedCloud& cloud);

const char* segment_label_name(SegmentLabel label);
Eigen::Vector3i segment_color_rgb(SegmentLabel label);
