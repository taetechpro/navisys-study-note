#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

struct LidarPoint {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float intensity = 0.0f;
};

enum class SegmentLabel : std::uint8_t {
    Other     = 0,
    Floor     = 1,
    LeftWall  = 2,
    RightWall = 3,
    FrontWall = 4,
    Wall      = 5,
};

struct LidarSegmentationOptions {
    // ─── ROI (region of interest) ───────────────────────
    double min_x = -5.0;
    double max_x = 50.0;
    double min_y = -25.0;
    double max_y = 25.0;
    double min_z = -3.0;
    double max_z = 3.0;

    // ─── Floor RANSAC 파라미터 ─────────────────────────
    double floor_search_min_z = -2.6;
    double floor_search_max_z = -0.4;
    double floor_distance_threshold = 0.18;
    double floor_normal_min_z = 0.85;
    std::size_t min_floor_inliers = 500;

    // ─── Wall RANSAC 파라미터 ──────────────────────────
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

    // ─── RANSAC 공통 ────────────────────────────────────
    int ransac_iterations = 180;
};

struct SegmentCounts {
    std::size_t other = 0;
    std::size_t floor = 0;
    std::size_t wall = 0;
    std::size_t left_wall = 0;
    std::size_t right_wall = 0;
    std::size_t front_wall = 0;
};

struct SegmentedCloud {
    std::vector<LidarPoint>   points;
    std::vector<SegmentLabel> labels;

    SegmentCounts counts() const;
};

class LidarSegmenter {
public:
    explicit LidarSegmenter(LidarSegmentationOptions options = {});

    SegmentedCloud process(const std::vector<LidarPoint>& points) const;

private:
    LidarSegmentationOptions options_;
};

std::vector<LidarPoint> load_kitti_velodyne_bin(const std::string& path);
std::vector<LidarPoint> filter_roi(const std::vector<LidarPoint>& points,
                                     const LidarSegmentationOptions& options);
void write_segmented_ply(const std::string& path, const SegmentedCloud& cloud);

// ─── Stage D: RANSAC 평면 fitting ──────────────────────
struct Plane {
    double a = 0.0;
    double b = 0.0;
    double c = 0.0;
    double d = 0.0;
    // 평면 방정식:  a*x + b*y + c*z + d = 0
    // (a, b, c) 는 단위 법선벡터로 약속 (길이 = 1)
};

double point_to_plane_distance(const Plane& plane, const LidarPoint& p);

bool fit_plane_3pt(const LidarPoint& p1,
                     const LidarPoint& p2,
                     const LidarPoint& p3,
                     Plane& out_plane);
std::size_t count_inliers(const Plane& plane,
    const std::vector<LidarPoint>& points,
    double threshold);
bool ransac_plane(const std::vector<LidarPoint>& points,
            int iterations,
            double distance_threshold,
            Plane& out_plane,
            std::size_t& out_inlier_count);
// ─── Stage E: Floor 분리 ───────────────────────────────
bool segment_floor(const std::vector<LidarPoint>& points,
                    const LidarSegmentationOptions& opt,
                    Plane& out_plane,
                    std::vector<std::size_t>& out_inlier_indices);

// ─── Stage F: Wall 3 종 그리디 피링 ─────────────────────
bool segment_wall(const std::vector<LidarPoint>& points,
                const std::vector<bool>& used_mask,
                const LidarSegmentationOptions& opt,
                SegmentLabel target_wall,
                Plane& out_plane,
                std::vector<std::size_t>& out_inlier_indices);

void segment_all_walls(const std::vector<LidarPoint>& points,
                        const LidarSegmentationOptions& opt,
                        const std::vector<std::size_t>& floor_inliers,
                        std::vector<std::size_t>& out_left_inliers,
                        std::vector<std::size_t>& out_right_inliers,
                        std::vector<std::size_t>& out_front_inliers);