#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

// Sensor-agnostic types shared by lidar/, depth/, and (planned) plane_constraint/ + msckf/ modules.
// 'LidarPoint' name is historical — used for both Velodyne points and stereo-depth points.

struct LidarPoint {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float intensity = 0.0f;
};

enum class SegmentLabel : std::uint8_t {
    Other = 0,
    Floor = 1,
    LeftWall = 2,
    RightWall = 3,
    FrontWall = 4,
    Wall = 5,
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
    std::vector<LidarPoint> points;
    std::vector<SegmentLabel> labels;

    SegmentCounts counts() const;
};
