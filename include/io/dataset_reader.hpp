#pragma once

#include <vector>

#include "core/types.hpp"

class DatasetReader {
public:
    virtual ~DatasetReader() = default;

    const std::vector<ImuData>& imu() const { return imu_; }
    const std::vector<CamData>& cam() const { return cam_; }
    const std::vector<GtData>& gt() const { return gt_; }
    const std::vector<std::string>& lidar() const { return lidar_; }

protected:
    std::vector<ImuData> imu_;
    std::vector<CamData> cam_;
    std::vector<GtData> gt_;
    std::vector<std::string> lidar_;
};
