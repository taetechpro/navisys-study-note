#pragma once
#include <string>

#include "io/dataset_reader.hpp"

// Reads an EuRoC ASL-format dataset directory.
// Expected layout:
//   <root>/
//     imu0/data.csv
//     cam0/data.csv  +  cam0/data/<timestamp>.png
//     cam1/data.csv  +  cam1/data/<timestamp>.png
//     state_groundtruth_estimate0/data.csv
class EuRoCReader : public DatasetReader {
public:
    explicit EuRoCReader(const std::string& dataset_root);

private:
    void load_imu(const std::string& root);
    void load_cam(const std::string& root);
    void load_gt (const std::string& root);
};
