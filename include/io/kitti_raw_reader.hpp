#pragma once

#include <string>

#include "io/dataset_reader.hpp"

// Reads a KITTI raw sync directory.
// Expected layout:
//   <root>/
//     image_00/data/*.png  + image_00/timestamps.txt
//     image_01/data/*.png  + image_01/timestamps.txt
//     oxts/data/*.txt      + oxts/timestamps.txt
class KittiRawReader : public DatasetReader {
public:
    explicit KittiRawReader(const std::string& dataset_root);

private:
    void load_cam(const std::string& root);
    void load_oxts(const std::string& root);
    void load_velodyne(const std::string& root);
};
