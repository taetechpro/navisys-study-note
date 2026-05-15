#include "my_lidar_segmenter.hpp"

#include<array>
#include<fstream>
#include<stdexcept>

std::vector<LidarPoint> load_kitti_velodyne_bin(const std::string& path) {
    // TODO B-2: 파일 열기 + 예외
        std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (!f.is_open()) {
        throw std::runtime_error("Cannot open KITTI Velodyne file: " + path);
    }
    // TODO B-3: 파일 크기 측정 + 점 개수 계산
    // TODO B-4: 점별 읽기 루프
    // TODO B-5: return
}