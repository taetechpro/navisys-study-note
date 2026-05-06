#include "io/kitti_raw_reader.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace fs = std::filesystem;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kEarthRadiusM = 6378137.0;

std::string trim(const std::string& text) {
    const auto first = text.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) return "";
    const auto last = text.find_last_not_of(" \t\r\n");
    return text.substr(first, last - first + 1);
}

double parse_timestamp_seconds(const std::string& line) {
    const std::string text = trim(line);
    if (text.empty()) return -1.0;

    const auto pos = text.find(' ');
    const std::string time_text = (pos == std::string::npos) ? text : text.substr(pos + 1);

    const auto c0 = time_text.find(':');
    const auto c1 = time_text.find(':', c0 + 1);
    if (c0 == std::string::npos || c1 == std::string::npos) {
        throw std::runtime_error("Invalid KITTI timestamp: " + text);
    }

    const int hh = std::stoi(time_text.substr(0, c0));
    const int mm = std::stoi(time_text.substr(c0 + 1, c1 - c0 - 1));
    const double ss = std::stod(time_text.substr(c1 + 1));
    return 3600.0 * hh + 60.0 * mm + ss;
}

std::vector<double> load_timestamps(const fs::path& path) {
    std::ifstream f(path);
    if (!f.is_open()) {
        throw std::runtime_error("Cannot open timestamp file: " + path.string());
    }

    std::vector<double> timestamps;
    std::string line;
    while (std::getline(f, line)) {
        const double ts = parse_timestamp_seconds(line);
        if (ts >= 0.0) timestamps.push_back(ts);
    }
    return timestamps;
}

std::vector<fs::path> collect_files(const fs::path& dir, const std::string& ext) {
    if (!fs::exists(dir)) {
        throw std::runtime_error("Missing directory: " + dir.string());
    }

    std::vector<fs::path> files;
    for (const auto& entry : fs::directory_iterator(dir)) {
        if (entry.is_regular_file() && entry.path().extension() == ext) {
            files.push_back(entry.path());
        }
    }

    std::sort(files.begin(), files.end(),
              [](const fs::path& a, const fs::path& b) {
                  return a.filename().string() < b.filename().string();
              });
    return files;
}

std::pair<fs::path, fs::path> select_stereo_roots(const fs::path& root) {
    const std::pair<fs::path, fs::path> gray{root / "image_00", root / "image_01"};
    if (fs::exists(gray.first / "data") && fs::exists(gray.second / "data")) {
        return gray;
    }

    const std::pair<fs::path, fs::path> color{root / "image_02", root / "image_03"};
    if (fs::exists(color.first / "data") && fs::exists(color.second / "data")) {
        return color;
    }

    throw std::runtime_error("Cannot find KITTI stereo image folders under " + root.string());
}

std::vector<double> load_numeric_tokens(const fs::path& path) {
    std::ifstream f(path);
    if (!f.is_open()) {
        throw std::runtime_error("Cannot open OXTS file: " + path.string());
    }

    std::vector<double> values;
    double value = 0.0;
    while (f >> value) values.push_back(value);
    return values;
}

double mercator_x(double lon_deg, double scale) {
    return scale * lon_deg * kPi * kEarthRadiusM / 180.0;
}

double mercator_y(double lat_deg, double scale) {
    return scale * kEarthRadiusM *
           std::log(std::tan((90.0 + lat_deg) * kPi / 360.0));
}

Eigen::Matrix3d rot_from_rpy(double roll, double pitch, double yaw) {
    const Eigen::AngleAxisd rx(roll, Eigen::Vector3d::UnitX());
    const Eigen::AngleAxisd ry(pitch, Eigen::Vector3d::UnitY());
    const Eigen::AngleAxisd rz(yaw, Eigen::Vector3d::UnitZ());
    return (rz * ry * rx).toRotationMatrix();
}

} // namespace

KittiRawReader::KittiRawReader(const std::string& root) {
    load_cam(root);
    load_oxts(root);
    load_velodyne(root);

    std::cout << "[KITTI raw] IMU=" << imu_.size()
              << "  CAM=" << cam_.size()
              << "  LIDAR=" << lidar_.size()
              << "  GT=" << gt_.size() << "\n";
}

void KittiRawReader::load_cam(const std::string& root) {
    const auto stereo_roots = select_stereo_roots(fs::path(root));
    const auto left_ts = load_timestamps(stereo_roots.first / "timestamps.txt");
    const auto right_ts = load_timestamps(stereo_roots.second / "timestamps.txt");
    const auto left_files = collect_files(stereo_roots.first / "data", ".png");
    const auto right_files = collect_files(stereo_roots.second / "data", ".png");

    const size_t count = std::min(
        std::min(left_ts.size(), right_ts.size()),
        std::min(left_files.size(), right_files.size()));

    if (count == 0) {
        throw std::runtime_error("No KITTI stereo frames found under " + root);
    }

    for (size_t i = 0; i < count; ++i) {
        CamData d;
        d.timestamp = left_ts[i];
        d.img_l = fs::absolute(left_files[i]).string();
        d.img_r = fs::absolute(right_files[i]).string();
        cam_.push_back(d);
    }
}

void KittiRawReader::load_velodyne(const std::string& root) {
    const fs::path velodyne_dir = fs::path(root) / "velodyne_points" / "data";
    if (!fs::exists(velodyne_dir)) {
        return;
    }

    const auto files = collect_files(velodyne_dir, ".bin");
    lidar_.reserve(files.size());
    for (const auto& file : files) {
        lidar_.push_back(fs::absolute(file).string());
    }
}

void KittiRawReader::load_oxts(const std::string& root) {
    const fs::path oxts_root = fs::path(root) / "oxts";
    const auto timestamps = load_timestamps(oxts_root / "timestamps.txt");
    const auto files = collect_files(oxts_root / "data", ".txt");
    const size_t count = std::min(timestamps.size(), files.size());

    if (count == 0) {
        throw std::runtime_error("No KITTI OXTS packets found under " + oxts_root.string());
    }

    bool have_origin = false;
    double mercator_scale = 1.0;
    Eigen::Vector3d origin = Eigen::Vector3d::Zero();

    for (size_t i = 0; i < count; ++i) {
        const auto values = load_numeric_tokens(files[i]);
        if (values.size() < 23) continue;

        const double lat = values[0];
        const double lon = values[1];
        const double alt = values[2];
        const double roll = values[3];
        const double pitch = values[4];
        const double yaw = values[5];

        if (!have_origin) {
            mercator_scale = std::cos(lat * kPi / 180.0);
            origin = Eigen::Vector3d(
                mercator_x(lon, mercator_scale),
                mercator_y(lat, mercator_scale),
                alt);
            have_origin = true;
        }

        ImuData imu;
        imu.timestamp = timestamps[i];
        imu.gyro = Eigen::Vector3d(values[20], values[21], values[22]); // forward-left-up
        imu.accel = Eigen::Vector3d(values[14], values[15], values[16]); // forward-left-up
        imu_.push_back(imu);

        GtData gt;
        gt.timestamp = timestamps[i];
        gt.p = Eigen::Vector3d(
            mercator_x(lon, mercator_scale) - origin.x(),
            mercator_y(lat, mercator_scale) - origin.y(),
            alt - origin.z());
        gt.q = Eigen::Quaterniond(rot_from_rpy(roll, pitch, yaw));
        gt.q.normalize();
        gt.v = Eigen::Vector3d(values[7], values[6], values[10]); // east, north, up
        gt_.push_back(gt);
    }

    if (!have_origin) {
        throw std::runtime_error("Failed to parse any valid KITTI OXTS packets under " +
                                 oxts_root.string());
    }

    std::sort(imu_.begin(), imu_.end(),
              [](const ImuData& a, const ImuData& b) { return a.timestamp < b.timestamp; });
    std::sort(gt_.begin(), gt_.end(),
              [](const GtData& a, const GtData& b) { return a.timestamp < b.timestamp; });
}
