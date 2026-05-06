#include "io/euroc_reader.hpp"

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <algorithm>
#include <iostream>

// ---- helpers ----

static std::vector<std::string> split_csv(const std::string& line) {
    std::vector<std::string> tokens;
    std::stringstream ss(line);
    std::string tok;
    while (std::getline(ss, tok, ',')) {
        // strip whitespace
        tok.erase(0, tok.find_first_not_of(" \t"));
        tok.erase(tok.find_last_not_of(" \t\r\n") + 1);
        tokens.push_back(tok);
    }
    return tokens;
}

// ---- EuRoCReader ----

EuRoCReader::EuRoCReader(const std::string& root) {
    load_imu(root);
    load_cam(root);
    load_gt(root);
    std::cout << "[EuRoC] IMU=" << imu_.size()
              << "  CAM=" << cam_.size()
              << "  GT=" << gt_.size() << "\n";
}

void EuRoCReader::load_imu(const std::string& root) {
    const std::string path = root + "/imu0/data.csv";
    std::ifstream f(path);
    if (!f.is_open())
        throw std::runtime_error("Cannot open IMU CSV: " + path);

    std::string line;
    std::getline(f, line); // skip header
    while (std::getline(f, line)) {
        if (line.empty() || line[0] == '#') continue;
        auto tok = split_csv(line);
        if (tok.size() < 7) continue;

        ImuData d;
        d.timestamp = std::stod(tok[0]) * 1e-9; // ns → s
        d.gyro  = {std::stod(tok[1]), std::stod(tok[2]), std::stod(tok[3])};
        d.accel = {std::stod(tok[4]), std::stod(tok[5]), std::stod(tok[6])};
        imu_.push_back(d);
    }
    std::sort(imu_.begin(), imu_.end(),
              [](const ImuData& a, const ImuData& b){ return a.timestamp < b.timestamp; });
}

void EuRoCReader::load_cam(const std::string& root) {
    // Load cam0 and cam1 CSV, then pair by nearest timestamp (within 5 ms).
    auto load_csv = [&](const std::string& cam) {
        const std::string csv = root + "/" + cam + "/data.csv";
        std::ifstream f(csv);
        if (!f.is_open())
            throw std::runtime_error("Cannot open cam CSV: " + csv);

        std::vector<std::pair<double, std::string>> entries;
        std::string line;
        std::getline(f, line); // skip header
        while (std::getline(f, line)) {
            if (line.empty() || line[0] == '#') continue;
            auto tok = split_csv(line);
            if (tok.size() < 2) continue;
            double ts = std::stod(tok[0]) * 1e-9;
            std::string img_path = root + "/" + cam + "/data/" + tok[1];
            entries.push_back({ts, img_path});
        }
        std::sort(entries.begin(), entries.end(),
                  [](auto& a, auto& b){ return a.first < b.first; });
        return entries;
    };

    auto cam0 = load_csv("cam0");
    auto cam1 = load_csv("cam1");

    // Camera period is 50 ms (20 Hz); use 25 ms as pairing tolerance.
    const double max_dt = 25e-3;
    for (auto& [ts0, path0] : cam0) {
        // Binary search for nearest cam1 timestamp
        auto it = std::lower_bound(cam1.begin(), cam1.end(), ts0,
                                   [](const std::pair<double,std::string>& a, double t){
                                       return a.first < t; });
        double best_dt = 1e9;
        decltype(it) best = cam1.end();

        if (it != cam1.end()) {
            double dt = std::abs(it->first - ts0);
            if (dt < best_dt) { best_dt = dt; best = it; }
        }
        if (it != cam1.begin()) {
            --it;
            double dt = std::abs(it->first - ts0);
            if (dt < best_dt) { best_dt = dt; best = it; }
        }

        if (best != cam1.end() && best_dt < max_dt) {
            CamData d;
            d.timestamp = ts0;
            d.img_l = path0;
            d.img_r = best->second;
            cam_.push_back(d);
        }
    }
    std::sort(cam_.begin(), cam_.end(),
              [](const CamData& a, const CamData& b){ return a.timestamp < b.timestamp; });
}

void EuRoCReader::load_gt(const std::string& root) {
    const std::string path = root + "/state_groundtruth_estimate0/data.csv";
    std::ifstream f(path);
    if (!f.is_open()) {
        std::cerr << "[EuRoC] Warning: no GT found at " << path << "\n";
        return;
    }

    std::string line;
    std::getline(f, line); // skip header
    while (std::getline(f, line)) {
        if (line.empty() || line[0] == '#') continue;
        auto tok = split_csv(line);
        if (tok.size() < 11) continue;

        GtData d;
        d.timestamp = std::stod(tok[0]) * 1e-9;
        d.p = {std::stod(tok[1]), std::stod(tok[2]), std::stod(tok[3])};
        // quaternion order in CSV: w, x, y, z
        d.q = Eigen::Quaterniond(std::stod(tok[4]),  // w
                                  std::stod(tok[5]),  // x
                                  std::stod(tok[6]),  // y
                                  std::stod(tok[7])); // z
        d.q.normalize();
        d.v = {std::stod(tok[8]), std::stod(tok[9]), std::stod(tok[10])};
        gt_.push_back(d);
    }
    std::sort(gt_.begin(), gt_.end(),
              [](const GtData& a, const GtData& b){ return a.timestamp < b.timestamp; });
}
