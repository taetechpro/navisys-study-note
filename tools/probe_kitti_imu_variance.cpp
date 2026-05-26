// P6 cycle 3 H1 — KITTI IMU variance probe (read-only).
//
// Reports sliding-window IMU sample std (OV StaticInitializer-style):
//   a_var = sqrt( sum |a - a_avg|^2 / (N-1) )
//   w_var = sqrt( sum |w - w_avg|^2 / (N-1) )
//
// Cross-references OpenVINS `init_imu_thresh` reference values:
//   UZH FPV       0.30   (small drone, indoor)
//   TUM VI        0.45   (handheld, indoor)
//   KAIST/KITTI   0.50   (outdoor vehicle)
//   KAIST_VIO     0.60
//   EuRoC MAV     1.50
//
// Usage:
//   probe_kitti_imu <kitti_root> [duration_s=5.0] [window_s=0.5]

#include <algorithm>
#include <cstdio>
#include <iostream>
#include <string>
#include <vector>

#include "io/kitti_raw_reader.hpp"

namespace {

struct WindowStat {
    double t_center = 0.0;
    int n = 0;
    double a_var = 0.0;
    double w_var = 0.0;
    Eigen::Vector3d a_var_xyz = Eigen::Vector3d::Zero();
};

WindowStat compute_window(const std::vector<ImuData>& imu, double t_lo, double t_hi) {
    WindowStat s;
    s.t_center = 0.5 * (t_lo + t_hi);

    Eigen::Vector3d a_sum = Eigen::Vector3d::Zero();
    Eigen::Vector3d w_sum = Eigen::Vector3d::Zero();
    int n = 0;
    for (const auto& d : imu) {
        if (d.timestamp < t_lo || d.timestamp > t_hi) continue;
        a_sum += d.accel;
        w_sum += d.gyro;
        ++n;
    }
    if (n < 2) return s;
    const Eigen::Vector3d a_avg = a_sum / n;
    const Eigen::Vector3d w_avg = w_sum / n;

    double a_sq = 0.0;
    double w_sq = 0.0;
    Eigen::Vector3d a_sq_xyz = Eigen::Vector3d::Zero();
    for (const auto& d : imu) {
        if (d.timestamp < t_lo || d.timestamp > t_hi) continue;
        const Eigen::Vector3d da = d.accel - a_avg;
        const Eigen::Vector3d dw = d.gyro - w_avg;
        a_sq += da.dot(da);
        w_sq += dw.dot(dw);
        a_sq_xyz += da.cwiseProduct(da);
    }
    s.n = n;
    s.a_var = std::sqrt(a_sq / (n - 1));
    s.w_var = std::sqrt(w_sq / (n - 1));
    s.a_var_xyz = (a_sq_xyz / (n - 1)).cwiseSqrt();
    return s;
}

const char* marker_for(double a_var) {
    if (a_var < 0.30) return "<0.30 (UZH FPV pass)";
    if (a_var < 0.45) return "<0.45 (TUM VI pass)";
    if (a_var < 0.50) return "<0.50 (KAIST/KITTI pass)";
    if (a_var < 0.60) return "<0.60 (KAIST_VIO pass)";
    if (a_var < 1.00) return "<1.00 (OV default pass)";
    if (a_var < 1.50) return "<1.50 (EuRoC pass)";
    return ">=1.50 (FAIL all configs)";
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "usage: probe_kitti_imu <kitti_root> [duration_s=5.0] [window_s=0.5]\n";
        return 1;
    }
    const std::string root = argv[1];
    const double duration_s = (argc >= 3) ? std::stod(argv[2]) : 5.0;
    const double window_s = (argc >= 4) ? std::stod(argv[3]) : 0.5;

    KittiRawReader reader(root);
    const auto& imu = reader.imu();
    if (imu.size() < 4) {
        std::cerr << "not enough IMU samples (" << imu.size() << ")\n";
        return 1;
    }
    const double t0 = imu.front().timestamp;
    const double t_end = std::min(imu.back().timestamp, t0 + duration_s);

    std::printf("# kitti_root=%s  duration_s=%.2f  window_s=%.2f\n",
                root.c_str(), duration_s, window_s);
    std::printf("# imu_total=%zu  t0=%.3f  t_end=%.3f  rate~%.1f Hz\n",
                imu.size(), t0, t_end,
                (imu.size() - 1) / (imu.back().timestamp - imu.front().timestamp));
    std::printf("# OV gate ref:  UZH=0.30  TUM=0.45  KAIST=0.50  EuRoC=1.50  default=1.00\n");
    std::printf("# columns: t_rel  n   a_var   w_var   a_var_x  a_var_y  a_var_z   marker\n");

    const double slide_dt = 0.1;  // slide window center by 100 ms
    double first_below_05 = -1.0;
    double first_below_10 = -1.0;
    double first_below_15 = -1.0;
    int rows_below_05 = 0;
    int rows_total = 0;

    for (double t_center = t0 + 0.5 * window_s; t_center <= t_end; t_center += slide_dt) {
        const double t_lo = t_center - 0.5 * window_s;
        const double t_hi = t_center + 0.5 * window_s;
        const WindowStat s = compute_window(imu, t_lo, t_hi);
        if (s.n < 2) continue;
        ++rows_total;
        std::printf("  %.3f  %3d  %7.4f  %7.4f   %7.4f  %7.4f  %7.4f   %s\n",
                    t_center - t0, s.n, s.a_var, s.w_var,
                    s.a_var_xyz.x(), s.a_var_xyz.y(), s.a_var_xyz.z(),
                    marker_for(s.a_var));
        if (s.a_var < 0.5 && first_below_05 < 0) first_below_05 = t_center - t0;
        if (s.a_var < 1.0 && first_below_10 < 0) first_below_10 = t_center - t0;
        if (s.a_var < 1.5 && first_below_15 < 0) first_below_15 = t_center - t0;
        if (s.a_var < 0.5) ++rows_below_05;
    }

    std::printf("\n# summary:\n");
    std::printf("#   first_window_below_KAIST(0.5):   %s\n",
                first_below_05 < 0 ? "NEVER (in scanned range)" :
                                     (std::to_string(first_below_05) + " s rel to t0").c_str());
    std::printf("#   first_window_below_OV_default(1.0): %s\n",
                first_below_10 < 0 ? "NEVER" :
                                     (std::to_string(first_below_10) + " s rel to t0").c_str());
    std::printf("#   first_window_below_EuRoC(1.5):   %s\n",
                first_below_15 < 0 ? "NEVER" :
                                     (std::to_string(first_below_15) + " s rel to t0").c_str());
    std::printf("#   rows_below_KAIST_0.5:            %d / %d  (%.1f%%)\n",
                rows_below_05, rows_total,
                rows_total > 0 ? (100.0 * rows_below_05 / rows_total) : 0.0);
    return 0;
}
