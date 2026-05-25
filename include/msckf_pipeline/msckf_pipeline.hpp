#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstddef>
#include <deque>
#include <memory>
#include <tuple>
#include <unordered_map>
#include <vector>

namespace ov_msckf {
class State;
class Propagator;
class UpdaterMSCKF;
struct UpdaterOptions;
}
namespace ov_core {
class Feature;
struct FeatureInitializerOptions;
}

// MsckfPipeline — adapter from this repo's stereo_tracker output to a
// monocular TC-MSCKF backend built from ported OpenVINS modules.
//
// Coordinate / convention notes:
//   * Hamilton on the boundary (Eigen::Matrix3d / Vector3d).
//   * JPL inside OpenVINS state: q_IG (IMU <- Global). We convert at the seam.
//   * "Global" = world frame supplied by the caller's stationary IMU init,
//     i.e. gravity-aligned, identical to the LC EKF's world frame.
//
// Frontend assumption: caller has already rectified the cam0 image and the
// (u, v) pixels are in the rectified frame. We register a CamRadtan with
// zero distortion and rectified intrinsics so OpenVINS treats the rectified
// stream as a single virtual camera.
class MsckfPipeline {
public:
    struct Pose {
        Eigen::Matrix3d R = Eigen::Matrix3d::Identity();  // R_world_imu (Hamilton, world <- body)
        Eigen::Vector3d p = Eigen::Vector3d::Zero();      // imu origin in world
        Eigen::Vector3d v = Eigen::Vector3d::Zero();      // imu velocity in world
        double timestamp = -1.0;
        bool valid = false;
    };

    struct TrackedFeat {
        std::size_t id;
        float u;  // rectified pixel
        float v;
    };

    MsckfPipeline(double t0,
                  const Eigen::Matrix3d& R0_wi,
                  const Eigen::Vector3d& p0,
                  const Eigen::Vector3d& v0,
                  const Eigen::Vector3d& bg0,
                  const Eigen::Vector3d& ba0,
                  const Eigen::Matrix4d& T_cam0_imu,
                  double fx_rect,
                  double fy_rect,
                  double cx_rect,
                  double cy_rect,
                  int img_width,
                  int img_height,
                  double gravity_mag = 9.81);

    ~MsckfPipeline();

    void feed_imu(double t,
                  const Eigen::Vector3d& gyro,
                  const Eigen::Vector3d& accel);

    // Feed one frame's tracked features (already rectified pixels). Triggers
    // propagation to t, clone augment, feature db update, MSCKF update on
    // features that just went out of view, and old-clone marginalization.
    void feed_camera(double t, const std::vector<TrackedFeat>& tracked);

    Pose latest_pose() const;
    int  num_clones() const;
    int  msckf_update_count() const { return msckf_updates_; }

private:
    std::shared_ptr<ov_msckf::State>    state_;
    std::unique_ptr<ov_msckf::Propagator> prop_;
    std::unique_ptr<ov_msckf::UpdaterMSCKF> updater_;

    // Owned options (UpdaterMSCKF/FeatureInitializer take non-const refs)
    std::unique_ptr<ov_msckf::UpdaterOptions>          upd_opts_;
    std::unique_ptr<ov_core::FeatureInitializerOptions> feat_opts_;

    // Adapter-owned feature DB. key = stereo_tracker track ID.
    std::unordered_map<std::size_t, std::shared_ptr<ov_core::Feature>> feature_db_;

    double fx_, fy_, cx_, cy_;
    int img_w_, img_h_;

    bool first_camera_ = true;
    int  msckf_updates_ = 0;

    // Cache for IMU boundary padding (Propagator needs a sample at/after cam.t)
    double          last_imu_t_ = -1.0;
    Eigen::Vector3d last_imu_w_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d last_imu_a_ = Eigen::Vector3d::Zero();
};
