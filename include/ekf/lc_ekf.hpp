#pragma once
#include <Eigen/Core>
#include <Eigen/Dense>
#include "ekf/imu_propagator.hpp"
#include "core/types.hpp"

// Loosely-Coupled EKF: wraps ImuPropagator and corrects it
// with VO position measurements.
//
// Measurement model:
//   z = p_world_cam0 = R_wI * p_IC + p_wI   + noise
// where p_IC = camera origin in IMU/body frame (lever arm),
//       p_wI = IMU position (EKF state),
//       R_wI = IMU orientation (EKF state).
//
// After computing the Kalman gain K and innovation δz, the
// error-state correction δx is applied and the state is reset.
class LcEkf {
public:
    LcEkf(ImuPropagator& imu,
          const Eigen::Matrix4d& T_cam0_imu,
          double sigma_vo = 0.5);

    void propagate(double t,
                   const Eigen::Vector3d& gyro,
                   const Eigen::Vector3d& accel);

    // Feed a VO position measurement (world frame, cam0 origin).
    // Returns false and skips update if covariance is ill-conditioned.
    bool update_vo(const Eigen::Vector3d& p_world_cam0);

    Eigen::Vector3d position()    const { return imu_.p; }
    Eigen::Vector3d velocity()    const { return imu_.v; }
    Eigen::Matrix3d orientation() const { return imu_.R; }

private:
    ImuPropagator& imu_;
    Eigen::Vector3d p_IC_;  // cam0 origin in IMU body frame
    double sigma_vo_;

    // Apply δx = [δp, δv, δθ, δbg, δba] to nominal state
    void apply_correction(const Eigen::Matrix<double,15,1>& dx);
};
