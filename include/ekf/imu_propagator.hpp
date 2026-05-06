#pragma once
#include <Eigen/Core>
#include <Eigen/Dense>
#include "core/types.hpp"

// 15-state error-state IMU propagator.
//
// Nominal state: p(3), v(3), R(3×3), b_g(3), b_a(3)
// Error  state:  δx = [δp, δv, δθ, δb_g, δb_a]  ∈ ℝ¹⁵
//
// Propagation via RK4 for the nominal state and
// first-order linearised ODE for the error-state covariance.
//
// Reference: Trawny & Roumeliotis, "Indirect Kalman Filter for 3D
//            Attitude Estimation", UMN Tech Report 2005.
class ImuPropagator {
public:
    // Nominal (best-estimate) state
    Eigen::Vector3d p;   // world frame [m]
    Eigen::Vector3d v;   // world frame [m/s]
    Eigen::Matrix3d R;   // world ← body (IMU)
    Eigen::Vector3d bg;  // gyro bias  [rad/s]
    Eigen::Vector3d ba;  // accel bias [m/s^2]

    // Error-state covariance  (15×15)
    using Mat15 = Eigen::Matrix<double, 15, 15>;
    Mat15 P;

    ImuPropagator(const Eigen::Vector3d& p0,
                  const Eigen::Vector3d& v0,
                  const Eigen::Matrix3d& R0,
                  const Eigen::Vector3d& bg0,
                  const Eigen::Vector3d& ba0,
                  const ImuNoiseParams&  noise,
                  const Eigen::Vector3d& g_world = Eigen::Vector3d(0,0,-9.81));

    // Integrate IMU measurement from t_prev to t_new.
    // Call once per IMU sample; handles dt internally.
    void propagate(double t_new,
                   const Eigen::Vector3d& gyro_raw,
                   const Eigen::Vector3d& accel_raw);

private:
    double t_prev_ = -1.0;
    ImuNoiseParams noise_;
    Eigen::Vector3d g_;   // gravity in world frame, set from constructor

    // RK4 nominal state derivative: [ṗ, v̇, Ṙ≡ω×, ḃ_g, ḃ_a]
    struct State { Eigen::Vector3d p, v; Eigen::Matrix3d R; };
    State state_derivative(const State& s,
                           const Eigen::Vector3d& gyro_corr,
                           const Eigen::Vector3d& accel_corr) const;

    // Advance nominal state by dt via RK4
    void rk4_step(double dt,
                  const Eigen::Vector3d& gyro_corr,
                  const Eigen::Vector3d& accel_corr);

    // Propagate covariance: P ← F·P·Fᵀ + G·Qd·Gᵀ (first-order Euler)
    void propagate_cov(double dt,
                       const Eigen::Vector3d& gyro_corr,
                       const Eigen::Vector3d& accel_corr);
};
