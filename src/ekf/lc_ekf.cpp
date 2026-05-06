#include "ekf/lc_ekf.hpp"
#include "core/so3.hpp"
#include <iostream>

using Eigen::Matrix3d;
using Eigen::Vector3d;
using Mat15 = Eigen::Matrix<double, 15, 15>;
using Vec15 = Eigen::Matrix<double, 15, 1>;

LcEkf::LcEkf(ImuPropagator& imu,
              const Eigen::Matrix4d& T_cam0_imu,
              double sigma_vo)
    : imu_(imu), sigma_vo_(sigma_vo)
{
    // Camera origin in IMU body frame: T_imu_cam0 * [0,0,0,1]ᵀ
    Eigen::Matrix4d T_imu_cam0 = T_cam0_imu.inverse();
    p_IC_ = T_imu_cam0.block<3,1>(0,3);
}

void LcEkf::propagate(double t,
                       const Vector3d& gyro,
                       const Vector3d& accel) {
    imu_.propagate(t, gyro, accel);
}

bool LcEkf::update_vo(const Vector3d& p_world_cam0) {
    // Predicted measurement: h = R_wI * p_IC + p_wI
    const Matrix3d& R_wI = imu_.R;
    const Vector3d& p_wI = imu_.p;
    Vector3d h = R_wI * p_IC_ + p_wI;

    // Innovation
    Vector3d innov = p_world_cam0 - h;

    // (debug removed)

    // Measurement Jacobian H (3×15)
    // h = R_wI * Exp(δθ) * p_IC + p_wI
    //   ≈ R_wI*(I+[δθ]×)*p_IC + p_wI = h₀ + R_wI*([δθ]×*p_IC) + δp
    // [δθ]×*p_IC = δθ×p_IC = -[p_IC]×*δθ
    // ∂h/∂δp = I₃,   ∂h/∂δθ = -R_wI * [p_IC]×
    Eigen::Matrix<double, 3, 15> H = Eigen::Matrix<double, 3, 15>::Zero();
    H.block<3,3>(0,0) = Matrix3d::Identity();              // ∂h/∂δp = I
    H.block<3,3>(0,6) = -R_wI * so3::hat(p_IC_);          // ∂h/∂δθ = -R*[p_IC]×

    // Innovation covariance S = H·P·Hᵀ + R
    Eigen::Matrix3d R_noise = (sigma_vo_*sigma_vo_) * Matrix3d::Identity();
    Eigen::Matrix3d S = H * imu_.P * H.transpose() + R_noise;

    // Kalman gain K = P·Hᵀ·S⁻¹
    Eigen::Matrix<double, 15, 3> K = imu_.P * H.transpose() * S.inverse();

    // Error-state correction δx = K·innov
    Vec15 dx = K * innov;

    // Apply correction
    apply_correction(dx);

    // Joseph form covariance update (numerically stable)
    Mat15 I_KH = Mat15::Identity() - K * H;
    imu_.P = I_KH * imu_.P * I_KH.transpose()
           + K * R_noise * K.transpose();

    double innov_norm = innov.norm();
    if (innov_norm > 5.0)
        std::cerr << "[EKF] Large VO innovation: " << innov_norm << " m\n";

    return true;
}

void LcEkf::apply_correction(const Vec15& dx) {
    imu_.p  += dx.segment<3>(0);
    imu_.v  += dx.segment<3>(3);
    imu_.R   = imu_.R * so3::Exp(dx.segment<3>(6));
    imu_.R   = Eigen::Quaterniond(imu_.R).normalized().toRotationMatrix();
    imu_.bg += dx.segment<3>(9);
    imu_.ba += dx.segment<3>(12);
}
