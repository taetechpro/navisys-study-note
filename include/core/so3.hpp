#pragma once
#include <Eigen/Core>
#include <Eigen/Dense>
#include <cmath>

// SO(3) utilities: Exp, Log, hat operator
namespace so3 {

// Skew-symmetric matrix: hat(v) * u  ==  v × u
inline Eigen::Matrix3d hat(const Eigen::Vector3d& v) {
    Eigen::Matrix3d M;
    M <<     0, -v[2],  v[1],
          v[2],     0, -v[0],
         -v[1],  v[0],     0;
    return M;
}

// Exponential map: Rodrigues formula, Exp(ω) → R ∈ SO(3)
// For small ‖ω‖ falls back to first-order approximation.
inline Eigen::Matrix3d Exp(const Eigen::Vector3d& omega) {
    const double angle = omega.norm();
    if (angle < 1e-8)
        return Eigen::Matrix3d::Identity() + hat(omega);
    const Eigen::Vector3d ax = omega / angle;
    const double s = std::sin(angle), c = std::cos(angle);
    return c * Eigen::Matrix3d::Identity()
         + (1.0 - c) * ax * ax.transpose()
         + s * hat(ax);
}

// Logarithm map: Log(R) → ω ∈ ℝ³  (angle-axis vector)
inline Eigen::Vector3d Log(const Eigen::Matrix3d& R) {
    double cos_a = std::clamp((R.trace() - 1.0) / 2.0, -1.0, 1.0);
    const double angle = std::acos(cos_a);
    if (angle < 1e-8) return Eigen::Vector3d::Zero();
    return (angle / (2.0 * std::sin(angle))) *
           Eigen::Vector3d(R(2,1)-R(1,2), R(0,2)-R(2,0), R(1,0)-R(0,1));
}

} // namespace so3
