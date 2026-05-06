#include "ekf/imu_propagator.hpp"
#include "core/so3.hpp"

using Eigen::Matrix3d;
using Eigen::Vector3d;
using Mat15 = Eigen::Matrix<double, 15, 15>;
using Vec15 = Eigen::Matrix<double, 15, 1>;

ImuPropagator::ImuPropagator(const Vector3d& p0,
                              const Vector3d& v0,
                              const Matrix3d& R0,
                              const Vector3d& bg0,
                              const Vector3d& ba0,
                              const ImuNoiseParams& noise,
                              const Vector3d& g_world)
    : p(p0), v(v0), R(R0), bg(bg0), ba(ba0), noise_(noise), g_(g_world)
{
    P.setZero();
    // Initial uncertainty: large for velocity/attitude, small for position
    P.block<3,3>(0,0)  = 1e-4 * Matrix3d::Identity(); // position
    P.block<3,3>(3,3)  = 1e-2 * Matrix3d::Identity(); // velocity
    P.block<3,3>(6,6)  = 1e-4 * Matrix3d::Identity(); // attitude
    P.block<3,3>(9,9)  = 1e-6 * Matrix3d::Identity(); // gyro bias
    P.block<3,3>(12,12)= 1e-4 * Matrix3d::Identity(); // accel bias
}

// ---- RK4 state derivative ----

ImuPropagator::State
ImuPropagator::state_derivative(const State& s,
                                 const Vector3d& gyro_c,
                                 const Vector3d& accel_c) const {
    State ds;
    ds.p = s.v;
    ds.v = s.R * accel_c + g_;        // world-frame accel (no bias, already subtracted)
    ds.R = s.R * so3::hat(gyro_c);    // Ṙ = R * [ω]×
    return ds;
}

void ImuPropagator::rk4_step(double dt,
                              const Vector3d& gyro_c,
                              const Vector3d& accel_c) {
    State s0{p, v, R};

    auto k1 = state_derivative(s0, gyro_c, accel_c);

    State s1{p + 0.5*dt*k1.p,
             v + 0.5*dt*k1.v,
             R * so3::Exp(0.5*dt*gyro_c)};
    auto k2 = state_derivative(s1, gyro_c, accel_c);

    State s2{p + 0.5*dt*k2.p,
             v + 0.5*dt*k2.v,
             R * so3::Exp(0.5*dt*gyro_c)};
    auto k3 = state_derivative(s2, gyro_c, accel_c);

    State s3{p + dt*k3.p,
             v + dt*k3.v,
             R * so3::Exp(dt*gyro_c)};
    auto k4 = state_derivative(s3, gyro_c, accel_c);

    p = p + (dt/6.0) * (k1.p + 2*k2.p + 2*k3.p + k4.p);
    v = v + (dt/6.0) * (k1.v + 2*k2.v + 2*k3.v + k4.v);
    R = R * so3::Exp(dt * gyro_c);
    R = Eigen::Quaterniond(R).normalized().toRotationMatrix(); // re-ortho
}

// ---- Covariance propagation ----
//
// Error-state ODE (continuous):
//   d/dt δx = F·δx + G·n,  n ~ N(0, Q_c)
//
// Block layout of δx = [δp(3), δv(3), δθ(3), δbg(3), δba(3)]:
//
//   F = | 0   I   0        0    0  |
//       | 0   0  -R[a×]   0   -R  |
//       | 0   0  -[ω×]   -R    0  |  (← using "small angle" for δbg effect on δθ)
//       | 0   0   0        0    0  |
//       | 0   0   0        0    0  |
//
//   G = | 0  0  0  0 |
//       | 0  R  0  0 |   (accel noise → δv)
//       | R  0  0  0 |   (gyro  noise → δθ)
//       | 0  0  I  0 |   (gyro  walk  → δbg)
//       | 0  0  0  I |   (accel walk  → δba)
//
// Discrete (Euler): P ← (I+F·dt)·P·(I+F·dt)ᵀ + G·Qd·Gᵀ

void ImuPropagator::propagate_cov(double dt,
                                   const Vector3d& gyro_c,
                                   const Vector3d& accel_c) {
    // Continuous-time F (15×15)
    Mat15 F = Mat15::Zero();
    F.block<3,3>(0,3)  =  Matrix3d::Identity();          // δṗ = δv
    F.block<3,3>(3,6)  = -R * so3::hat(accel_c);         // δv̇ ← -R[a×]δθ
    F.block<3,3>(3,12) = -R;                             // δv̇ ← -R·δba
    F.block<3,3>(6,6)  = -so3::hat(gyro_c);              // δθ̇ ← -[ω×]δθ
    F.block<3,3>(6,9)  = -R;                             // δθ̇ ← -R·δbg  (approx = -I for small angles, but -R is correct)
    // biases: Ḃg = 0, Ḃa = 0

    // Discrete transition (first-order Euler)
    Mat15 Phi = Mat15::Identity() + F * dt;

    // Continuous-time process noise density matrix Qc (12×12)
    // channels: gyro_noise(3), accel_noise(3), gyro_walk(3), accel_walk(3)
    Eigen::Matrix<double, 15, 12> G = Eigen::Matrix<double, 15, 12>::Zero();
    G.block<3,3>(3,3)  = R;                              // accel noise → δv
    G.block<3,3>(6,0)  = R;                              // gyro  noise → δθ
    G.block<3,3>(9,6)  = Matrix3d::Identity();           // gyro  walk  → δbg
    G.block<3,3>(12,9) = Matrix3d::Identity();           // accel walk  → δba

    double sg = noise_.gyro_noise,  sa = noise_.accel_noise;
    double sbg = noise_.gyro_walk,  sba = noise_.accel_walk;

    Eigen::Matrix<double, 12, 12> Qc = Eigen::Matrix<double, 12, 12>::Zero();
    Qc.block<3,3>(0,0)  = sg*sg  * Matrix3d::Identity();
    Qc.block<3,3>(3,3)  = sa*sa  * Matrix3d::Identity();
    Qc.block<3,3>(6,6)  = sbg*sbg* Matrix3d::Identity();
    Qc.block<3,3>(9,9)  = sba*sba* Matrix3d::Identity();

    // Discrete noise: Qd = G·Qc·Gᵀ·dt
    Mat15 Qd = (G * Qc * G.transpose()) * dt;

    P = Phi * P * Phi.transpose() + Qd;
}

// ---- public: propagate ----

void ImuPropagator::propagate(double t_new,
                               const Vector3d& gyro_raw,
                               const Vector3d& accel_raw) {
    if (t_prev_ < 0.0) {
        t_prev_ = t_new;
        return;
    }
    double dt = t_new - t_prev_;
    if (dt <= 0.0 || dt > 0.5) { // sanity gate
        t_prev_ = t_new;
        return;
    }
    t_prev_ = t_new;

    // Subtract current bias estimates
    Vector3d gyro_c  = gyro_raw  - bg;
    Vector3d accel_c = accel_raw - ba;

    rk4_step(dt, gyro_c, accel_c);
    propagate_cov(dt, gyro_c, accel_c);
}
