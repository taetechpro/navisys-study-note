#pragma once
#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>

struct ImuData {
    double timestamp;       // seconds
    Eigen::Vector3d gyro;   // rad/s, body (IMU) frame
    Eigen::Vector3d accel;  // m/s^2, body (IMU) frame
};

struct CamData {
    double timestamp;  // seconds
    std::string img_l; // absolute path to cam0 image
    std::string img_r; // absolute path to cam1 image
};

struct GtData {
    double timestamp;    // seconds
    Eigen::Vector3d p;   // position in world frame [m]
    Eigen::Quaterniond q; // orientation: world-from-body quaternion [w,x,y,z]
    Eigen::Vector3d v;   // velocity in world frame [m/s]
};

struct CameraParams {
    double fx, fy, cx, cy;          // intrinsics [px]
    double k1, k2, p1, p2;          // radtan distortion
    Eigen::Matrix4d T_cam_imu = Eigen::Matrix4d::Identity(); // T_CI: IMU → cam
    int width = 752, height = 480;
};

struct ImuNoiseParams {
    double gyro_noise  = 1.6968e-4; // σ_g  [rad/s/√Hz]
    double accel_noise = 2.0000e-3; // σ_a  [m/s^2/√Hz]
    double gyro_walk   = 1.9393e-5; // σ_bg [rad/s^2/√Hz]
    double accel_walk  = 3.0000e-3; // σ_ba [m/s^3/√Hz]
};
