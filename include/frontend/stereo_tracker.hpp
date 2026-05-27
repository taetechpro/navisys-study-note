#pragma once
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include "core/types.hpp"

// Sparse stereo VO: KLT tracking + stereo triangulation + PnP pose estimation.
//
// Invariant: prev_pts_l_.size() == map_pts_world_.size() at all times.
// Every element pair (prev_pts_l_[i], map_pts_world_[i]) is a valid 2D-3D correspondence.
//
// World frame = cam0 frame of the very first processed image.
class StereoTracker {
public:
    struct Pose {
        Eigen::Matrix3d R = Eigen::Matrix3d::Identity(); // world ← cam0
        Eigen::Vector3d t = Eigen::Vector3d::Zero();     // cam0 origin in world
        bool valid = false;
    };

    StereoTracker(const CameraParams& cam0, const CameraParams& cam1);

    Pose process(const cv::Mat& img_l, const cv::Mat& img_r);

    int tracked_count() const { return static_cast<int>(prev_pts_l_.size()); }
    const std::vector<cv::Point2f>& tracked_points() const { return prev_pts_l_; }
    const std::vector<std::size_t>& tracked_ids() const { return prev_track_ids_; }
    // Stereo accessors (cycle 5): cam1 (right rectified) pixel for each track.
    // Invariant: same size as tracked_points(); stereo_valid()[i] == false means
    // the right match was lost this frame (use cam0 measurement only).
    const std::vector<cv::Point2f>& tracked_points_right() const { return prev_pts_r_; }
    const std::vector<bool>&        stereo_valid()        const { return stereo_valid_; }
    const cv::Mat& rectified_left_image() const { return prev_img_l_; }
    const cv::Mat& rectified_right_image() const { return prev_img_r_; }
    double rectified_fx() const { return fx_rect_; }
    double rectified_fy() const { return fy_rect_; }
    double rectified_cx() const { return cx_rect_; }
    double rectified_cy() const { return cy_rect_; }
    double rectified_baseline() const { return baseline_rect_; }
    Eigen::Matrix3d rectified_to_cam0_rotation() const;

private:
    // ---- camera params ----
    cv::Mat K0_, K1_, dist0_, dist1_;
    cv::Mat P0_, P1_;              // projection matrices for triangulatePoints (rectified)
    cv::Mat F_;                    // fundamental matrix (not used after rectification)
    Eigen::Matrix4d T_cam1_cam0_;  // cam1 from cam0 transform

    // ---- stereo rectification ----
    cv::Mat R0_rect_, R1_rect_;    // rotation to rectified frame (cam0, cam1)
    cv::Mat map0x_, map0y_;        // cam0 undistort+rectify maps
    cv::Mat map1x_, map1y_;        // cam1 undistort+rectify maps
    cv::Mat K_rect_;               // 3×3 rectified camera matrix
    cv::Mat zero_dist_;            // zero distortion (rectified space)
    double baseline_rect_;         // rectified baseline [m]
    double fx_rect_;               // rectified focal length [px]
    double fy_rect_;               // rectified focal length [px]
    double cx_rect_;               // rectified principal point [px]
    double cy_rect_;               // rectified principal point [px]

    // ---- ORB descriptor matching (stereo) ----
    cv::Ptr<cv::ORB> orb_;
    cv::Ptr<cv::BFMatcher> matcher_;

    // ---- tracker state (always aligned: size N) ----
    cv::Mat prev_img_l_;
    cv::Mat prev_img_r_;
    std::vector<cv::Point2f> prev_pts_l_;         // [N] left pixel coords (rectified)
    std::vector<cv::Point2f> prev_pts_r_;         // [N] right pixel coords (rectified, invalid when !stereo_valid_[i])
    std::vector<bool>        stereo_valid_;       // [N] whether prev_pts_r_[i] holds a real cam1 match
    std::vector<Eigen::Vector3d> map_pts_world_;  // [N] 3D in world frame
    std::vector<std::size_t> prev_track_ids_;     // [N] persistent feature IDs

    std::size_t next_id_ = 0;                     // monotonic ID source

    Pose curr_pose_;
    bool initialized_ = false;

    int target_features_ = 250;
    int min_pnp_inliers_ = 10;

    // ---- helpers ----
    // KLT optical flow from src to dst; returns tracked positions, sets status
    std::vector<cv::Point2f> klt_track(const cv::Mat& src,
                                        const cv::Mat& dst,
                                        const std::vector<cv::Point2f>& pts_src,
                                        std::vector<uchar>& status);

    // Stereo-specific KLT: initialises search in right image at expected disparity
    // offset to avoid the zero-disparity local minimum.
    std::vector<cv::Point2f> stereo_klt(const cv::Mat& img_l,
                                         const cv::Mat& img_r,
                                         const std::vector<cv::Point2f>& pts_l,
                                         std::vector<uchar>& status);

    float disparity_offset_px_ = 6.0f;  // initial guess: fx*baseline/z_expected

    // Undistort pixel coords using camera K and distortion coeffs
    std::vector<cv::Point2f> undistort(const std::vector<cv::Point2f>& px,
                                        const cv::Mat& K,
                                        const cv::Mat& dist) const;

    // Triangulate stereo point pairs → 3D in cam0 frame
    std::vector<Eigen::Vector3d> stereo_triangulate(
        const std::vector<cv::Point2f>& ud_l,
        const std::vector<cv::Point2f>& ud_r) const;

    // solvePnPRansac wrapper; returns false if not enough inliers
    bool solve_pnp(const std::vector<Eigen::Vector3d>& pts3d,
                   const std::vector<cv::Point2f>& pts2d,
                   Eigen::Matrix3d& R_cw, Eigen::Vector3d& t_cw,
                   std::vector<int>& inlier_idx) const;

    // FAST feature detection avoiding existing feature positions
    void detect_features(const cv::Mat& gray,
                         const std::vector<cv::Point2f>& existing,
                         int n_needed,
                         std::vector<cv::Point2f>& out);

    // ORB-based stereo matching on rectified images. Detects ORB keypoints in
    // both images, matches via Hamming-distance brute-force with ratio test,
    // and filters by epipolar constraint (|dv|<2px) and disparity sign.
    // Returns matched (pts_l, pts_r) with 1:1 correspondence.
    struct StereoMatch {
        std::vector<cv::Point2f> pts_l;
        std::vector<cv::Point2f> pts_r;
    };
    StereoMatch orb_stereo_match(const cv::Mat& img_l,
                                  const cv::Mat& img_r,
                                  const std::vector<cv::Point2f>& avoid,
                                  int max_features);

    // Epipolar constraint filter: set status[i]=0 if right point is too far
    // from its epipolar line in cam1 (uses undistorted pixel coords + F_).
    void epipolar_filter(const std::vector<cv::Point2f>& pts_l,
                         const std::vector<cv::Point2f>& pts_r,
                         std::vector<uchar>& status,
                         double max_dist_px = 2.0) const;
};
