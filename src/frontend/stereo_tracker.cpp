#include "frontend/stereo_tracker.hpp"

#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <iostream>

using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::Matrix4d;

// ---- constructor ----

StereoTracker::StereoTracker(const CameraParams& cam0,
                             const CameraParams& cam1) {
    K0_ = (cv::Mat_<double>(3,3) <<
           cam0.fx, 0, cam0.cx,
           0, cam0.fy, cam0.cy,
           0, 0, 1);
    dist0_ = (cv::Mat_<double>(1,4) <<
              cam0.k1, cam0.k2, cam0.p1, cam0.p2);

    K1_ = (cv::Mat_<double>(3,3) <<
           cam1.fx, 0, cam1.cx,
           0, cam1.fy, cam1.cy,
           0, 0, 1);
    dist1_ = (cv::Mat_<double>(1,4) <<
              cam1.k1, cam1.k2, cam1.p1, cam1.p2);

    // T_cam1_cam0 = T_C1_I * T_C0_I^{-1}
    Matrix4d T_C0_I = cam0.T_cam_imu;
    Matrix4d T_C1_I = cam1.T_cam_imu;
    T_cam1_cam0_ = T_C1_I * T_C0_I.inverse();

    // Stereo projection matrices (both in cam0 frame)
    // P0 = K0 * [I|0],  P1 = K1 * [R10|t10]
    cv::Mat R10(3,3,CV_64F), t10(3,1,CV_64F);
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) R10.at<double>(i,j) = T_cam1_cam0_(i,j);
        t10.at<double>(i,0) = T_cam1_cam0_(i,3);
    }
    // Fundamental matrix (kept for reference, not used after rectification)
    cv::Mat t_skew = (cv::Mat_<double>(3,3) <<
         0,                    -t10.at<double>(2,0),  t10.at<double>(1,0),
         t10.at<double>(2,0),   0,                   -t10.at<double>(0,0),
        -t10.at<double>(1,0),   t10.at<double>(0,0),  0);
    cv::Mat E = t_skew * R10;
    F_ = K1_.t().inv() * E * K0_.inv();

    // ---- Stereo Rectification ----
    // Computes R0_rect, R1_rect so that rectified epipolar lines are horizontal.
    // Also produces new projection matrices P0_rect, P1_rect with common focal length.
    cv::Size img_size(cam0.width, cam0.height);
    cv::Mat Q;
    cv::stereoRectify(K0_, dist0_, K1_, dist1_, img_size,
                      R10, t10,
                      R0_rect_, R1_rect_, P0_, P1_, Q,
                      cv::CALIB_ZERO_DISPARITY, -1);
    // After stereoRectify, P0_ and P1_ are the NEW 3×4 projection matrices
    // for rectified cam0 and cam1 (in the rectified coordinate system).

    // Build per-pixel undistort+rectify maps (used at runtime via cv::remap)
    cv::initUndistortRectifyMap(K0_, dist0_, R0_rect_, P0_, img_size,
                                 CV_32FC1, map0x_, map0y_);
    cv::initUndistortRectifyMap(K1_, dist1_, R1_rect_, P1_, img_size,
                                 CV_32FC1, map1x_, map1y_);

    fx_rect_       = P0_.at<double>(0, 0);
    fy_rect_       = P0_.at<double>(1, 1);
    cx_rect_       = P0_.at<double>(0, 2);
    cy_rect_       = P0_.at<double>(1, 2);
    // P1_[0,3] = -fx * baseline (translation term in rectified frame)
    baseline_rect_ = -P1_.at<double>(0, 3) / fx_rect_;

    // Rectified camera matrix K and zero distortion (for PnP in rectified space)
    K_rect_    = P0_(cv::Rect(0, 0, 3, 3)).clone();
    zero_dist_ = cv::Mat::zeros(1, 4, CV_64F);

    // Guided-KLT initial offset (rectified: cam1 to the right → features shift LEFT in cam1)
    // Negative offset for standard stereo rectification layout.
    disparity_offset_px_ = -static_cast<float>(fx_rect_ * baseline_rect_ / 3.0);

    // ORB detector + Hamming brute-force matcher for stereo matching
    // Args: nfeatures=1500, scaleFactor=1.2, nlevels=8, edgeThreshold=15,
    //       firstLevel=0, WTA_K=2, scoreType=HARRIS, patchSize=31, fastThreshold=7
    orb_     = cv::ORB::create(1500, 1.2f, 8, 15, 0, 2, cv::ORB::HARRIS_SCORE, 31, 7);
    matcher_ = cv::BFMatcher::create(cv::NORM_HAMMING, false);

    std::cout << "[StereoTracker] rectified fx=" << fx_rect_
              << " baseline=" << baseline_rect_ * 100 << " cm"
              << " disp_init@3m=" << disparity_offset_px_ << " px  ORB ready\n";
}

// ---- public: process ----
// Invariant maintained throughout: prev_pts_l_.size() == map_pts_world_.size()
// and every pair (prev_pts_l_[i], map_pts_world_[i]) is a valid 2D-3D correspondence.

StereoTracker::Pose StereoTracker::process(const cv::Mat& img_l,
                                            const cv::Mat& img_r) {
    cv::Mat raw_l, raw_r;
    cv::cvtColor(img_l, raw_l, cv::COLOR_BGR2GRAY);
    cv::cvtColor(img_r, raw_r, cv::COLOR_BGR2GRAY);

    // Apply stereo rectification: now both images are undistorted AND
    // horizontally aligned (epipolar lines are scanlines).
    cv::Mat gray_l, gray_r;
    cv::remap(raw_l, gray_l, map0x_, map0y_, cv::INTER_LINEAR);
    cv::remap(raw_r, gray_r, map1x_, map1y_, cv::INTER_LINEAR);

    // ---- Bootstrap (first frame) ----
    if (!initialized_) {
        // ORB-based stereo matching on rectified images (robust to repeated patterns)
        auto match = orb_stereo_match(gray_l, gray_r, {}, target_features_);
        auto pts3d = stereo_triangulate(match.pts_l, match.pts_r);

        auto& kept_l = match.pts_l;  // alias so the loop below still works

        for (size_t i = 0; i < pts3d.size(); ++i) {
            double z = pts3d[i][2];
            if (z > 0.1 && z < 50.0) {
                prev_pts_l_.push_back(kept_l[i]);
                prev_pts_r_.push_back(match.pts_r[i]);  // cycle 5: cam1 px from ORB stereo match
                stereo_valid_.push_back(true);          // cycle 5: bootstrap match always valid
                map_pts_world_.push_back(pts3d[i]); // world = cam0_0 frame
                prev_track_ids_.push_back(next_id_++);
            }
        }

        prev_img_l_ = gray_l.clone();
        prev_img_r_ = gray_r.clone();
        curr_pose_ = Pose{Matrix3d::Identity(), Vector3d::Zero(), true};
        initialized_ = true;
        return curr_pose_;
    }

    // ---- Step 1: temporal KLT tracking ----
    std::vector<uchar> temp_ok;
    auto curr_pts = klt_track(prev_img_l_, gray_l, prev_pts_l_, temp_ok);

    // Filter: keep valid tracked + aligned map points + IDs
    std::vector<cv::Point2f>    tracked_pts;
    std::vector<Vector3d>       tracked_map;
    std::vector<std::size_t>    tracked_ids;
    for (size_t i = 0; i < temp_ok.size(); ++i) {
        if (temp_ok[i]) {
            tracked_pts.push_back(curr_pts[i]);
            tracked_map.push_back(map_pts_world_[i]);
            tracked_ids.push_back(prev_track_ids_[i]);
        }
    }

    // ---- Step 2: PnP pose estimation ----
    Pose pose = curr_pose_;
    if (static_cast<int>(tracked_pts.size()) >= min_pnp_inliers_) {
        Matrix3d R_cw; Vector3d t_cw;
        std::vector<int> inliers;
        if (solve_pnp(tracked_map, tracked_pts, R_cw, t_cw, inliers)) {
            pose.R = R_cw.transpose();           // R_wc
            pose.t = -R_cw.transpose() * t_cw;  // camera origin in world
            pose.valid = true;
            curr_pose_ = pose;

            // Keep only inlier correspondences (incl. IDs)
            std::vector<cv::Point2f>    in_pts;
            std::vector<Vector3d>       in_map;
            std::vector<std::size_t>    in_ids;
            for (int idx : inliers) {
                in_pts.push_back(tracked_pts[idx]);
                in_map.push_back(tracked_map[idx]);
                in_ids.push_back(tracked_ids[idx]);
            }
            tracked_pts = in_pts;
            tracked_map = in_map;
            tracked_ids = in_ids;
        }
    }

    // ---- Step 3: skipped ----
    // Re-triangulating tracked features via KLT stereo fails on repeated textures.
    // Keep 3D positions from the original bootstrap / step-4 addition; let PnP RANSAC
    // weed out stale points.

    // ---- Cycle 5: stereo KLT for surviving (existing) tracks ----
    // tracked_pts here holds only features that survived temporal KLT + PnP
    // inlier filter. Compute their cam1 (right) pixels via stereo KLT and
    // record validity. New features added in Step 4 below already carry their
    // cam1 pixel from ORB stereo matching, so we push those directly.
    std::vector<cv::Point2f> tracked_pts_r;
    std::vector<bool>        tracked_valid;
    tracked_pts_r.reserve(tracked_pts.size());
    tracked_valid.reserve(tracked_pts.size());
    if (!tracked_pts.empty()) {
        std::vector<uchar> stereo_ok;
        auto pts_r_curr = stereo_klt(gray_l, gray_r, tracked_pts, stereo_ok);
        for (size_t i = 0; i < tracked_pts.size(); ++i) {
            const bool ok = (i < stereo_ok.size()) && stereo_ok[i];
            tracked_pts_r.push_back(ok ? pts_r_curr[i] : cv::Point2f(-1.f, -1.f));
            tracked_valid.push_back(ok);
        }
    }

    // ---- Step 4: detect+triangulate new features via ORB if below threshold ----
    int need = target_features_ - static_cast<int>(tracked_pts.size());
    if (need > 0) {
        auto match = orb_stereo_match(gray_l, gray_r, tracked_pts, need);
        if (!match.pts_l.empty()) {
            auto pts3d_cam = stereo_triangulate(match.pts_l, match.pts_r);
            for (size_t i = 0; i < pts3d_cam.size(); ++i) {
                double z = pts3d_cam[i][2];
                if (z < 0.1 || z > 50.0) continue;
                tracked_pts.push_back(match.pts_l[i]);
                tracked_pts_r.push_back(match.pts_r[i]);  // cycle 5: cam1 px from ORB
                tracked_valid.push_back(true);             // cycle 5
                tracked_map.push_back(pose.R * pts3d_cam[i] + pose.t);
                tracked_ids.push_back(next_id_++);
            }
        }
    }

    // ---- Update state (invariant: sizes always match) ----
    prev_img_l_      = gray_l.clone();
    prev_img_r_      = gray_r.clone();
    prev_pts_l_      = tracked_pts;
    prev_pts_r_      = tracked_pts_r;   // cycle 5
    stereo_valid_    = tracked_valid;   // cycle 5
    map_pts_world_   = tracked_map;
    prev_track_ids_  = tracked_ids;

    // Convert pose from rectified frame back to original cam0 frame for output.
    // Internally we work in rectified frame (3D points, map, curr_pose_). Caller
    // (run_euroc.cpp) expects pose in original cam0 frame.
    Eigen::Matrix3d R0_rect_e;
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            R0_rect_e(r, c) = R0_rect_.at<double>(r, c);

    Pose pose_out;
    pose_out.valid = pose.valid;
    pose_out.t     = R0_rect_e.transpose() * pose.t;
    pose_out.R     = R0_rect_e.transpose() * pose.R * R0_rect_e;
    return pose_out;
}

Matrix3d StereoTracker::rectified_to_cam0_rotation() const {
    Matrix3d R0_rect_e;
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            R0_rect_e(r, c) = R0_rect_.at<double>(r, c);
        }
    }
    return R0_rect_e.transpose();
}

// ---- private helpers ----

std::vector<cv::Point2f>
StereoTracker::klt_track(const cv::Mat& src, const cv::Mat& dst,
                          const std::vector<cv::Point2f>& pts_src,
                          std::vector<uchar>& status) {
    std::vector<cv::Point2f> pts_dst;
    if (pts_src.empty()) { status.clear(); return pts_dst; }

    std::vector<float> err;
    cv::calcOpticalFlowPyrLK(src, dst, pts_src, pts_dst,
                              status, err,
                              cv::Size(21,21), 4,
                              cv::TermCriteria(cv::TermCriteria::COUNT|cv::TermCriteria::EPS, 30, 0.01));

    // Forward-backward consistency check
    std::vector<cv::Point2f> back_pts;
    std::vector<uchar> back_ok;
    cv::calcOpticalFlowPyrLK(dst, src, pts_dst, back_pts,
                              back_ok, err, cv::Size(21,21), 4);
    for (size_t i = 0; i < status.size(); ++i) {
        if (!status[i] || !back_ok[i]) { status[i] = 0; continue; }
        float dx = pts_src[i].x - back_pts[i].x;
        float dy = pts_src[i].y - back_pts[i].y;
        if (dx*dx + dy*dy > 1.0f) status[i] = 0;
    }
    return pts_dst;
}

std::vector<cv::Point2f>
StereoTracker::undistort(const std::vector<cv::Point2f>& px,
                          const cv::Mat& K,
                          const cv::Mat& dist) const {
    if (px.empty()) return {};
    std::vector<cv::Point2f> out;
    // Output in undistorted pixel space using the same K (for use with P=K[R|t])
    cv::undistortPoints(px, out, K, dist, cv::noArray(), K);
    return out;
}

std::vector<Vector3d>
StereoTracker::stereo_triangulate(const std::vector<cv::Point2f>& ud_l,
                                   const std::vector<cv::Point2f>& ud_r) const {
    std::vector<Vector3d> result;
    if (ud_l.empty()) return result;

    cv::Mat pts4d;
    cv::triangulatePoints(P0_, P1_, ud_l, ud_r, pts4d);

    result.resize(ud_l.size());
    for (int i = 0; i < pts4d.cols; ++i) {
        double w = pts4d.at<float>(3,i);
        if (std::abs(w) < 1e-6) { result[i] = {0,0,-1}; continue; }
        result[i] = {pts4d.at<float>(0,i)/w,
                     pts4d.at<float>(1,i)/w,
                     pts4d.at<float>(2,i)/w};
    }
    return result;
}

bool StereoTracker::solve_pnp(const std::vector<Vector3d>& pts3d,
                               const std::vector<cv::Point2f>& pts2d,
                               Matrix3d& R_cw, Vector3d& t_cw,
                               std::vector<int>& inlier_idx) const {
    if (static_cast<int>(pts3d.size()) < min_pnp_inliers_) return false;

    std::vector<cv::Point3f> obj;
    obj.reserve(pts3d.size());
    for (auto& p : pts3d)
        obj.emplace_back(float(p[0]), float(p[1]), float(p[2]));

    cv::Mat rvec, tvec, inliers_mat;
    // Rectified images → use rectified K and zero distortion
    bool ok = cv::solvePnPRansac(obj, pts2d, K_rect_, zero_dist_,
                                  rvec, tvec,
                                  false, 100, 4.0f, 0.99,
                                  inliers_mat, cv::SOLVEPNP_EPNP);
    if (!ok || inliers_mat.rows < min_pnp_inliers_) return false;

    cv::Mat R_cv;
    cv::Rodrigues(rvec, R_cv);
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            R_cw(r,c) = R_cv.at<double>(r,c);
    t_cw = {tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2)};

    inlier_idx.clear();
    inlier_idx.reserve(inliers_mat.rows);
    for (int i = 0; i < inliers_mat.rows; ++i)
        inlier_idx.push_back(inliers_mat.at<int>(i));
    return true;
}

void StereoTracker::detect_features(const cv::Mat& gray,
                                     const std::vector<cv::Point2f>& existing,
                                     int n_needed,
                                     std::vector<cv::Point2f>& out) {
    if (n_needed <= 0) return;

    // Mask out regions around existing features (radius 10px)
    cv::Mat mask = cv::Mat::ones(gray.size(), CV_8UC1) * 255;
    for (auto& pt : existing)
        cv::circle(mask, pt, 10, cv::Scalar(0), -1);

    std::vector<cv::KeyPoint> kps;
    auto det = cv::FastFeatureDetector::create(15);
    det->detect(gray, kps, mask);

    std::sort(kps.begin(), kps.end(),
              [](const cv::KeyPoint& a, const cv::KeyPoint& b){
                  return a.response > b.response; });

    int take = std::min(n_needed, (int)kps.size());
    out.reserve(take);
    for (int i = 0; i < take; ++i)
        out.push_back(kps[i].pt);
}

void StereoTracker::epipolar_filter(const std::vector<cv::Point2f>& pts_l,
                                     const std::vector<cv::Point2f>& pts_r,
                                     std::vector<uchar>& status,
                                     double max_dist_px) const {
    if (pts_l.empty()) return;

    // Undistort to get ideal pixel coords (F_ is built from K0^{-1}, K1^{-T})
    auto ud_l = undistort(pts_l, K0_, dist0_);
    auto ud_r = undistort(pts_r, K1_, dist1_);

    // Epipolar lines in cam1 for all cam0 points: l = F * p_l
    std::vector<cv::Vec3f> lines;
    cv::computeCorrespondEpilines(ud_l, 1, F_, lines);

    for (size_t i = 0; i < status.size(); ++i) {
        if (!status[i]) continue;
        float a = lines[i][0], b = lines[i][1], c = lines[i][2];
        float dist = std::abs(a * ud_r[i].x + b * ud_r[i].y + c)
                     / std::sqrt(a*a + b*b);
        if (dist > static_cast<float>(max_dist_px))
            status[i] = 0;
    }
}

// Stereo-specific KLT: initialise right image search at expected disparity
// offset to avoid the zero-disparity local minimum that plain KLT falls into.
std::vector<cv::Point2f>
StereoTracker::stereo_klt(const cv::Mat& img_l, const cv::Mat& img_r,
                           const std::vector<cv::Point2f>& pts_l,
                           std::vector<uchar>& status) {
    std::vector<cv::Point2f> pts_r;
    if (pts_l.empty()) { status.clear(); return pts_r; }

    // Initial guess: shift by expected disparity (in cam1, features appear
    // shifted by fx * |t10_x| / z_expected).
    pts_r.reserve(pts_l.size());
    for (const auto& p : pts_l)
        pts_r.emplace_back(p.x + disparity_offset_px_, p.y);

    std::vector<float> err;
    cv::calcOpticalFlowPyrLK(
        img_l, img_r, pts_l, pts_r,
        status, err,
        cv::Size(21, 21), 4,
        cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 30, 0.01),
        cv::OPTFLOW_USE_INITIAL_FLOW);

    // Forward-backward consistency check
    std::vector<cv::Point2f> back_pts;
    std::vector<uchar> back_ok;
    cv::calcOpticalFlowPyrLK(img_r, img_l, pts_r, back_pts,
                              back_ok, err, cv::Size(21, 21), 4);
    for (size_t i = 0; i < status.size(); ++i) {
        if (!status[i] || !back_ok[i]) { status[i] = 0; continue; }
        float dx = pts_l[i].x - back_pts[i].x;
        float dy = pts_l[i].y - back_pts[i].y;
        if (dx*dx + dy*dy > 1.0f) status[i] = 0;
    }
    return pts_r;
}

// ORB-based stereo matching on rectified images. Robust to repeated textures
// unlike KLT, because it matches by descriptor distance (Hamming) rather than
// pixel gradient descent.
StereoTracker::StereoMatch
StereoTracker::orb_stereo_match(const cv::Mat& img_l,
                                  const cv::Mat& img_r,
                                  const std::vector<cv::Point2f>& avoid,
                                  int max_features) {
    StereoMatch result;

    // Avoid-region mask so we don't re-detect near existing features
    cv::Mat mask = cv::Mat::ones(img_l.size(), CV_8UC1) * 255;
    for (const auto& p : avoid)
        cv::circle(mask, p, 10, cv::Scalar(0), -1);

    // Detect ORB keypoints + descriptors in both rectified images
    std::vector<cv::KeyPoint> kp_l, kp_r;
    cv::Mat desc_l, desc_r;
    orb_->detectAndCompute(img_l, mask, kp_l, desc_l);
    orb_->detectAndCompute(img_r, cv::noArray(), kp_r, desc_r);

    if (kp_l.empty() || kp_r.empty() || desc_l.empty() || desc_r.empty())
        return result;

    // Epipolar-constrained matching: for each cam0 keypoint, only compare against
    // cam1 keypoints in a narrow y-band (|dv|<2px) with valid disparity (-150 < du < 0).
    // This is MUCH more robust than brute-force BFMatcher + post-filter, because
    // the "best match" is chosen from geometrically-valid candidates only.
    constexpr float MAX_DV      = 2.0f;    // |Δy|    px
    constexpr float MIN_DU      = -150.0f; // du ≥ -150 (max 150px disparity)
    constexpr float MAX_DU      = -1.0f;   // du ≤  -1  (min 1px disparity)
    constexpr int   MAX_HAMMING = 60;      // descriptor distance threshold (32-byte = 256 bits)

    int n_candidates_found = 0;
    for (int i = 0; i < (int)kp_l.size(); ++i) {
        const cv::Point2f& p_l = kp_l[i].pt;
        const cv::Mat& d_l = desc_l.row(i);

        int   best_j = -1;
        int   best_d = MAX_HAMMING + 1;
        int   second_d = MAX_HAMMING + 1;

        for (int j = 0; j < (int)kp_r.size(); ++j) {
            const cv::Point2f& p_r = kp_r[j].pt;
            float dv = p_r.y - p_l.y;
            if (std::abs(dv) > MAX_DV) continue;
            float du = p_r.x - p_l.x;
            if (du < MIN_DU || du > MAX_DU) continue;

            int d = cv::norm(d_l, desc_r.row(j), cv::NORM_HAMMING);
            if (d < best_d) {
                second_d = best_d;
                best_d   = d;
                best_j   = j;
            } else if (d < second_d) {
                second_d = d;
            }
            n_candidates_found++;
        }

        if (best_j < 0) continue;
        if (best_d > MAX_HAMMING) continue;
        // Ratio test: if ambiguous, reject
        if (best_d > 0.85f * second_d) continue;

        result.pts_l.push_back(p_l);
        result.pts_r.push_back(kp_r[best_j].pt);
        if ((int)result.pts_l.size() >= max_features) break;
    }
    return result;
}
