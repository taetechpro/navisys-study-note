#include "msckf_pipeline/msckf_pipeline.hpp"

#include "msckf/cam/CamRadtan.hpp"
#include "msckf/feat/Feature.hpp"
#include "msckf/feat/FeatureInitializerOptions.hpp"
#include "msckf/init/InitializerHelper.hpp"
#include "msckf/state/Propagator.hpp"
#include "msckf/state/State.hpp"
#include "msckf/state/StateHelper.hpp"
#include "msckf/state/StateOptions.hpp"
#include "msckf/types/IMU.hpp"
#include "msckf/types/PoseJPL.hpp"
#include "msckf/types/Vec.hpp"
#include "msckf/update/UpdaterMSCKF.hpp"
#include "msckf/update/UpdaterOptions.hpp"
#include "msckf/utils/NoiseManager.hpp"
#include "msckf/utils/quat_ops.hpp"
#include "msckf/utils/sensor_data.hpp"

#include <iostream>
#include <unordered_set>

namespace {
constexpr std::size_t kCamId = 0;  // single (rectified mono) camera
}  // namespace

MsckfPipeline::MsckfPipeline(double t0,
                             const Eigen::Matrix3d& R0_wi,
                             const Eigen::Vector3d& mean_accel,
                             const Eigen::Vector3d& mean_gyro,
                             double gravity_mag,
                             const Eigen::Matrix4d& T_cam0_imu,
                             double fx_rect, double fy_rect,
                             double cx_rect, double cy_rect,
                             int img_width, int img_height)
    : fx_(fx_rect), fy_(fy_rect), cx_(cx_rect), cy_(cy_rect),
      img_w_(img_width), img_h_(img_height) {

    // ---- StateOptions ----
    ov_msckf::StateOptions opts;
    opts.do_fej = true;
    opts.do_calib_camera_pose       = false;
    opts.do_calib_camera_intrinsics = false;
    opts.do_calib_camera_timeoffset = false;
    opts.do_calib_imu_intrinsics    = false;
    opts.do_calib_imu_g_sensitivity = false;
    opts.max_clone_size       = 11;
    opts.max_slam_features    = 0;
    opts.max_msckf_in_update  = 1000;
    opts.num_cameras          = 1;
    opts.feat_rep_msckf =
        ov_type::LandmarkRepresentation::Representation::GLOBAL_3D;

    state_ = std::make_shared<ov_msckf::State>(opts);
    state_->_timestamp = t0;

    // ---- P6 cycle 3 H2: revert to cycle 1 init (ba=0) ----
    // OV-style ba = mean_accel - R*g was the cycle 2 attempt that made ATE
    // worse (KITTI 0117 |a|=10.07 vs g=9.81 magnitude mismatch puts +0.26 m/s²
    // into ba.z). H1 probe showed KITTI 0117 init window IS stationary
    // (a_var < 0.5 KAIST gate), so the issue is NOT motion contamination.
    // For H2 we isolate update path: keep LC R_GtoI (yaw-aligned with GT) and
    // bg = mean_gyro, but force ba = 0 so propagate residual reflects raw
    // |a| - g mismatch and the update has to correct it.
    const Eigen::Matrix3d R_GtoI = R0_wi.transpose();  // world->body
    const Eigen::Vector4d q_GtoI = ov_core::rot_2_quat(R_GtoI);

    Eigen::Vector3d gravity_inG;
    gravity_inG << 0.0, 0.0, gravity_mag;
    const Eigen::Vector3d bg = mean_gyro;
    const Eigen::Vector3d ba = Eigen::Vector3d::Zero();  // H2: cycle 1 init
    const Eigen::Vector3d ba_ov = mean_accel - R_GtoI * gravity_inG;  // diag

    // ---- IMU 16-dim [q(4), p(3), v(3), bg(3), ba(3)] ----
    Eigen::Matrix<double, 16, 1> imu0;
    imu0.block<4, 1>(0, 0)  = q_GtoI;
    imu0.block<3, 1>(4, 0)  = Eigen::Vector3d::Zero();
    imu0.block<3, 1>(7, 0)  = Eigen::Vector3d::Zero();
    imu0.block<3, 1>(10, 0) = bg;
    imu0.block<3, 1>(13, 0) = ba;
    state_->_imu->set_value(imu0);
    state_->_imu->set_fej(imu0);

    // ---- diagnostic prints ----
    std::cerr << "[msckf:init] mean_accel=" << mean_accel.transpose()
              << "  |a|=" << mean_accel.norm()
              << "  gravity_mag=" << gravity_mag << "\n";
    std::cerr << "[msckf:init] mean_gyro=" << mean_gyro.transpose() << "\n";
    std::cerr << "[msckf:init] R_GtoI (Global z is body-up direction) =\n" << R_GtoI << "\n";
    std::cerr << "[msckf:init] bg=" << bg.transpose() << "  ba=" << ba.transpose()
              << " (H2: forced to zero)\n";
    std::cerr << "[msckf:init] ba_ov_would_be=" << ba_ov.transpose()
              << "  |ba_ov|=" << ba_ov.norm() << " (diagnostic only)\n";
    // Sanity: predicted body accel at rest with ba=0 = R_GtoI * gravity_inG
    {
        const Eigen::Vector3d predicted = R_GtoI * gravity_inG + ba;
        const double rest_err = (predicted - mean_accel).norm();
        std::cerr << "[msckf:init] |R_GtoI*g + ba - mean_accel| = " << rest_err
                  << " (this is the residual the update must correct)\n";
    }

    // ---- Camera extrinsic: T_cam0_imu (Hamilton) -> q_CtoI in JPL convention ----
    // OpenVINS PoseJPL stores q_CtoI, p_IinC. T_cam0_imu : cam0 <- imu, so:
    //   R_CI = T_cam0_imu.block<3,3>(0,0)  (cam <- imu)
    //   p_IinC = -R_CI * t_cam0_imu        but easier: t_IC stored as state's p
    // OpenVINS extrinsic conventions place R_CtoI in the JPL quat and the
    // camera position in the IMU body frame as p. We follow that:
    //   q_CtoI  = R_IC -> rot_2_quat  (i.e., quat of R_imu_from_cam transposed)
    //   p_IinC  = -R_CI * t_CI            (cam origin expressed in cam0 frame... )
    // Simpler & verified by OpenVINS sources: store [q_CtoI_as_JPL, p_IinC].
    Eigen::Matrix3d R_CI = T_cam0_imu.block<3, 3>(0, 0);  // cam <- imu
    Eigen::Vector3d t_CI = T_cam0_imu.block<3, 1>(0, 3);  // imu origin in cam
    Eigen::Matrix<double, 7, 1> ext;
    ext.block<4, 1>(0, 0) = ov_core::rot_2_quat(R_CI);  // q_CtoI as JPL of R_CI
    ext.block<3, 1>(4, 0) = t_CI;                        // p_IinC
    state_->_calib_IMUtoCAM.at(kCamId)->set_value(ext);
    state_->_calib_IMUtoCAM.at(kCamId)->set_fej(ext);

    // ---- Camera intrinsic + distortion (we registered a rectified stream) ----
    Eigen::Matrix<double, 8, 1> intr;
    intr << fx_rect, fy_rect, cx_rect, cy_rect, 0.0, 0.0, 0.0, 0.0;
    state_->_cam_intrinsics.at(kCamId)->set_value(intr);
    state_->_cam_intrinsics.at(kCamId)->set_fej(intr);

    auto cam = std::make_shared<ov_core::CamRadtan>(img_width, img_height);
    cam->set_value(intr);
    state_->_cam_intrinsics_cameras.insert({kCamId, cam});

    // ---- Propagator + UpdaterMSCKF ----
    // H3a: NoiseManager default sigma_a=2.0e-3 is 3× lower than KAIST yaml
    // (5.886e-3). KITTI uses an OXTS RT3003 — similar grade IMU to KAIST. With
    // sigma_a too small the propagator's state covariance grows too slowly, the
    // chi² gate's S = HPH^T + sigma_pix² is too small, and ~92% of features
    // get dropped (cycle 3 H2). Bump sigma_a to KAIST value; leave everything
    // else at OV defaults so any improvement is attributable to this knob.
    ov_msckf::NoiseManager noises;
    noises.sigma_a   = 5.886e-3;
    noises.sigma_a_2 = noises.sigma_a * noises.sigma_a;
    std::cerr << "[msckf:noise] H3a sigma_a=" << noises.sigma_a
              << " (vs OV default 2.0e-3, KAIST yaml 5.886e-3)\n";
    prop_ = std::make_unique<ov_msckf::Propagator>(noises, gravity_mag);

    upd_opts_ = std::make_unique<ov_msckf::UpdaterOptions>();
    upd_opts_->chi2_multipler = 5.0;
    upd_opts_->sigma_pix      = 1.5;  // H3b: was 1.0, KAIST yaml uses 1.5
    upd_opts_->sigma_pix_sq   = upd_opts_->sigma_pix * upd_opts_->sigma_pix;
    std::cerr << "[msckf:noise] H3b sigma_pix=" << upd_opts_->sigma_pix
              << " (vs OV header 1.0, KAIST yaml 1.5)\n";

    feat_opts_ = std::make_unique<ov_core::FeatureInitializerOptions>();
    // Use defaults (refine_features = true, max_runs = 5, min_dist = 0.10, ...)

    updater_ = std::make_unique<ov_msckf::UpdaterMSCKF>(*upd_opts_, *feat_opts_);
}

MsckfPipeline::~MsckfPipeline() = default;

void MsckfPipeline::feed_imu(double t,
                             const Eigen::Vector3d& gyro,
                             const Eigen::Vector3d& accel) {
    ov_core::ImuData data;
    data.timestamp = t;
    data.wm = gyro;
    data.am = accel;
    prop_->feed_imu(data);
    last_imu_t_ = t;
    last_imu_w_ = gyro;
    last_imu_a_ = accel;
}

void MsckfPipeline::feed_camera(double t,
                                const std::vector<TrackedFeat>& tracked) {
    // ---- Step 1: propagate + clone ----
    // OpenVINS Propagator::propagate_and_clone aborts on dt<=0, so we cannot
    // re-call it at the constructor-supplied t0. For the very first camera
    // frame we just augment a clone at t0 directly. From the second frame on,
    // we let the propagator advance state to t and clone there.
    //
    // Also: select_imu_readings inside the propagator needs an IMU sample at
    // or after `t` to close the boundary. The host main loop only feeds IMU
    // <= cam.timestamp, so we duplicate the most recent IMU at time `t`
    // before propagating. dt of that final segment is small (typically the
    // few-ms gap between the last 100Hz sample and the camera frame).
    if (first_camera_) {
        first_camera_ = false;
        ov_msckf::StateHelper::augment_clone(state_, Eigen::Vector3d::Zero());
    } else {
        prop_->propagate_and_clone(state_, t);
    }

    // ---- Step 2: update feature DB with this frame's observations ----
    std::unordered_set<std::size_t> seen_this_frame;
    seen_this_frame.reserve(tracked.size());

    for (const auto& tf : tracked) {
        seen_this_frame.insert(tf.id);

        auto it = feature_db_.find(tf.id);
        if (it == feature_db_.end()) {
            auto feat = std::make_shared<ov_core::Feature>();
            feat->featid    = tf.id;
            feat->to_delete = false;
            it = feature_db_.emplace(tf.id, feat).first;
        }
        auto& feat = it->second;

        Eigen::Matrix<float, 2, 1> uv;
        uv << tf.u, tf.v;
        Eigen::Matrix<float, 2, 1> uv_n;
        uv_n << static_cast<float>((tf.u - cx_) / fx_),
                static_cast<float>((tf.v - cy_) / fy_);

        feat->uvs[kCamId].push_back(uv);
        feat->uvs_norm[kCamId].push_back(uv_n);
        feat->timestamps[kCamId].push_back(t);
    }

    // ---- Step 3: collect *lost* features (in DB but not seen this frame),
    // require at least 2 observations for triangulation. ----
    std::vector<std::shared_ptr<ov_core::Feature>> feats_to_update;
    std::vector<std::size_t> ids_to_erase;
    for (auto& kv : feature_db_) {
        if (seen_this_frame.count(kv.first)) continue;  // still tracked
        if (kv.second->timestamps[kCamId].size() < 2) {
            // not enough views for triangulation; just drop
            ids_to_erase.push_back(kv.first);
            continue;
        }
        feats_to_update.push_back(kv.second);
        ids_to_erase.push_back(kv.first);
    }
    for (auto id : ids_to_erase) feature_db_.erase(id);

    // ---- Step 4: MSCKF update ----
    if (!feats_to_update.empty()) {
        const int submitted = static_cast<int>(feats_to_update.size());
        updater_->update(state_, feats_to_update);
        // UpdaterMSCKF mutates feats_to_update in place via erase():
        //   - features with <2 measurements after clean → erased
        //   - features that fail triangulation → erased
        //   - features that fail chi² gate → erased
        //   - features that pass all gates → remain in vector AND get
        //     `to_delete=true` at the end.
        // So `feats_to_update.size()` after the call == count of features
        // that actually contributed to the EKF update.
        const int consumed = static_cast<int>(feats_to_update.size());
        feats_submitted_total_ += submitted;
        feats_consumed_total_  += consumed;
        ++frames_with_update_;
        ++msckf_updates_;
        if (frames_with_update_ <= 5 || frames_with_update_ % 25 == 0) {
            std::cerr << "[msckf:upd] f_with_upd=" << frames_with_update_
                      << " submitted=" << submitted
                      << " consumed=" << consumed
                      << "  cumul submitted=" << feats_submitted_total_
                      << " consumed=" << feats_consumed_total_
                      << "  accept_pct=" << (feats_submitted_total_ > 0 ?
                          100.0 * feats_consumed_total_ / feats_submitted_total_ : 0.0)
                      << "%\n";
        }
    }

    // ---- Step 5: marginalize old clone if sliding window is full ----
    while (static_cast<int>(state_->_clones_IMU.size()) > state_->_options.max_clone_size) {
        ov_msckf::StateHelper::marginalize_old_clone(state_);
    }

    // ---- R1 diagnostic: first 5 frames, log state ----
    if (diag_frame_count_ < 5) {
        const Eigen::Matrix3d R_wi = state_->_imu->Rot().transpose();
        const Eigen::Vector3d p    = state_->_imu->pos();
        const Eigen::Vector3d v    = state_->_imu->vel();
        std::cerr << "[msckf:R1] f" << diag_frame_count_ << " t=" << t
                  << "  p=" << p.transpose()
                  << "  v=" << v.transpose()
                  << "  Rwi_diag=" << R_wi(0,0) << "," << R_wi(1,1) << "," << R_wi(2,2)
                  << "  updates=" << msckf_updates_
                  << "\n";
        ++diag_frame_count_;
    }
}

MsckfPipeline::Pose MsckfPipeline::latest_pose() const {
    Pose out;
    if (!state_) return out;
    // state->_imu->Rot() == R_IG (IMU <- Global). We expose R_world_imu.
    out.R         = state_->_imu->Rot().transpose();
    out.p         = state_->_imu->pos();
    out.v         = state_->_imu->vel();
    out.timestamp = state_->_timestamp;
    out.valid     = (state_->_timestamp > 0.0);
    return out;
}

int MsckfPipeline::num_clones() const {
    return state_ ? static_cast<int>(state_->_clones_IMU.size()) : 0;
}
