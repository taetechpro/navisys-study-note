#include "msckf_pipeline/msckf_pipeline.hpp"

#include "msckf/cam/CamRadtan.hpp"
#include "msckf/feat/Feature.hpp"
#include "msckf/feat/FeatureInitializerOptions.hpp"
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

// Hamilton R_wi (world<-imu) -> JPL q_IG (imu<-global, since OpenVINS uses
// I_R_G internally and exposes Rot() == R_IG).
Eigen::Matrix<double, 4, 1> hamilton_R_wi_to_jpl_q_IG(const Eigen::Matrix3d& R_wi) {
    return ov_core::rot_2_quat(R_wi.transpose());  // R_IG = R_wi^T
}
}  // namespace

MsckfPipeline::MsckfPipeline(double t0,
                             const Eigen::Matrix3d& R0_wi,
                             const Eigen::Vector3d& p0,
                             const Eigen::Vector3d& v0,
                             const Eigen::Vector3d& bg0,
                             const Eigen::Vector3d& ba0,
                             const Eigen::Matrix4d& T_cam0_imu,
                             double fx_rect, double fy_rect,
                             double cx_rect, double cy_rect,
                             int img_width, int img_height,
                             double gravity_mag)
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

    // ---- IMU initial value: 16-dim [q(4), p(3), v(3), bg(3), ba(3)] ----
    Eigen::Matrix<double, 16, 1> imu0;
    imu0.block<4, 1>(0, 0)  = hamilton_R_wi_to_jpl_q_IG(R0_wi);
    imu0.block<3, 1>(4, 0)  = p0;
    imu0.block<3, 1>(7, 0)  = v0;
    imu0.block<3, 1>(10, 0) = bg0;
    imu0.block<3, 1>(13, 0) = ba0;
    state_->_imu->set_value(imu0);
    state_->_imu->set_fej(imu0);

    // ---- R1 diagnostic: state R should equal input R_wi^T (= R_IG) ----
    {
        const Eigen::Matrix3d R_state_IG = state_->_imu->Rot();
        const Eigen::Matrix3d R_expected_IG = R0_wi.transpose();
        const double err = (R_state_IG - R_expected_IG).norm();
        std::cerr << "[msckf:R1] |R_state_IG - R_wi^T| = " << err
                  << "  (close to 0 means quat round-trip OK)\n";
        std::cerr << "[msckf:R1] R_wi (world<-body) input =\n" << R0_wi << "\n";
        std::cerr << "[msckf:R1] R_state_IG (body<-world) =\n" << R_state_IG << "\n";
        std::cerr << "[msckf:R1] p0=" << p0.transpose()
                  << "  v0=" << v0.transpose()
                  << "  bg0=" << bg0.transpose()
                  << "  ba0=" << ba0.transpose() << "\n";
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
    ov_msckf::NoiseManager noises;  // defaults — caller can tune later
    prop_ = std::make_unique<ov_msckf::Propagator>(noises, gravity_mag);

    upd_opts_ = std::make_unique<ov_msckf::UpdaterOptions>();
    upd_opts_->chi2_multipler = 5.0;
    upd_opts_->sigma_pix      = 1.0;
    upd_opts_->sigma_pix_sq   = 1.0;

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
        updater_->update(state_, feats_to_update);
        ++msckf_updates_;
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
