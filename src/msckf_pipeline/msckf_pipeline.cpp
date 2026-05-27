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

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <unordered_set>

namespace {
constexpr std::size_t kCamId = 0;  // single (rectified mono) camera

int env_int(const char* name, int fallback) {
    const char* value = std::getenv(name);
    return value ? std::atoi(value) : fallback;
}

double env_double(const char* name, double fallback) {
    const char* value = std::getenv(name);
    return value ? std::atof(value) : fallback;
}
}  // namespace

MsckfPipeline::MsckfPipeline(double t0,
                             const Eigen::Matrix3d& R0_wi,
                             const Eigen::Vector3d& mean_accel,
                             const Eigen::Vector3d& mean_gyro,
                             double gravity_mag,
                             const Eigen::Matrix4d& T_cam0_imu,
                             const Eigen::Matrix4d& T_cam1_imu,
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
    opts.max_clone_size       = env_int("MSCKF_MAX_CLONES", 30);
    opts.max_slam_features    = 0;
    opts.max_msckf_in_update  = 1000;
    opts.num_cameras          = 2;  // cycle 5: stereo (rectified cam0 + cam1)
    opts.feat_rep_msckf =
        ov_type::LandmarkRepresentation::Representation::GLOBAL_3D;
    std::cerr << "[msckf:opts] max_clone_size=" << opts.max_clone_size << "\n";

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

    // OpenVINS StaticInitializer does not leave the State ctor's tiny default
    // covariance in place. Match its IMU startup covariance so bg/ba can move
    // enough to absorb KITTI's initial accel magnitude mismatch.
    Eigen::MatrixXd init_cov =
        std::pow(0.02, 2) * Eigen::MatrixXd::Identity(state_->_imu->size(), state_->_imu->size());
    init_cov.block<3, 3>(0, 0) = std::pow(0.02, 2) * Eigen::Matrix3d::Identity();  // q
    init_cov.block<3, 3>(3, 3) = std::pow(0.05, 2) * Eigen::Matrix3d::Identity();  // p
    init_cov.block<3, 3>(6, 6) = std::pow(0.01, 2) * Eigen::Matrix3d::Identity();  // v
    ov_msckf::StateHelper::set_initial_covariance(state_, init_cov, {state_->_imu});

    // ---- diagnostic prints ----
    std::cerr << "[msckf:init] mean_accel=" << mean_accel.transpose()
              << "  |a|=" << mean_accel.norm()
              << "  gravity_mag=" << gravity_mag << "\n";
    std::cerr << "[msckf:init] mean_gyro=" << mean_gyro.transpose() << "\n";
    std::cerr << "[msckf:init] R_GtoI (Global z is body-up direction) =\n" << R_GtoI << "\n";
    std::cerr << "[msckf:init] bg=" << bg.transpose() << "  ba=" << ba.transpose()
              << " (H2: forced to zero)\n";
    std::cerr << "[msckf:init] cov std q/p/v/bg/ba = "
              << std::sqrt(init_cov(0, 0)) << " / "
              << std::sqrt(init_cov(3, 3)) << " / "
              << std::sqrt(init_cov(6, 6)) << " / "
              << std::sqrt(init_cov(9, 9)) << " / "
              << std::sqrt(init_cov(12, 12)) << "\n";
    std::cerr << "[msckf:init] ba_ov_would_be=" << ba_ov.transpose()
              << "  |ba_ov|=" << ba_ov.norm() << " (diagnostic only)\n";
    // Sanity: predicted body accel at rest with ba=0 = R_GtoI * gravity_inG
    {
        const Eigen::Vector3d predicted = R_GtoI * gravity_inG + ba;
        const double rest_err = (predicted - mean_accel).norm();
        std::cerr << "[msckf:init] |R_GtoI*g + ba - mean_accel| = " << rest_err
                  << " (this is the residual the update must correct)\n";
    }

    // ---- Camera extrinsics (cam0 + cam1) -> JPL convention ----
    // OpenVINS PoseJPL stores [q_CtoI, p_IinC]. The same encoding is used for
    // every cam_id. With stereoRectify(alpha=0) the two rectified cameras
    // share K but differ in T_cam_imu by the baseline shift in x.
    auto set_calib = [&](std::size_t cam_id, const Eigen::Matrix4d& T_cam_imu) {
        Eigen::Matrix3d R_CI = T_cam_imu.block<3, 3>(0, 0);
        Eigen::Vector3d t_CI = T_cam_imu.block<3, 1>(0, 3);
        Eigen::Matrix<double, 7, 1> ext;
        ext.block<4, 1>(0, 0) = ov_core::rot_2_quat(R_CI);  // q_CtoI as JPL
        ext.block<3, 1>(4, 0) = t_CI;                        // p_IinC
        state_->_calib_IMUtoCAM.at(cam_id)->set_value(ext);
        state_->_calib_IMUtoCAM.at(cam_id)->set_fej(ext);
    };
    set_calib(0, T_cam0_imu);
    set_calib(1, T_cam1_imu);

    // ---- Camera intrinsic + distortion (rectified, shared K for cam0/cam1) ----
    Eigen::Matrix<double, 8, 1> intr;
    intr << fx_rect, fy_rect, cx_rect, cy_rect, 0.0, 0.0, 0.0, 0.0;
    for (std::size_t cam_id : {0u, 1u}) {
        state_->_cam_intrinsics.at(cam_id)->set_value(intr);
        state_->_cam_intrinsics.at(cam_id)->set_fej(intr);
        auto cam = std::make_shared<ov_core::CamRadtan>(img_width, img_height);
        cam->set_value(intr);
        state_->_cam_intrinsics_cameras.insert({cam_id, cam});
    }

    // ---- Propagator + UpdaterMSCKF ----
    // H3a: NoiseManager default sigma_a=2.0e-3 is 3× lower than KAIST yaml
    // (5.886e-3). KITTI uses an OXTS RT3003 — similar grade IMU to KAIST. With
    // sigma_a too small the propagator's state covariance grows too slowly, the
    // chi² gate's S = HPH^T + sigma_pix² is too small, and ~92% of features
    // get dropped (cycle 3 H2). Bump sigma_a to KAIST value; leave everything
    // else at OV defaults so any improvement is attributable to this knob.
    ov_msckf::NoiseManager noises;
    noises.sigma_a   = env_double("MSCKF_SIGMA_A", 5.886e-3);
    noises.sigma_a_2 = noises.sigma_a * noises.sigma_a;
    std::cerr << "[msckf:noise] H3a sigma_a=" << noises.sigma_a
              << " (vs OV default 2.0e-3, KAIST yaml 5.886e-3)\n";
    prop_ = std::make_unique<ov_msckf::Propagator>(noises, gravity_mag);

    upd_opts_ = std::make_unique<ov_msckf::UpdaterOptions>();
    upd_opts_->chi2_multipler = env_double("MSCKF_CHI2_MULT", 5.0);
    upd_opts_->sigma_pix      = env_double("MSCKF_SIGMA_PIX", 5.0);
    upd_opts_->sigma_pix_sq   = upd_opts_->sigma_pix * upd_opts_->sigma_pix;
    std::cerr << "[msckf:noise] chi2_multipler=" << upd_opts_->chi2_multipler << "\n";
    std::cerr << "[msckf:noise] sigma_pix=" << upd_opts_->sigma_pix
              << " (KITTI 0117 tuned; OV header 1.0, KAIST yaml 1.5)\n";

    feat_opts_ = std::make_unique<ov_core::FeatureInitializerOptions>();
    // Use defaults (refine_features = true, max_runs = 5, min_dist = 0.10, ...)

    updater_ = std::make_unique<ov_msckf::UpdaterMSCKF>(*upd_opts_, *feat_opts_);
}

MsckfPipeline::~MsckfPipeline() = default;

void MsckfPipeline::seed_initial_velocity(const Eigen::Vector3d& v_world) {
    if (!state_ || velocity_seeded_) return;

    Eigen::Matrix<double, 16, 1> imu = state_->_imu->value();
    imu.block<3, 1>(7, 0) = v_world;
    state_->_imu->set_value(imu);
    state_->_imu->set_fej(imu);
    velocity_seeded_ = true;

    std::cerr << "[msckf:init] seeded v0 from stereo frontend: "
              << v_world.transpose() << "  |v0|=" << v_world.norm() << "\n";
}

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
                                const std::vector<TrackedFeat>& left,
                                const std::vector<TrackedFeat>& right) {
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
    // Cycle 5 stereo: cam0 measurements come from `left` (every alive track),
    // cam1 measurements come from `right` (subset where stereo match was
    // valid). Both share the same track ID. A feature gets cam0 entries every
    // frame it's seen; cam1 entries appear only on frames where its stereo
    // match held. OV's UpdaterMSCKF iterates feature.timestamps as a cam_id
    // map, so independent insertion per cam is exactly the expected shape.
    std::unordered_set<std::size_t> seen_this_frame;
    seen_this_frame.reserve(left.size());

    auto fetch_or_create = [&](std::size_t id) -> std::shared_ptr<ov_core::Feature>& {
        auto it = feature_db_.find(id);
        if (it == feature_db_.end()) {
            auto feat = std::make_shared<ov_core::Feature>();
            feat->featid    = id;
            feat->to_delete = false;
            // anchor_cam_id stays at the OV default (-1); we use GLOBAL_3D
            // representation so the anchor path is never taken. Set to 0 as
            // a defensive default in case representation changes later.
            feat->anchor_cam_id = 0;
            it = feature_db_.emplace(id, feat).first;
        }
        return it->second;
    };

    auto push_measurement = [&](std::shared_ptr<ov_core::Feature>& feat,
                                std::size_t cam_id, float u, float v) {
        Eigen::Matrix<float, 2, 1> uv;        uv << u, v;
        Eigen::Matrix<float, 2, 1> uv_n;
        // Both rectified cams share the same K, so the normalization uses the
        // shared fx_/fy_/cx_/cy_ regardless of cam_id.
        uv_n << static_cast<float>((u - cx_) / fx_),
                static_cast<float>((v - cy_) / fy_);
        feat->uvs[cam_id].push_back(uv);
        feat->uvs_norm[cam_id].push_back(uv_n);
        feat->timestamps[cam_id].push_back(t);
    };

    // 2a. cam0 measurements (mandatory: defines "feature is alive this frame")
    for (const auto& tf : left) {
        seen_this_frame.insert(tf.id);
        auto& feat = fetch_or_create(tf.id);
        push_measurement(feat, /*cam_id=*/0, tf.u, tf.v);
    }
    // 2b. cam1 measurements (only IDs that already have a cam0 entry; we never
    // create a feature from a right-only observation because triangulation
    // would have no temporal continuity)
    for (const auto& tf : right) {
        auto it = feature_db_.find(tf.id);
        if (it == feature_db_.end()) continue;  // right without left — drop
        push_measurement(it->second, /*cam_id=*/1, tf.u, tf.v);
    }

    // ---- Step 3: collect *lost* features (in DB but no cam0 entry this
    // frame). cam1-only sightings don't keep a feature alive — once cam0 stops
    // tracking it, the temporal sequence ends.
    // Need >= 2 cam0 observations OR the combined cam0+cam1 measurement count
    // >= 2 for triangulation. We use the OV-style check: a feature passes if
    // its *total* measurement count (across all cams) >= 2 (Updater itself
    // imposes a stricter per-cam check during clean_old_measurements).
    std::vector<std::shared_ptr<ov_core::Feature>> feats_to_update;
    std::vector<std::size_t> ids_to_erase;
    for (auto& kv : feature_db_) {
        if (seen_this_frame.count(kv.first)) continue;  // still tracked
        std::size_t total_meas = 0;
        for (const auto& pair : kv.second->timestamps) total_meas += pair.second.size();
        if (total_meas < 2) {
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
