#!/usr/bin/env python3
"""
Rerun 시각화 — KITTI 0117 의 LC EKF vs MSCKF vs GT 비교.

Usage:
    # 라이브 viewer (spawn)
    python tools/visualize_results_rerun.py

    # .rrd 파일로 저장 (offline)
    python tools/visualize_results_rerun.py --save out.rrd

    # 다른 결과 디렉토리 지정
    python tools/visualize_results_rerun.py \
        --msckf-dir results/kitti_raw/2011_09_26_drive_0117/full_msckf \
        --lc-dir    results/kitti_raw/2011_09_26_drive_0117/full_lc

읽는 파일 (각 dir 안):
  - trajectory_aligned_tum.txt  (GT 정렬된 추정 궤적, TUM 포맷)
  - gt_tum.txt                  (KITTI oxts GT, TUM 포맷)
  - state_log.txt               (매 frame 의 p/v/euler/q/feat)
  - imu_log.txt                 (optional, IMU 입력 로그)

시각화 구성:
  /world/gt          GT 궤적 (회색 line strip)
  /world/lc          LC EKF 추정 궤적 (파랑)
  /world/msckf       MSCKF 추정 궤적 (주황)
  /world/imu_pose    각 시점의 IMU pose (transform3D)
  /plots/ate_error   per-frame ATE error (scalar)
  /plots/velocity    velocity 3축
  /plots/euler       roll/pitch/yaw
  /plots/feat_count  tracked feature 수
"""

from __future__ import annotations

import argparse
import os
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np
import rerun as rr


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_LC_DIR = REPO_ROOT / "results/kitti_raw/2011_09_26_drive_0117/full_lc"
DEFAULT_MSCKF_DIR = REPO_ROOT / "results/kitti_raw/2011_09_26_drive_0117/full_msckf"


@dataclass
class TumPose:
    timestamps: np.ndarray  # (N,)
    positions: np.ndarray   # (N, 3) tx ty tz
    quats: np.ndarray       # (N, 4) qx qy qz qw


@dataclass
class StateLog:
    timestamps: np.ndarray  # (N,)
    positions: np.ndarray   # (N, 3)
    velocities: np.ndarray  # (N, 3)
    eulers_deg: np.ndarray  # (N, 3) roll pitch yaw (deg)
    quats: np.ndarray       # (N, 4) qx qy qz qw
    feat_counts: np.ndarray # (N,)


def read_tum(path: Path) -> Optional[TumPose]:
    if not path.exists():
        print(f"[skip] not found: {path}", file=sys.stderr)
        return None
    data = np.loadtxt(path, comments="#")
    if data.ndim == 1:
        data = data.reshape(1, -1)
    if data.shape[1] < 8:
        raise ValueError(f"{path}: expected >= 8 columns, got {data.shape[1]}")
    return TumPose(
        timestamps=data[:, 0],
        positions=data[:, 1:4],
        quats=data[:, 4:8],  # qx qy qz qw
    )


def read_state_log(path: Path) -> Optional[StateLog]:
    if not path.exists():
        print(f"[skip] not found: {path}", file=sys.stderr)
        return None
    data = np.loadtxt(path, comments="#")
    if data.ndim == 1:
        data = data.reshape(1, -1)
    if data.shape[1] < 15:
        raise ValueError(f"{path}: expected >= 15 columns, got {data.shape[1]}")
    return StateLog(
        timestamps=data[:, 0],
        positions=data[:, 1:4],
        velocities=data[:, 4:7],
        eulers_deg=data[:, 7:10],
        quats=data[:, 10:14],
        feat_counts=data[:, 14].astype(int),
    )


def nearest_index(timestamps: np.ndarray, t: float) -> int:
    return int(np.argmin(np.abs(timestamps - t)))


def log_static_trajectory(entity: str, traj: TumPose, color: list[int]) -> None:
    """전체 궤적을 static line strip 으로 한 번 등록 — scrubbing 해도 항상 보임."""
    if traj is None or len(traj.positions) < 2:
        return
    rr.log(
        entity,
        rr.LineStrips3D(
            [traj.positions.tolist()],
            colors=[color],
            radii=0.05,
        ),
        static=True,
    )


def log_streaming_pose(entity_prefix: str,
                        state: StateLog,
                        engine_name: str,
                        color: list[int]) -> None:
    """매 frame 의 pose / scalars 를 timeline 에 stream."""
    if state is None:
        return
    for i, t in enumerate(state.timestamps):
        rr.set_time("kitti", duration=float(t))
        # 3D pose marker (sphere at position)
        rr.log(
            f"{entity_prefix}/{engine_name}/pos",
            rr.Points3D(
                positions=[state.positions[i].tolist()],
                colors=[color],
                radii=0.15,
            ),
        )
        # Scalars
        rr.log(f"/plots/{engine_name}/vx", rr.Scalars(state.velocities[i, 0]))
        rr.log(f"/plots/{engine_name}/vy", rr.Scalars(state.velocities[i, 1]))
        rr.log(f"/plots/{engine_name}/vz", rr.Scalars(state.velocities[i, 2]))
        rr.log(f"/plots/{engine_name}/speed",
               rr.Scalars(float(np.linalg.norm(state.velocities[i]))))
        rr.log(f"/plots/{engine_name}/roll",  rr.Scalars(state.eulers_deg[i, 0]))
        rr.log(f"/plots/{engine_name}/pitch", rr.Scalars(state.eulers_deg[i, 1]))
        rr.log(f"/plots/{engine_name}/yaw",   rr.Scalars(state.eulers_deg[i, 2]))
        rr.log(f"/plots/{engine_name}/feat_count",
               rr.Scalars(float(state.feat_counts[i])))


def log_per_frame_ate(engine_name: str,
                       est: TumPose,
                       gt: TumPose) -> None:
    """추정값-GT 의 per-frame Euclidean error 를 scalar timeline 으로."""
    if est is None or gt is None:
        return
    for i, t in enumerate(est.timestamps):
        j = nearest_index(gt.timestamps, t)
        err = float(np.linalg.norm(est.positions[i] - gt.positions[j]))
        rr.set_time("kitti", duration=float(t))
        rr.log(f"/plots/ate_error/{engine_name}", rr.Scalars(err))


def compute_rmse(est: TumPose, gt: TumPose) -> float:
    if est is None or gt is None:
        return float("nan")
    errs = []
    for i, t in enumerate(est.timestamps):
        j = nearest_index(gt.timestamps, t)
        errs.append(np.linalg.norm(est.positions[i] - gt.positions[j]))
    return float(np.sqrt(np.mean(np.square(errs))))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--msckf-dir", type=Path, default=DEFAULT_MSCKF_DIR,
                        help="MSCKF run result directory")
    parser.add_argument("--lc-dir", type=Path, default=DEFAULT_LC_DIR,
                        help="LC EKF run result directory")
    parser.add_argument("--save", type=Path, default=None,
                        help="Save to .rrd file instead of spawning viewer")
    parser.add_argument("--no-stream-pose", action="store_true",
                        help="Skip per-frame streaming (lighter visualization)")
    args = parser.parse_args()

    # ---- Init Rerun ----
    rr.init("kitti_0117_lc_vs_msckf", spawn=(args.save is None))
    if args.save is not None:
        rr.save(str(args.save))
        print(f"[rerun] saving to {args.save}")
    else:
        print("[rerun] spawning viewer")

    # ---- Load all data ----
    msckf_traj  = read_tum(args.msckf_dir / "trajectory_aligned_tum.txt")
    msckf_state = read_state_log(args.msckf_dir / "state_log.txt")
    msckf_gt    = read_tum(args.msckf_dir / "gt_tum.txt")
    lc_traj     = read_tum(args.lc_dir / "trajectory_aligned_tum.txt")
    lc_state    = read_state_log(args.lc_dir / "state_log.txt")
    lc_gt       = read_tum(args.lc_dir / "gt_tum.txt")

    # ---- ATE summary ----
    msckf_rmse = compute_rmse(msckf_traj, msckf_gt)
    lc_rmse    = compute_rmse(lc_traj, lc_gt)
    print(f"[ate] LC RMSE    = {lc_rmse:.4f} m  ({lc_rmse*100:.2f} cm)")
    print(f"[ate] MSCKF RMSE = {msckf_rmse:.4f} m  ({msckf_rmse*100:.2f} cm)")
    if msckf_rmse > 0 and lc_rmse > 0:
        if msckf_rmse < lc_rmse:
            print(f"[ate] MSCKF 가 LC 보다 {lc_rmse/msckf_rmse:.2f}× 좋음")
        else:
            print(f"[ate] LC 가 MSCKF 보다 {msckf_rmse/lc_rmse:.2f}× 좋음")

    # ---- World axes ----
    rr.log("/world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)

    # ---- Static trajectories (always visible) ----
    log_static_trajectory("/world/gt",    msckf_gt or lc_gt, color=[180, 180, 180])
    log_static_trajectory("/world/lc",    lc_traj,           color=[ 60, 130, 250])
    log_static_trajectory("/world/msckf", msckf_traj,        color=[250, 140,  40])

    # ---- Per-frame ATE error scalars ----
    log_per_frame_ate("lc",    lc_traj,    lc_gt)
    log_per_frame_ate("msckf", msckf_traj, msckf_gt)

    # ---- Per-frame state streaming (heavier) ----
    if not args.no_stream_pose:
        log_streaming_pose("/world",  lc_state,    "lc",    color=[ 60, 130, 250])
        log_streaming_pose("/world",  msckf_state, "msckf", color=[250, 140,  40])

    # ---- Summary text ----
    summary = (
        f"KITTI 0117 (660 frames)\n"
        f"  LC EKF    : {lc_rmse*100:7.2f} cm\n"
        f"  MSCKF     : {msckf_rmse*100:7.2f} cm\n"
        f"  ratio     : {msckf_rmse/lc_rmse:.2f}× (MSCKF/LC)"
    )
    rr.log("/summary", rr.TextDocument(summary), static=True)

    print("[rerun] done.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
