"""
Trajectory visualizer for cpp_lc_vio results.

Usage:
    python tools/plot_trajectory.py <result_dir>

Reads:
    <result_dir>/trajectory_tum.txt   (estimated)
    <result_dir>/gt_tum.txt           (ground truth)
    <result_dir>/state_log.txt        (pos + vel + euler, optional)
    <result_dir>/metrics.txt          (ATE numbers)

Outputs:
    <result_dir>/trajectory_plot.png
    <result_dir>/state_plot.png       (if state_log.txt exists)
"""

import sys
import pathlib
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec


# ──────────────────────────────────────────
# I/O helpers
# ──────────────────────────────────────────

def load_tum(path: pathlib.Path):
    """Return (timestamps[N], positions[N,3]) from a TUM-format file."""
    ts, pos = [], []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            ts.append(float(parts[0]))
            pos.append([float(parts[1]), float(parts[2]), float(parts[3])])
    return np.array(ts), np.array(pos)


def load_state_log(path: pathlib.Path):
    """Return dict: ts, pos(N,3), vel(N,3), euler(N,3)[deg], quat(N,4)[xyzw], feat(N)."""
    ts, pos, vel, euler, quat, feat = [], [], [], [], [], []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            p = line.split()
            ts.append(float(p[0]))
            pos.append([float(p[1]), float(p[2]), float(p[3])])
            vel.append([float(p[4]), float(p[5]), float(p[6])])
            euler.append([float(p[7]), float(p[8]), float(p[9])])
            # quaternion optional (older logs may not have it)
            if len(p) >= 14:
                quat.append([float(p[10]), float(p[11]), float(p[12]), float(p[13])])
            else:
                quat.append([0.0, 0.0, 0.0, 1.0])
            # feature count optional
            feat.append(int(p[14]) if len(p) >= 15 else 0)
    return {
        "ts":    np.array(ts),
        "pos":   np.array(pos),
        "vel":   np.array(vel),
        "euler": np.array(euler),
        "quat":  np.array(quat),          # columns: qx qy qz qw
        "feat":  np.array(feat, dtype=int),
    }


def load_metrics(path: pathlib.Path) -> dict:
    m = {}
    if not path.exists():
        return m
    with open(path) as f:
        for line in f:
            if ":" in line:
                k, v = line.strip().split(":", 1)
                m[k.strip()] = v.strip()
    return m


# ──────────────────────────────────────────
# Alignment & ATE
# ──────────────────────────────────────────

def rigid_align(est: np.ndarray, gt: np.ndarray):
    mu_est = est.mean(axis=0)
    mu_gt  = gt.mean(axis=0)
    est_c  = est - mu_est
    gt_c   = gt  - mu_gt

    cov = (gt_c.T @ est_c) / est.shape[0]
    u, _, vt = np.linalg.svd(cov)
    s = np.eye(3)
    if np.linalg.det(u) * np.linalg.det(vt) < 0:
        s[-1, -1] = -1.0

    r = u @ s @ vt
    t = mu_gt - r @ mu_est
    aligned = (r @ est.T).T + t
    return aligned, r, t


def align_and_ate(est: np.ndarray, gt_interp: np.ndarray):
    aligned, r, t = rigid_align(est, gt_interp)
    err = np.linalg.norm(aligned - gt_interp, axis=1)
    return aligned, r, t, float(np.sqrt(np.mean(err ** 2))), err


def interp_gt(gt_ts, gt_pos, est_ts):
    out = np.zeros((len(est_ts), 3))
    for ax in range(3):
        out[:, ax] = np.interp(est_ts, gt_ts, gt_pos[:, ax])
    return out


# ──────────────────────────────────────────
# Plot 1: trajectory_plot.png  (XY + XYZ)
# ──────────────────────────────────────────

def plot_trajectory(result_dir: pathlib.Path,
                    est_ts, est_pos, gt_interp, est_aligned,
                    ate_cm, metrics):
    out_png = result_dir / "trajectory_plot.png"
    has_gt  = gt_interp is not None

    t = est_ts - est_ts[0]
    labels     = ["X [m]", "Y [m]", "Z [m]"]
    colors_est = ["#2196F3", "#4CAF50", "#F44336"]
    colors_gt  = ["#90CAF9", "#A5D6A7", "#EF9A9A"]

    fig = plt.figure(figsize=(15, 6))
    gs  = gridspec.GridSpec(1, 2, figure=fig, wspace=0.35)

    ax_xy = fig.add_subplot(gs[0])
    if has_gt:
        ax_xy.plot(gt_interp[:, 0], gt_interp[:, 1],
                   color="#FF9800", lw=1.2, ls="--", label="GT", alpha=0.8)
    ax_xy.plot(est_aligned[:, 0], est_aligned[:, 1],
               color="#2196F3", lw=1.4, label="Estimated")
    ax_xy.scatter(*est_aligned[0, :2],  marker="*", s=150, color="#2196F3", zorder=5)
    ax_xy.scatter(*est_aligned[-1, :2], marker="s", s=80,  color="#2196F3", zorder=5)
    if has_gt:
        ax_xy.scatter(*gt_interp[0, :2],  marker="*", s=150, color="#FF9800", zorder=5)
        ax_xy.scatter(*gt_interp[-1, :2], marker="s", s=80,  color="#FF9800", zorder=5)
    ax_xy.set_xlabel("X [m]"); ax_xy.set_ylabel("Y [m]")
    ax_xy.set_title("XY Top-view")
    ax_xy.legend(fontsize=9); ax_xy.set_aspect("equal")
    ax_xy.grid(True, lw=0.4, alpha=0.5)

    ax_xyz = fig.add_subplot(gs[1])
    for i in range(3):
        if has_gt:
            ax_xyz.plot(t, gt_interp[:, i], color=colors_gt[i], lw=1.0, ls="--", alpha=0.7)
        ax_xyz.plot(t, est_aligned[:, i], color=colors_est[i], lw=1.2, label=labels[i])
    if has_gt:
        ax_xyz.plot([], [], color="gray", lw=1.0, ls="--", label="GT")
    ax_xyz.plot([], [], color="gray", lw=1.2, label="Est")
    ax_xyz.set_xlabel("Time [s]"); ax_xyz.set_ylabel("Position [m]")
    ax_xyz.set_title("XYZ vs Time")
    ax_xyz.legend(fontsize=8, ncol=2)
    ax_xyz.grid(True, lw=0.4, alpha=0.5)

    frames = metrics.get("frames", str(len(est_ts)))
    fig.suptitle(
        f"LC-EKF VIO  ·  {metrics.get('dataset_type', result_dir.name)}"
        f"  ·  {frames} frames  ·  ATE RMSE: {ate_cm:.1f} cm",
        fontsize=13, fontweight="bold"
    )
    plt.savefig(out_png, dpi=150, bbox_inches="tight")
    print(f"[plot] Saved → {out_png}")
    plt.close(fig)


# ──────────────────────────────────────────
# Plot 2: state_plot.png  (pos / vel / euler / error)
# ──────────────────────────────────────────

def origin_align_error(est_pos: np.ndarray, gt_interp: np.ndarray,
                       r_align: np.ndarray) -> np.ndarray:
    """
    Fix coordinate-frame rotation (Umeyama R) but shift only at t=0.
    Shows true cumulative drift without global translation optimization.
    """
    est_rot = (r_align @ est_pos.T).T          # correct frame rotation
    shift   = gt_interp[0] - est_rot[0]        # translate so t=0 error = 0
    return np.linalg.norm(est_rot + shift - gt_interp, axis=1)


def plot_state(result_dir: pathlib.Path, state: dict,
               gt_interp, est_pos_raw, est_aligned, per_frame_err,
               r_align, ate_cm, metrics):
    out_png = result_dir / "state_plot.png"
    has_gt  = gt_interp is not None

    t = state["ts"] - state["ts"][0]

    colors     = ["#2196F3", "#4CAF50", "#F44336"]        # X, Y, Z
    colors_gt  = ["#90CAF9", "#A5D6A7", "#EF9A9A"]
    euler_colors = ["#9C27B0", "#FF9800", "#009688"]       # roll, pitch, yaw
    quat_colors  = ["#E91E63", "#FF5722", "#795548", "#607D8B"]  # qx qy qz qw

    fig, axes = plt.subplots(6, 1, figsize=(14, 24), sharex=True)
    fig.subplots_adjust(hspace=0.38)

    # ── Row 0: Position ──────────────────────
    ax = axes[0]
    labels_pos = ["X [m]", "Y [m]", "Z [m]"]
    for i in range(3):
        if has_gt:
            ax.plot(t, gt_interp[:, i], color=colors_gt[i],
                    lw=1.0, ls="--", alpha=0.8)
        ax.plot(t, est_aligned[:, i], color=colors[i],
                lw=1.2, label=labels_pos[i])
    if has_gt:
        ax.plot([], [], color="gray", lw=1.0, ls="--", label="GT")
    ax.set_ylabel("Position [m]")
    ax.set_title("Position XYZ")
    ax.legend(fontsize=8, ncol=4)
    ax.grid(True, lw=0.4, alpha=0.5)

    # ── Row 1: Velocity ──────────────────────
    ax = axes[1]
    labels_vel = ["Vx [m/s]", "Vy [m/s]", "Vz [m/s]"]
    for i in range(3):
        ax.plot(t, state["vel"][:, i], color=colors[i],
                lw=1.2, label=labels_vel[i])
    ax.set_ylabel("Velocity [m/s]")
    ax.set_title("Velocity XYZ (Estimated)")
    ax.legend(fontsize=8, ncol=3)
    ax.grid(True, lw=0.4, alpha=0.5)

    # ── Row 2: Euler angles ──────────────────
    ax = axes[2]
    euler_labels = ["Roll [°]", "Pitch [°]", "Yaw [°]"]
    for i in range(3):
        ax.plot(t, state["euler"][:, i], color=euler_colors[i],
                lw=1.2, label=euler_labels[i])
    ax.set_ylabel("Angle [deg]")
    ax.set_title("Euler Angles ZYX (Estimated)")
    ax.legend(fontsize=8, ncol=3)
    ax.grid(True, lw=0.4, alpha=0.5)

    # ── Row 3: Quaternion ────────────────────
    ax = axes[3]
    quat_labels = ["qx", "qy", "qz", "qw"]
    for i in range(4):
        ax.plot(t, state["quat"][:, i], color=quat_colors[i],
                lw=1.2, label=quat_labels[i])
    ax.set_ylabel("[-]")
    ax.set_title("Quaternion (Estimated)")
    ax.legend(fontsize=8, ncol=4)
    ax.grid(True, lw=0.4, alpha=0.5)

    # ── Row 4: Per-frame position error ──────
    ax = axes[4]
    if has_gt and per_frame_err is not None:
        raw_err = origin_align_error(est_pos_raw, gt_interp, r_align)
        ax.plot(t, raw_err, color="#FF9800", lw=1.4,
                label="Raw drift (origin-aligned)", zorder=3)
        ax.plot(t, per_frame_err, color="#E91E63", lw=1.2,
                label="Umeyama-aligned error", zorder=2)
        ax.fill_between(t, raw_err, alpha=0.10, color="#FF9800")
        ax.fill_between(t, per_frame_err, alpha=0.10, color="#E91E63")
        ax.set_ylabel("Error [m]")
        ax.set_title(f"Per-frame Position Error  (ATE RMSE = {ate_cm:.1f} cm)")
        ax.legend(fontsize=8)
    else:
        ax.text(0.5, 0.5, "No GT available", ha="center", va="center",
                transform=ax.transAxes, fontsize=12, color="gray")
        ax.set_title("Per-frame Position Error")
    ax.set_xlabel("")
    ax.grid(True, lw=0.4, alpha=0.5)

    # ── Row 5: Feature count ─────────────
    ax = axes[5]
    feat = state["feat"]
    ax.plot(t, feat, color="#3F51B5", lw=1.2, label="Tracked features")
    ax.axhline(250, color="gray",    lw=0.8, ls="--", label="Target (250)")
    ax.axhline(10,  color="#F44336", lw=0.8, ls="--", label="Min PnP (10)")
    ax.fill_between(t, feat, where=(feat < 10), color="#F44336", alpha=0.25)
    ax.set_ylabel("Count")
    ax.set_title("Tracked Feature Count")
    ax.legend(fontsize=8, ncol=3)
    ax.set_xlabel("Time [s]")
    ax.grid(True, lw=0.4, alpha=0.5)

    frames = metrics.get("frames", str(len(t)))
    fig.suptitle(
        f"LC-EKF VIO  ·  {metrics.get('dataset_type', result_dir.name)}"
        f"  ·  {frames} frames  ·  ATE RMSE: {ate_cm:.1f} cm",
        fontsize=13, fontweight="bold"
    )
    plt.savefig(out_png, dpi=150, bbox_inches="tight")
    print(f"[plot] Saved → {out_png}")
    plt.close(fig)


# ──────────────────────────────────────────
# Entry point
# ──────────────────────────────────────────

def plot(result_dir: pathlib.Path):
    est_file   = result_dir / "trajectory_tum.txt"
    gt_file    = result_dir / "gt_tum.txt"
    met_file   = result_dir / "metrics.txt"
    state_file = result_dir / "state_log.txt"

    if not est_file.exists():
        sys.exit(f"[ERROR] Not found: {est_file}")

    est_ts, est_pos = load_tum(est_file)
    metrics = load_metrics(met_file)

    has_gt = gt_file.exists()
    per_frame_err = None
    r_align = np.eye(3)
    t_align = np.zeros(3)

    if has_gt:
        gt_ts, gt_pos = load_tum(gt_file)
        gt_interp = interp_gt(gt_ts, gt_pos, est_ts)
        est_aligned, r_align, t_align, ate_m, per_frame_err = align_and_ate(est_pos, gt_interp)
        ate_cm = ate_m * 100
    else:
        gt_interp   = None
        est_aligned = est_pos
        ate_cm = float(metrics.get("ate_rmse_cm", 0))

    plot_trajectory(result_dir, est_ts, est_pos, gt_interp, est_aligned,
                    ate_cm, metrics)

    if state_file.exists():
        state = load_state_log(state_file)
        aligned_state_pos = (r_align @ state["pos"].T).T + t_align
        state_for_plot = dict(state)
        state_for_plot["pos"] = aligned_state_pos
        plot_state(result_dir, state_for_plot, gt_interp, est_pos,
                   est_aligned, per_frame_err, r_align, ate_cm, metrics)
    else:
        print("[plot] state_log.txt not found — skipping state_plot.png")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python plot_trajectory.py <result_dir>")
        sys.exit(1)
    plot(pathlib.Path(sys.argv[1]))
