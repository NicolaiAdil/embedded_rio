#!/usr/bin/env python3
"""Compare offline replay output against PX4 EKF2 ground truth from a .ulg.

Run from the venv that has evo + pyulog installed, e.g.:
    ~/ntnu/master/ulogs/.venv/bin/python3 \
        tools/replay/scripts/compare_with_ulog.py \
        runs/baseline /path/to/flight.ulg --save_results runs/baseline/cmp \
        [--include_live] [--max_time_diff 0.2]

Inputs:
    <replay_dir>/state.csv  — produced by tools/replay/rio_replay
                              (Teensy world frame, t in Teensy seconds)
    <ulg_file>              — PX4 .ulg with vehicle_visual_odometry,
                              estimator_local_position, estimator_attitude

Method:
    1. Read all topics from .ulg.
    2. Compute Teensy→PX4 time offset from
       median(vvo.timestamp − vvo.timestamp_sample).
    3. Apply offset to replay's t and rotate replay state from Teensy
       world frame into PX4 NED via q_t = (w=0, x=0, y=1, z=0).
    4. Build evo PoseTrajectory3D for replay and EKF2, associate by
       timestamp, compute APE / RPE / per-axis errors and emit plots.
"""
import os
os.environ.setdefault("MPLBACKEND", "Agg")

import argparse
import copy
import json
import pprint
import sys
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from pyulog import ULog
from scipy.spatial.transform import Rotation

from evo.tools import log as evo_log
evo_log.configure_logging(verbose=True, debug=False, silent=False)
from evo.tools.settings import SETTINGS
SETTINGS.plot_backend = "Agg"
SETTINGS.plot_usetex = False

from evo.tools import plot
from evo.core import sync, metrics
from evo.core.units import Unit
from evo.core.trajectory import PoseTrajectory3D


# ── ulog helpers (lifted verbatim from /home/nicolai/ntnu/master/ulogs/plotter.py)

def get_topic(ulog, name, instance=0):
    for d in ulog.data_list:
        if d.name == name and d.multi_id == instance:
            return pd.DataFrame(d.data)
    raise ValueError(f"Topic '{name}' not found in log")


def load_visual_odometry(ulog):
    df = get_topic(ulog, "vehicle_visual_odometry")
    pos = np.column_stack([df["position[0]"], df["position[1]"], df["position[2]"]])
    q_wxyz = np.column_stack([df["q[0]"], df["q[1]"], df["q[2]"], df["q[3]"]])
    vel = np.column_stack([df["velocity[0]"], df["velocity[1]"], df["velocity[2]"]])
    return df["timestamp"].values, pos, q_wxyz, vel, df


def load_estimator_local_position(ulog):
    df_lpos = get_topic(ulog, "estimator_local_position")
    df_att  = get_topic(ulog, "estimator_attitude")
    pos = np.column_stack([df_lpos["x"], df_lpos["y"], df_lpos["z"]])
    vel = np.column_stack([df_lpos["vx"], df_lpos["vy"], df_lpos["vz"]])
    ts_lpos = df_lpos["timestamp"].values
    ts_att  = df_att["timestamp"].values
    q_wxyz = np.column_stack([
        np.interp(ts_lpos, ts_att, df_att["q[0]"]),
        np.interp(ts_lpos, ts_att, df_att["q[1]"]),
        np.interp(ts_lpos, ts_att, df_att["q[2]"]),
        np.interp(ts_lpos, ts_att, df_att["q[3]"]),
    ])
    return ts_lpos, pos, q_wxyz, vel, df_lpos


def make_pose_trajectory(timestamps_us, positions, quaternions_wxyz):
    t_s = timestamps_us / 1e6
    norms = np.linalg.norm(quaternions_wxyz, axis=1)
    valid = ((norms > 1e-6)
             & np.all(np.isfinite(quaternions_wxyz), axis=1)
             & np.all(np.isfinite(positions), axis=1))
    if (n_drop := int(np.sum(~valid))) > 0:
        print(f"  dropping {n_drop} invalid rows (zero/NaN quat or pos)")
    t_s, positions, quaternions_wxyz = t_s[valid], positions[valid], quaternions_wxyz[valid]

    poses = []
    for p, q in zip(positions, quaternions_wxyz):
        w, x, y, z = q
        T = np.eye(4)
        T[:3, :3] = Rotation.from_quat([x, y, z, w]).as_matrix()
        T[:3,  3] = p
        poses.append(T)
    return PoseTrajectory3D(poses_se3=poses, timestamps=t_s)


def vel_to_body_frame(vel_ned, q_wxyz):
    vel_body = np.empty_like(vel_ned)
    for i, (v, q) in enumerate(zip(vel_ned, q_wxyz)):
        w, x, y, z = q
        R = Rotation.from_quat([x, y, z, w]).as_matrix()
        vel_body[i] = R.T @ v
    return vel_body


def handle_fig(fig, tag, save_dir):
    if save_dir is not None:
        Path(save_dir).mkdir(parents=True, exist_ok=True)
        out = Path(save_dir) / f"{tag}.svg"
        fig.savefig(out, bbox_inches="tight")
        print(f"  saved → {out}")
    else:
        out = f"{tag}.png"
        fig.savefig(out, bbox_inches="tight", dpi=150)
        print(f"  saved → {out}")
    plt.close(fig)


# ── new helpers: replay state.csv + frame transform + time offset

def load_replay_state(replay_dir: Path):
    """Read replay's state.csv. Returns:
        t_s   : Teensy seconds since boot (float64, N)
        pos_W : (N, 3)   Teensy world frame position (z up, gravity-aligned)
        vel_W : (N, 3)   Teensy world frame velocity
        q_W   : (N, 4)   attitude quaternion (w, x, y, z)
    """
    csv = replay_dir / "state.csv"
    if not csv.is_file():
        raise SystemExit(f"missing {csv}")
    df = pd.read_csv(csv)
    pos_W = df[["p_x", "p_y", "p_z"]].to_numpy()
    vel_W = df[["v_x", "v_y", "v_z"]].to_numpy()
    q_W   = df[["q_w", "q_x", "q_y", "q_z"]].to_numpy()
    return df["t"].to_numpy(), pos_W, vel_W, q_W


# Frame transform applied in firmware src/main.cpp:108-112 to send ODOMETRY
# in PX4 NED. Replay's state.csv is in raw Teensy world frame, so we apply
# the same rotation here.
#
#   q_t = Quat(w=0, x=0, y=1, z=0)  (180° about y, maps (x,y,z) → (-x, y, -z))
#   p_NED = q_t · p_W
#   q_NED = q_t · q_W · q_t.inv()
_Q_T_XYZW = np.array([0.0, 1.0, 0.0, 0.0])  # scipy uses (x, y, z, w)
_R_T = Rotation.from_quat(_Q_T_XYZW)


def transform_replay_to_ned(pos_W, vel_W, q_W_wxyz):
    pos_N = _R_T.apply(pos_W)
    vel_N = _R_T.apply(vel_W)
    q_W_xyzw = q_W_wxyz[:, [1, 2, 3, 0]]
    R_W = Rotation.from_quat(q_W_xyzw)
    R_N = _R_T * R_W * _R_T.inv()
    q_N_xyzw = R_N.as_quat()
    q_N_wxyz = q_N_xyzw[:, [3, 0, 1, 2]]
    return pos_N, vel_N, q_N_wxyz


def compute_time_offset(df_vvo):
    """Diagnostic only: median(timestamp − timestamp_sample) (µs).
    Unreliable when the sender doesn't echo MAVLink TIMESYNC: PX4 then
    fills timestamp_sample with its own clock (= timestamp), so this
    returns ≈ 0 even when there's a real Teensy↔PX4 boot-time delta."""
    ts_vvo  = df_vvo["timestamp"].to_numpy().astype(np.int64)
    ts_samp = df_vvo["timestamp_sample"].to_numpy().astype(np.int64)
    diff = ts_vvo - ts_samp
    offset = float(np.median(diff))
    return offset, float(np.std(diff)), int(diff.size)


def compute_time_offset_via_first_publish(replay_dir, ts_vvo):
    """Anchor the first 'RAD'-source row of replay/state.csv to the first
    live VVO entry in the .ulg. The firmware publishes ODOMETRY on every
    radar frame after attitude init (see processRadar in src/main.cpp),
    so these two events refer to the same instant — regardless of what
    PX4 stored in timestamp_sample.

    Returns offset_us such that t_px4 = t_teensy + offset_us.
    """
    df = pd.read_csv(replay_dir / "state.csv", usecols=["t", "source"])
    rad = df[df["source"] == "RAD"]
    if rad.empty:
        raise SystemExit(
            "no 'RAD'-source rows in state.csv — can't anchor replay time")
    first_replay_us = float(rad["t"].iloc[0]) * 1e6
    first_vvo_us    = float(ts_vvo[0])
    return first_vvo_us - first_replay_us


# ── main ──────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("replay_dir", type=Path, help="Replay output directory (state.csv inside)")
    ap.add_argument("ulg",        type=Path, help="PX4 .ulg log file")
    ap.add_argument("--save_results", type=Path, default=None,
                    help="Output dir for SVGs (default: <replay_dir>/cmp)")
    ap.add_argument("--include_live", action="store_true",
                    help="Also overlay the live VVO trace as a 3rd line")
    ap.add_argument("--max_time_diff", type=float, default=0.2,
                    help="evo association tolerance [s]")
    args = ap.parse_args()

    if not args.replay_dir.is_dir():
        raise SystemExit(f"not a directory: {args.replay_dir}")
    if not args.ulg.is_file():
        raise SystemExit(f"missing .ulg: {args.ulg}")
    save_dir = args.save_results or (args.replay_dir / "cmp")

    # ── load .ulg ──────────────────────────────────────────────────────────
    ulog = ULog(str(args.ulg))
    ts_vvo, pos_vvo, q_vvo, vel_vvo, df_vvo = load_visual_odometry(ulog)
    ts_ekf, pos_ekf, q_ekf, vel_ekf, _      = load_estimator_local_position(ulog)
    print(f"vehicle_visual_odometry  : {len(ts_vvo)} msgs, "
          f"{(ts_vvo[-1]-ts_vvo[0])/1e6:.2f}s")
    print(f"estimator_local_position : {len(ts_ekf)} msgs, "
          f"{(ts_ekf[-1]-ts_ekf[0])/1e6:.2f}s")

    # ── time offset Teensy → PX4 ───────────────────────────────────────────
    # Primary: anchor first replay 'RAD' publish to first live VVO entry.
    offset_us = compute_time_offset_via_first_publish(args.replay_dir, ts_vvo)
    # Diagnostic: timestamp_sample method (unreliable without TIMESYNC).
    offset_diag, std_us, n_samp = compute_time_offset(df_vvo)
    print(f"\nTime offset (PX4 − Teensy):")
    print(f"  via first VVO publish : {offset_us/1e6:+.6f} s   "
          f"({offset_us:+.0f} µs)   [USED]")
    print(f"  via timestamp_sample  : {offset_diag/1e6:+.6f} s   "
          f"std={std_us/1e3:.3f} ms  ({n_samp} samples)  [diagnostic]")

    # Persist for downstream scripts.
    Path(save_dir).mkdir(parents=True, exist_ok=True)
    with (Path(save_dir) / "time_offset.json").open("w") as f:
        json.dump({"offset_us": offset_us,
                   "offset_us_via_timestamp_sample": offset_diag,
                   "std_us_via_timestamp_sample": std_us,
                   "n_vvo_samples": n_samp}, f, indent=2)

    # ── load replay, transform to NED, apply offset ───────────────────────
    t_replay_s, pos_W, vel_W, q_W = load_replay_state(args.replay_dir)
    pos_N, vel_N, q_N = transform_replay_to_ned(pos_W, vel_W, q_W)
    ts_replay_us = t_replay_s * 1e6 + offset_us
    print(f"\nreplay/state.csv         : {len(t_replay_s)} rows, "
          f"{t_replay_s[-1]-t_replay_s[0]:.2f}s   "
          f"first PX4-aligned t = {ts_replay_us[0]/1e6:.3f}s "
          f"(EKF2 first = {ts_ekf[0]/1e6:.3f}s)")

    # ── build trajectories and associate ───────────────────────────────────
    traj_ref = make_pose_trajectory(ts_ekf,        pos_ekf, q_ekf)
    traj_est = make_pose_trajectory(ts_replay_us,  pos_N,   q_N)

    n_replay_before = traj_est.num_poses
    traj_ref_a, traj_est_a = sync.associate_trajectories(
        traj_ref, traj_est,
        max_diff=args.max_time_diff,
        first_name="EKF2 (GT)", snd_name="replay")
    matched_pct = 100.0 * traj_est_a.num_poses / max(n_replay_before, 1)
    print(f"\nassociated {traj_est_a.num_poses} / {n_replay_before} replay samples "
          f"to EKF2 within {args.max_time_diff*1000:.0f} ms ({matched_pct:.1f}%)")
    if matched_pct < 50:
        print("  fewer than 50% matched — time sync is likely wrong.")

    traj_est_aligned = copy.deepcopy(traj_est_a)
    traj_est_aligned.align_origin(traj_ref_a)

    # Common time origin = first EKF2 sample. Time axis is "seconds since
    # EKF2 start" so EKF2 spans 0..end and replay/VVO sit at their actual
    # offset (replay/VVO often start much later than EKF2).
    t0_s = float(ts_ekf[0]) / 1e6

    # Full traces (not post-association) for time-series plots.
    t_ekf_rel    = ts_ekf       / 1e6 - t0_s
    t_replay_rel = ts_replay_us / 1e6 - t0_s

    # Per-trace origin-align positions (each trace starts at 0,0,0).
    pos_ekf_rel = pos_ekf - pos_ekf[0]
    pos_N_rel   = pos_N   - pos_N[0]

    def _euler_deg(q_wxyz):
        q_xyzw = q_wxyz[:, [1, 2, 3, 0]]
        return np.rad2deg(np.unwrap(
            Rotation.from_quat(q_xyzw).as_euler("xyz"), axis=0))

    eul_ekf_full    = _euler_deg(q_ekf)
    eul_replay_full = _euler_deg(q_N)

    # Post-association arrays — used only for paired-error metrics
    # (position error, APE, RPE).
    p_ref         = traj_ref_a.positions_xyz
    p_est         = traj_est_aligned.positions_xyz
    t_paired_rel  = np.asarray(traj_ref_a.timestamps) - t0_s

    # Optional live VVO overlay (already in PX4 frame & clock).
    t_vvo_rel = None
    pos_vvo_rel = None
    eul_vvo_full = None
    if args.include_live:
        t_vvo_rel    = ts_vvo / 1e6 - t0_s
        pos_vvo_rel  = pos_vvo - pos_vvo[0]
        eul_vvo_full = _euler_deg(q_vvo)

    # ── plots ──────────────────────────────────────────────────────────────
    pos_labels = ["x [m]", "y [m]", "z [m]"]
    rpy_labels = ["roll [deg]", "pitch [deg]", "yaw [deg]"]

    # Pose vs time — full traces, starting at their actual time offsets.
    fig, axs = plt.subplots(3, 2, sharex="col", figsize=(12, 8))
    for i in range(3):
        axs[i, 0].plot(t_ekf_rel,    pos_ekf_rel[:, i], label="EKF2 (GT)")
        axs[i, 0].plot(t_replay_rel, pos_N_rel[:, i],   label="replay", alpha=0.85)
        if t_vvo_rel is not None:
            axs[i, 0].plot(t_vvo_rel, pos_vvo_rel[:, i], ":",
                           color="darkorange", alpha=0.7, label="live VVO")
        axs[i, 0].set_ylabel(pos_labels[i]); axs[i, 0].grid(True, alpha=0.3)

        axs[i, 1].plot(t_ekf_rel,    eul_ekf_full[:, i],    label="EKF2 (GT)")
        axs[i, 1].plot(t_replay_rel, eul_replay_full[:, i], label="replay", alpha=0.85)
        if t_vvo_rel is not None:
            axs[i, 1].plot(t_vvo_rel, eul_vvo_full[:, i], ":",
                           color="darkorange", alpha=0.7, label="live VVO")
        axs[i, 1].set_ylabel(rpy_labels[i]); axs[i, 1].grid(True, alpha=0.3)

    axs[0, 0].set_title("Position vs time (per-trace origin-aligned)")
    axs[0, 1].set_title("Attitude vs time")
    axs[0, 0].legend(fontsize=8); axs[0, 1].legend(fontsize=8)
    axs[-1, 0].set_xlabel("t [s] (since EKF2 start)")
    axs[-1, 1].set_xlabel("t [s] (since EKF2 start)")
    plt.tight_layout(); handle_fig(fig, "compare_pose", save_dir)

    # XY trajectory overlay — per-trace origin-aligned, no time axis.
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.plot(pos_ekf_rel[:, 0], pos_ekf_rel[:, 1], "--", color="gray",
            lw=0.8, label="EKF2 (GT)")
    ax.plot(pos_N_rel[:, 0],   pos_N_rel[:, 1],          color="steelblue",
            lw=0.8, label="replay")
    if pos_vvo_rel is not None:
        ax.plot(pos_vvo_rel[:, 0], pos_vvo_rel[:, 1], ":",
                color="darkorange", lw=0.8, alpha=0.7, label="live VVO")
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    ax.set_title("Trajectory XY (per-trace origin-aligned)"); ax.set_aspect("equal")
    ax.grid(True, alpha=0.3); ax.legend(fontsize=8)
    plt.tight_layout(); handle_fig(fig, "compare_trajectory_xy", save_dir)

    # Body-frame velocity vs time — full traces, common t-axis.
    vel_ekf_body    = vel_to_body_frame(vel_ekf, q_ekf)
    vel_replay_body = vel_to_body_frame(vel_N,   q_N)
    fig, axs = plt.subplots(3, 1, sharex=True, figsize=(12, 7))
    for i, lbl in enumerate(["vx_body [m/s]", "vy_body [m/s]", "vz_body [m/s]"]):
        axs[i].plot(t_ekf_rel,    vel_ekf_body[:, i],    label="EKF2 (GT)")
        axs[i].plot(t_replay_rel, vel_replay_body[:, i], label="replay", alpha=0.85)
        if t_vvo_rel is not None:
            vel_vvo_body = vel_to_body_frame(vel_vvo, q_vvo)
            axs[i].plot(t_vvo_rel, vel_vvo_body[:, i], ":",
                        color="darkorange", alpha=0.7, label="live VVO")
        axs[i].set_ylabel(lbl); axs[i].grid(True, alpha=0.3); axs[i].legend(fontsize=8)
    axs[0].set_title("Velocity vs time (body frame)")
    axs[-1].set_xlabel("t [s] (since EKF2 start)")
    plt.tight_layout(); handle_fig(fig, "compare_velocity", save_dir)

    # Position error — needs paired data (post-association).
    err      = p_est - p_ref
    err_norm = np.linalg.norm(err, axis=1)
    rms      = np.sqrt(np.mean(err**2, axis=0))
    fig, axs = plt.subplots(4, 1, sharex=True, figsize=(12, 9))
    for i, (lbl, col) in enumerate(zip(
            ["ΔX [m]", "ΔY [m]", "ΔZ [m]"],
            ["steelblue", "darkorange", "forestgreen"])):
        axs[i].plot(t_paired_rel, err[:, i], color=col, lw=0.8)
        axs[i].axhline(0, color="black", lw=0.5, ls=":")
        axs[i].set_ylabel(lbl); axs[i].grid(True, alpha=0.3)
    axs[3].plot(t_paired_rel, err_norm, color="crimson", lw=0.8)
    axs[3].set_ylabel("|ΔP| [m]"); axs[3].grid(True, alpha=0.3)
    axs[0].set_title(
        f"Position error: replay − EKF2  "
        f"(RMS: X={rms[0]:.3f}  Y={rms[1]:.3f}  Z={rms[2]:.3f} m)")
    axs[-1].set_xlabel("t [s] (since EKF2 start)")
    plt.tight_layout(); handle_fig(fig, "compare_position_error", save_dir)

    # APE
    ape = metrics.APE(metrics.PoseRelation.translation_part)
    ape.process_data((traj_ref_a, traj_est_aligned))
    ape_stats = ape.get_all_statistics()
    print("\n── APE (translation) ──")
    pprint.pprint(ape_stats)
    fig = plt.figure(figsize=(10, 4))
    plot.error_array(
        fig.gca(), ape.error, x_array=t_paired_rel,
        statistics={s: v for s, v in ape_stats.items() if s != "sse"},
        name="APE", title="APE w.r.t. translation  (GT = EKF2, est = replay)",
        xlabel="t [s] (since EKF2 start)")
    handle_fig(fig, "compare_ape", save_dir)

    fig = plt.figure(figsize=(8, 8))
    ax = plot.prepare_axis(fig, plot.PlotMode.xy)
    plot.traj(ax, plot.PlotMode.xy, traj_ref_a, "--", "gray", "EKF2 (GT)")
    plot.traj_colormap(ax, traj_est_aligned, ape.error, plot.PlotMode.xy,
                       title="Replay trajectory coloured by APE",
                       min_map=ape_stats["min"], max_map=ape_stats["max"])
    ax.legend()
    handle_fig(fig, "compare_ape_trajectory", save_dir)

    # RPE
    traj_length = float(np.sum(np.linalg.norm(np.diff(p_ref, axis=0), axis=1)))
    rpe_delta   = float(np.clip(traj_length * 0.1, 0.05, 10.0))
    print(f"\nGT length: {traj_length:.3f} m  →  RPE δ = {rpe_delta:.3f} m")
    rpe = metrics.RPE(pose_relation=metrics.PoseRelation.translation_part,
                      delta=rpe_delta, delta_unit=Unit.meters, all_pairs=True)
    rpe.process_data((traj_ref_a, traj_est_aligned))
    rpe_stats = rpe.get_all_statistics()
    print(f"\n── RPE (translation, δ={rpe_delta:.3f} m, all_pairs) ──")
    pprint.pprint(rpe_stats)
    fig = plt.figure(figsize=(10, 4))
    plot.error_array(
        fig.gca(), rpe.error, x_array=np.arange(len(rpe.error)),
        statistics={s: v for s, v in rpe_stats.items() if s != "sse"},
        name="RPE", title=f"RPE  δ={rpe_delta:.3f} m, all_pairs",
        xlabel="pair index")
    handle_fig(fig, "compare_rpe", save_dir)

    # ── 2×3 overview (mirrors tools/replay/scripts/plot_runs.py) ────────────
    # EKF2 / replay / live-VVO overlaid on trajectory, altitude, speed.
    # Replay-only on σ-evolution, radar innovation hist, baro innovation.
    cov_df  = pd.read_csv(args.replay_dir / "cov_diag.csv")
    rinn_df = pd.read_csv(args.replay_dir / "radar_innov.csv")
    binn_df = pd.read_csv(args.replay_dir / "baro_innov.csv")
    cov_t_rel  = cov_df["t"].to_numpy()  + offset_us / 1e6 - t0_s
    binn_t_rel = binn_df["t"].to_numpy() + offset_us / 1e6 - t0_s

    fig, axes = plt.subplots(2, 3, figsize=(17, 10))

    # [0,0] xy trajectory (top-down)
    ax = axes[0, 0]
    ax.plot(pos_ekf_rel[:, 0], pos_ekf_rel[:, 1], "--", color="gray",
            lw=0.8, label="EKF2 (GT)")
    ax.plot(pos_N_rel[:, 0],   pos_N_rel[:, 1],   color="steelblue",
            lw=0.8, label="replay")
    if pos_vvo_rel is not None:
        ax.plot(pos_vvo_rel[:, 0], pos_vvo_rel[:, 1], ":",
                color="darkorange", lw=0.8, alpha=0.8, label="live VVO")
    ax.set_xlabel("p_x [m]"); ax.set_ylabel("p_y [m]")
    ax.set_title("Trajectory (top-down)")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(alpha=0.3); ax.legend(fontsize=8)

    # [0,1] altitude vs t (with replay ±σ_z band)
    ax = axes[0, 1]
    sig_z = np.sqrt(np.clip(cov_df["P02"].to_numpy(), 0, None))
    pz_replay = pos_N_rel[:, 2]
    n = min(len(pz_replay), len(sig_z), len(t_replay_rel))
    ax.fill_between(t_replay_rel[:n], pz_replay[:n] - sig_z[:n],
                    pz_replay[:n] + sig_z[:n], color="steelblue", alpha=0.15)
    ax.plot(t_ekf_rel,    pos_ekf_rel[:, 2], "--", color="gray",
            lw=0.8, label="EKF2 (GT)")
    ax.plot(t_replay_rel, pz_replay,         color="steelblue",
            lw=0.6, label="replay (±σ_z)")
    if pos_vvo_rel is not None:
        ax.plot(t_vvo_rel, pos_vvo_rel[:, 2], ":", color="darkorange",
                lw=0.8, alpha=0.8, label="live VVO")
    ax.set_xlabel("t [s]"); ax.set_ylabel("p_z [m]")
    ax.set_title("Altitude")
    ax.grid(alpha=0.3); ax.legend(fontsize=8)

    # [0,2] speed |v|
    ax = axes[0, 2]
    spd_ekf    = np.linalg.norm(vel_ekf, axis=1)
    spd_replay = np.linalg.norm(vel_N,   axis=1)
    ax.plot(t_ekf_rel,    spd_ekf,    "--", color="gray",
            lw=0.8, label="EKF2 (GT)")
    ax.plot(t_replay_rel, spd_replay, color="steelblue",
            lw=0.6, label="replay")
    if pos_vvo_rel is not None:
        spd_vvo = np.linalg.norm(vel_vvo, axis=1)
        ax.plot(t_vvo_rel, spd_vvo, ":", color="darkorange",
                lw=0.8, alpha=0.8, label="live VVO")
    ax.set_xlabel("t [s]"); ax.set_ylabel("|v| [m/s]")
    ax.set_title("Speed")
    ax.grid(alpha=0.3); ax.legend(fontsize=8)

    # [1,0] replay covariance evolution (log-scale) — pos / vel / att RMS
    ax = axes[1, 0]
    sp = np.sqrt(np.clip(cov_df["P00"] + cov_df["P01"] + cov_df["P02"], 0, None) / 3)
    sv = np.sqrt(np.clip(cov_df["P03"] + cov_df["P04"] + cov_df["P05"], 0, None) / 3)
    sa = np.sqrt(np.clip(cov_df["P09"] + cov_df["P10"] + cov_df["P11"], 0, None) / 3)
    ax.plot(cov_t_rel, sp, color="steelblue",   lw=0.7, label="σ_p")
    ax.plot(cov_t_rel, sv, color="forestgreen", lw=0.7, ls="--", label="σ_v")
    ax.plot(cov_t_rel, sa, color="crimson",     lw=0.7, ls=":",  label="σ_θ")
    ax.set_yscale("log")
    ax.set_xlabel("t [s]"); ax.set_ylabel("σ (rms over xyz)")
    ax.set_title("Replay covariance evolution")
    ax.grid(alpha=0.3, which="both"); ax.legend(fontsize=8)

    # [1,1] radar normalized innovation histogram (accepted only)
    ax = axes[1, 1]
    acc = rinn_df[rinn_df["status"] == 0]
    z = acc["residual"] / np.sqrt(acc["S"].clip(lower=1e-12))
    z = np.asarray(z[np.isfinite(z)], dtype=np.float64)
    bins = np.linspace(-5, 5, 80)
    ax.hist(z, bins=bins, density=True, alpha=0.55, color="steelblue",
            label=f"n={len(z)}  std={z.std():.3f}")
    xs = np.linspace(-5, 5, 200)
    ax.plot(xs, np.exp(-xs**2 / 2) / np.sqrt(2 * np.pi),
            color="k", lw=1, ls="--", label="N(0,1)")

    # Fraction of |residual / √S| inside 1, 2, 3 σ (N(0,1) reference:
    # 68.27 / 95.45 / 99.73 %). Higher than reference → filter is too
    # pessimistic (S over-estimated); lower → too optimistic.
    if len(z):
        abs_z = np.abs(z)
        n     = len(z)
        pct1  = 100.0 * np.sum(abs_z < 1) / n
        pct2  = 100.0 * np.sum(abs_z < 2) / n
        pct3  = 100.0 * np.sum(abs_z < 3) / n
        ax.text(0.02, 0.98,
                f"|z|<1σ: {pct1:5.1f}%  (N: 68.3%)\n"
                f"|z|<2σ: {pct2:5.1f}%  (N: 95.4%)\n"
                f"|z|<3σ: {pct3:5.1f}%  (N: 99.7%)",
                transform=ax.transAxes, va="top", ha="left",
                fontsize=7, family="monospace",
                bbox=dict(facecolor="white", edgecolor="0.7",
                          alpha=0.85, pad=4))

    ax.set_yscale("log")
    ax.set_xlabel("residual / √S"); ax.set_ylabel("density (log)")
    ax.set_title("Radar normalized innovation (accepted)")
    ax.grid(alpha=0.3, which="both"); ax.legend(fontsize=8, loc="upper right")

    # [1,2] baro innovation timeseries (residual + ±√S band)
    ax = axes[1, 2]
    bmask = ((binn_df["accepted"] == 1) | (binn_df["rejected"] == 1)).to_numpy().astype(bool)
    if bmask.any():
        # Force float64 — pandas occasionally returns object-dtype arrays
        # when columns contain mixed int/float printing, which breaks
        # matplotlib's internal np.isfinite() during fill_between.
        btsub = np.asarray(binn_t_rel[bmask],                 dtype=np.float64)
        sig   = np.sqrt(np.clip(
                np.asarray(binn_df.loc[bmask, "S"],           dtype=np.float64),
                0, None))
        res   = np.asarray(binn_df.loc[bmask, "residual"],    dtype=np.float64)
        ax.fill_between(btsub, -sig, sig, color="steelblue", alpha=0.15)
        ax.plot(btsub, res, color="steelblue", lw=0.5, label="replay")
    ax.axhline(0, color="k", lw=0.5)
    ax.set_xlabel("t [s]"); ax.set_ylabel("Δz residual [m]")
    ax.set_title("Baro innovation (±√S band)")
    ax.grid(alpha=0.3); ax.legend(fontsize=8)

    fig.suptitle(f"compare overview — {args.replay_dir.name} vs EKF2"
                 + ("  (+ live VVO)" if pos_vvo_rel is not None else ""))
    fig.tight_layout()
    out = Path(save_dir) / "compare.png"
    fig.savefig(out, dpi=120, bbox_inches="tight")
    plt.close(fig)
    print(f"  saved → {out}")

    print(f"\nDone — outputs in {save_dir}/")


if __name__ == "__main__":
    sys.exit(main())
