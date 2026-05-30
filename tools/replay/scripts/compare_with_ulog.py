#!/usr/bin/env python3
"""Compare offline replay output against PX4 EKF2 ground truth from a .ulg.

Usage:
    .venv/bin/python3 tools/replay/scripts/compare_with_ulog.py \
        RUN_DIR FLIGHT.ulg [-o OUT_DIR] [--no-vvo] [-t SECONDS]

Examples:
    # default — overlays live VVO, writes plots to <RUN_DIR>/cmp/
    .venv/bin/python3 tools/replay/scripts/compare_with_ulog.py \
        runs/log24_sigma068 data/field/2105/ulog/log_24_*.ulg

    # suppress live VVO trace, custom output dir
    .venv/bin/python3 tools/replay/scripts/compare_with_ulog.py \
        runs/baseline flight.ulg --no-vvo -o /tmp/cmp

Inputs:
    RUN_DIR                 — Replay output directory containing state.csv
                              (produced by tools/replay/rio_replay; Teensy
                              world frame, t in Teensy seconds).
    FLIGHT.ulg              — PX4 .ulg with vehicle_visual_odometry,
                              estimator_local_position, estimator_attitude.

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
import json
import pprint
import sys
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from pyulog import ULog
from scipy.spatial.transform import Rotation
from scipy.signal import correlate, correlation_lags

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


# Frame transform applied in firmware sendOdometry() (src/main.cpp) to send
# ODOMETRY in PX4 NED. Replay's state.csv is in raw Teensy world frame, so
# we apply the same rotation here.
#
# Firmware uses Eigen's Quat(w,x,y,z) = (0, √2/2, √2/2, 0): a 180° rotation
# about the axis (1,1,0)/√2 — the standard ENU↔NED swap, mapping
# (x, y, z) → (y, x, −z). (The inline comment in src/main.cpp saying
# "(0,0,1,0)" is stale; the constant itself is the source of truth.)
#
#   p_NED = q_t · p_W
#   q_NED = q_t · q_W · q_t.inv()
_INV_SQRT2 = 1.0 / np.sqrt(2.0)
_Q_T_XYZW = np.array([_INV_SQRT2, _INV_SQRT2, 0.0, 0.0])  # scipy uses (x, y, z, w)
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


def align_origin_pose(pos, q_wxyz, ref_idx=0,
                      target_pos=None, target_q_wxyz=None):
    """SE(3) alignment: apply a single rigid transform so that the pose at
    ref_idx becomes (target_pos, target_q_wxyz). Default target is the
    canonical origin (0, identity).

    Used to align all three traces (EKF2, replay, live VVO) to a common
    reference at the first moment all three exist. Setting the target's
    rotation to EKF2's actual orientation at ref_idx (rather than identity)
    keeps EKF2 in its native NED frame — it becomes a pure translation by
    -pos_ekf[ref_idx] — while replay/VVO get full SE(3) rotated+translated
    to land on the same pose. That rotation absorbs static mount
    misalignment between the filter and EKF2 (e.g. a few degrees of pitch
    from an imperfect sensor mount): without it, the constant body-frame
    offset shows up as a growing trajectory rotation when the vehicle
    moves through different attitudes."""
    if target_pos is None:
        target_pos = np.zeros(3)
    if target_q_wxyz is None:
        target_q_wxyz = np.array([1.0, 0.0, 0.0, 0.0])

    p_local       = pos[ref_idx]
    q_local_xyzw  = q_wxyz[ref_idx, [1, 2, 3, 0]]
    q_target_xyzw = np.asarray(target_q_wxyz)[[1, 2, 3, 0]]

    R_local  = Rotation.from_quat(q_local_xyzw)
    R_target = Rotation.from_quat(q_target_xyzw)
    R_align  = R_target * R_local.inv()

    pos_a    = R_align.apply(pos - p_local) + target_pos
    q_xyzw   = q_wxyz[:, [1, 2, 3, 0]]
    q_a_xyzw = (R_align * Rotation.from_quat(q_xyzw)).as_quat()
    q_a_wxyz = q_a_xyzw[:, [3, 0, 1, 2]]
    return pos_a, q_a_wxyz


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


def load_mocap_bag(bag_path: Path, topic: str = "/qualisys/Body_1/odom"):
    """Load a ROS1 mocap bag (nav_msgs/Odometry). Returns:
        ts_s   : (N,) ROS header timestamp in wall-clock seconds (float64)
        pos    : (N, 3) position in mocap world frame
        q_wxyz : (N, 4) attitude quaternion
        vel    : (N, 3) linear velocity in body frame (Odometry convention:
                 twist is in child_frame_id, which Qualisys publishes
                 wrt the rigid-body local frame).
    """
    from rosbags.rosbag1 import Reader
    from rosbags.typesys import Stores, get_typestore
    typestore = get_typestore(Stores.ROS1_NOETIC)

    ts_list, pos_list, q_list, vel_list = [], [], [], []
    with Reader(bag_path) as reader:
        conns = [c for c in reader.connections if c.topic == topic]
        if not conns:
            avail = sorted({c.topic for c in reader.connections})
            raise SystemExit(
                f"topic '{topic}' not found in {bag_path}.\n"
                f"available topics: {avail}")
        if conns[0].msgtype != "nav_msgs/msg/Odometry":
            raise SystemExit(
                f"unsupported mocap msgtype {conns[0].msgtype!r} on {topic} "
                f"— expected nav_msgs/msg/Odometry")
        for cn, _, raw in reader.messages(connections=conns):
            m = typestore.deserialize_ros1(raw, cn.msgtype)
            stamp = m.header.stamp
            ts_list.append(stamp.sec + stamp.nanosec * 1e-9)
            p = m.pose.pose.position;       pos_list.append((p.x, p.y, p.z))
            q = m.pose.pose.orientation;    q_list.append((q.w, q.x, q.y, q.z))
            v = m.twist.twist.linear;       vel_list.append((v.x, v.y, v.z))
    return (np.asarray(ts_list, dtype=np.float64),
            np.asarray(pos_list, dtype=np.float64),
            np.asarray(q_list,   dtype=np.float64),
            np.asarray(vel_list, dtype=np.float64))


def auto_align_mocap_via_velocity(ts_mocap_s, vel_mocap,
                                  ts_ref_us, vel_ref, ref_name, save_dir):
    """Find the time offset that best aligns |v_mocap| with |v_ref| via
    cross-correlation on a 50 Hz uniform grid. |v| is frame-invariant, so
    it doesn't matter that mocap twist is body-frame and the reference may
    be NED (replay/EKF2) or body FRD (VVO). Returns offset_s such that
        t_px4_us = (ts_mocap_s - ts_mocap_s[0]) * 1e6
                   + ts_ref_us[0] + offset_s * 1e6
    Also saves a before/after |v| overlay so the alignment can be eyeballed.
    """
    spd_mocap = np.linalg.norm(vel_mocap, axis=1)
    spd_ref   = np.linalg.norm(vel_ref,   axis=1)
    t_mocap_rel = ts_mocap_s - ts_mocap_s[0]
    t_ref_rel   = (ts_ref_us - ts_ref_us[0]) / 1e6

    dt    = 0.02
    t_max = max(float(t_mocap_rel[-1]), float(t_ref_rel[-1]))
    grid  = np.arange(0.0, t_max + dt, dt)
    s_m   = np.interp(grid, t_mocap_rel, spd_mocap, left=0.0, right=0.0)
    s_r   = np.interp(grid, t_ref_rel,   spd_ref,   left=0.0, right=0.0)
    # Zero-mean + unit-std → normalized cross-correlation, robust to
    # different absolute speed magnitudes between sources.
    s_m = (s_m - s_m.mean()) / (s_m.std() + 1e-12)
    s_r = (s_r - s_r.mean()) / (s_r.std() + 1e-12)

    corr = correlate(s_r, s_m, mode="full")
    lags = correlation_lags(len(s_r), len(s_m), mode="full")
    best_lag = int(lags[int(np.argmax(corr))])
    offset_s = float(best_lag) * dt
    print(f"  |v| cross-correlation (ref = {ref_name}) → mocap offset = "
          f"{offset_s:+.3f} s   (peak lag = {best_lag} samples @ {1/dt:.0f} Hz)")

    Path(save_dir).mkdir(parents=True, exist_ok=True)
    fig, axs = plt.subplots(2, 1, sharex=True, figsize=(12, 6))
    axs[0].plot(t_ref_rel,   spd_ref,   color="steelblue",  lw=0.8, label=f"{ref_name} |v|")
    axs[0].plot(t_mocap_rel, spd_mocap, color="darkorange", lw=0.8, label="mocap |v|")
    axs[0].set_title("Before alignment (each trace from its own t=0)")
    axs[0].set_ylabel("|v| [m/s]"); axs[0].grid(alpha=0.3); axs[0].legend(fontsize=8)
    axs[1].plot(t_ref_rel,             spd_ref,   color="steelblue", lw=0.8, label=f"{ref_name} |v|")
    axs[1].plot(t_mocap_rel + offset_s, spd_mocap, color="darkorange", lw=0.8,
                label=f"mocap |v|  (shifted {offset_s:+.3f} s)")
    axs[1].set_title("After alignment")
    axs[1].set_ylabel("|v| [m/s]"); axs[1].set_xlabel(f"t [s] ({ref_name}-relative)")
    axs[1].grid(alpha=0.3); axs[1].legend(fontsize=8)
    fig.tight_layout()
    out = Path(save_dir) / "mocap_velocity_sync.svg"
    fig.savefig(out, bbox_inches="tight"); plt.close(fig)
    print(f"  saved → {out}")
    return offset_s


def evaluate_pair(gt_traj, gt_name, est_traj, est_name, t0_s, tol, save_dir):
    """Compute and plot position-error + APE + RPE of `est_traj` against
    `gt_traj`. Saves three SVGs per pair (suffixed with `est_name`) so
    multiple estimates against a common GT don't clobber each other."""
    from evo.core.filters import FilterException
    if est_traj.num_poses == 0 or gt_traj.num_poses == 0:
        print(f"  {est_name} vs {gt_name}: empty trajectory, skipping")
        return
    n_est_before = est_traj.num_poses
    a_gt, a_est = sync.associate_trajectories(
        gt_traj, est_traj, max_diff=tol,
        first_name=f"{gt_name} (GT)", snd_name=est_name)
    if a_est.num_poses < 3:
        print(f"  {est_name} vs {gt_name}: only {a_est.num_poses} paired "
              f"within {tol*1000:.0f} ms — skipping metrics")
        return
    matched_pct = 100.0 * a_est.num_poses / max(n_est_before, 1)
    print(f"\n=== {est_name} vs {gt_name}  (matched "
          f"{a_est.num_poses}/{n_est_before}, {matched_pct:.1f}%) ===")

    p_ref = a_gt.positions_xyz
    p_est = a_est.positions_xyz
    t_paired_rel = np.asarray(a_gt.timestamps) - t0_s

    # Position error
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
        f"Position error: {est_name} − {gt_name}  "
        f"(RMS: X={rms[0]:.3f}  Y={rms[1]:.3f}  Z={rms[2]:.3f} m)")
    axs[-1].set_xlabel("t [s] (since EKF2 start)")
    plt.tight_layout()
    handle_fig(fig, f"compare_position_error_{est_name}", save_dir)

    # APE
    ape = metrics.APE(metrics.PoseRelation.translation_part)
    ape.process_data((a_gt, a_est))
    ape_stats = ape.get_all_statistics()
    print(f"── APE ({est_name} vs {gt_name}) ──"); pprint.pprint(ape_stats)
    fig = plt.figure(figsize=(10, 4))
    plot.error_array(
        fig.gca(), ape.error, x_array=t_paired_rel,
        statistics={s: v for s, v in ape_stats.items() if s != "sse"},
        name="APE",
        title=f"APE  (GT = {gt_name}, est = {est_name})",
        xlabel="t [s] (since EKF2 start)")
    handle_fig(fig, f"compare_ape_{est_name}", save_dir)

    fig = plt.figure(figsize=(8, 8))
    ax = plot.prepare_axis(fig, plot.PlotMode.xy)
    plot.traj(ax, plot.PlotMode.xy, a_gt, "--", "gray", f"{gt_name} (GT)")
    plot.traj_colormap(ax, a_est, ape.error, plot.PlotMode.xy,
                       title=f"{est_name} trajectory coloured by APE",
                       min_map=ape_stats["min"], max_map=ape_stats["max"])
    ax.legend()
    handle_fig(fig, f"compare_ape_trajectory_{est_name}", save_dir)

    # RPE
    traj_length = float(np.sum(np.linalg.norm(np.diff(p_ref, axis=0), axis=1)))
    rpe_delta   = float(np.clip(traj_length * 0.1, 0.05, 10.0))
    print(f"GT length: {traj_length:.3f} m  →  RPE δ = {rpe_delta:.3f} m")
    rpe = metrics.RPE(pose_relation=metrics.PoseRelation.translation_part,
                      delta=rpe_delta, delta_unit=Unit.meters, all_pairs=True)
    try:
        rpe.process_data((a_gt, a_est))
        rpe_stats = rpe.get_all_statistics()
        print(f"── RPE ({est_name} vs {gt_name}, δ={rpe_delta:.3f} m) ──")
        pprint.pprint(rpe_stats)
        fig = plt.figure(figsize=(10, 4))
        plot.error_array(
            fig.gca(), rpe.error, x_array=np.arange(len(rpe.error)),
            statistics={s: v for s, v in rpe_stats.items() if s != "sse"},
            name="RPE",
            title=f"RPE: {est_name} vs {gt_name}  (δ={rpe_delta:.3f} m)",
            xlabel="pair index")
        handle_fig(fig, f"compare_rpe_{est_name}", save_dir)
    except FilterException as exc:
        print(f"  RPE skipped: {exc}")


def render_raw_sources(out_dir, ts_ekf, pos_ekf, q_ekf, vel_ekf,
                       ts_vvo, pos_vvo, q_vvo, vel_vvo,
                       t_replay_s, pos_W, vel_W, q_W,
                       pos_N, vel_N, q_N,
                       mocap=None):
    """Sanity-check plots of each input source in its native frame, with
    no trimming, alignment, or cross-source transformation applied. One
    figure per source under <out_dir>/raw/ with position, attitude,
    velocity, and XY trajectory."""
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    def _euler_deg(q_wxyz):
        q_xyzw = q_wxyz[:, [1, 2, 3, 0]]
        return np.rad2deg(np.unwrap(
            Rotation.from_quat(q_xyzw).as_euler("xyz"), axis=0))

    def _one(tag, t_s, pos, q, vel, pos_frame, vel_frame):
        eul = _euler_deg(q)
        t_rel = t_s - t_s[0]
        fig, axs = plt.subplots(2, 2, figsize=(13, 9))
        for i, lbl in enumerate(["x", "y", "z"]):
            axs[0, 0].plot(t_rel, pos[:, i], label=lbl)
        axs[0, 0].set_xlabel("t [s] (since first sample)")
        axs[0, 0].set_ylabel(f"position [m]  ({pos_frame})")
        axs[0, 0].grid(alpha=0.3); axs[0, 0].legend(fontsize=8)
        for i, lbl in enumerate(["roll", "pitch", "yaw"]):
            axs[0, 1].plot(t_rel, eul[:, i], label=lbl)
        axs[0, 1].set_xlabel("t [s] (since first sample)")
        axs[0, 1].set_ylabel("attitude [deg]")
        axs[0, 1].grid(alpha=0.3); axs[0, 1].legend(fontsize=8)
        for i, lbl in enumerate(["vx", "vy", "vz"]):
            axs[1, 0].plot(t_rel, vel[:, i], label=lbl)
        axs[1, 0].set_xlabel("t [s] (since first sample)")
        axs[1, 0].set_ylabel(f"velocity [m/s]  ({vel_frame})")
        axs[1, 0].grid(alpha=0.3); axs[1, 0].legend(fontsize=8)
        axs[1, 1].plot(pos[:, 0], pos[:, 1], lw=0.6, color="steelblue")
        axs[1, 1].scatter(pos[0, 0],  pos[0, 1],  c="green", s=25, zorder=3, label="start")
        axs[1, 1].scatter(pos[-1, 0], pos[-1, 1], c="red",   s=25, zorder=3, label="end")
        axs[1, 1].set_xlabel("x [m]"); axs[1, 1].set_ylabel("y [m]")
        axs[1, 1].set_title(f"XY trajectory  ({pos_frame})")
        axs[1, 1].set_aspect("equal", adjustable="datalim")
        axs[1, 1].grid(alpha=0.3); axs[1, 1].legend(fontsize=8)
        fig.suptitle(f"raw {tag}  ({len(t_s)} samples, {t_rel[-1]:.2f}s span)")
        fig.tight_layout()
        out = out_dir / f"raw_{tag}.svg"
        fig.savefig(out, bbox_inches="tight")
        plt.close(fig)
        print(f"  saved → {out}")

    _one("ekf2",                ts_ekf / 1e6, pos_ekf, q_ekf, vel_ekf,
         pos_frame="PX4 NED", vel_frame="PX4 NED")
    _one("vvo",                 ts_vvo / 1e6, pos_vvo, q_vvo, vel_vvo,
         pos_frame="PX4 NED", vel_frame="body FRD")
    if t_replay_s is not None:
        _one("replay_teensy_world", t_replay_s, pos_W, q_W, vel_W,
             pos_frame="Teensy world (z-up)", vel_frame="Teensy world (z-up)")
        _one("replay_ned",          t_replay_s, pos_N, q_N, vel_N,
             pos_frame="NED (q_t · Teensy)", vel_frame="NED (q_t · Teensy)")
    if mocap is not None:
        ts_moc_s, pos_moc, q_moc, vel_moc = mocap
        _one("mocap", ts_moc_s, pos_moc, q_moc, vel_moc,
             pos_frame="mocap world", vel_frame="body (Odometry child_frame)")


# ── main ──────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("replay_dir", type=Path,
                    help="Replay output directory (must contain state.csv)")
    ap.add_argument("ulg", type=Path,
                    help="PX4 .ulg log file")
    ap.add_argument("-o", "--out", "--save_results", type=Path, default=None,
                    dest="out",
                    help="Output dir for plots (default: <replay_dir>/cmp)")
    ap.add_argument("--no-vvo", "--no-live", dest="plot_vvo",
                    action="store_false",
                    help="Suppress the live VVO overlay (on by default).")
    ap.add_argument("-t", "--tol", "--max_time_diff", type=float, default=0.2,
                    dest="tol",
                    help="evo association tolerance in seconds (default 0.2)")
    ap.add_argument("--skip-seconds", type=float, default=0.0,
                    dest="skip_seconds",
                    help="Skip this many seconds of data after the first VVO "
                         "publish before aligning/plotting (default 0). The "
                         "first samples can be noisy (filter still settling) "
                         "and contaminate the origin alignment.")
    ap.add_argument("--mocap", type=Path, default=None,
                    help="Optional ROS1 bag with mocap odometry to overlay "
                         "on the plots (no APE/RPE computed against mocap).")
    ap.add_argument("--mocap-topic", default="/qualisys/Body_1/odom",
                    help="nav_msgs/Odometry topic in the mocap bag.")
    ap.add_argument("--mocap-offset", type=float, default=None,
                    help="Manual override (seconds) for the mocap→PX4 time "
                         "offset. Default: auto via |v| cross-correlation. "
                         "Eyeball the value from cmp/mocap_velocity_sync.svg.")
    ap.add_argument("--mocap-sync-with", choices=("replay", "vvo", "ekf2"),
                    default="replay",
                    help="Which trace's |v| to cross-correlate mocap against "
                         "for the auto time-sync (default: replay). Ignored "
                         "when --mocap-offset is given.")
    ap.add_argument("--mocap-body-frame", choices=("frd", "bld"),
                    default="frd",
                    help="Mocap rigid-body frame convention (default: frd). "
                         "Use 'bld' for bags whose rigid body was set up "
                         "Back-Left-Down: a 180°-about-Z body rotation is "
                         "applied to convert Back-Left-Down → Forward-Right-"
                         "Down so body-frame velocity/attitude match VVO.")
    args = ap.parse_args()

    if not args.replay_dir.is_dir():
        raise SystemExit(f"not a directory: {args.replay_dir}")
    if not args.ulg.is_file():
        raise SystemExit(f"missing .ulg: {args.ulg}")
    save_dir = args.out or (args.replay_dir / "cmp")

    # ── load .ulg ──────────────────────────────────────────────────────────
    ulog = ULog(str(args.ulg))
    ts_vvo, pos_vvo, q_vvo, vel_vvo, df_vvo = load_visual_odometry(ulog)
    ts_ekf, pos_ekf, q_ekf, vel_ekf, _      = load_estimator_local_position(ulog)
    print(f"vehicle_visual_odometry  : {len(ts_vvo)} msgs, "
          f"{(ts_vvo[-1]-ts_vvo[0])/1e6:.2f}s")
    print(f"estimator_local_position : {len(ts_ekf)} msgs, "
          f"{(ts_ekf[-1]-ts_ekf[0])/1e6:.2f}s")

    # Alignment cutoff: first VVO publish, plus optional skip window.
    # Computed BEFORE any trimming so the original first VVO timestamp
    # remains available for the time-offset anchor below. Pre-cutoff
    # samples are dropped from all traces after we've used the original
    # ts_vvo[0] for offset computation.
    t_cut_us = float(ts_vvo[0]) + args.skip_seconds * 1e6
    if args.skip_seconds:
        print(f"\n--skip-seconds={args.skip_seconds}s  →  alignment cutoff at "
              f"t={t_cut_us/1e6:.3f}s (first VVO = {ts_vvo[0]/1e6:.3f}s)")

    # ── detect replay availability ────────────────────────────────────────
    # state.csv may be missing entirely or present-but-empty (header only,
    # no RAD rows — e.g. the input CSV had no radar frames). When absent
    # we skip all replay-only work (time offset, APE/RPE, σ/innov panels)
    # and produce a VVO+EKF2(+mocap) comparison.
    has_replay = False
    state_csv = args.replay_dir / "state.csv"
    if state_csv.is_file():
        try:
            df_chk = pd.read_csv(state_csv, usecols=["t", "source"])
            if (df_chk["source"] == "RAD").any():
                has_replay = True
        except (ValueError, pd.errors.EmptyDataError):
            pass
    if not has_replay:
        print("\nreplay state.csv missing or has no 'RAD' rows — running "
              "in no-replay mode (VVO + EKF2"
              + (" + mocap" if args.mocap is not None else "") + ").")

    Path(save_dir).mkdir(parents=True, exist_ok=True)

    # ── time offset Teensy → PX4 (replay only) ────────────────────────────
    offset_us = 0.0
    t_replay_s = pos_W = vel_W = q_W = None
    pos_N = vel_N = q_N = ts_replay_us = None
    if has_replay:
        # Primary: anchor first replay 'RAD' publish to first live VVO.
        offset_us = compute_time_offset_via_first_publish(args.replay_dir, ts_vvo)
        # Diagnostic: timestamp_sample method (unreliable without TIMESYNC).
        offset_diag, std_us, n_samp = compute_time_offset(df_vvo)
        print(f"\nTime offset (PX4 − Teensy):")
        print(f"  via first VVO publish : {offset_us/1e6:+.6f} s   "
              f"({offset_us:+.0f} µs)   [USED]")
        print(f"  via timestamp_sample  : {offset_diag/1e6:+.6f} s   "
              f"std={std_us/1e3:.3f} ms  ({n_samp} samples)  [diagnostic]")
        with (Path(save_dir) / "time_offset.json").open("w") as f:
            json.dump({"offset_us": offset_us,
                       "offset_us_via_timestamp_sample": offset_diag,
                       "std_us_via_timestamp_sample": std_us,
                       "n_vvo_samples": n_samp}, f, indent=2)

        # ── load replay, transform to NED, apply offset ───────────────────
        t_replay_s, pos_W, vel_W, q_W = load_replay_state(args.replay_dir)
        pos_N, vel_N, q_N = transform_replay_to_ned(pos_W, vel_W, q_W)
        ts_replay_us = t_replay_s * 1e6 + offset_us
        print(f"\nreplay/state.csv         : {len(t_replay_s)} rows, "
              f"{t_replay_s[-1]-t_replay_s[0]:.2f}s   "
              f"first PX4-aligned t = {ts_replay_us[0]/1e6:.3f}s "
              f"(EKF2 first = {ts_ekf[0]/1e6:.3f}s)")

    # ── optional mocap bag ─────────────────────────────────────────────────
    # Loaded BEFORE trimming so the full mocap trajectory is available for
    # |v| cross-correlation against the chosen reference (replay by default,
    # optionally vvo or ekf2). Time mapping uses the reference's first
    # un-trimmed timestamp as the PX4-time anchor.
    ts_mocap_us = pos_mocap = q_mocap = vel_mocap = None
    mocap_raw_pkg = None
    if args.mocap is not None:
        if not args.mocap.is_file():
            raise SystemExit(f"missing --mocap bag: {args.mocap}")
        print(f"\nmocap bag                 : {args.mocap}")
        ts_mocap_s, pos_mocap, q_mocap, vel_mocap = load_mocap_bag(
            args.mocap, args.mocap_topic)
        print(f"  {args.mocap_topic}: {len(ts_mocap_s)} msgs, "
              f"{ts_mocap_s[-1] - ts_mocap_s[0]:.2f}s")

        # Body-frame correction. Qualisys rigid bodies are sometimes set
        # up with "forward" pointing toward the drone's back, producing
        # Back-Left-Down (BLD) body conventions. To match PX4 FRD we
        # right-multiply each quaternion by R_z(180°) and negate the
        # body-frame velocity's X/Y components. World frame is left to
        # the Umeyama trajectory alignment below.
        if args.mocap_body_frame == "bld":
            q_z180_R = Rotation.from_quat([0., 0., 1., 0.])
            q_xyzw   = q_mocap[:, [1, 2, 3, 0]]
            q_corr_xyzw = (Rotation.from_quat(q_xyzw) * q_z180_R).as_quat()
            q_mocap   = q_corr_xyzw[:, [3, 0, 1, 2]]
            vel_mocap = vel_mocap * np.array([-1., -1., 1.])
            print("  applied BLD→FRD body-frame correction "
                  "(neg vx/vy, right-multiplied q by R_z(180°))")
        mocap_raw_pkg = (ts_mocap_s, pos_mocap, q_mocap, vel_mocap)

        # Pick the |v| reference for sync. All three are in PX4 µs already.
        # Auto-fall-back when the requested ref isn't available (no replay
        # in this run, or --no-vvo was passed).
        sync_refs = {
            "replay": (ts_replay_us, vel_N,   "replay")  if has_replay   else None,
            "vvo":    (ts_vvo,       vel_vvo, "live VVO") if args.plot_vvo else None,
            "ekf2":   (ts_ekf,       vel_ekf, "EKF2"),
        }
        chosen = args.mocap_sync_with
        if sync_refs[chosen] is None:
            fb = "vvo" if sync_refs["vvo"] is not None else "ekf2"
            print(f"  --mocap-sync-with={chosen} unavailable → "
                  f"falling back to {fb}")
            chosen = fb
        ref_ts, ref_vel, ref_name = sync_refs[chosen]
        args.mocap_sync_with = chosen

        if args.mocap_offset is not None:
            mocap_offset_s = float(args.mocap_offset)
            print(f"  using manual --mocap-offset = {mocap_offset_s:+.3f} s "
                  f"(relative to {ref_name})")
        else:
            mocap_offset_s = auto_align_mocap_via_velocity(
                ts_mocap_s, vel_mocap, ref_ts, ref_vel, ref_name, save_dir)
        ts_mocap_us = ((ts_mocap_s - ts_mocap_s[0]) * 1e6
                       + float(ref_ts[0]) + mocap_offset_s * 1e6)

    # ── raw sanity-check plots (pre-trim, native frames) ─────────────────
    print(f"\nraw plots → {save_dir}/raw/")
    render_raw_sources(
        Path(save_dir) / "raw",
        ts_ekf, pos_ekf, q_ekf, vel_ekf,
        ts_vvo, pos_vvo, q_vvo, vel_vvo,
        t_replay_s, pos_W, vel_W, q_W,
        pos_N, vel_N, q_N,
        mocap=mocap_raw_pkg)

    # ── trim all three traces at t_cut_us ─────────────────────────────────
    # Done now (after offset is computed and replay is in PX4 µs) so the
    # cutoff applies uniformly across EKF2, replay, and VVO.
    def _trim(name, ts, *arrays):
        keep = ts >= t_cut_us
        n = int(np.sum(~keep))
        if n:
            print(f"  trimming {n} {name} samples before cutoff")
        return (ts[keep],) + tuple(a[keep] for a in arrays)

    ts_ekf, pos_ekf, q_ekf, vel_ekf = _trim(
        "EKF2", ts_ekf, pos_ekf, q_ekf, vel_ekf)
    ts_vvo, pos_vvo, q_vvo, vel_vvo = _trim(
        "VVO",  ts_vvo, pos_vvo, q_vvo, vel_vvo)
    if has_replay:
        ts_replay_us, pos_N, vel_N, q_N = _trim(
            "replay", ts_replay_us, pos_N, vel_N, q_N)
    if ts_mocap_us is not None:
        # Mocap typically records well before VVO starts and after VVO
        # stops; clip to the VVO window so the overlay covers only the
        # interval where the live odometry stream is meaningful.
        ts_mocap_us, pos_mocap, q_mocap, vel_mocap = _trim(
            "mocap (pre-VVO)", ts_mocap_us, pos_mocap, q_mocap, vel_mocap)
        if len(ts_mocap_us) > 0:
            t_vvo_end_us = float(ts_vvo[-1])
            keep = ts_mocap_us <= t_vvo_end_us
            n_drop = int(np.sum(~keep))
            if n_drop:
                print(f"  trimming {n_drop} mocap samples after VVO end")
            ts_mocap_us = ts_mocap_us[keep]
            pos_mocap   = pos_mocap[keep]
            q_mocap     = q_mocap[keep]
            vel_mocap   = vel_mocap[keep]
        if len(ts_mocap_us) == 0:
            print("  mocap empty after trim — dropping from plots")
            ts_mocap_us = None

    n_replay_left = len(ts_replay_us) if has_replay else -1
    if len(ts_ekf) == 0 or len(ts_vvo) == 0 or (has_replay and n_replay_left == 0):
        raise SystemExit(
            f"--skip-seconds={args.skip_seconds}s left one of the traces "
            f"empty (EKF2={len(ts_ekf)}, VVO={len(ts_vvo)}, "
            f"replay={n_replay_left}). Pick a smaller value.")

    # ── SE(3) align all traces so they start at (0, identity) ─────────────
    # The first sample of each trace is at-or-just-after t_cut_us, so they
    # all describe the same physical instant (the first moment EKF2 and
    # VVO both exist, after the optional --skip-seconds). Aligning each
    # trace's pose at index 0 to (0, identity) means every trace's xyz
    # starts at 0 and rpy starts at (0, 0, 0) — the plots show motion
    # since the alignment instant rather than absolute pose. Mocap goes
    # through Umeyama first (to absorb its frame's rotation w.r.t. PX4
    # NED) and is then origin-aligned via the same identity target below.
    ref_q_wxyz = np.array([1., 0., 0., 0.])
    pos_ekf_rel, q_ekf_rel = align_origin_pose(
        pos_ekf, q_ekf, ref_idx=0, target_q_wxyz=ref_q_wxyz)
    pos_N_rel = q_N_rel = None
    if has_replay:
        pos_N_rel, q_N_rel = align_origin_pose(
            pos_N, q_N, ref_idx=0, target_q_wxyz=ref_q_wxyz)

    # Common time origin = first EKF2 sample.
    t0_s = float(ts_ekf[0]) / 1e6

    # Full traces (not post-association) for time-series plots.
    t_ekf_rel    = ts_ekf / 1e6 - t0_s
    t_replay_rel = (ts_replay_us / 1e6 - t0_s) if has_replay else None

    def _euler_deg(q_wxyz):
        q_xyzw = q_wxyz[:, [1, 2, 3, 0]]
        return np.rad2deg(np.unwrap(
            Rotation.from_quat(q_xyzw).as_euler("xyz"), axis=0))

    eul_ekf_full    = _euler_deg(q_ekf_rel)
    eul_replay_full = _euler_deg(q_N_rel) if has_replay else None

    # Optional live VVO overlay (already in PX4 frame & clock).
    t_vvo_rel = None
    pos_vvo_rel = None
    eul_vvo_full = None
    if args.plot_vvo:
        t_vvo_rel    = ts_vvo / 1e6 - t0_s
        pos_vvo_rel, q_vvo_rel = align_origin_pose(
            pos_vvo, q_vvo, ref_idx=0, target_q_wxyz=ref_q_wxyz)
        eul_vvo_full = _euler_deg(q_vvo_rel)

    # Optional mocap overlay. Aligned to the chosen sync reference (replay
    # by default) via Umeyama (rotation + translation, no scale) on the
    # time-associated subset; the resulting SE(3) is then applied to the
    # full mocap trace.
    #
    # Why trajectory-wide Umeyama, not align_origin_pose at sample 0:
    # the mocap rigid-body's local frame is Qualisys-defined (set by the
    # marker pattern) and won't generally match PX4 body frame — a single-
    # pose attitude alignment leaves a residual yaw that sends the
    # trajectory in the wrong direction. Umeyama on the trajectory absorbs
    # that residual yaw plus any small mocap-world → PX4-NED rotation.
    #
    # Per-axis body-frame velocity isn't overlaid (same mocap-body frame
    # mismatch); only |v| (frame-invariant) goes on the Speed panel.
    t_mocap_rel = pos_mocap_rel = eul_mocap_full = spd_mocap_full = None
    if ts_mocap_us is not None:
        from evo.core.lie_algebra import se3
        align_refs = {
            "replay": (ts_replay_us, pos_N_rel)    if has_replay         else None,
            "vvo":    (ts_vvo,       pos_vvo_rel)  if pos_vvo_rel is not None else None,
            "ekf2":   (ts_ekf,       pos_ekf_rel),
        }
        align_pair = align_refs.get(args.mocap_sync_with) or align_refs["ekf2"]
        align_ts, align_pos = align_pair

        identity_q = np.tile(np.array([1., 0., 0., 0.]), (len(align_ts), 1))
        traj_moc     = make_pose_trajectory(ts_mocap_us, pos_mocap, q_mocap)
        traj_ref_moc = make_pose_trajectory(align_ts,    align_pos, identity_q)
        sa, sb = sync.associate_trajectories(
            traj_ref_moc, traj_moc, max_diff=args.tol,
            first_name=args.mocap_sync_with, snd_name="mocap")
        if sb.num_poses < 3:
            print(f"  mocap alignment skipped: only {sb.num_poses} paired "
                  f"poses within {args.tol*1000:.0f} ms — sync likely wrong")
            r_a = np.eye(3)
        else:
            r_a, _, _ = sb.align(sa, correct_scale=False)
            print(f"  mocap aligned to {args.mocap_sync_with} via Umeyama "
                  f"({sb.num_poses} paired samples)")
        # Apply Umeyama R (no translation — discarded so origin-alignment
        # is the sole source of position offset, matching other traces),
        # then origin-align via the same identity target used for EKF2/
        # replay/VVO. Result: mocap starts at (0,0,0) with rpy=(0,0,0).
        traj_moc = make_pose_trajectory(ts_mocap_us, pos_mocap, q_mocap)
        traj_moc.transform(se3(r_a, np.zeros(3)))
        pos_mocap_rel, q_mocap_rel = align_origin_pose(
            traj_moc.positions_xyz, traj_moc.orientations_quat_wxyz,
            ref_idx=0, target_q_wxyz=ref_q_wxyz)
        t_mocap_rel    = ts_mocap_us / 1e6 - t0_s
        eul_mocap_full = _euler_deg(q_mocap_rel)
        spd_mocap_full = np.linalg.norm(vel_mocap, axis=1)

    # ── plots ──────────────────────────────────────────────────────────────
    pos_labels = ["x [m]", "y [m]", "z [m]"]
    rpy_labels = ["roll [deg]", "pitch [deg]", "yaw [deg]"]

    # Body-frame velocities — reused by overlay velocity plot below.
    # EKF2's estimator_local_position.{vx,vy,vz} and replay's vel_N are both
    # in NED world frame, so we rotate them into body FRD via R(q).T.
    # vehicle_visual_odometry.velocity is ALREADY in body FRD frame (the
    # firmware sets child_frame_id = MAV_FRAME_BODY_FRD and publishes
    # v_body = q_t · (q_WI.inv · v_WI); see src/main.cpp sendOdometry).
    # Re-rotating it would double-transform and produce garbage.
    vel_ekf_body    = vel_to_body_frame(vel_ekf, q_ekf)
    vel_replay_body = vel_to_body_frame(vel_N, q_N) if has_replay else None
    vel_vvo_body    = vel_vvo if t_vvo_rel is not None else None

    # ── APE / RPE / position error ────────────────────────────────────────
    # GT preference: mocap when available (true ground truth), otherwise
    # EKF2 (best available estimate of truth). Every other available trace
    # is treated as an estimate and evaluated against the GT.
    sources = {"ekf2": (ts_ekf, pos_ekf_rel, q_ekf_rel)}
    if has_replay:
        sources["replay"] = (ts_replay_us, pos_N_rel, q_N_rel)
    if t_vvo_rel is not None:
        pos_vvo_rel_full, q_vvo_rel_full = pos_vvo_rel, q_vvo_rel
        sources["vvo"] = (ts_vvo, pos_vvo_rel_full, q_vvo_rel_full)
    if t_mocap_rel is not None:
        sources["mocap"] = (ts_mocap_us, pos_mocap_rel, q_mocap_rel)

    gt_name = "mocap" if "mocap" in sources else "ekf2"
    ts_gt, pos_gt, q_gt = sources[gt_name]
    gt_traj = make_pose_trajectory(ts_gt, pos_gt, q_gt)
    print(f"\nGT for APE/RPE: {gt_name}  ({gt_traj.num_poses} poses)")
    for est_name, (ts_e, pos_e, q_e) in sources.items():
        if est_name == gt_name:
            continue
        est_traj = make_pose_trajectory(ts_e, pos_e, q_e)
        evaluate_pair(gt_traj, gt_name, est_traj, est_name,
                      t0_s, args.tol, save_dir)

    cov_df = rinn_df = binn_df = binn_t_rel = None
    if has_replay:
        # Replay-only diagnostic CSVs (σ band, radar/baro innovations).
        cov_df  = pd.read_csv(args.replay_dir / "cov_diag.csv")
        rinn_df = pd.read_csv(args.replay_dir / "radar_innov.csv")
        binn_df = pd.read_csv(args.replay_dir / "baro_innov.csv")
        binn_t_rel = binn_df["t"].to_numpy() + offset_us / 1e6 - t0_s

    def render_overlays(out_root: Path, with_ekf: bool, with_vvo: bool = True):
        out_root.mkdir(parents=True, exist_ok=True)
        # Local "show VVO?" predicate — short-circuits when either the run had
        # no VVO data at all (t_vvo_rel is None) or the caller explicitly asks
        # to hide it (with_vvo=False, the "no_vvo" pass).
        show_vvo = with_vvo and (t_vvo_rel is not None)
        show_mocap = t_mocap_rel is not None
        show_replay = has_replay
        MOCAP_KW = dict(color="forestgreen", lw=0.8, alpha=0.85, label="mocap")
        if not with_ekf:
            sfx = "  (no EKF2)"
        elif not with_vvo:
            sfx = "  (no live VVO)"
        else:
            sfx = ""

        # Pose vs time — full traces, starting at their actual time offsets.
        fig, axs = plt.subplots(3, 2, sharex="col", figsize=(12, 8))
        for i in range(3):
            if with_ekf:
                axs[i, 0].plot(t_ekf_rel, pos_ekf_rel[:, i], label="EKF2 (GT)")
            if show_replay:
                axs[i, 0].plot(t_replay_rel, pos_N_rel[:, i], label="replay", alpha=0.85)
            if show_vvo:
                axs[i, 0].plot(t_vvo_rel, pos_vvo_rel[:, i], ":",
                               color="darkorange", alpha=0.7, label="live VVO")
            if show_mocap:
                axs[i, 0].plot(t_mocap_rel, pos_mocap_rel[:, i], **MOCAP_KW)
            axs[i, 0].set_ylabel(pos_labels[i]); axs[i, 0].grid(True, alpha=0.3)

            if with_ekf:
                axs[i, 1].plot(t_ekf_rel, eul_ekf_full[:, i], label="EKF2 (GT)")
            if show_replay:
                axs[i, 1].plot(t_replay_rel, eul_replay_full[:, i], label="replay", alpha=0.85)
            if show_vvo:
                axs[i, 1].plot(t_vvo_rel, eul_vvo_full[:, i], ":",
                               color="darkorange", alpha=0.7, label="live VVO")
            if show_mocap:
                axs[i, 1].plot(t_mocap_rel, eul_mocap_full[:, i], **MOCAP_KW)
            axs[i, 1].set_ylabel(rpy_labels[i]); axs[i, 1].grid(True, alpha=0.3)

        axs[0, 0].set_title("Position vs time (per-trace origin-aligned)" + sfx)
        axs[0, 1].set_title("Attitude vs time" + sfx)
        axs[0, 0].legend(fontsize=8); axs[0, 1].legend(fontsize=8)
        axs[-1, 0].set_xlabel("t [s] (since EKF2 start)")
        axs[-1, 1].set_xlabel("t [s] (since EKF2 start)")
        plt.tight_layout(); handle_fig(fig, "compare_pose", out_root)

        # XY trajectory overlay — per-trace origin-aligned, no time axis.
        fig, ax = plt.subplots(figsize=(8, 8))
        if with_ekf:
            ax.plot(pos_ekf_rel[:, 0], pos_ekf_rel[:, 1], "--", color="gray",
                    lw=0.8, label="EKF2 (GT)")
        if show_replay:
            ax.plot(pos_N_rel[:, 0], pos_N_rel[:, 1], color="steelblue",
                    lw=0.8, label="replay")
        if show_vvo:
            ax.plot(pos_vvo_rel[:, 0], pos_vvo_rel[:, 1], ":",
                    color="darkorange", lw=0.8, alpha=0.7, label="live VVO")
        if show_mocap:
            ax.plot(pos_mocap_rel[:, 0], pos_mocap_rel[:, 1], **MOCAP_KW)
        ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
        ax.set_title("Trajectory XY (per-trace origin-aligned)" + sfx)
        ax.set_aspect("equal")
        ax.grid(True, alpha=0.3); ax.legend(fontsize=8)
        plt.tight_layout(); handle_fig(fig, "compare_trajectory_xy", out_root)

        # Body-frame velocity vs time — full traces, common t-axis.
        fig, axs = plt.subplots(3, 1, sharex=True, figsize=(12, 7))
        for i, lbl in enumerate(["vx_body [m/s]", "vy_body [m/s]", "vz_body [m/s]"]):
            if with_ekf:
                axs[i].plot(t_ekf_rel, vel_ekf_body[:, i], label="EKF2 (GT)")
            if show_replay:
                axs[i].plot(t_replay_rel, vel_replay_body[:, i], label="replay", alpha=0.85)
            if show_vvo and vel_vvo_body is not None:
                axs[i].plot(t_vvo_rel, vel_vvo_body[:, i], ":",
                            color="darkorange", alpha=0.7, label="live VVO")
            axs[i].set_ylabel(lbl); axs[i].grid(True, alpha=0.3); axs[i].legend(fontsize=8)
        axs[0].set_title("Velocity vs time (body frame)" + sfx)
        axs[-1].set_xlabel("t [s] (since EKF2 start)")
        plt.tight_layout(); handle_fig(fig, "compare_velocity", out_root)

        # Overview grid: 2×3 with replay (top = overlays, bottom = replay-only
        # diagnostics); 1×3 without replay (top row only).
        nrows = 2 if has_replay else 1
        figsize = (17, 10) if has_replay else (17, 5)
        fig, axes = plt.subplots(nrows, 3, figsize=figsize, squeeze=False)

        # [0,0] xy trajectory (top-down)
        ax = axes[0, 0]
        if with_ekf:
            ax.plot(pos_ekf_rel[:, 0], pos_ekf_rel[:, 1], "--", color="gray",
                    lw=0.8, label="EKF2 (GT)")
        if show_replay:
            ax.plot(pos_N_rel[:, 0], pos_N_rel[:, 1], color="steelblue",
                    lw=0.8, label="replay")
        if show_vvo:
            ax.plot(pos_vvo_rel[:, 0], pos_vvo_rel[:, 1], ":",
                    color="darkorange", lw=0.8, alpha=0.8, label="live VVO")
        if show_mocap:
            ax.plot(pos_mocap_rel[:, 0], pos_mocap_rel[:, 1], **MOCAP_KW)
        ax.set_xlabel("p_x [m]"); ax.set_ylabel("p_y [m]")
        ax.set_title("Trajectory (top-down)")
        ax.set_aspect("equal", adjustable="datalim")
        ax.grid(alpha=0.3); ax.legend(fontsize=8)

        # [0,1] altitude vs t (with replay ±σ_z band when available)
        ax = axes[0, 1]
        if show_replay:
            sig_z = np.sqrt(np.clip(cov_df["P02"].to_numpy(), 0, None))
            pz_replay = pos_N_rel[:, 2]
            n = min(len(pz_replay), len(sig_z), len(t_replay_rel))
            ax.fill_between(t_replay_rel[:n], pz_replay[:n] - sig_z[:n],
                            pz_replay[:n] + sig_z[:n], color="steelblue", alpha=0.15)
        if with_ekf:
            ax.plot(t_ekf_rel, pos_ekf_rel[:, 2], "--", color="gray",
                    lw=0.8, label="EKF2 (GT)")
        if show_replay:
            ax.plot(t_replay_rel, pz_replay, color="steelblue",
                    lw=0.6, label="replay (±σ_z)")
        if show_vvo:
            ax.plot(t_vvo_rel, pos_vvo_rel[:, 2], ":", color="darkorange",
                    lw=0.8, alpha=0.8, label="live VVO")
        if show_mocap:
            ax.plot(t_mocap_rel, pos_mocap_rel[:, 2], **MOCAP_KW)
        ax.set_xlabel("t [s]"); ax.set_ylabel("p_z [m]")
        ax.set_title("Altitude")
        ax.grid(alpha=0.3); ax.legend(fontsize=8)

        # [0,2] speed |v|
        ax = axes[0, 2]
        if with_ekf:
            spd_ekf = np.linalg.norm(vel_ekf, axis=1)
            ax.plot(t_ekf_rel, spd_ekf, "--", color="gray",
                    lw=0.8, label="EKF2 (GT)")
        if show_replay:
            spd_replay = np.linalg.norm(vel_N, axis=1)
            ax.plot(t_replay_rel, spd_replay, color="steelblue",
                    lw=0.6, label="replay")
        if show_vvo:
            spd_vvo = np.linalg.norm(vel_vvo, axis=1)
            ax.plot(t_vvo_rel, spd_vvo, ":", color="darkorange",
                    lw=0.8, alpha=0.8, label="live VVO")
        if show_mocap:
            ax.plot(t_mocap_rel, spd_mocap_full, **MOCAP_KW)
        ax.set_xlabel("t [s]"); ax.set_ylabel("|v| [m/s]")
        ax.set_title("Speed")
        ax.grid(alpha=0.3); ax.legend(fontsize=8)

        if not has_replay:
            goto_bottom_row = False
        else:
            goto_bottom_row = True
        if not goto_bottom_row:
            fig.suptitle(f"compare overview — {args.replay_dir.name}  "
                         "(no replay)")
            fig.tight_layout()
            out = Path(out_root) / "compare.png"
            fig.savefig(out, dpi=120, bbox_inches="tight")
            plt.close(fig)
            print(f"  saved → {out}")
            return

        # [1,0] accepted vs rejected radar points per frame — stacked bar.
        # One bar per radar frame, total height = points in that frame,
        # blue = accepted (status==0), red = rejected (status==1).
        ax = axes[1, 0]
        rinn_g  = rinn_df.groupby("t", sort=True)["status"]
        t_frame = np.asarray(rinn_g.size().index, dtype=np.float64)
        n_total = rinn_g.size().to_numpy()
        n_acc   = rinn_g.apply(lambda s: int((s == 0).sum())).to_numpy()
        n_rej   = n_total - n_acc
        t_frame_rel = t_frame + offset_us / 1e6 - t0_s
        # Bar width = median frame spacing (fall back to 0.05 s if a single
        # frame). Slight under-fill so adjacent bars don't merge visually.
        dt = np.median(np.diff(t_frame_rel)) if len(t_frame_rel) > 1 else 0.05
        width = 0.9 * dt
        ax.bar(t_frame_rel, n_acc, width=width, color="steelblue",
               linewidth=0, label="accepted")
        ax.bar(t_frame_rel, n_rej, width=width, bottom=n_acc, color="crimson",
               linewidth=0, label="rejected")
        n_acc_all = int(n_acc.sum())
        n_tot_all = int(n_total.sum())
        rate = 100.0 * n_acc_all / max(n_tot_all, 1)
        ax.text(0.02, 0.98,
                f"accepted: {n_acc_all}/{n_tot_all}  ({rate:.1f}%)",
                transform=ax.transAxes, va="top", ha="left",
                fontsize=8, family="monospace",
                bbox=dict(facecolor="white", edgecolor="0.7",
                          alpha=0.85, pad=4))
        ax.set_xlabel("t [s]"); ax.set_ylabel("# radar points / frame")
        ax.set_title("Accepted vs rejected radar measurements")
        ax.grid(alpha=0.3, axis="y"); ax.legend(fontsize=8, loc="upper right")

        # [1,1] radar normalized innovation histogram.
        # Includes accepted + rejected (skipped points have NaN residual/S
        # and are dropped by np.isfinite). Filtering to accepted-only would
        # tautologically put 100% inside the gate's σ-threshold.
        ax = axes[1, 1]
        z_all = rinn_df["residual"] / np.sqrt(rinn_df["S"].clip(lower=1e-12))
        z = np.asarray(z_all[np.isfinite(z_all)], dtype=np.float64)
        n_acc = int((rinn_df["status"] == 0).sum())
        n_rej = int((rinn_df["status"] == 1).sum())
        bins = np.linspace(-5, 5, 80)
        ax.hist(np.clip(z, -5, 5), bins=bins, density=True, alpha=0.55,
                color="steelblue",
                label=f"n={len(z)} (acc {n_acc}, rej {n_rej})  std={z.std():.3f}")
        xs = np.linspace(-5, 5, 200)
        ax.plot(xs, np.exp(-xs**2 / 2) / np.sqrt(2 * np.pi),
                color="k", lw=1, ls="--", label="N(0,1)")

        # Fraction of |residual / √S| inside 1, 2, 3 σ (N(0,1) reference:
        # 68.27 / 95.45 / 99.73 %). Higher than reference → filter is too
        # pessimistic (S over-estimated); lower → too optimistic.
        if len(z):
            abs_z = np.abs(z)
            n_z   = len(z)
            pct1  = 100.0 * np.sum(abs_z < 1) / n_z
            pct2  = 100.0 * np.sum(abs_z < 2) / n_z
            pct3  = 100.0 * np.sum(abs_z < 3) / n_z
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
        ax.set_title("Radar normalized innovation (accepted + rejected)")
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

        if not with_ekf:
            title_ref = "(replay vs live VVO, no EKF2)"
        elif not with_vvo:
            title_ref = "vs EKF2 (no live VVO)"
        else:
            title_ref = "vs EKF2" + ("  (+ live VVO)" if show_vvo else "")
        fig.suptitle(f"compare overview — {args.replay_dir.name} {title_ref}")
        fig.tight_layout()
        out = Path(out_root) / "compare.png"
        fig.savefig(out, dpi=120, bbox_inches="tight")
        plt.close(fig)
        print(f"  saved → {out}")

    render_overlays(save_dir,                with_ekf=True,  with_vvo=True)
    render_overlays(save_dir / "no_ekf",     with_ekf=False, with_vvo=True)
    render_overlays(save_dir / "no_vvo",     with_ekf=True,  with_vvo=False)

    print(f"\nDone — outputs in {save_dir}/  (+ no_ekf/, no_vvo/)")


if __name__ == "__main__":
    sys.exit(main())
