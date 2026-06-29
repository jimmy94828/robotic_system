#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Timestamp-based map alignment between pose3d.csv and pose2d.csv.

Main difference from map_alignment_v3.py:
  - pose2d is matched by timestamp/stamp
  - pose3d is matched by origin_timestamp
  - no idx offset is required

Expected pose3d columns:
  idx, tx, ty, tz, qx, qy, qz, qw, origin_timestamp
or:
  idx, x, y, z, qx, qy, qz, qw, origin_timestamp

Expected pose2d columns:
  idx, stamp, x, y, yaw_rag
or:
  idx, timestamp, x, y, yaw_rad / yaw

Output:
  - YAML Sim(2) transform from projected 3D trajectory to 2D map trajectory
  - alignment plot
  - optional matched_pairs.csv for debugging
"""

import argparse
import math
from pathlib import Path
from typing import Optional, Tuple

import numpy as np
import pandas as pd
import yaml

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


# -----------------------------
# SE(3) helpers
# -----------------------------
def quat_to_R(qx, qy, qz, qw):
    n = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if n <= 0:
        raise ValueError("Quaternion norm is zero.")
    qx, qy, qz, qw = qx/n, qy/n, qz/n, qw/n

    xx, yy, zz = qx*qx, qy*qy, qz*qz
    xy, xz, yz = qx*qy, qx*qz, qy*qz
    wx, wy, wz = qw*qx, qw*qy, qw*qz

    R = np.array([
        [1 - 2*(yy+zz),     2*(xy - wz),     2*(xz + wy)],
        [    2*(xy + wz), 1 - 2*(xx+zz),     2*(yz - wx)],
        [    2*(xz - wy),     2*(yz + wx), 1 - 2*(xx+yy)],
    ], dtype=np.float64)
    return R


def make_T(txyz, qxyzw):
    tx, ty, tz = txyz
    qx, qy, qz, qw = qxyzw
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = quat_to_R(qx, qy, qz, qw)
    T[:3, 3] = [tx, ty, tz]
    return T


def inv_T(T):
    R = T[:3, :3]
    t = T[:3, 3]
    Ti = np.eye(4, dtype=np.float64)
    Ti[:3, :3] = R.T
    Ti[:3, 3] = -R.T @ t
    return Ti


# -----------------------------
# Timestamp helpers
# -----------------------------
def normalize_timestamp_series(s: pd.Series, unit: str = "auto") -> pd.Series:
    """
    Convert a timestamp series to seconds.

    auto rule:
      Unix ns: ~1e18 -> /1e9
      Unix us: ~1e15 -> /1e6
      Unix ms: ~1e12 -> /1e3
      Unix sec or relative sec: unchanged
    """
    v = pd.to_numeric(s, errors="coerce").astype(float)
    med = float(np.nanmedian(np.abs(v.to_numpy()))) if len(v) else 0.0

    if unit == "auto":
        if med > 1e17:
            div = 1e9
        elif med > 1e14:
            div = 1e6
        elif med > 1e11:
            div = 1e3
        else:
            div = 1.0
    elif unit == "s":
        div = 1.0
    elif unit == "ms":
        div = 1e3
    elif unit == "us":
        div = 1e6
    elif unit == "ns":
        div = 1e9
    else:
        raise ValueError(f"Unknown timestamp unit: {unit}")

    return v / div


def resolve_column(df: pd.DataFrame, candidates, explicit: Optional[str], required=True) -> Optional[str]:
    cols = list(df.columns)
    lower_map = {c.lower(): c for c in cols}

    if explicit and explicit != "auto":
        key = explicit.lower()
        if key in lower_map:
            return lower_map[key]
        if required:
            raise ValueError(f"Column '{explicit}' not found. columns={cols}")
        return None

    for c in candidates:
        if c.lower() in lower_map:
            return lower_map[c.lower()]

    if required:
        raise ValueError(f"None of columns {candidates} found. columns={cols}")
    return None


# -----------------------------
# Plane / projection
# -----------------------------
def fit_ground_plane_basis(P3: np.ndarray):
    P3 = np.asarray(P3, dtype=np.float64)
    if P3.ndim != 2 or P3.shape[1] != 3:
        raise ValueError(f"P3 must be (N,3), got {P3.shape}")
    if P3.shape[0] < 3:
        raise ValueError("Need at least 3 points to fit plane.")

    mu = P3.mean(axis=0)
    Q = P3 - mu
    C = (Q.T @ Q) / P3.shape[0]
    _, V = np.linalg.eigh(C)
    n = V[:, 0]
    n = n / (np.linalg.norm(n) + 1e-12)

    x_axis = np.array([1.0, 0.0, 0.0], dtype=np.float64)
    e1 = x_axis - (x_axis @ n) * n
    if np.linalg.norm(e1) < 1e-6:
        y_axis = np.array([0.0, 1.0, 0.0], dtype=np.float64)
        e1 = y_axis - (y_axis @ n) * n
    e1 = e1 / (np.linalg.norm(e1) + 1e-12)
    e2 = np.cross(n, e1)
    e2 = e2 / (np.linalg.norm(e2) + 1e-12)

    z_axis = np.array([0.0, 0.0, 1.0], dtype=np.float64)
    angle_deg = math.degrees(math.acos(np.clip(abs(n @ z_axis), -1.0, 1.0)))
    return mu, n, e1, e2, angle_deg


def project_to_plane_uv(p3, mu, e1, e2):
    d = p3 - mu
    return np.array([d @ e1, d @ e2], dtype=np.float64)


def project_points(P3: np.ndarray, mode: str, plane=None) -> Tuple[np.ndarray, dict]:
    """Project 3D points to 2D. mode: pca, xy, xz, yz, yx, zx, zy."""
    P3 = np.asarray(P3, dtype=np.float64)
    info = {"projection_mode": mode}

    if mode == "xy":
        return P3[:, [0, 1]], info
    if mode == "xz":
        return P3[:, [0, 2]], info
    if mode == "yz":
        return P3[:, [1, 2]], info
    if mode == "yx":
        return P3[:, [1, 0]], info
    if mode == "zx":
        return P3[:, [2, 0]], info
    if mode == "zy":
        return P3[:, [2, 1]], info

    if mode == "pca":
        if plane is None:
            mu, n, e1, e2, angle_deg = fit_ground_plane_basis(P3)
        else:
            mu, n, e1, e2, angle_deg = plane
        P2 = np.vstack([project_to_plane_uv(p, mu, e1, e2) for p in P3])
        info.update({
            "plane_angle_normal_to_Z_deg": float(angle_deg),
            "plane_mu": [float(x) for x in mu],
            "plane_normal": [float(x) for x in n],
            "plane_e1": [float(x) for x in e1],
            "plane_e2": [float(x) for x in e2],
        })
        return P2, info

    raise ValueError(f"Unknown projection mode: {mode}")


# -----------------------------
# Parsing
# -----------------------------
def load_extrinsic_yaml_as_T(path: str) -> np.ndarray:
    data = yaml.safe_load(Path(path).read_text())
    t = data["translation"]
    r = data["rotation"]
    return make_T((t["x"], t["y"], t["z"]), (r["x"], r["y"], r["z"], r["w"]))


def load_cam_pose(path: str, time_col: str, timestamp_unit: str) -> pd.DataFrame:
    p = Path(path)
    lines = p.read_text().strip().splitlines()
    if not lines:
        raise ValueError(f"Empty cam pose file: {path}")
    first = lines[0]
    has_alpha = any(c.isalpha() for c in first)

    if has_alpha:
        df = pd.read_csv(path, sep=r"[,\s\t]+", engine="python")
    else:
        arr = []
        for line in lines:
            if not line.strip() or line.strip().startswith("#"):
                continue
            toks = line.replace(",", " ").split()
            if len(toks) < 8:
                raise ValueError(f"cam pose line needs at least 8 fields: idx tx ty tz qx qy qz qw, got: {line}")
            arr.append([float(x) for x in toks[:8]])
        df = pd.DataFrame(arr, columns=["idx", "tx", "ty", "tz", "qx", "qy", "qz", "qw"])

    df = df.rename(columns={c: c.strip().lower() for c in df.columns})

    if "index" in df.columns and "idx" not in df.columns:
        df = df.rename(columns={"index": "idx"})
    if "frame" in df.columns and "idx" not in df.columns:
        df = df.rename(columns={"frame": "idx"})
    if "idx" not in df.columns:
        df["idx"] = np.arange(len(df), dtype=int)

    rename_xyz = {}
    for src, dst in (("x", "tx"), ("y", "ty"), ("z", "tz")):
        if src in df.columns and dst not in df.columns:
            rename_xyz[src] = dst
    if rename_xyz:
        df = df.rename(columns=rename_xyz)

    needed = ["idx", "tx", "ty", "tz", "qx", "qy", "qz", "qw"]
    missing = [c for c in needed if c not in df.columns]
    if missing:
        raise ValueError(f"cam pose missing columns: {missing}. columns={list(df.columns)}")

    tcol = resolve_column(
        df,
        candidates=["origin_timestamp", "timestamp", "stamp", "t_sec", "time", "ts", "ts_system"],
        explicit=time_col,
        required=True,
    )
    df["time_sec"] = normalize_timestamp_series(df[tcol], timestamp_unit)
    df["raw_time"] = pd.to_numeric(df[tcol], errors="coerce")
    df["idx"] = df["idx"].astype(int)

    out = df[needed + ["time_sec", "raw_time"]].copy()
    out = out.dropna(subset=["time_sec"])
    return out


def load_base2d_pose_csv(path: str, time_col: str, timestamp_unit: str) -> pd.DataFrame:
    df = pd.read_csv(path)
    df = df.rename(columns={c: c.strip().lower() for c in df.columns})

    if "index" in df.columns and "idx" not in df.columns:
        df = df.rename(columns={"index": "idx"})
    if "idx" not in df.columns:
        df["idx"] = np.arange(len(df), dtype=int)

    # Accept yaw_rag because the current generated CSV uses this spelling.
    if "yaw_rad" in df.columns and "yaw" not in df.columns:
        df = df.rename(columns={"yaw_rad": "yaw"})
    if "yaw_rag" in df.columns and "yaw" not in df.columns:
        df = df.rename(columns={"yaw_rag": "yaw"})
    if "theta" in df.columns and "yaw" not in df.columns:
        df = df.rename(columns={"theta": "yaw"})

    needed = ["idx", "x", "y", "yaw"]
    missing = [c for c in needed if c not in df.columns]
    if missing:
        raise ValueError(
            f"base2d pose missing columns: {missing}. Need idx,x,y and yaw/yaw_rad/yaw_rag. columns={list(df.columns)}"
        )

    tcol = resolve_column(
        df,
        candidates=["stamp", "timestamp", "origin_timestamp", "t_sec", "time", "ts", "ts_system"],
        explicit=time_col,
        required=True,
    )
    df["time_sec"] = normalize_timestamp_series(df[tcol], timestamp_unit)
    df["raw_time"] = pd.to_numeric(df[tcol], errors="coerce")
    df["idx"] = df["idx"].astype(int)

    out = df[needed + ["time_sec", "raw_time"]].copy()
    out = out.dropna(subset=["time_sec"])
    return out


# -----------------------------
# Matching
# -----------------------------
def match_by_timestamp(
    cam: pd.DataFrame,
    base2d: pd.DataFrame,
    max_time_diff: float,
    time_offset_3d_to_2d: float,
    direction: str,
) -> pd.DataFrame:
    """
    Match pose3d to nearest pose2d timestamp.

    Matching equation:
      pose2d.time_sec ~= pose3d.time_sec + time_offset_3d_to_2d
    """
    c = cam.copy()
    b = base2d.copy()
    c["match_time"] = c["time_sec"] + float(time_offset_3d_to_2d)

    c = c.sort_values("match_time").reset_index(drop=True)
    b = b.sort_values("time_sec").reset_index(drop=True)

    merged = pd.merge_asof(
        c,
        b,
        left_on="match_time",
        right_on="time_sec",
        direction=direction,
        tolerance=float(max_time_diff),
        suffixes=("_cam", "_base"),
    )

    # Drop unmatched rows. x/y/yaw come from base2d and become unsuffixed or suffixed depending on collisions.
    if "x" in merged.columns:
        xcol, ycol = "x", "y"
    else:
        xcol, ycol = "x_base", "y_base"
    merged = merged.dropna(subset=[xcol, ycol]).copy()

    # Avoid using the same 2D pose multiple times. Keep the closest time match.
    merged["abs_time_diff"] = np.abs(merged["match_time"] - merged["time_sec_base"])
    merged = merged.sort_values("abs_time_diff").drop_duplicates(subset=["idx_base"], keep="first")
    merged = merged.sort_values("match_time").reset_index(drop=True)

    return merged


# -----------------------------
# Sim(2)
# -----------------------------
def estimate_sim2(Psrc, Pdst, with_scale=True):
    Psrc = np.asarray(Psrc, dtype=np.float64)
    Pdst = np.asarray(Pdst, dtype=np.float64)

    mask = np.isfinite(Psrc).all(axis=1) & np.isfinite(Pdst).all(axis=1)
    Psrc = Psrc[mask]
    Pdst = Pdst[mask]
    n = Psrc.shape[0]
    if n < 2:
        raise ValueError(f"Not enough finite points: n={n}")

    mu_s = Psrc.mean(axis=0)
    mu_d = Pdst.mean(axis=0)
    X = Psrc - mu_s
    Y = Pdst - mu_d

    cov = (X.T @ Y) / n
    U, S, Vt = np.linalg.svd(cov)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T

    if with_scale:
        varX = (X * X).sum() / n
        if varX <= 1e-12:
            raise ValueError("Degenerate configuration: varX too small")
        s = float(S.sum() / varX)
    else:
        s = 1.0

    t = mu_d - s * (R @ mu_s)
    return s, R, t


def apply_sim2(P, s, R, t):
    P = np.asarray(P, dtype=np.float64)
    return (s * (R @ P.T)).T + t


def rmse(Psrc, Pdst, s, R, t):
    pred = apply_sim2(Psrc, s, R, t)
    e = pred - Pdst
    return float(math.sqrt((e * e).sum() / Psrc.shape[0]))


def robust_estimate_sim2(Psrc, Pdst, with_scale=True, trim_keep=0.8, iters=3, min_inliers=10):
    Psrc = np.asarray(Psrc, dtype=np.float64)
    Pdst = np.asarray(Pdst, dtype=np.float64)
    if Psrc.shape[0] < min_inliers:
        raise ValueError(f"Too few points for robust: {Psrc.shape[0]} < {min_inliers}")

    mask = np.ones(Psrc.shape[0], dtype=bool)
    s, R, t = 1.0, np.eye(2), np.zeros(2)

    for _ in range(iters):
        if mask.sum() < min_inliers:
            break
        s, R, t = estimate_sim2(Psrc[mask], Pdst[mask], with_scale=with_scale)
        pred = apply_sim2(Psrc, s, R, t)
        r = np.linalg.norm(pred - Pdst, axis=1)
        thr = np.quantile(r, trim_keep)
        new_mask = r <= thr
        if new_mask.sum() == mask.sum():
            mask = new_mask
            break
        mask = new_mask

    if mask.sum() < min_inliers:
        raise ValueError("Too few inliers after trimming.")

    s, R, t = estimate_sim2(Psrc[mask], Pdst[mask], with_scale=with_scale)
    err = rmse(Psrc[mask], Pdst[mask], s, R, t)
    return err, s, R, t, mask


# -----------------------------
# Main solving
# -----------------------------
def cam_row_to_T(r):
    return make_T((r.tx, r.ty, r.tz), (r.qx, r.qy, r.qz, r.qw))


def build_3d_base_points(merged: pd.DataFrame, T_cam_base: np.ndarray) -> np.ndarray:
    P3 = []
    for _, r in merged.iterrows():
        T_m3_c = cam_row_to_T(r)
        T_m3_b = T_m3_c @ T_cam_base
        P3.append(T_m3_b[:3, 3])
    return np.asarray(P3, dtype=np.float64)


def solve_alignment_for_candidate(
    merged_all: pd.DataFrame,
    T_cam_base: np.ndarray,
    tag: str,
    projection_modes,
    with_scale: bool,
    robust: bool,
    trim_keep: float,
    robust_iters: int,
    min_matched: int,
    pose_start: int,
    first_n_poses: int,
):
    if len(merged_all) < min_matched:
        return []

    merged = merged_all.copy().sort_values("match_time").reset_index(drop=True)

    start = int(max(0, pose_start))
    if first_n_poses and first_n_poses > 0:
        merged_used = merged.iloc[start:start + int(first_n_poses)].copy()
    else:
        merged_used = merged.iloc[start:].copy()

    if len(merged_used) < min_matched:
        return []

    P3_3d = build_3d_base_points(merged_used, T_cam_base)
    P2_xy = merged_used[["x", "y"]].to_numpy(np.float64)

    results = []
    for proj_mode in projection_modes:
        # PCA has sign ambiguity. Try four sign variants.
        if proj_mode == "pca":
            try:
                mu, n, e1, e2, angle_deg = fit_ground_plane_basis(P3_3d)
            except Exception:
                continue
            variants = [
                ("(+,+)", e1, e2),
                ("(+,-)", e1, -e2),
                ("(-,+)", -e1, e2),
                ("(-,-)", -e1, -e2),
            ]
            variant_items = []
            for btag, e1v, e2v in variants:
                P3_uv, info = project_points(P3_3d, "pca", plane=(mu, n, e1v, e2v, angle_deg))
                info["projection_mode"] = f"pca|basis{btag}"
                variant_items.append((P3_uv, info))
        else:
            try:
                P3_uv, info = project_points(P3_3d, proj_mode)
            except Exception:
                continue
            variant_items = [(P3_uv, info)]

        for P3_uv, proj_info in variant_items:
            try:
                if robust:
                    err, s, R, t, in_mask = robust_estimate_sim2(
                        P3_uv,
                        P2_xy,
                        with_scale=with_scale,
                        trim_keep=trim_keep,
                        iters=robust_iters,
                        min_inliers=min_matched,
                    )
                else:
                    s, R, t = estimate_sim2(P3_uv, P2_xy, with_scale=with_scale)
                    err = rmse(P3_uv, P2_xy, s, R, t)
                    in_mask = np.ones(P3_uv.shape[0], dtype=bool)
            except Exception:
                continue

            results.append({
                "err": float(err),
                "tag": tag,
                "T_cam_base": T_cam_base,
                "projection_info": proj_info,
                "s": float(s),
                "R": R,
                "t": t,
                "theta": float(math.atan2(R[1, 0], R[0, 0])),
                "n_all": int(P3_uv.shape[0]),
                "n_in": int(in_mask.sum()),
                "in_mask": in_mask,
                "merged_all": merged,
                "merged_used": merged_used,
                "pose_start": int(pose_start),
                "first_n_poses": int(first_n_poses),
            })
    return results


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cam_pose", default="./input/pose3d.csv", help="camera trajectory CSV, e.g. pose3d.csv")
    ap.add_argument("--base2d_pose", default="./input/pose2d.csv")
    ap.add_argument("--cam_to_base", default="./input/cam_to_base.yaml")

    ap.add_argument("--pose3d_time_col", default="origin_timestamp", help="pose3d timestamp column. Default: origin_timestamp")
    ap.add_argument("--pose2d_time_col", default="stamp", help="pose2d timestamp column. Default: stamp")
    ap.add_argument("--pose3d_time_unit", default="auto", choices=["auto", "s", "ms", "us", "ns"])
    ap.add_argument("--pose2d_time_unit", default="auto", choices=["auto", "s", "ms", "us", "ns"])
    ap.add_argument("--max_time_diff", type=float, default=0.2, help="maximum allowed timestamp difference in seconds")
    ap.add_argument("--time_offset_3d_to_2d", type=float, default=0.0,
                    help="match equation: pose2d_time ~= pose3d_origin_timestamp + this offset")
    ap.add_argument("--match_direction", default="nearest", choices=["nearest", "backward", "forward"])

    ap.add_argument("--auto_time_offset", action="store_true")
    ap.add_argument("--time_offset_min", type=float, default=-5.0)
    ap.add_argument("--time_offset_max", type=float, default=5.0)
    ap.add_argument("--time_offset_step", type=float, default=0.05)

    ap.add_argument("--extrinsic_mode", choices=["cam_to_base", "base_to_cam", "auto"], default="cam_to_base",
                    help="cam_to_base: YAML is camera->base. base_to_cam: YAML is base->camera. auto: try both.")
    ap.add_argument("--projection_mode", default="pca", choices=["pca", "xy", "xz", "yz", "yx", "zx", "zy", "auto"],
                    help="3D-to-2D pre-projection before Sim(2). auto tries axis pairs and PCA.")
    ap.add_argument("--min_matched", type=int, default=10)
    ap.add_argument("--pose_start", type=int, default=0)
    ap.add_argument("--first_n_poses", type=int, default=0, help="0 means use all matched poses after pose_start")

    ap.add_argument("--yaw_deg", action="store_true", help="If pose2d yaw is stored in degrees; currently only affects future yaw use.")
    ap.add_argument("--with_scale", action="store_true")
    ap.add_argument("--robust", action="store_true")
    ap.add_argument("--trim_keep", type=float, default=0.8)
    ap.add_argument("--robust_iters", type=int, default=3)

    ap.add_argument("--out", default="./output/map3d_to_map2d_sim2_time.yaml")
    ap.add_argument("--plot_out", default="./output/traj_alignment_time.png")
    ap.add_argument("--matched_out", default="", help="Optional CSV path for matched timestamp pairs.")
    ap.add_argument("--top_k", type=int, default=10)
    args = ap.parse_args()

    cam = load_cam_pose(args.cam_pose, args.pose3d_time_col, args.pose3d_time_unit)
    base2d = load_base2d_pose_csv(args.base2d_pose, args.pose2d_time_col, args.pose2d_time_unit)
    if args.yaw_deg:
        base2d["yaw"] = np.deg2rad(base2d["yaw"].astype(float))

    print(f"[load] pose3d rows={len(cam)}, time range={cam['time_sec'].min():.6f}..{cam['time_sec'].max():.6f}")
    print(f"[load] pose2d rows={len(base2d)}, time range={base2d['time_sec'].min():.6f}..{base2d['time_sec'].max():.6f}")

    T_yaml = load_extrinsic_yaml_as_T(args.cam_to_base)
    candidates = []
    if args.extrinsic_mode in ("cam_to_base", "auto"):
        T_base_cam = T_yaml
        candidates.append(("cam_to_base(inv)", inv_T(T_base_cam)))
    if args.extrinsic_mode in ("base_to_cam", "auto"):
        candidates.append(("base_to_cam(direct)", T_yaml))

    if args.projection_mode == "auto":
        projection_modes = ["xy", "xz", "yz", "yx", "zx", "zy", "pca"]
    else:
        projection_modes = [args.projection_mode]

    if args.auto_time_offset:
        offsets = np.arange(args.time_offset_min, args.time_offset_max + 0.5 * args.time_offset_step, args.time_offset_step)
        offsets = [float(x) for x in offsets]
    else:
        offsets = [float(args.time_offset_3d_to_2d)]

    all_results = []
    match_cache = {}
    for off in offsets:
        merged = match_by_timestamp(
            cam,
            base2d,
            max_time_diff=args.max_time_diff,
            time_offset_3d_to_2d=off,
            direction=args.match_direction,
        )
        match_cache[off] = merged
        if len(merged) < args.min_matched:
            continue

        for tag, T_cam_base in candidates:
            res = solve_alignment_for_candidate(
                merged_all=merged,
                T_cam_base=T_cam_base,
                tag=tag,
                projection_modes=projection_modes,
                with_scale=args.with_scale,
                robust=args.robust,
                trim_keep=args.trim_keep,
                robust_iters=args.robust_iters,
                min_matched=args.min_matched,
                pose_start=args.pose_start,
                first_n_poses=args.first_n_poses,
            )
            for r in res:
                r["time_offset_3d_to_2d"] = float(off)
                r["matched_total"] = int(len(merged))
                all_results.append(r)

    if not all_results:
        raise RuntimeError(
            "No valid solution. Check timestamp columns, timestamp unit, max_time_diff, time_offset, min_matched, extrinsic."
        )

    all_results.sort(key=lambda x: x["err"])
    best = all_results[0]

    print("\n[top candidates]")
    for i, r in enumerate(all_results[:args.top_k]):
        print(
            f"#{i:02d} rmse={r['err']:.6f}, matched_used={r['n_all']}, inliers={r['n_in']}, "
            f"matched_total={r['matched_total']}, dt_offset={r['time_offset_3d_to_2d']:.6f}, "
            f"extrinsic={r['tag']}, projection={r['projection_info']['projection_mode']}, "
            f"s={r['s']:.6f}, theta_deg={math.degrees(r['theta']):.3f}"
        )

    print("\n[best]")
    print(
        f"extrinsic={best['tag']}, projection={best['projection_info']['projection_mode']}, "
        f"time_offset_3d_to_2d={best['time_offset_3d_to_2d']:.6f}, "
        f"matched_total={best['matched_total']}, matched_used={best['n_all']}, "
        f"inliers={best['n_in']}, rmse={best['err']:.6f}"
    )
    print(f"scale s = {best['s']:.8f}")
    print(f"theta deg = {math.degrees(best['theta']):.6f}")
    print(f"t = [{best['t'][0]:.6f}, {best['t'][1]:.6f}]")
    if "plane_angle_normal_to_Z_deg" in best["projection_info"]:
        print(f"plane angle(normal,Z) = {best['projection_info']['plane_angle_normal_to_Z_deg']:.3f} deg")

    # Prepare full trajectory plot using best transform.
    merged_best = best["merged_all"].copy()
    merged_used = best["merged_used"].copy()
    T_cam_base = best["T_cam_base"]

    base_xy = base2d.sort_values("time_sec")[["x", "y"]].to_numpy(np.float64)

    P3_all_3d = build_3d_base_points(merged_best, T_cam_base)
    P3_used_3d = build_3d_base_points(merged_used, T_cam_base)
    P2_used = merged_used[["x", "y"]].to_numpy(np.float64)

    proj_mode = best["projection_info"]["projection_mode"]
    proj_base = proj_mode.split("|")[0]

    # Reuse PCA basis from best if PCA was selected.
    if proj_base == "pca":
        info = best["projection_info"]
        plane = (
            np.asarray(info["plane_mu"], dtype=np.float64),
            np.asarray(info["plane_normal"], dtype=np.float64),
            np.asarray(info["plane_e1"], dtype=np.float64),
            np.asarray(info["plane_e2"], dtype=np.float64),
            float(info["plane_angle_normal_to_Z_deg"]),
        )
        P3_all_uv, _ = project_points(P3_all_3d, "pca", plane=plane)
        P3_used_uv, _ = project_points(P3_used_3d, "pca", plane=plane)
    else:
        P3_all_uv, _ = project_points(P3_all_3d, proj_base)
        P3_used_uv, _ = project_points(P3_used_3d, proj_base)

    P3_all_xy = apply_sim2(P3_all_uv, best["s"], best["R"], best["t"])
    P3_used_xy = apply_sim2(P3_used_uv, best["s"], best["R"], best["t"])

    in_mask = best["in_mask"]
    P2_in = P2_used[in_mask]
    P3_in = P3_used_xy[in_mask]

    plt.figure(figsize=(8, 8))
    plt.plot(base_xy[:, 0], base_xy[:, 1], linewidth=2, label="2D base trajectory (map2d)")
    plt.plot(P3_all_xy[:, 0], P3_all_xy[:, 1], linewidth=2, label="timestamp-matched 3D->2D trajectory")
    plt.scatter(P2_used[:, 0], P2_used[:, 1], s=12, alpha=0.35, label="used base2d pts")
    plt.scatter(P3_used_xy[:, 0], P3_used_xy[:, 1], s=12, alpha=0.35, label="used transformed pts")
    plt.scatter(P2_in[:, 0], P2_in[:, 1], s=20, alpha=0.9, label="inlier base2d pts")
    plt.scatter(P3_in[:, 0], P3_in[:, 1], s=20, alpha=0.9, label="inlier transformed pts")
    plt.axis("equal")
    plt.grid(True)
    plt.xlabel("x")
    plt.ylabel("y")
    plt.title(
        f"Timestamp Alignment: RMSE={best['err']:.4f}, dt={best['time_offset_3d_to_2d']:.3f}s, "
        f"s={best['s']:.4f}, theta={math.degrees(best['theta']):.2f} deg\n"
        f"extrinsic={best['tag']}, projection={best['projection_info']['projection_mode']}, robust={args.robust}"
    )
    plt.legend()
    plt.tight_layout()
    Path(args.plot_out).parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(args.plot_out, dpi=200)
    print(f"[plot] {args.plot_out}")

    if args.matched_out:
        mo = Path(args.matched_out)
        mo.parent.mkdir(parents=True, exist_ok=True)
        dbg = merged_best[[
            "idx_cam", "raw_time_cam", "time_sec_cam",
            "idx_base", "raw_time_base", "time_sec_base",
            "match_time", "abs_time_diff", "x", "y", "yaw",
        ]].copy()
        dbg.to_csv(mo, index=False)
        print(f"[matched] {mo}")

    out = {
        "matching": {
            "method": "timestamp_nearest",
            "pose3d_time_col": args.pose3d_time_col,
            "pose2d_time_col": args.pose2d_time_col,
            "pose3d_time_unit": args.pose3d_time_unit,
            "pose2d_time_unit": args.pose2d_time_unit,
            "max_time_diff_sec": float(args.max_time_diff),
            "time_offset_3d_to_2d_sec": float(best["time_offset_3d_to_2d"]),
            "match_direction": args.match_direction,
        },
        "extrinsic_mode_selected": best["tag"],
        "projection": best["projection_info"],
        "with_scale": bool(args.with_scale),
        "robust": bool(args.robust),
        "trim_keep": float(args.trim_keep),
        "robust_iters": int(args.robust_iters),
        "rmse_xy": float(best["err"]),
        "sim2": {
            "s": float(best["s"]),
            "R": [[float(best["R"][0, 0]), float(best["R"][0, 1])],
                  [float(best["R"][1, 0]), float(best["R"][1, 1])]],
            "t": {"x": float(best["t"][0]), "y": float(best["t"][1])},
            "theta_rad": float(best["theta"]),
            "theta_deg": float(math.degrees(best["theta"])),
        },
        "pose_subset": {
            "pose_start": int(args.pose_start),
            "first_n_poses": int(args.first_n_poses),
            "matched_total": int(best["matched_total"]),
            "matched_used": int(best["n_all"]),
            "inliers": int(best["n_in"]),
        },
    }
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(yaml.safe_dump(out, sort_keys=False, allow_unicode=True))
    print(f"[write] {args.out}")


if __name__ == "__main__":
    main()
