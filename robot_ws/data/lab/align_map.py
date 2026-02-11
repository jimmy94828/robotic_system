#!/usr/bin/env python3
import os
import json
import math
import argparse
import numpy as np

import yaml
import cv2
from scipy.optimize import least_squares


# ----------------------------
# Utils: rotation + plane align
# ----------------------------
def rodrigues_from_a_to_b(a, b):
    """Rotation matrix R such that R @ a == b (both unit vectors)."""
    a = a / (np.linalg.norm(a) + 1e-12)
    b = b / (np.linalg.norm(b) + 1e-12)
    v = np.cross(a, b)
    c = float(np.dot(a, b))
    s = float(np.linalg.norm(v))
    if s < 1e-12:
        # a parallel to b (or opposite)
        if c > 0:
            return np.eye(3, dtype=np.float64)
        # 180 deg rotation around any axis orthogonal to a
        axis = np.array([1, 0, 0], dtype=np.float64)
        if abs(a[0]) > 0.9:
            axis = np.array([0, 1, 0], dtype=np.float64)
        v = np.cross(a, axis)
        v = v / (np.linalg.norm(v) + 1e-12)
        K = np.array([[0, -v[2], v[1]],
                      [v[2], 0, -v[0]],
                      [-v[1], v[0], 0]], dtype=np.float64)
        return np.eye(3) + 2 * (K @ K)

    v = v / s
    K = np.array([[0, -v[2], v[1]],
                  [v[2], 0, -v[0]],
                  [-v[1], v[0], 0]], dtype=np.float64)
    R = np.eye(3, dtype=np.float64) + s * K + (1 - c) * (K @ K)
    return R


def fit_plane_normal_svd(points_xyz):
    """Fit plane normal using SVD of covariance. Return unit normal."""
    c = points_xyz.mean(axis=0)
    X = points_xyz - c
    # covariance
    C = (X.T @ X) / max(len(points_xyz), 1)
    # eigenvector with smallest eigenvalue
    w, V = np.linalg.eigh(C)
    n = V[:, np.argmin(w)]
    n = n / (np.linalg.norm(n) + 1e-12)
    return n


# ----------------------------
# Map YAML/PGM loading + edge extraction
# ----------------------------
def load_map_yaml(yaml_path):
    with open(yaml_path, "r") as f:
        meta = yaml.safe_load(f)

    image_path = meta["image"]
    # image path may be relative to yaml folder
    if not os.path.isabs(image_path):
        image_path = os.path.join(os.path.dirname(yaml_path), image_path)

    res = float(meta["resolution"])
    origin = meta["origin"]  # [x, y, yaw]
    ox, oy, oyaw = float(origin[0]), float(origin[1]), float(origin[2])

    occ_th = float(meta.get("occupied_thresh", 0.65))
    free_th = float(meta.get("free_thresh", 0.196))
    negate = int(meta.get("negate", 0))

    img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise RuntimeError(f"Failed to read map image: {image_path}")

    # Convert to occupancy probability style:
    # Typical PGM: 0=occupied, 254=free, 205=unknown (but varies)
    # nav2 convention with occupied_thresh/free_thresh uses normalized [0,1].
    img_f = img.astype(np.float32) / 255.0
    if negate == 1:
        img_f = 1.0 - img_f

    # Occupied mask: darker means more occupied (after negate handling)
    # Use thresholds in yaml if present
    occ = (img_f < (1.0 - occ_th)).astype(np.uint8)  # 1 where occupied
    # Unknown often mid-gray; we ignore it for edges by default.

    # Extract edge of occupied regions
    occ255 = (occ * 255).astype(np.uint8)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    dil = cv2.dilate(occ255, kernel, iterations=1)
    ero = cv2.erode(occ255, kernel, iterations=1)
    edge = cv2.subtract(dil, ero)
    # thin and denoise
    edge = cv2.threshold(edge, 10, 255, cv2.THRESH_BINARY)[1]

    H, W = img.shape[:2]
    return {
        "meta": meta,
        "image_path": image_path,
        "img": img,
        "occ": occ,
        "edge": edge,
        "resolution": res,
        "origin": (ox, oy, oyaw),
        "H": H,
        "W": W
    }


def map_pixel_to_map_xy(u, v, map_info):
    """
    Convert pixel (u,v) in PGM (origin top-left) to map frame (x,y) in meters.
    YAML origin is at map's bottom-left corner in map frame, with yaw.
    """
    res = map_info["resolution"]
    ox, oy, oyaw = map_info["origin"]
    H = map_info["H"]

    # grid coords in meters with (0,0) at bottom-left
    gx = (u + 0.5) * res
    gy = ( (H - 1 - v) + 0.5 ) * res

    cy = math.cos(oyaw)
    sy = math.sin(oyaw)
    x = ox + cy * gx - sy * gy
    y = oy + sy * gx + cy * gy
    return x, y


def map_xy_to_map_pixel(x, y, map_info):
    """
    Convert map frame (x,y) meters to fractional pixel (u,v) in PGM.
    Return (u, v) float where u right, v down.
    """
    res = map_info["resolution"]
    ox, oy, oyaw = map_info["origin"]
    H = map_info["H"]

    # inverse rotate/translate to grid meters
    dx = x - ox
    dy = y - oy
    cy = math.cos(-oyaw)
    sy = math.sin(-oyaw)
    gx = cy * dx - sy * dy
    gy = sy * dx + cy * dy

    u = gx / res - 0.5
    v_grid = gy / res - 0.5  # v in grid-up coordinate (bottom-left origin)
    v = (H - 1) - v_grid
    return u, v


# ----------------------------
# Fourier-Mellin init (rotation + scale)
# ----------------------------
def hanning_window(H, W):
    wx = np.hanning(W)
    wy = np.hanning(H)
    w = np.outer(wy, wx).astype(np.float32)
    return w


def fft_logmag(img01):
    """
    img01: float32 image in [0,1]
    return log magnitude spectrum (float32)
    """
    H, W = img01.shape
    w = hanning_window(H, W)
    f = np.fft.fft2(img01 * w)
    f = np.fft.fftshift(f)
    mag = np.abs(f) + 1e-6
    logmag = np.log(mag).astype(np.float32)
    return logmag


def estimate_scale_rotation_fm(edge2, edge3):
    """
    edge2, edge3: uint8 0/255 edges, same shape (H,W)
    Return (scale, theta_rad) as initial estimate.
    """
    H, W = edge2.shape
    I2 = (edge2.astype(np.float32) / 255.0)
    I3 = (edge3.astype(np.float32) / 255.0)

    L2 = fft_logmag(I2)
    L3 = fft_logmag(I3)

    center = (W / 2.0, H / 2.0)
    maxRadius = min(center[0], center[1])

    # log-polar
    lp2 = cv2.warpPolar(L2, (W, H), center, maxRadius, cv2.WARP_POLAR_LOG)
    lp3 = cv2.warpPolar(L3, (W, H), center, maxRadius, cv2.WARP_POLAR_LOG)

    (dx, dy), resp = cv2.phaseCorrelate(lp2, lp3)

    # OpenCV log-polar mapping: radius = exp(x / M), M = W / log(maxRadius)
    M = W / (math.log(maxRadius + 1e-6) + 1e-12)
    scale = math.exp(dx / (M + 1e-12))

    # angle axis is along y: 0..H corresponds to 0..360 deg
    rot_deg = 360.0 * (dy / H)
    theta = math.radians(rot_deg)

    # Sign ambiguities exist. We'll return candidates and let later scoring/optimization fix it.
    return scale, theta, resp


# ----------------------------
# Distance transform objective & refinement
# ----------------------------
def build_distance_transform(edge):
    """
    edge: uint8 0/255, 255 means edge pixel
    distanceTransform computes distance to nearest zero pixel, so invert.
    """
    inv = cv2.threshold(edge, 1, 255, cv2.THRESH_BINARY_INV)[1]
    D = cv2.distanceTransform(inv, cv2.DIST_L2, 3).astype(np.float32)
    return D


def bilinear_sample(img, u, v):
    """Bilinear sampling for img at fractional (u,v). u->x, v->y."""
    H, W = img.shape
    u0 = np.floor(u).astype(np.int32)
    v0 = np.floor(v).astype(np.int32)
    u1 = u0 + 1
    v1 = v0 + 1

    # clip
    u0c = np.clip(u0, 0, W - 1)
    u1c = np.clip(u1, 0, W - 1)
    v0c = np.clip(v0, 0, H - 1)
    v1c = np.clip(v1, 0, H - 1)

    Ia = img[v0c, u0c]
    Ib = img[v0c, u1c]
    Ic = img[v1c, u0c]
    Id = img[v1c, u1c]

    wa = (u1 - u) * (v1 - v)
    wb = (u - u0) * (v1 - v)
    wc = (u1 - u) * (v - v0)
    wd = (u - u0) * (v - v0)

    return wa * Ia + wb * Ib + wc * Ic + wd * Id


def residuals_sim2(params, pts_xy, dist_img, map_info):
    """
    params = [log_s, theta, tx, ty] where tx,ty in map frame meters
    pts_xy: (N,2) in 3D frame (after floor align) units
    dist_img: distance transform image in pixel units
    returns residuals (N,)
    """
    log_s, theta, tx, ty = params
    s = math.exp(log_s)
    c = math.cos(theta)
    sn = math.sin(theta)
    R = np.array([[c, -sn],
                  [sn,  c]], dtype=np.float64)

    p = pts_xy.astype(np.float64)
    p2 = (s * (p @ R.T)) + np.array([tx, ty], dtype=np.float64)

    # map (x,y) -> pixel (u,v)
    u = np.empty((len(p2),), dtype=np.float64)
    v = np.empty((len(p2),), dtype=np.float64)
    for i in range(len(p2)):
        ui, vi = map_xy_to_map_pixel(float(p2[i, 0]), float(p2[i, 1]), map_info)
        u[i] = ui
        v[i] = vi

    # keep only points inside image bounds to avoid bias; outside -> large penalty
    H, W = dist_img.shape
    inside = (u >= 0) & (u <= W - 1) & (v >= 0) & (v <= H - 1)

    res = np.ones((len(p2),), dtype=np.float64) * 50.0  # large penalty for outside
    if inside.any():
        res_in = bilinear_sample(dist_img, u[inside], v[inside])
        res[inside] = res_in

    return res


# ----------------------------
# Rasterize 3D points to edge image in map canvas (for FM init)
# ----------------------------
def rasterize_points_to_edge_image(pts_xy_mapframe, map_info, thickness=1):
    """
    pts_xy_mapframe: (N,2) in map frame meters (approx), we draw them into map image pixels.
    return uint8 edge mask 0/255.
    """
    H, W = map_info["H"], map_info["W"]
    img = np.zeros((H, W), dtype=np.uint8)

    # draw points
    for x, y in pts_xy_mapframe:
        u, v = map_xy_to_map_pixel(float(x), float(y), map_info)
        ui = int(round(u))
        vi = int(round(v))
        if 0 <= ui < W and 0 <= vi < H:
            img[vi, ui] = 255

    if thickness > 1:
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (thickness, thickness))
        img = cv2.dilate(img, k, iterations=1)

    # edge-like (thin)
    img = cv2.threshold(img, 10, 255, cv2.THRESH_BINARY)[1]
    return img


# ----------------------------
# Main
# ----------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--npz", required=True, help="3DGS npz path (contains pts/means3D and semantic_ids)")
    ap.add_argument("--map_yaml", required=True, help="2D map yaml path (with image pgm/png)")
    ap.add_argument("--floor_id", type=int, default=2, help="semantic id for floor (default: 2)")
    ap.add_argument("--z_min", type=float, default=-5.0, help="min height above floor to keep")
    ap.add_argument("--z_max", type=float, default=1.0, help="max height above floor to keep")
    ap.add_argument("--max_points", type=int, default=200000, help="max points used in optimization (subsample)")
    ap.add_argument("--out_dir", default="align_out", help="output directory")
    ap.add_argument("--manual", action="store_true", help="enter interactive manual adjustment after automatic fit")
    ap.add_argument("--step_trans", type=float, default=0.05, help="translation step in meters for manual mode")
    ap.add_argument("--step_rot_deg", type=float, default=1.0, help="rotation step in degrees for manual mode")
    ap.add_argument("--step_scale", type=float, default=0.01, help="scale step fraction for manual mode (e.g. 0.01 = 1%)")
    args = ap.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)

    # Load map
    map_info = load_map_yaml(args.map_yaml)
    edge2 = map_info["edge"]
    dist_img = build_distance_transform(edge2)

    # Compute map edge centroid in map frame (meters)
    ys, xs = np.where(edge2 > 0)
    num_edges = len(xs)
    if num_edges < 1000:
        print(f"Warning: map edge pixels too few ({num_edges}). Trying Canny fallback.")
        # try a Canny-based edge extraction fallback from the original grayscale map image
        try:
            img_gray = map_info["img"]
            blur = cv2.GaussianBlur(img_gray, (5, 5), 0)
            can = cv2.Canny(blur, 50, 150)
            # dilate a bit to make edges thicker
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
            can = cv2.dilate(can, kernel, iterations=1)
            edge2 = (can > 0).astype(np.uint8) * 255
            dist_img = build_distance_transform(edge2)
            ys, xs = np.where(edge2 > 0)
            num_edges = len(xs)
            print(f"Canny fallback produced {num_edges} edge pixels.")
        except Exception as e:
            raise RuntimeError("Map edge pixels too few and Canny fallback failed: " + str(e))

    if num_edges < 100:
        raise RuntimeError("Map edge pixels too few even after fallback. Check your map image or yaml thresholds.")

    edge_xy = np.array([map_pixel_to_map_xy(int(u), int(v), map_info) for u, v in zip(xs, ys)], dtype=np.float64)
    c_map = edge_xy.mean(axis=0)

    # Load 3DGS
    data = np.load(args.npz)
    pts = data["pts"] if "pts" in data.files else data["means3D"]
    sem = data["semantic_ids"] if "semantic_ids" in data.files else data["pan"]

    pts = pts.astype(np.float64)
    sem = sem.astype(np.int32)

    # Floor alignment
    floor_mask = (sem == args.floor_id)
    floor_pts = pts[floor_mask]
    if len(floor_pts) < 5000:
        raise RuntimeError("Floor points too few. Check floor_id or semantic_ids.")
    n = fit_plane_normal_svd(floor_pts)

    # rotate normal -> +Z
    Rg = rodrigues_from_a_to_b(n, np.array([0, 0, 1], dtype=np.float64))
    pts_rot = (Rg @ pts.T).T

    # shift so floor at z=0
    z0 = np.median(pts_rot[floor_mask, 2])
    pts_rot[:, 2] -= z0

    # Filter non-floor + height slice
    keep = (~floor_mask) & (pts_rot[:, 2] >= args.z_min) & (pts_rot[:, 2] <= args.z_max)
    pts_use = pts_rot[keep]
    sem_use = sem[keep]

    if len(pts_use) < 50000:
        raise RuntimeError("Too few points after filtering. Try wider z_min/z_max or check floor_id.")

    # Subsample for speed
    if len(pts_use) > args.max_points:
        idx = np.random.choice(len(pts_use), args.max_points, replace=False)
        pts_use = pts_use[idx]
        sem_use = sem_use[idx]

    pts_xy = pts_use[:, :2]
    c_3d = pts_xy.mean(axis=0)

    # Initial translation by centroid alignment (assume s=1, theta=0)
    tx0, ty0 = (c_map - c_3d)

    # Create an initial rasterized edge image for 3D points in map canvas
    pts_xy_init_map = pts_xy + np.array([tx0, ty0], dtype=np.float64)
    edge3_init = rasterize_points_to_edge_image(pts_xy_init_map, map_info, thickness=2)

    # Fourier-Mellin init (scale + rotation)
    s_fm, th_fm, resp = estimate_scale_rotation_fm(edge2, edge3_init)

    # resolve ambiguity by trying a few candidates and picking the best by distance score
    candidates = [
        (s_fm,  th_fm),
        (s_fm, -th_fm),
        (1.0 / max(s_fm, 1e-6),  th_fm),
        (1.0 / max(s_fm, 1e-6), -th_fm),
    ]

    def quick_score(s, th):
        c = math.cos(th); sn = math.sin(th)
        R = np.array([[c, -sn],[sn, c]], dtype=np.float64)
        t = c_map - (s * (c_3d @ R.T))
        # sample 10k points
        M = min(len(pts_xy), 10000)
        sel = np.random.choice(len(pts_xy), M, replace=False)
        p = pts_xy[sel]
        p2 = (s * (p @ R.T)) + t
        # evaluate distance transform
        u = np.zeros((M,), dtype=np.float64)
        v = np.zeros((M,), dtype=np.float64)
        for i in range(M):
            ui, vi = map_xy_to_map_pixel(float(p2[i,0]), float(p2[i,1]), map_info)
            u[i] = ui; v[i] = vi
        H, W = dist_img.shape
        inside = (u>=0)&(u<=W-1)&(v>=0)&(v<=H-1)
        if inside.any():
            return float(np.mean(bilinear_sample(dist_img, u[inside], v[inside])))
        return 1e9

    best = None
    best_score = 1e18
    for s0, th0 in candidates:
        sc = quick_score(s0, th0)
        if sc < best_score:
            best_score = sc
            best = (s0, th0)

    s0, theta0 = best
    # initial translation using centroid under (s0, theta0)
    c = math.cos(theta0); sn = math.sin(theta0)
    R2 = np.array([[c, -sn],[sn, c]], dtype=np.float64)
    t0 = c_map - (s0 * (c_3d @ R2.T))
    tx_init, ty_init = float(t0[0]), float(t0[1])

    # Nonlinear refinement using distance transform
    x0 = np.array([math.log(max(s0, 1e-6)), theta0, tx_init, ty_init], dtype=np.float64)

    # bounds for stability
    lb = np.array([math.log(1e-3), -math.pi, -1e6, -1e6], dtype=np.float64)
    ub = np.array([math.log(1e3),  math.pi,  1e6,  1e6], dtype=np.float64)

    print("Initial guess:")
    print(f"  scale s0 = {math.exp(x0[0]):.6f}, yaw(deg) = {math.degrees(x0[1]):.3f}, tx={x0[2]:.3f}, ty={x0[3]:.3f}")
    print(f"  FM response = {resp:.6f}, quick_score = {best_score:.3f}")

    sol = least_squares(
        fun=residuals_sim2,
        x0=x0,
        bounds=(lb, ub),
        args=(pts_xy, dist_img, map_info),
        loss="soft_l1",
        f_scale=5.0,
        max_nfev=60,
        verbose=2
    )

    log_s, theta, tx, ty = sol.x
    s = math.exp(log_s)
    yaw_deg = math.degrees(theta)
    # Allow manual tweaking of the found similarity transform if requested.
    if args.manual:
        print("Entering manual adjustment mode. Keys: a/d rotate -/+; w/s scale +/-; ,/. tx -/+: [/ ] ty -/+: space/enter accept; q/ESC cancel; r reset")
        cur_s = float(s)
        cur_theta = float(theta)
        cur_tx = float(tx)
        cur_ty = float(ty)
        step_t = float(args.step_trans)
        step_rot = math.radians(float(args.step_rot_deg))
        step_scale = float(args.step_scale)

        title = "align_manual"
        H, W = map_info["H"], map_info["W"]
        while True:
            overlay = cv2.cvtColor(map_info["img"], cv2.COLOR_GRAY2BGR)
            overlay[map_info["edge"] > 0] = (120, 120, 120)

            M = min(len(pts_xy), 200000)
            sel = np.random.choice(len(pts_xy), M, replace=False)
            p = pts_xy[sel]
            c_m = math.cos(cur_theta); s_m = math.sin(cur_theta)
            Rm = np.array([[c_m, -s_m],[s_m, c_m]], dtype=np.float64)
            p_map = (cur_s * (p @ Rm.T)) + np.array([cur_tx, cur_ty], dtype=np.float64)

            for x, y in p_map:
                u, v = map_xy_to_map_pixel(float(x), float(y), map_info)
                ui = int(round(u)); vi = int(round(v))
                if 0 <= ui < W and 0 <= vi < H:
                    overlay[vi, ui] = (0, 0, 255)

            info = f"s={cur_s:.6f} yaw={math.degrees(cur_theta):.3f} tx={cur_tx:.3f} ty={cur_ty:.3f}"
            cv2.putText(overlay, info, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)
            cv2.putText(overlay, "Controls: a/d rot, w/s scale, ,/. tx, [/ ] ty, r reset, space/enter accept, q/ESC cancel", (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200,200,200), 1)

            cv2.imshow(title, overlay)
            k = cv2.waitKey(0) & 0xFF
            if k in (27, ord('q')):
                print("Manual mode cancelled; keeping automatic solution.")
                break
            if k == ord(' ') or k == 13:
                tx, ty, s, theta = cur_tx, cur_ty, cur_s, cur_theta
                print("Manual transform accepted.")
                break
            if k == ord('r'):
                cur_s = float(s); cur_theta = float(theta); cur_tx = float(tx); cur_ty = float(ty)
                continue
            if k == ord('a'):
                cur_theta -= step_rot
                continue
            if k == ord('d'):
                cur_theta += step_rot
                continue
            if k == ord('w'):
                cur_s *= (1.0 + step_scale)
                continue
            if k == ord('s'):
                cur_s *= max(1e-6, (1.0 - step_scale))
                continue
            # translation keys: ,/. and [/] as alternatives
            if k == ord(','):
                cur_tx -= step_t; continue
            if k == ord('.'):
                cur_tx += step_t; continue
            if k == ord('['):
                cur_ty -= step_t; continue
            if k == ord(']'):
                cur_ty += step_t; continue
        try:
            cv2.destroyWindow(title)
        except Exception:
            pass

    # finalize results (may have been updated by manual mode)
    yaw_deg = math.degrees(theta)

    result = {
        "npz": args.npz,
        "map_yaml": args.map_yaml,
        "floor_id": args.floor_id,
        "z_slice": [args.z_min, args.z_max],
        "scale_s": float(s),
        "yaw_rad": float(theta),
        "yaw_deg": float(yaw_deg),
        "tx_m": float(tx),
        "ty_m": float(ty),
        "rmse_pixels": float(np.sqrt(np.mean(sol.fun**2))),
        "num_points_opt": int(len(pts_xy)),
        "success": bool(sol.success),
        "message": sol.message
    }

    with open(os.path.join(args.out_dir, "result.json"), "w") as f:
        json.dump(result, f, indent=2)

    # Create overlay.png for sanity check
    # Draw map edge in gray, transformed 3D points in red
    overlay = cv2.cvtColor(map_info["img"], cv2.COLOR_GRAY2BGR)
    # draw edges slightly darker for visibility
    overlay[map_info["edge"] > 0] = (120, 120, 120)

    # transform a subset of points and draw
    M = min(len(pts_xy), 200000)
    sel = np.random.choice(len(pts_xy), M, replace=False)
    p = pts_xy[sel]
    c = math.cos(theta); sn = math.sin(theta)
    R = np.array([[c, -sn],[sn, c]], dtype=np.float64)
    p_map = (s * (p @ R.T)) + np.array([tx, ty], dtype=np.float64)

    H, W = map_info["H"], map_info["W"]
    for x, y in p_map:
        u, v = map_xy_to_map_pixel(float(x), float(y), map_info)
        ui = int(round(u))
        vi = int(round(v))
        if 0 <= ui < W and 0 <= vi < H:
            overlay[vi, ui] = (0, 0, 255)  # red

    cv2.imwrite(os.path.join(args.out_dir, "overlay.png"), overlay)

    # Produce aligned npz (transform full 3D points)
    # Apply floor alignment rotation + z shift already done in pts_rot; then apply sim2 on x,y and scale on z.
    pts_all = pts_rot.copy()
    # sim2 on xy
    pts_all_xy = pts_all[:, :2]
    pts_all_xy_map = (s * (pts_all_xy @ R.T)) + np.array([tx, ty], dtype=np.float64)
    pts_all[:, :2] = pts_all_xy_map
    # scale z too (common similarity assumption)
    pts_all[:, 2] *= s

    out_npz = os.path.join(args.out_dir, "aligned_sem_gs.npz")
    out_dict = dict(data)  # keep original fields
    # overwrite coords with aligned coords in map frame
    if "pts" in out_dict:
        out_dict["pts"] = pts_all.astype(np.float32)
    if "means3D" in out_dict:
        out_dict["means3D"] = pts_all.astype(np.float32)

    # also store transform metadata
    out_dict["align_scale_s"] = np.array([s], dtype=np.float32)
    out_dict["align_yaw_rad"] = np.array([theta], dtype=np.float32)
    out_dict["align_tx_ty"] = np.array([tx, ty], dtype=np.float32)

    np.savez_compressed(out_npz, **out_dict)

    # write DecisionMaker-compatible YAML (plane_fit + sim2 + metadata)
    # construct mu point on plane from normal and z0: mu = n * z0
    mu_pt = (n * float(z0))

    # build basis vectors on plane: e1 = projection of x-axis onto plane (fallback to y-axis)
    vx = np.array([1.0, 0.0, 0.0], dtype=np.float64)
    proj = vx - (vx.dot(n)) * n
    if np.linalg.norm(proj) < 1e-6:
        vx = np.array([0.0, 1.0, 0.0], dtype=np.float64)
        proj = vx - (vx.dot(n)) * n
    e1 = (proj / (np.linalg.norm(proj) + 1e-12))
    e2 = np.cross(n, e1)

    # angle of normal to map3d Z axis (degrees)
    dot_z = float(np.clip(np.dot(n, np.array([0.0, 0.0, 1.0], dtype=np.float64)), -1.0, 1.0))
    angle_deg = float(math.degrees(math.acos(dot_z)))

    # sim2 components
    s_val = float(s)
    yaw_rad = float(theta)
    yaw_deg = float(math.degrees(theta))
    R2x2 = [[float(math.cos(yaw_rad)), float(-math.sin(yaw_rad))],
            [float(math.sin(yaw_rad)), float(math.cos(yaw_rad))]]
    t_dict = {"x": float(tx), "y": float(ty)}

    # rmse in map meters (convert pixels->meters)
    try:
        rmse_pixels = float(np.sqrt(np.mean(sol.fun**2)))
    except Exception:
        rmse_pixels = float(result.get("rmse_pixels", 0.0))
    rmse_xy_m = rmse_pixels * float(map_info.get("resolution", 1.0))

    # inlier counts: threshold in pixels
    try:
        matched_cnt = int(len(sol.fun))
        inliers_cnt = int(np.sum(np.array(sol.fun) <= 5.0))
    except Exception:
        matched_cnt = int(len(pts_xy))
        inliers_cnt = int(matched_cnt)

    out_structure = {
        "extrinsic_mode_selected": "cam_to_base(inv)|basis(+,-)",
        "index_offset_cam_to_base2d": -7,
        "with_scale": True,
        "robust": True,
        "trim_keep": 0.8,
        "robust_iters": 3,
        "rmse_xy": float(rmse_xy_m),
        "plane_fit": {
            "angle_normal_to_map3d_Z_deg": float(angle_deg),
            "mu": {"x": float(mu_pt[0]), "y": float(mu_pt[1]), "z": float(mu_pt[2])},
            "normal_n": [float(v) for v in n.tolist()],
            "basis_e1": [float(v) for v in e1.tolist()],
            "basis_e2": [float(v) for v in e2.tolist()],
            "uv_definition": "u=dot(p-mu,e1), v=dot(p-mu,e2)"
        },
        "sim2": {
            "s": float(s_val),
            "R": R2x2,
            "t": t_dict,
            "theta_rad": float(yaw_rad),
            "theta_deg": float(yaw_deg)
        },
        "counts": {"matched": int(matched_cnt), "inliers": int(inliers_cnt)}
    }

    with open(os.path.join(args.out_dir, "align.yaml"), "w") as f:
        yaml.safe_dump(out_structure, f, default_flow_style=False)

    print("\nDone.")
    print(f"  result.json  : {os.path.join(args.out_dir, 'result.json')}")
    print(f"  overlay.png  : {os.path.join(args.out_dir, 'overlay.png')}")
    print(f"  aligned npz  : {out_npz}")
    print("\nFinal transform (3D -> map):")
    print(f"  scale s  = {s:.6f}")
    print(f"  yaw(deg) = {yaw_deg:.3f}")
    print(f"  t (m)    = ({tx:.3f}, {ty:.3f})")


if __name__ == "__main__":
    main()
