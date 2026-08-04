#!/usr/bin/env python3
"""Build a 2D occupancy map by projecting the aligned 3D point cloud onto the
Kachaka 2D map grid.

The Kachaka native map is NOT a complete occupancy map:
  - white  (253, 253, 253) -> traversable (lidar-confirmed free)
  - beige  (244, 232, 219) -> unknown / unexplored
  - brown  (191, 170, 155) -> obstacle (wall)

This tool transforms the 3D point cloud into the Kachaka map frame using the
calibrated plane_fit + Sim(2) from alignment.yaml, removes the floor (and
everything above the robot's height band), rasterizes the remaining obstacle
points onto the same grid as the Kachaka map, and merges both sources into a
standard trinary occupancy map (0 = occupied, 205 = unknown, 254 = free).

Floor points are kept as free-space evidence: a cell densely observed as floor
with no obstacle points above it is marked free even where the Kachaka map is
still unexplored.

Outputs (next to --output-prefix):
  <prefix>.png          trinary occupancy image (same grid as the Kachaka map)
  <prefix>.yaml         map_server-style metadata (same origin/resolution)
  <prefix>_preview.png  color overlay for visual verification (rotated 90 deg
                        clockwise to match the usual MapVisualizer view)

Only numpy / PIL / yaml are required (no cv2), so it runs outside the
container.
"""

import argparse
import os
from pathlib import Path

import numpy as np
import yaml
from PIL import Image

# Kachaka native map palette (RGB).
KACHAKA_FREE_RGB = (253, 253, 253)
KACHAKA_UNKNOWN_RGB = (244, 232, 219)
KACHAKA_OCCUPIED_RGB = (191, 170, 155)

# Standard map_server trinary values.
OCCUPIED = 0
UNKNOWN = 205
FREE = 254


def load_alignment(alignment_path: Path):
    alignment_path = Path(alignment_path)
    with open(alignment_path, 'r') as f:
        data = yaml.safe_load(f)

    mu = np.array(
        [
            data['plane_fit']['mu']['x'],
            data['plane_fit']['mu']['y'],
            data['plane_fit']['mu']['z'],
        ],
        dtype=float,
    )
    e1 = np.array(data['plane_fit']['basis_e1'], dtype=float)
    e2 = np.array(data['plane_fit']['basis_e2'], dtype=float)
    normal_n = np.array(data['plane_fit']['normal_n'], dtype=float)
    s = float(data['sim2']['s'])
    rot = np.array(data['sim2']['R'], dtype=float)
    trans = np.array([data['sim2']['t']['x'], data['sim2']['t']['y']], dtype=float)
    return mu, e1, e2, normal_n, s, rot, trans


def load_map_info(map_yaml_path: Path):
    map_yaml_path = Path(map_yaml_path)
    with open(map_yaml_path, 'r') as f:
        cfg = yaml.safe_load(f)

    image_path = map_yaml_path.parent / cfg['image']
    image = np.array(Image.open(image_path).convert('RGB'))
    origin = cfg.get('origin', [0.0, 0.0, 0.0])
    resolution = float(cfg['resolution'])
    return image, resolution, (float(origin[0]), float(origin[1]), float(origin[2]))


def classify_kachaka_pixels(image: np.ndarray):
    """Map each pixel to the nearest of the three known Kachaka colors."""
    palette = np.array(
        [KACHAKA_FREE_RGB, KACHAKA_UNKNOWN_RGB, KACHAKA_OCCUPIED_RGB],
        dtype=np.int32,
    )
    flat = image.reshape(-1, 3).astype(np.int32)
    distances = np.linalg.norm(flat[:, None, :] - palette[None, :, :], axis=2)
    nearest = np.argmin(distances, axis=1).reshape(image.shape[:2])
    return nearest == 0, nearest == 1, nearest == 2  # free, unknown, occupied


def transform_points_to_map(points: np.ndarray, mu, e1, e2, normal_n, s, rot, trans):
    """Same formula as object_query_server.transform_points_to_map."""
    delta = points.astype(np.float64) - mu[None, :]
    uv = np.column_stack((delta @ e1, delta @ e2))
    height = delta @ normal_n
    xy = s * (uv @ rot.T) + trans[None, :]
    z = s * height
    return xy, z


def _low_envelope(xy: np.ndarray, z: np.ndarray, cell: float, min_points: int = 30):
    """Per-cell low-percentile z on a coarse grid -> candidate floor samples."""
    gx = np.floor(xy[:, 0] / cell).astype(np.int64)
    gy = np.floor(xy[:, 1] / cell).astype(np.int64)
    key = (gx - gx.min()) * (gy.max() - gy.min() + 1) + (gy - gy.min())

    order = np.lexsort((z, key))
    key_sorted = key[order]
    z_sorted = z[order]
    _, start, counts = np.unique(key_sorted, return_index=True, return_counts=True)

    keep = counts >= min_points
    start, counts = start[keep], counts[keep]
    # 2nd percentile within each cell (skip the very lowest outlier points)
    env_idx = start + np.maximum(1, (counts * 0.02).astype(np.int64))
    env_z = z_sorted[env_idx]
    env_xy = xy[order][env_idx]
    return env_xy, env_z


def _fit_plane(env_xy: np.ndarray, env_z: np.ndarray, iters: int = 3):
    """Robust least-squares plane z = a*x + b*y + c with residual trimming."""
    mask = np.ones(env_z.shape[0], dtype=bool)
    coeffs = np.array([0.0, 0.0, float(np.median(env_z))])
    for _ in range(iters):
        design = np.column_stack(
            (env_xy[mask, 0], env_xy[mask, 1], np.ones(int(mask.sum()))))
        coeffs, *_ = np.linalg.lstsq(design, env_z[mask], rcond=None)
        residual = env_z - (env_xy @ coeffs[:2] + coeffs[2])
        sigma = max(0.02, 1.4826 * float(np.median(np.abs(residual - np.median(residual)))))
        mask = np.abs(residual) < 2.5 * sigma
    return coeffs, mask


def estimate_floor_heights(xy: np.ndarray, z: np.ndarray, cell: float = 0.25):
    """Height of every point above the locally fitted floor plane.

    The alignment's plane_fit leaves a residual tilt, so the floor is not at a
    constant z across the room.  A robust plane is fitted to the per-cell low
    envelope instead, and heights are measured relative to it.  Both height
    orientations are tried; the real floor is flat, so the orientation whose
    envelope fits a plane with the smaller residual wins (point counts are
    misleading here: walls put mass at every height).
    Returns (heights, sign, coeffs, diagnostics).
    """
    best = None
    for sign in (1.0, -1.0):
        zs = sign * z
        env_xy, env_z = _low_envelope(xy, zs, cell)
        if env_z.size < 10:
            continue
        coeffs, inlier_mask = _fit_plane(env_xy, env_z)
        residual = env_z - (env_xy @ coeffs[:2] + coeffs[2])
        fit_rms = float(np.sqrt(np.mean(residual[inlier_mask] ** 2)))
        if best is None or fit_rms < best[0]:
            heights = zs - (xy @ coeffs[:2] + coeffs[2])
            diag = {
                'envelope_cells': int(env_z.size),
                'envelope_inlier_cells': int(inlier_mask.sum()),
                'fit_rms_m': fit_rms,
                'tilt_m_per_m': float(np.hypot(coeffs[0], coeffs[1])),
            }
            best = (fit_rms, heights, sign, coeffs, diag)

    if best is None:
        raise RuntimeError('floor plane estimation failed: too few envelope cells')
    _, heights, sign, coeffs, diag = best
    if diag['tilt_m_per_m'] > 0.05:
        print(f"WARNING: fitted floor tilt {diag['tilt_m_per_m'] * 100:.1f} cm/m is "
              'unusually large; check the alignment quality.')
    return heights, sign, coeffs, diag


def rasterize_counts(xy: np.ndarray, resolution: float, origin, width: int, height: int):
    """Count points per cell on the map_server grid (origin = bottom-left)."""
    px = np.floor((xy[:, 0] - origin[0]) / resolution).astype(np.int64)
    py_from_bottom = np.floor((xy[:, 1] - origin[1]) / resolution).astype(np.int64)
    py = height - 1 - py_from_bottom

    inside = (px >= 0) & (px < width) & (py >= 0) & (py < height)
    counts = np.zeros((height, width), dtype=np.int32)
    np.add.at(counts, (py[inside], px[inside]), 1)
    return counts, int(np.count_nonzero(~inside))


def wall_agreement(kachaka_occupied: np.ndarray, obstacle_cells: np.ndarray, radius_px: int = 2):
    """Fraction of Kachaka wall pixels with a point-cloud obstacle nearby.

    Sanity check that the alignment actually places the cloud on the map.
    """
    if not np.any(kachaka_occupied):
        return float('nan')
    dilated = obstacle_cells.copy()
    for _ in range(radius_px):
        shifted = dilated.copy()
        shifted[1:, :] |= dilated[:-1, :]
        shifted[:-1, :] |= dilated[1:, :]
        shifted[:, 1:] |= dilated[:, :-1]
        shifted[:, :-1] |= dilated[:, 1:]
        dilated = shifted
    hits = np.count_nonzero(kachaka_occupied & dilated)
    return hits / np.count_nonzero(kachaka_occupied)


def build_preview(kachaka_image: np.ndarray, occupancy: np.ndarray,
                  pcd_obstacle: np.ndarray, pcd_free_only: np.ndarray,
                  scale: int = 4):
    """Overlay: faded Kachaka map + red pcd obstacles + green pcd-only free."""
    preview = (kachaka_image.astype(np.float32) * 0.55 + 255.0 * 0.45).astype(np.uint8)
    preview[pcd_free_only] = (170, 235, 170)
    preview[occupancy == OCCUPIED] = (60, 60, 60)
    preview[pcd_obstacle] = (220, 40, 40)

    preview = np.rot90(preview, k=-1)  # match MapVisualizer's clockwise view
    preview = np.repeat(np.repeat(preview, scale, axis=0), scale, axis=1)
    return preview


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    default_root = Path(__file__).resolve().parents[1]
    parser.add_argument('--alignment', type=Path,
                        default=default_root / 'data/Util/alignment.yaml')
    parser.add_argument('--pointcloud', type=Path,
                        default=default_root / 'data/lab/demo/robot_deploy_robot_run_rgb_color.npz')       # TODO change to new map
    parser.add_argument('--map-yaml', type=Path,
                        default=default_root / 'data/lab/kachaka_native.yaml')
    parser.add_argument('--output-prefix', type=Path,
                        default=default_root / 'data/lab/demo/occupancy_from_pcd')
    parser.add_argument('--floor-clearance', type=float, default=0.05,
                        help='height above the detected floor treated as floor [m]')
    parser.add_argument('--max-height', type=float, default=1.5,
                        help='obstacle band upper bound above the floor [m]')
    parser.add_argument('--min-obstacle-points', type=int, default=5,
                        help='points in the obstacle band needed to mark a cell occupied')
    parser.add_argument('--min-floor-points', type=int, default=5,
                        help='floor points needed to mark an unexplored cell free')
    parser.add_argument('--no-free-from-floor', action='store_true',
                        help='do not use floor points as free-space evidence')
    parser.add_argument('--preview-scale', type=int, default=4)
    return parser.parse_args()


def main():
    args = parse_args()

    mu, e1, e2, normal_n, s, rot, trans = load_alignment(args.alignment)
    kachaka_image, resolution, origin = load_map_info(args.map_yaml)
    height, width = kachaka_image.shape[:2]
    kachaka_free, _, kachaka_occupied = classify_kachaka_pixels(kachaka_image)

    points = np.load(args.pointcloud)['points']
    finite = np.all(np.isfinite(points), axis=1)
    points = points[finite]
    xy, z = transform_points_to_map(points, mu, e1, e2, normal_n, s, rot, trans)
    heights, z_sign, floor_coeffs, floor_diag = estimate_floor_heights(xy, z)

    floor_mask = (heights >= -0.30) & (heights < args.floor_clearance)
    obstacle_mask = (heights >= args.floor_clearance) & (heights <= args.max_height)

    obstacle_counts, obstacle_oob = rasterize_counts(
        xy[obstacle_mask], resolution, origin, width, height)
    floor_counts, _ = rasterize_counts(
        xy[floor_mask], resolution, origin, width, height)

    pcd_obstacle = obstacle_counts >= args.min_obstacle_points
    pcd_floor = floor_counts >= args.min_floor_points

    occupancy = np.full((height, width), UNKNOWN, dtype=np.uint8)
    free_cells = kachaka_free.copy()
    if not args.no_free_from_floor:
        free_cells |= pcd_floor
    occupancy[free_cells] = FREE
    occupancy[kachaka_occupied | pcd_obstacle] = OCCUPIED  # occupied wins

    prefix = args.output_prefix
    os.makedirs(prefix.parent, exist_ok=True)
    png_path = prefix.with_suffix('.png')
    yaml_path = prefix.with_suffix('.yaml')
    preview_path = prefix.parent / (prefix.name + '_preview.png')

    Image.fromarray(occupancy, mode='L').save(png_path)
    with open(yaml_path, 'w') as f:
        yaml.safe_dump(
            {
                'image': png_path.name,
                'resolution': resolution,
                'origin': list(origin),
                'negate': 0,
                'occupied_thresh': 0.65,
                'free_thresh': 0.196,
                'source': {
                    'kind': 'pointcloud_occupancy',
                    'alignment': str(args.alignment),
                    'pointcloud': str(args.pointcloud),
                    'kachaka_map': str(args.map_yaml),
                    'height_axis_sign': float(z_sign),
                    'floor_plane_z_eq_ax_by_c': [float(v) for v in floor_coeffs],
                    'floor_fit': floor_diag,
                    'floor_clearance_m': args.floor_clearance,
                    'max_height_m': args.max_height,
                },
            },
            f,
            sort_keys=False,
        )

    pcd_free_only = pcd_floor & ~kachaka_free & ~(kachaka_occupied | pcd_obstacle)
    preview = build_preview(
        kachaka_image, occupancy, pcd_obstacle, pcd_free_only, args.preview_scale)
    Image.fromarray(preview).save(preview_path)

    agreement = wall_agreement(kachaka_occupied, pcd_obstacle)
    total = occupancy.size
    print(f'points total/finite      : {finite.size} / {points.shape[0]}')
    print(f'height axis sign         : {z_sign:+.0f}')
    print(f'floor plane              : z = {floor_coeffs[0]:.4f}x + {floor_coeffs[1]:.4f}y '
          f'+ {floor_coeffs[2]:.3f}  (tilt {floor_diag["tilt_m_per_m"] * 100:.1f} cm/m, '
          f'fit rms {floor_diag["fit_rms_m"] * 100:.1f} cm)')
    print(f'floor / obstacle points  : {int(floor_mask.sum())} / {int(obstacle_mask.sum())}')
    print(f'obstacle pts outside map : {obstacle_oob}')
    print(f'grid {width}x{height} @ {resolution:.4f} m/px, origin={origin}')
    print(f'cells occupied           : {int((occupancy == OCCUPIED).sum())} '
          f'({100.0 * (occupancy == OCCUPIED).sum() / total:.1f}%)  '
          f'[kachaka walls {int(kachaka_occupied.sum())}, pcd {int(pcd_obstacle.sum())}]')
    print(f'cells free               : {int((occupancy == FREE).sum())} '
          f'({100.0 * (occupancy == FREE).sum() / total:.1f}%)  '
          f'[kachaka {int(kachaka_free.sum())}, pcd-only {int(pcd_free_only.sum())}]')
    print(f'cells unknown            : {int((occupancy == UNKNOWN).sum())} '
          f'({100.0 * (occupancy == UNKNOWN).sum() / total:.1f}%)')
    print(f'kachaka walls matched by pcd (<=2px): {100.0 * agreement:.1f}%')
    print(f'wrote: {png_path}')
    print(f'wrote: {yaml_path}')
    print(f'wrote: {preview_path}')


if __name__ == '__main__':
    main()
