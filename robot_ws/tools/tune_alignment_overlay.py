#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, List, Optional, Sequence, Tuple

import cv2
import numpy as np
import yaml


@dataclass
class MapInfo:
    image: np.ndarray
    resolution: float
    origin: Tuple[float, float, float]
    width: int
    height: int


@dataclass
class AlignmentState:
    scale: float
    theta_rad: float
    tx: float
    ty: float

    @property
    def rot(self) -> np.ndarray:
        c = math.cos(self.theta_rad)
        s = math.sin(self.theta_rad)
        return np.array([[c, -s], [s, c]], dtype=np.float64)

    @property
    def theta_deg(self) -> float:
        return math.degrees(self.theta_rad)

    def copy(self) -> "AlignmentState":
        return AlignmentState(
            scale=self.scale,
            theta_rad=self.theta_rad,
            tx=self.tx,
            ty=self.ty,
        )


@dataclass(frozen=True)
class PlaneProjection:
    uv: np.ndarray
    height: np.ndarray
    colors_bgr: np.ndarray


@dataclass(frozen=True)
class InstancePoint:
    label: str
    instance_id: str
    uv: np.ndarray
    height: float


def load_map_yaml(map_yaml_path: Path) -> MapInfo:
    with map_yaml_path.open("r", encoding="utf-8") as file_obj:
        cfg = yaml.safe_load(file_obj)

    image_name = cfg.get("image")
    if not image_name:
        raise ValueError(f"Map YAML has no image field: {map_yaml_path}")

    image_path = Path(image_name)
    if not image_path.is_absolute():
        image_path = map_yaml_path.parent / image_path

    image = cv2.imread(str(image_path), cv2.IMREAD_UNCHANGED)
    if image is None:
        raise ValueError(f"Failed to load map image: {image_path}")
    if image.ndim == 2:
        image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    elif image.shape[2] == 4:
        image = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)

    height, width = image.shape[:2]
    origin = cfg.get("origin", [0.0, 0.0, 0.0])
    return MapInfo(
        image=image,
        resolution=float(cfg.get("resolution", 0.05)),
        origin=(float(origin[0]), float(origin[1]), float(origin[2])),
        width=width,
        height=height,
    )


def load_alignment_yaml(alignment_path: Path) -> Tuple[dict, np.ndarray, np.ndarray, np.ndarray, np.ndarray, AlignmentState]:
    with alignment_path.open("r", encoding="utf-8") as file_obj:
        data = yaml.safe_load(file_obj)

    plane = data["plane_fit"]
    sim2 = data["sim2"]
    mu = np.array([plane["mu"]["x"], plane["mu"]["y"], plane["mu"]["z"]], dtype=np.float64)
    e1 = np.array(plane["basis_e1"], dtype=np.float64)
    e2 = np.array(plane["basis_e2"], dtype=np.float64)
    normal = np.array(plane["normal_n"], dtype=np.float64)
    rot = np.array(sim2["R"], dtype=np.float64)
    theta = float(sim2.get("theta_rad", math.atan2(rot[1, 0], rot[0, 0])))
    state = AlignmentState(
        scale=float(sim2["s"]),
        theta_rad=theta,
        tx=float(sim2["t"]["x"]),
        ty=float(sim2["t"]["y"]),
    )
    return data, mu, e1, e2, normal, state


def load_point_cloud_npz(pointcloud_path: Path) -> Tuple[np.ndarray, np.ndarray]:
    data = np.load(pointcloud_path)

    points = None
    for key in ("points", "pts", "means3D", "xyz"):
        if key in data:
            points = np.asarray(data[key], dtype=np.float32)
            break
    if points is None or points.ndim != 2 or points.shape[1] < 3:
        raise ValueError(f"Could not find Nx3 point data in: {pointcloud_path}")

    colors = None
    for key in ("colors", "rgb", "color", "rgb_colors"):
        if key in data:
            raw = np.asarray(data[key])
            if raw.ndim == 2 and raw.shape[1] >= 3:
                colors = raw[:, :3]
                break

    if colors is None:
        colors = np.full((points.shape[0], 3), 255, dtype=np.uint8)
    elif colors.dtype != np.uint8:
        if float(np.nanmax(colors)) <= 1.0:
            colors = np.clip(colors * 255.0, 0.0, 255.0).astype(np.uint8)
        else:
            colors = np.clip(colors, 0.0, 255.0).astype(np.uint8)

    if colors.shape[0] != points.shape[0]:
        raise ValueError(
            f"Point/color count mismatch: {points.shape[0]} points vs {colors.shape[0]} colors"
        )

    return points[:, :3], colors[:, :3]


def pick_point_subset(
    points: np.ndarray,
    colors_rgb: np.ndarray,
    stride: int,
    max_points: int,
) -> Tuple[np.ndarray, np.ndarray]:
    if stride <= 0:
        raise ValueError("--stride must be greater than 0")
    points = points[::stride]
    colors_rgb = colors_rgb[::stride]

    if max_points > 0 and points.shape[0] > max_points:
        idx = np.linspace(0, points.shape[0] - 1, max_points, dtype=np.int64)
        points = points[idx]
        colors_rgb = colors_rgb[idx]

    return points, colors_rgb


def project_points_to_plane(
    points: np.ndarray,
    colors_rgb: np.ndarray,
    mu: np.ndarray,
    e1: np.ndarray,
    e2: np.ndarray,
    normal: np.ndarray,
) -> PlaneProjection:
    delta = points.astype(np.float64) - mu[None, :]
    uv = np.column_stack((delta @ e1, delta @ e2))
    height = delta @ normal
    colors_bgr = colors_rgb[:, ::-1].copy()
    return PlaneProjection(uv=uv, height=height, colors_bgr=colors_bgr)


def parse_bgr(text: str) -> Tuple[int, int, int]:
    values = [int(part.strip()) for part in text.split(",")]
    if len(values) != 3:
        raise argparse.ArgumentTypeError("Expected B,G,R, for example 0,165,255")
    if any(value < 0 or value > 255 for value in values):
        raise argparse.ArgumentTypeError("Color channels must be 0..255")
    return values[0], values[1], values[2]


def world_to_pixels(xy: np.ndarray, map_info: MapInfo) -> Tuple[np.ndarray, np.ndarray]:
    px = ((xy[:, 0] - map_info.origin[0]) / map_info.resolution).astype(np.int32)
    py_from_bottom = ((xy[:, 1] - map_info.origin[1]) / map_info.resolution).astype(np.int32)
    py = map_info.height - py_from_bottom
    return px, py


def transform_uv_to_map_xy(projection: PlaneProjection, state: AlignmentState) -> np.ndarray:
    return state.scale * (projection.uv @ state.rot.T) + np.array([state.tx, state.ty], dtype=np.float64)


def filter_by_height(
    projection: PlaneProjection,
    state: AlignmentState,
    min_z: Optional[float],
    max_z: Optional[float],
) -> np.ndarray:
    z = state.scale * projection.height
    mask = np.ones(z.shape[0], dtype=bool)
    if min_z is not None:
        mask &= z >= min_z
    if max_z is not None:
        mask &= z <= max_z
    return mask


def load_instances(instance_path: Optional[Path], mu: np.ndarray, e1: np.ndarray, e2: np.ndarray, normal: np.ndarray) -> List[InstancePoint]:
    if instance_path is None:
        return []
    if not instance_path.exists():
        raise FileNotFoundError(f"Instance JSON not found: {instance_path}")

    with instance_path.open("r", encoding="utf-8") as file_obj:
        data = json.load(file_obj)

    raw_instances = data.get("instances", data)
    records: Iterable
    if isinstance(raw_instances, dict):
        records = raw_instances.items()
    elif isinstance(raw_instances, list):
        records = enumerate(raw_instances)
    else:
        raise ValueError(f"Unsupported instance JSON format: {instance_path}")

    result: List[InstancePoint] = []
    for fallback_id, inst in records:
        if not isinstance(inst, dict):
            continue
        if bool(inst.get("filtered", False)):
            continue

        label = (
            inst.get("semantic_name")
            or inst.get("class_name")
            or inst.get("category_name")
            or inst.get("label")
        )
        centroid = inst.get("centroid") or inst.get("center")
        if centroid is None and "bbox_min" in inst and "bbox_max" in inst:
            bbox_min = np.asarray(inst["bbox_min"], dtype=np.float64)
            bbox_max = np.asarray(inst["bbox_max"], dtype=np.float64)
            if bbox_min.shape[0] >= 3 and bbox_max.shape[0] >= 3:
                centroid = ((bbox_min[:3] + bbox_max[:3]) * 0.5).tolist()

        if not label or not isinstance(centroid, list) or len(centroid) < 3:
            continue

        point = np.asarray(centroid[:3], dtype=np.float64)
        delta = point - mu
        uv = np.array([delta @ e1, delta @ e2], dtype=np.float64)
        height = float(delta @ normal)
        instance_id = str(inst.get("instance_id", inst.get("inst_id", fallback_id)))
        result.append(
            InstancePoint(
                label=str(label).strip().lower(),
                instance_id=instance_id,
                uv=uv,
                height=height,
            )
        )
    return result


def project_instance(instance: InstancePoint, state: AlignmentState) -> Tuple[float, float, float]:
    xy = state.scale * (state.rot @ instance.uv) + np.array([state.tx, state.ty], dtype=np.float64)
    z = state.scale * instance.height
    return float(xy[0]), float(xy[1]), float(z)


def draw_instances(
    image: np.ndarray,
    map_info: MapInfo,
    instances: Sequence[InstancePoint],
    state: AlignmentState,
    labels: Sequence[str],
    draw_all: bool,
) -> None:
    wanted = {label.strip().lower() for label in labels if label.strip()}
    for instance in instances:
        if not draw_all and instance.label not in wanted:
            continue
        x, y, _ = project_instance(instance, state)
        px = int((x - map_info.origin[0]) / map_info.resolution)
        py = map_info.height - int((y - map_info.origin[1]) / map_info.resolution)
        if 0 <= px < map_info.width and 0 <= py < map_info.height:
            cv2.circle(image, (px, py), 5, (0, 255, 255), -1)
            cv2.circle(image, (px, py), 7, (0, 0, 0), 1)
            text = f"{instance.label}:{instance.instance_id}"
            cv2.putText(image, text, (px + 8, py - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 0, 0), 2)
            cv2.putText(image, text, (px + 8, py - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 255, 255), 1)


def render_overlay(
    map_info: MapInfo,
    projection: PlaneProjection,
    state: AlignmentState,
    min_z: Optional[float],
    max_z: Optional[float],
    alpha: float,
    cloud_color_bgr: Tuple[int, int, int],
    use_point_colors: bool,
    point_radius: int,
    instances: Sequence[InstancePoint],
    instance_labels: Sequence[str],
    draw_all_instances: bool,
    status_lines: Sequence[str],
) -> np.ndarray:
    base = map_info.image.copy()
    overlay = base.copy()

    mask = filter_by_height(projection, state, min_z, max_z)
    visible_projection = PlaneProjection(
        uv=projection.uv[mask],
        height=projection.height[mask],
        colors_bgr=projection.colors_bgr[mask],
    )
    xy = transform_uv_to_map_xy(visible_projection, state)
    px, py = world_to_pixels(xy, map_info)
    in_bounds = (px >= 0) & (px < map_info.width) & (py >= 0) & (py < map_info.height)
    px = px[in_bounds]
    py = py[in_bounds]

    if use_point_colors:
        colors = visible_projection.colors_bgr[in_bounds]
        overlay[py, px] = colors
    else:
        overlay[py, px] = cloud_color_bgr

    if point_radius > 0:
        draw_color = cloud_color_bgr
        for x_pix, y_pix in zip(px[::4], py[::4]):
            cv2.circle(overlay, (int(x_pix), int(y_pix)), point_radius, draw_color, -1)

    frame = cv2.addWeighted(overlay, alpha, base, 1.0 - alpha, 0.0)
    draw_instances(frame, map_info, instances, state, instance_labels, draw_all_instances)

    y = 18
    for line in status_lines:
        cv2.putText(frame, line, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 0, 0), 2)
        cv2.putText(frame, line, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 255, 255), 1)
        y += 18

    return frame


def update_alignment_data(data: dict, state: AlignmentState) -> dict:
    updated = dict(data)
    updated["sim2"] = dict(updated["sim2"])
    updated["sim2"]["s"] = float(state.scale)
    updated["sim2"]["R"] = state.rot.tolist()
    updated["sim2"]["t"] = {
        "x": float(state.tx),
        "y": float(state.ty),
    }
    updated["sim2"]["theta_rad"] = float(state.theta_rad)
    updated["sim2"]["theta_deg"] = float(state.theta_deg)
    updated["manual_tuning"] = {
        "tool": "robot_ws/tools/tune_alignment_overlay.py",
        "note": "Only sim2 was adjusted; plane_fit was kept unchanged.",
    }
    return updated


def save_alignment(output_path: Path, data: dict, state: AlignmentState) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    updated = update_alignment_data(data, state)
    with output_path.open("w", encoding="utf-8") as file_obj:
        yaml.safe_dump(updated, file_obj, sort_keys=False, allow_unicode=False)


def print_controls() -> None:
    print(
        "\nControls:\n"
        "  W/S        move projected cloud +Y / -Y in map frame\n"
        "  A/D        move projected cloud -X / +X in map frame\n"
        "  Q/E        rotate counter-clockwise / clockwise\n"
        "  Z/X        scale down / up\n"
        "  [/]        translation step half / double\n"
        "  ;/'        rotation step half / double\n"
        "  -/=        scale step half / double\n"
        "  R          reset to input alignment\n"
        "  P          print current sim2\n"
        "  O          save overlay preview image\n"
        "  S/Enter    save tuned alignment yaml\n"
        "  H          show this help\n"
        "  Esc        quit\n"
    )


def format_state(state: AlignmentState, translation_step: float, rotation_step_deg: float, scale_step: float) -> List[str]:
    return [
        f"s={state.scale:.9f} theta={state.theta_deg:.4f} deg tx={state.tx:.4f} ty={state.ty:.4f}",
        f"steps: move={translation_step:.3f} m rot={rotation_step_deg:.3f} deg scale={scale_step * 100.0:.3f}%",
        "WASD move | Q/E rotate | Z/X scale | S save | O preview | H help | Esc quit",
    ]


def build_arg_parser() -> argparse.ArgumentParser:
    workspace_root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(
        description=(
            "Interactively fine-tune the existing 3D point cloud to Kachaka 2D map alignment. "
            "The tool keeps plane_fit fixed and only edits sim2."
        )
    )
    parser.add_argument(
        "--alignment",
        default=str(workspace_root / "data/Util/alignment.yaml"),
        help="Input alignment YAML containing plane_fit and sim2",
    )
    parser.add_argument(
        "--map-yaml",
        default=str(workspace_root / "data/lab/kachaka_native.yaml"),
        help="Kachaka 2D map YAML used by decision_maker_node",
    )
    parser.add_argument(
        "--pointcloud",
        default=str(workspace_root / "data/lab/demo/robot_deploy_slam_demo_rgb_color.npz"),
        help="Source 3D point cloud NPZ",
    )
    parser.add_argument(
        "--output",
        default=str(workspace_root / "data/Util/alignment_tuned.yaml"),
        help="Output YAML path for the tuned alignment",
    )
    parser.add_argument(
        "--preview-output",
        default=str(workspace_root / "data/Util/alignment_tuned_preview.png"),
        help="Output PNG path for overlay preview",
    )
    parser.add_argument(
        "--instance-path",
        default=None,
        help="Optional instance semantic JSON for drawing projected instance centroids",
    )
    parser.add_argument(
        "--instance-label",
        action="append",
        default=[],
        help="Draw projected centroids for this label. Can be repeated, e.g. --instance-label desk",
    )
    parser.add_argument(
        "--draw-all-instances",
        action="store_true",
        help="Draw all projected instance centroids from --instance-path",
    )
    parser.add_argument(
        "--stride",
        type=int,
        default=8,
        help="Use every Nth point before rendering",
    )
    parser.add_argument(
        "--max-points",
        type=int,
        default=250000,
        help="Maximum projected points used for interactive rendering. Use 0 for no cap",
    )
    parser.add_argument(
        "--min-z",
        type=float,
        default=None,
        help="Keep transformed points with z >= this value",
    )
    parser.add_argument(
        "--max-z",
        type=float,
        default=1.8,
        help="Keep transformed points with z <= this value",
    )
    parser.add_argument(
        "--alpha",
        type=float,
        default=0.72,
        help="Point cloud overlay alpha in 0..1",
    )
    parser.add_argument(
        "--cloud-color",
        type=parse_bgr,
        default=(0, 165, 255),
        help="Overlay point color as B,G,R when --use-point-colors is not set",
    )
    parser.add_argument(
        "--use-point-colors",
        action="store_true",
        help="Use colors from the point cloud NPZ instead of a single overlay color",
    )
    parser.add_argument(
        "--point-radius",
        type=int,
        default=0,
        help="Optional radius for sparse point drawing. 0 is fastest",
    )
    parser.add_argument(
        "--translation-step",
        type=float,
        default=0.05,
        help="Initial translation step in map meters",
    )
    parser.add_argument(
        "--rotation-step-deg",
        type=float,
        default=0.25,
        help="Initial rotation step in degrees",
    )
    parser.add_argument(
        "--scale-step",
        type=float,
        default=0.002,
        help="Initial multiplicative scale step. 0.002 means 0.2 percent",
    )
    parser.add_argument(
        "--save-preview-only",
        action="store_true",
        help="Render one overlay image and exit without opening an interactive window",
    )
    return parser


def main() -> None:
    args = build_arg_parser().parse_args()

    alignment_path = Path(args.alignment).expanduser()
    map_yaml_path = Path(args.map_yaml).expanduser()
    pointcloud_path = Path(args.pointcloud).expanduser()
    output_path = Path(args.output).expanduser()
    preview_output = Path(args.preview_output).expanduser()
    instance_path = Path(args.instance_path).expanduser() if args.instance_path else None

    if args.alpha < 0.0 or args.alpha > 1.0:
        raise SystemExit("--alpha must be in range 0..1")
    if args.point_radius < 0:
        raise SystemExit("--point-radius must be >= 0")

    map_info = load_map_yaml(map_yaml_path)
    alignment_data, mu, e1, e2, normal, original_state = load_alignment_yaml(alignment_path)
    state = original_state.copy()

    points, colors_rgb = load_point_cloud_npz(pointcloud_path)
    points, colors_rgb = pick_point_subset(points, colors_rgb, args.stride, args.max_points)
    projection = project_points_to_plane(points, colors_rgb, mu, e1, e2, normal)
    instances = load_instances(instance_path, mu, e1, e2, normal)

    print(f"Loaded map: {map_yaml_path} ({map_info.width}x{map_info.height}, res={map_info.resolution})")
    print(f"Loaded alignment: {alignment_path}")
    print(f"Loaded point cloud: {pointcloud_path} ({projection.uv.shape[0]} displayed points)")
    if instance_path:
        print(f"Loaded instances: {instance_path} ({len(instances)} centroids)")

    translation_step = float(args.translation_step)
    rotation_step = math.radians(float(args.rotation_step_deg))
    scale_step = float(args.scale_step)

    def make_frame() -> np.ndarray:
        return render_overlay(
            map_info=map_info,
            projection=projection,
            state=state,
            min_z=args.min_z,
            max_z=args.max_z,
            alpha=args.alpha,
            cloud_color_bgr=args.cloud_color,
            use_point_colors=args.use_point_colors,
            point_radius=args.point_radius,
            instances=instances,
            instance_labels=args.instance_label,
            draw_all_instances=args.draw_all_instances,
            status_lines=format_state(state, translation_step, math.degrees(rotation_step), scale_step),
        )

    if args.save_preview_only:
        preview_output.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(preview_output), make_frame())
        print(f"Saved preview: {preview_output}")
        return

    print_controls()
    window_name = "tune_alignment_overlay"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

    dirty = True
    frame = None
    while True:
        if dirty or frame is None:
            frame = make_frame()
            dirty = False
        cv2.imshow(window_name, frame)
        key = cv2.waitKey(30) & 0xFF
        if key == 255:
            continue

        if key == 27:
            break
        if key in (ord("h"), ord("H")):
            print_controls()
            continue
        if key in (ord("w"), ord("W")):
            state.ty += translation_step
        elif key in (ord("s"),):
            state.ty -= translation_step
        elif key in (ord("a"), ord("A")):
            state.tx -= translation_step
        elif key in (ord("d"), ord("D")):
            state.tx += translation_step
        elif key in (ord("q"), ord("Q")):
            state.theta_rad += rotation_step
        elif key in (ord("e"), ord("E")):
            state.theta_rad -= rotation_step
        elif key in (ord("z"), ord("Z")):
            state.scale *= max(0.000001, 1.0 - scale_step)
        elif key in (ord("x"), ord("X")):
            state.scale *= 1.0 + scale_step
        elif key == ord("["):
            translation_step *= 0.5
        elif key == ord("]"):
            translation_step *= 2.0
        elif key == ord(";"):
            rotation_step *= 0.5
        elif key == ord("'"):
            rotation_step *= 2.0
        elif key == ord("-"):
            scale_step *= 0.5
        elif key == ord("="):
            scale_step *= 2.0
        elif key in (ord("r"), ord("R")):
            state = original_state.copy()
        elif key in (ord("p"), ord("P")):
            print(
                f"s={state.scale:.12f}, theta_rad={state.theta_rad:.12f}, "
                f"theta_deg={state.theta_deg:.6f}, tx={state.tx:.6f}, ty={state.ty:.6f}"
            )
            continue
        elif key in (ord("o"), ord("O")):
            preview_output.parent.mkdir(parents=True, exist_ok=True)
            cv2.imwrite(str(preview_output), make_frame())
            print(f"Saved preview: {preview_output}")
            continue
        elif key in (ord("S"), 10, 13):
            save_alignment(output_path, alignment_data, state)
            print(f"Saved tuned alignment: {output_path}")
            continue
        else:
            continue
        dirty = True

    cv2.destroyWindow(window_name)


if __name__ == "__main__":
    main()
