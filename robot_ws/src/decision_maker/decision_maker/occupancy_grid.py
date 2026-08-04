"""Occupancy-grid helper for navigation-goal selection.

Loads the trinary occupancy map produced by ``tools/build_occupancy_map.py``
(0 = occupied, 205 = unknown, 254 = free, same grid/origin as the Kachaka
native map) and provides:

  - world <-> pixel conversion (map_server convention, origin = bottom-left)
  - ``is_free(x, y)`` / ``clearance_at(x, y)`` (metres to the nearest obstacle,
    from an exact Euclidean distance transform)
  - free-space navigation-goal search on rings around the projected target
    point, constrained to an obstacle-clearance band (semantic bboxes are
    deliberately not used: their extent/orientation proved unreliable)

Deliberately ROS-free and cv2-optional so it can be exercised outside the
robot container.
"""

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple

import numpy as np
import yaml

OCCUPIED_MAX_VALUE = 100   # pixel values <= this are obstacles
FREE_MIN_VALUE = 240       # pixel values >= this are free


def _load_gray_image(path: Path) -> np.ndarray:
    try:
        import cv2
        image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
        if image is None:
            raise ValueError(f'failed to load occupancy image: {path}')
        return image
    except ImportError:
        from PIL import Image
        return np.array(Image.open(path).convert('L'))


def _squared_edt_1d(f: np.ndarray) -> np.ndarray:
    """Felzenszwalb & Huttenlocher 1D squared distance transform."""
    n = f.size
    d = np.zeros(n)
    v = np.zeros(n, dtype=np.int64)
    z = np.zeros(n + 1)
    k = 0
    z[0] = -np.inf
    z[1] = np.inf
    for q in range(1, n):
        s = ((f[q] + q * q) - (f[v[k]] + v[k] * v[k])) / (2 * q - 2 * v[k])
        while s <= z[k]:
            k -= 1
            s = ((f[q] + q * q) - (f[v[k]] + v[k] * v[k])) / (2 * q - 2 * v[k])
        k += 1
        v[k] = q
        z[k] = s
        z[k + 1] = np.inf
    k = 0
    for q in range(n):
        while z[k + 1] < q:
            k += 1
        d[q] = (q - v[k]) ** 2 + f[v[k]]
    return d


def _distance_transform(occupied: np.ndarray) -> np.ndarray:
    """Exact Euclidean distance (in cells) to the nearest occupied cell."""
    height, width = occupied.shape
    large = float(height * height + width * width)
    f = np.where(occupied, 0.0, large)
    for x in range(width):
        f[:, x] = _squared_edt_1d(f[:, x])
    for y in range(height):
        f[y, :] = _squared_edt_1d(f[y, :])
    return np.sqrt(f)


class OccupancyGrid:
    """Trinary occupancy map with metric clearance queries."""

    def __init__(self, image: np.ndarray, resolution: float, origin):
        self.image = image
        self.resolution = float(resolution)
        self.origin = (float(origin[0]), float(origin[1]))
        self.height, self.width = image.shape
        self.free_mask = image >= FREE_MIN_VALUE
        self.occupied_mask = image <= OCCUPIED_MAX_VALUE
        self.clearance_m = _distance_transform(self.occupied_mask) * self.resolution

    @classmethod
    def from_yaml(cls, yaml_path) -> 'OccupancyGrid':
        yaml_path = Path(yaml_path)
        with open(yaml_path, 'r') as f:
            cfg = yaml.safe_load(f)
        image = _load_gray_image(yaml_path.parent / cfg['image'])
        origin = cfg.get('origin', [0.0, 0.0, 0.0])
        return cls(image, float(cfg['resolution']), origin)

    def world_to_pixel(self, x: float, y: float) -> Tuple[int, int]:
        px = int(math.floor((x - self.origin[0]) / self.resolution))
        py = self.height - 1 - int(math.floor((y - self.origin[1]) / self.resolution))
        return px, py

    def pixel_to_world(self, px: int, py: int) -> Tuple[float, float]:
        x = self.origin[0] + (px + 0.5) * self.resolution
        y = self.origin[1] + (self.height - 1 - py + 0.5) * self.resolution
        return x, y

    def contains(self, x: float, y: float) -> bool:
        px, py = self.world_to_pixel(x, y)
        return 0 <= px < self.width and 0 <= py < self.height

    def is_free(self, x: float, y: float) -> bool:
        px, py = self.world_to_pixel(x, y)
        if not (0 <= px < self.width and 0 <= py < self.height):
            return False
        return bool(self.free_mask[py, px])

    def is_occupied(self, x: float, y: float) -> bool:
        px, py = self.world_to_pixel(x, y)
        if not (0 <= px < self.width and 0 <= py < self.height):
            return False
        return bool(self.occupied_mask[py, px])

    def clearance_at(self, x: float, y: float) -> float:
        px, py = self.world_to_pixel(x, y)
        if not (0 <= px < self.width and 0 <= py < self.height):
            return 0.0
        return float(self.clearance_m[py, px])


@dataclass
class GoalSearchConfig:
    """Tunables for free-space goal selection (all distances in metres)."""
    min_clearance: float = 0.20        # goal-to-obstacle distance band, lower
    max_clearance: float = 0.40        # ... and upper bound
    max_standoff: float = 1.50         # farthest allowed goal from the target
    standoff_step: float = 0.05        # ring radius sampling step
    ring_angle_step_deg: float = 10.0
    ring_min_radius: float = 0.20
    free_gap_reject: float = 0.15      # free stretch behind the frontal obstacle
                                       # that marks it as NOT the target surface
    weight_clearance: float = 2.0
    weight_standoff: float = 0.8
    weight_robot_distance: float = 0.15


def _facing_ray_ok(grid: OccupancyGrid, goal_xy, target_xy,
                   free_gap_reject: float):
    """Check that the obstacle in front of the goal is the target's own surface.

    Walks from the goal to the projected target point (which usually lies
    inside the object's occupied cells).  Once the ray enters an obstacle it
    must not re-emerge into free space for ``free_gap_reject`` metres or more
    before reaching the target -- such a gap means the frontal obstacle is
    something else (e.g. a wall) with the object beyond it.

    Returns (ok, distance_to_first_hit_or_None).
    """
    dx = target_xy[0] - goal_xy[0]
    dy = target_xy[1] - goal_xy[1]
    length = math.hypot(dx, dy)
    if length < 1e-6:
        return True, None
    step = grid.resolution * 0.5
    steps = max(1, int(length / step))
    first_hit = None
    entered = False
    free_gap = 0.0
    for i in range(1, steps + 1):
        travelled = i * step
        x = goal_xy[0] + dx / length * travelled
        y = goal_xy[1] + dy / length * travelled
        if grid.is_occupied(x, y):
            if first_hit is None:
                first_hit = travelled
            entered = True
            free_gap = 0.0
        elif entered and grid.is_free(x, y):
            free_gap += step
            if free_gap >= free_gap_reject:
                return False, first_hit
        # unknown cells neither break nor extend the gap
    return True, first_hit


def select_goal_around_point(grid: OccupancyGrid, target_xy, robot_xy,
                             cfg: GoalSearchConfig) -> Optional[dict]:
    """Best free-space goal on rings around the projected target point.

    Candidates on rings (radius x angle) must lie in free space with an
    obstacle clearance inside ``[min_clearance, max_clearance]`` and must face
    the target surface directly (``_facing_ray_ok``).  Lowest score wins:
    clearance near the band centre, small ring radius (stay close to the
    object), and mild preference for the robot's side.  The returned heading
    responsibility stays with the caller (face the target point).
    """
    radii = np.arange(max(cfg.ring_min_radius, grid.resolution),
                      cfg.max_standoff + 1e-6, cfg.standoff_step)
    angles = np.deg2rad(np.arange(0.0, 360.0, cfg.ring_angle_step_deg))
    band_mid = 0.5 * (cfg.min_clearance + cfg.max_clearance)
    best = None
    checked = 0
    for angle in angles:
        direction = np.array([math.cos(angle), math.sin(angle)])
        for radius in radii:
            goal = np.asarray(target_xy, dtype=float) + radius * direction
            checked += 1
            if not grid.is_free(goal[0], goal[1]):
                continue
            clearance = grid.clearance_at(goal[0], goal[1])
            if not cfg.min_clearance <= clearance <= cfg.max_clearance:
                continue
            ok, first_hit = _facing_ray_ok(
                grid, goal, target_xy, cfg.free_gap_reject)
            if not ok:
                continue
            score = cfg.weight_clearance * abs(clearance - band_mid)
            score += cfg.weight_standoff * radius
            if robot_xy is not None:
                score += cfg.weight_robot_distance * math.hypot(
                    goal[0] - robot_xy[0], goal[1] - robot_xy[1])
            if best is None or score < best['score']:
                best = {
                    'x': float(goal[0]),
                    'y': float(goal[1]),
                    'boundary': None,
                    'clearance': clearance,
                    'standoff': float(radius),
                    'frontal_hit': first_hit,
                    'score': score,
                }
    if best is not None:
        best['candidates_checked'] = checked
    return best
