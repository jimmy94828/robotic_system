import argparse, json, os, re
import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt
import pandas as pd
import random

from matplotlib.image import imread
from matplotlib.colors import ListedColormap
import matplotlib.patches as mpatches

########################## helper ######################################

# 0.28209479177 is the constant for SH0
def sh_dc_to_rgb(dc):
    # dc: shape (N,3)
    C0 = 0.28209479177387814
    rgb_standard = dc * C0 + 0.5

    if rgb_standard.min() >= -0.5 and rgb_standard.max() <= 1.5:
        # print(f"[INFO] Using standard SH conversion")
        rgb = np.clip(rgb_standard, 0, 1)
    else:
        # print(f"[INFO] DC values out of standard range, using normalization")
        # Normalize each channel separately to maintain color ratio
        rgb = np.zeros_like(dc)
        for i in range(3):
            ch_min, ch_max = dc[:, i].min(), dc[:, i].max()
            if ch_max > ch_min:
                rgb[:, i] = (dc[:, i] - ch_min) / (ch_max - ch_min)
            else:
                rgb[:, i] = 0.5
    return rgb.astype(np.float32)

from plyfile import PlyData
def load_ply_pointcloud(ply_path, max_points=None, voxel_size=None):
    ply = PlyData.read(ply_path)
    v = ply['vertex']
    pts = np.vstack([v['x'], v['y'], v['z']]).T.astype(np.float32)
    names = v.data.dtype.names
    if ('f_dc_0' in names) and ('f_dc_1' in names) and ('f_dc_2' in names):
        dc = np.vstack([v['f_dc_0'], v['f_dc_1'], v['f_dc_2']]).T.astype(np.float32)
        colors = sh_dc_to_rgb(dc)
    else:
        colors = np.ones((pts.shape[0], 3), dtype=np.float32) * 0.5

    if max_points is not None and pts.shape[0] > max_points:
        idx = np.random.choice(pts.shape[0], max_points, replace=False)
        pts = pts[idx]; colors = colors[idx]

    if voxel_size is not None:
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts)
        pcd.colors = o3d.utility.Vector3dVector(colors)
        pcd = pcd.voxel_down_sample(voxel_size=voxel_size)
        pts = np.asarray(pcd.points).astype(np.float32)
        colors = np.asarray(pcd.colors).astype(np.float32)

    return pts, colors

########################## Load Data ##########################

# Load 2D Map & Meta data
def load_2dmap(map_path, meta_path):
    img = imread(map_path)
    with open(meta_path, "r") as f:
        meta = json.load(f)

    # create coordinate function (easy to transform)
    W = meta["W"]; H = meta["H"]
    x_min, x_max = meta["x_min"], meta["x_max"]
    z_min, z_max = meta["z_min"], meta["z_max"]

    meta["to_world"] = lambda uu, vv: (
        x_min + (1.0 - vv / (H - 1)) * (x_max - x_min),     # x
        z_min + (uu / (W - 1))       * (z_max - z_min)      # z
    )
    meta["to_pixel"] = lambda xx, zz: (
        int(round((zz - z_min) / (z_max - z_min) * (W - 1))),           # u
        int(round((1.0 - (xx - x_min) / (x_max - x_min)) * (H - 1)))    # v
    )
    return img, meta

# Point transform: uv to xz coord
def uv_to_xz(u, v, meta):
    '''transform uv to xz'''
    to_world = meta['to_world']
    x, z = to_world(int(u), int(v))
    return float(x), float(z)

# Point transform: xz to uv coord
def xz_to_uv(x, z, meta):
    '''transform xz to uv'''
    to_pixel = meta['to_pixel']
    u, v = to_pixel(x, z)
    return u, v

def to_uint8(img):
    img = np.asarray(img)
    if img.dtype != np.uint8:
        img = np.clip(img, 0, 1)
        img = (img * 255).astype(np.uint8)
    return img

######################## Get Location ########################

def _bresenham_line(u0,v0,u1,v1):
    u0, v0, u1, v1 = int(u0), int(v0), int(u1), int(v1)
    du = abs(u1-u0); dv = -abs(v1-v0)
    su = 1 if u0 < u1 else -1
    sv = 1 if v0 < v1 else -1
    err = du + dv
    u, v = u0, v0
    pts = []
    while True:
        pts.append((u,v))
        if u == u1 and v == v1: break
        e2 = 2*err
        if e2 >= dv:
            err += dv; u += su
        if e2 <= du:
            err += du; v += sv
    return pts

def hits_wall(wall_mask, u0, v0, u1, v1):
    H, W = wall_mask.shape
    line_pts = _bresenham_line(u0, v0, u1, v1)

    for (u, v) in line_pts:
        if not (0 <= u < W and 0 <= v < H):
            return True
        if wall_mask[v, u] == 1:
            return True
    return False

def build_binary_map(
    map_2d,
    meta,
    robot_radius_m: float = 0.02,
    floor_rgb=(255, 255, 255),
    color_tol=12
):
    # --- to uint8 ---
    img = to_uint8(map_2d)
    H, W = img.shape[:2]

    # --- build free_raw / obstacle_raw ---
    target = np.array(floor_rgb, dtype=np.float32).reshape(1,1,3)
    diff = img.astype(np.float32) - target
    dist2 = np.sum(diff * diff, axis=2)
    free_raw = (dist2 <= color_tol**2).astype(np.uint8)
    obstacle_raw = (1 - free_raw).astype(np.uint8)

    # --- get scale (m->pixel) ---
    z_span = meta["z_max"] - meta["z_min"]
    x_span = meta["x_max"] - meta["x_min"]
    s_u = z_span / max(1, (W - 1))
    s_v = x_span / max(1, (H - 1))
    s = 0.5 * (s_u + s_v)   # scale
    eps_m = 0.05
    r_px = float((robot_radius_m + eps_m) / max(s, 1e-9))

    # --- larger the obstacle ---
    obs = obstacle_raw.astype(np.uint8)
    # Simple dilation if cv2 is not available (omitted for pure python speed, or add back if needed)
    
    free_safe = (1 - obs).astype(np.uint8)

    return {
        "free_raw": free_raw,
        "obstacle_raw": obstacle_raw,
        "free_safe": free_safe,
        "r_px": r_px,
        "scale_m_per_px": s
    }

######################## RRT ########################
def cal_dist(u0, v0, u1, v1):
    return ((u0-u1)**2 + (v0-v1)**2)**0.5

def collision(free_safe, u0, v0, u1, v1):
    H, W = free_safe.shape
    line_pts = _bresenham_line(u0, v0, u1, v1)

    for (u, v) in line_pts:
        if not (0 <= u < W and 0 <= v < H):
            return True
        if free_safe[v, u] == 0:
            return True
    return False

def random_sample_node(free_safe):
    H, W = free_safe.shape
    while True:
        v = random.randint(0, H-1)
        u = random.randint(0, W-1)
        if free_safe[v, u] == 1:
            return (u,v)

def get_nearest_node(tree, x_rand):
    nearest_dist = None
    nearest_node = None
    nearest_idx = 0
    for i, node in enumerate(tree):
        u_rand, v_rand = float(x_rand[0]), float(x_rand[1])
        u_tree, v_tree = float(node._get_uv()[0]), float(node._get_uv()[1])
        dist = cal_dist(u_rand, v_rand, u_tree, v_tree)
        if nearest_dist == None:
            nearest_dist = dist
            nearest_node = node
            nearest_idx = i
        elif nearest_dist > dist:
            nearest_dist = dist
            nearest_node = node
            nearest_idx = i
    return nearest_node, nearest_idx

def sample_new_node(x_near, x_rand, step_size=10):
    u_near, v_near = float(x_near[0]), float(x_near[1])
    u_rand, v_rand = float(x_rand[0]), float(x_rand[1])
    dist = cal_dist(u_rand, v_rand, u_near, v_near)
    if dist < 1e-9:
        u_new, v_new = u_rand, v_rand
    else:
        scale = min(1.0, float(step_size) / dist)
        u_new = u_near + (u_rand-u_near) * scale
        v_new = v_near + (v_rand-v_near) * scale
    return (int(round(u_new)), int(round(v_new)))

def is_arrive_target(free_safe, target_point, node_point, threshold=8, target_is_center=False):
    dist = cal_dist(target_point[0], target_point[1], node_point[0], node_point[1])
    
    if not target_is_center:
        coll = collision(free_safe, target_point[0], target_point[1], node_point[0], node_point[1])
        if (dist <= threshold) and (not coll):
            return True
        else:
            return False
    else:
        if (dist <= threshold):
            return True
        else:
            return False

class Node:
    def __init__(self, pos):
        self.u = pos[0]
        self.v = pos[1]
        self.parent = None
        self.idx = 0
        self.cost = 0
        
    def _get_uv(self):
        return (self.u, self.v)
    def _set_parent(self, parent_node):
        self.parent = parent_node
    def _get_parent(self):
        return self.parent

def RRT_Core(start_point, target_point, free_safe, meta, step_size=10, 
        arrive_threshold=8, target_is_center=False, bias=0.05):
    # init tree (create init node)
    start_node = Node(start_point)
    tree = [start_node]
    idx = 0

    max_iter = 5000
    for i in range(max_iter):
        
        # (1). Random Sampling
        x_rand = 0
        if random.random() < (1-bias):
            x_rand = random_sample_node(free_safe)
        else:
            x_rand = target_point
            
        # (2). Find Nearest Node
        x_near, idx_near = get_nearest_node(tree, x_rand)
        
        # (3). Sample New Node
        x_new = sample_new_node(x_near=x_near._get_uv(),
                                x_rand=x_rand,
                                step_size=step_size)
        
        # (4). Collision Check
        if collision(free_safe, x_near._get_uv()[0], x_near._get_uv()[1], x_new[0], x_new[1]):
            continue
        
        # (5). Create Node and Update Tree
        idx += 1
        new_node = Node(x_new)
        new_node._set_parent(x_near)
        new_node.idx = idx
        new_node.cost = cal_dist(x_near._get_uv()[0], x_near._get_uv()[1], x_new[0], x_new[1]) + x_near.cost
        
        tree.append(new_node)
        
        # (6). Check is arrive
        if is_arrive_target(free_safe, target_point, x_new, threshold=arrive_threshold, target_is_center=target_is_center):
            print("Target Reached!")
            break

    # get path
    planning_path = []
    node = tree[-1]
    while node is not None:
        planning_path.append(node)
        node = node._get_parent()
    
    real_world_x_z = []
    for p in planning_path[-1::-1]:
        real_world_x_z.append(uv_to_xz(p._get_uv()[0], p._get_uv()[1], meta))
    
    return real_world_x_z

######################## Main Function ########################

def rrt_planning(start:tuple, target:tuple, map_path, meta_path, points_path):
    """
    Main entry point for ROS node.
    Args:
        start: (x, z) tuple in world coordinates
        target: (x, z) tuple in world coordinates
        map_path: str, path to .png map
        meta_path: str, path to .json meta
        points_path: str, path to .ply file
    """
    random.seed(0)

    # 1. Load Data
    pts, colors = load_ply_pointcloud(points_path)
    # print(f'PTS SHAPE: {pts.shape}')

    # 2. Load Map
    map_2d, meta = load_2dmap(map_path, meta_path)

    # 3. Convert Start/Target to UV (Pixels)
    start_point = xz_to_uv(start[0], start[1], meta)
    target_uv = xz_to_uv(target[0], target[1], meta)
    
    # print(f'Start UV: {start_point}')
    # print(f'Target UV: {target_uv}')

    # 4. Build Collision Map
    robot_radius_m = 0.2  # Slightly larger for safety
    occ = build_binary_map(map_2d, meta, robot_radius_m=robot_radius_m, floor_rgb=(255,255,255))
    
    # Check if start/target are valid
    H, W = occ["free_safe"].shape
    if not (0 <= start_point[0] < W and 0 <= start_point[1] < H):
        raise ValueError("Start point is out of map bounds")
    if not (0 <= target_uv[0] < W and 0 <= target_uv[1] < H):
        raise ValueError("Target point is out of map bounds")
        
    # 5. RRT Planning
    # print('Start RRT planning...')
    step_size = 30 # px
    arrive_threshold = 30 # px
    
    path_realworld_x_z = RRT_Core(
        start_point=start_point, 
        target_point=target_uv, 
        free_safe=occ["free_safe"], 
        meta=meta,
        step_size=step_size, 
        arrive_threshold=arrive_threshold, 
        target_is_center=True
    )

    return path_realworld_x_z