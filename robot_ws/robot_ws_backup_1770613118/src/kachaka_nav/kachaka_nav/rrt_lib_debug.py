import numpy as np
import random
import traceback
from matplotlib.image import imread

######################## Helper: Coordinate Transforms ########################

def create_meta_from_ros(resolution, origin, width, height):
    """
    Creates the metadata dictionary directly from ROS values.
    """
    # ROS Origin is usually bottom-left (x_min, y_min)
    x_min = origin[0]
    z_min = origin[1] # We map ROS y -> RRT z
    
    x_max = x_min + (width * resolution)
    z_max = z_min + (height * resolution)

    meta = {
        "W": width, "H": height,
        "x_min": x_min, "x_max": x_max,
        "z_min": z_min, "z_max": z_max
    }

    # Define coordinate transform lambdas
    meta["to_world"] = lambda uu, vv: (
        x_min + (1.0 - vv / (height - 1)) * (x_max - x_min),    # x
        z_min + (uu / (width - 1))        * (z_max - z_min)     # z (ros y)
    )
    meta["to_pixel"] = lambda xx, zz: (
        int(round((zz - z_min) / (z_max - z_min) * (width - 1))),          # u
        int(round((1.0 - (xx - x_min) / (x_max - x_min)) * (height - 1)))  # v
    )
    return meta

def uv_to_xz(u, v, meta):
    x, z = meta['to_world'](int(u), int(v))
    return float(x), float(z)

def xz_to_uv(x, z, meta):
    return meta['to_pixel'](x, z)

######################## Map Processing ########################
def to_uint8(img):
    img = np.asarray(img)
    if img.dtype != np.uint8:
        img = np.clip(img, 0, 1)
        img = (img * 255).astype(np.uint8)
    return img

def build_binary_map(map_2d, meta, robot_radius_m=0.2, color_tol=12):
    img = to_uint8(map_2d)
    H, W = img.shape[:2]

    # Convert white/grey map to binary obstacle map
    # Assuming floor is white-ish (255,255,255)
    target = np.array((255, 255, 255), dtype=np.float32).reshape(1,1,3)
    diff = img.astype(np.float32) - target
    dist2 = np.sum(diff * diff, axis=2)
    
    # 1 = Free, 0 = Obstacle (Standard) -> We need the reverse for internal logic often
    free_raw = (dist2 <= color_tol**2).astype(np.uint8)
    obstacle_raw = (1 - free_raw).astype(np.uint8)

    return {"free_safe": free_raw} # Simplified for speed

######################## RRT Core Logic ########################
# (Kept mostly same, but cleaned up)

class Node:
    def __init__(self, pos):
        self.u = pos[0]; self.v = pos[1]
        self.parent = None
        
def _bresenham_line(u0,v0,u1,v1):
    # Standard line algorithm
    u0,v0,u1,v1 = int(u0), int(v0), int(u1), int(v1)
    du = abs(u1-u0); dv = -abs(v1-v0)
    su = 1 if u0<u1 else -1; sv = 1 if v0<v1 else -1
    err = du+dv
    u,v = u0,v0
    pts = []
    while True:
        pts.append((u,v))
        if u==u1 and v==v1: break
        e2=2*err
        if e2>=dv: err+=dv; u+=su
        if e2<=du: err+=du; v+=sv
    return pts

def collision(free_safe, u0, v0, u1, v1):
    H, W = free_safe.shape
    for (u, v) in _bresenham_line(u0, v0, u1, v1):
        if not (0 <= u < W and 0 <= v < H): return True
        if free_safe[v, u] == 0: return True # 0 means obstacle here
    return False

def get_nearest_node(tree, x_rand):
    # Linear search (slow for massive trees, fine for Nav)
    # Optimized to use min() with key
    return min(tree, key=lambda n: (n.u - x_rand[0])**2 + (n.v - x_rand[1])**2)

def RRT_Core(start_uv, target_uv, free_safe, meta, step_size=15, max_iter=3000):
    start_node = Node(start_uv)
    tree = [start_node]
    H, W = free_safe.shape

    for _ in range(max_iter):
        # print(tree)
        # 1. Sample
        if random.random() < 0.1: # 10% Goal Bias
            rand_uv = target_uv
        else:
            rand_uv = (random.randint(0, W-1), random.randint(0, H-1))
        
        # 2. Nearest
        nearest = get_nearest_node(tree, rand_uv)
        
        # 3. Steer (Move towards sample)
        dist = ((rand_uv[0]-nearest.u)**2 + (rand_uv[1]-nearest.v)**2)**0.5
        if dist < 1e-9: continue

        scale = min(1.0, float(step_size) / dist)
        new_u = nearest.u + (rand_uv[0]-nearest.u) * scale
        new_v = nearest.v + (rand_uv[1]-nearest.v) * scale
        # round to nearest pixel (avoid truncation bias)
        new_uv = (int(round(new_u)), int(round(new_v)))

        # ensure the new_uv is actually different from nearest; if not, step one pixel towards the sample
        if new_uv[0] == nearest.u and new_uv[1] == nearest.v:
            dx = rand_uv[0] - nearest.u
            dy = rand_uv[1] - nearest.v
            if dx == 0 and dy == 0:
                continue
            su = 1 if dx > 0 else -1
            sv = 1 if dy > 0 else -1
            new_u_px = nearest.u + su if 0 <= nearest.u + su < W else nearest.u
            new_v_px = nearest.v + sv if 0 <= nearest.v + sv < H else nearest.v
            # if still same, skip
            if new_u_px == nearest.u and new_v_px == nearest.v:
                continue
            new_uv = (int(new_u_px), int(new_v_px))

        # 4. Check Collision
        if collision(free_safe, nearest.u, nearest.v, new_uv[0], new_uv[1]):
            # collision, skip
            continue

        # 5. Add Node
        new_node = Node(new_uv)
        new_node.parent = nearest
        tree.append(new_node)

        # 6. Check Arrival
        dist_to_goal = ((new_uv[0]-target_uv[0])**2 + (new_uv[1]-target_uv[1])**2)**0.5
        if dist_to_goal < step_size:
            # Check final segment to goal
            if not collision(free_safe, new_uv[0], new_uv[1], target_uv[0], target_uv[1]):
                goal_node = Node(target_uv)
                goal_node.parent = new_node
                tree.append(goal_node)
                # Reconstruct Path
                path = []
                curr = goal_node
                while curr:
                    path.append(uv_to_xz(curr.u, curr.v, meta))
                    curr = curr.parent
                return path[::-1] # Return start -> end

    return [] # Failed

######################## Main Function ########################

def rrt_planning(start, target, map_path, resolution, origin):
    """
    Optimized Entry Point.
    start/target: (x, y) floats
    map_path: path to png
    resolution: float (m/px)
    origin: [x, y, z]
    """
    random.seed(0)

    # 1. Load Image
    img = imread(map_path) # Returns normalized float or uint8
    H, W = img.shape[:2]
    
    # 2. Create Meta directly
    meta = create_meta_from_ros(resolution, origin, W, H)

    # 3. Convert Coords
    try:
        start_uv = xz_to_uv(start[0], start[1], meta)
        target_uv = xz_to_uv(target[0], target[1], meta)
    except Exception as e:
        print(f"[RRT] Error: failed to convert start/target to map pixels: {e}")
        traceback.print_exc()
        return None

    # 4. Build Map
    occ = build_binary_map(img, meta)
    
    # Bounds Check
    if not (0 <= start_uv[0] < W and 0 <= start_uv[1] < H): 
        print("start out")
        return []
    if not (0 <= target_uv[0] < W and 0 <= target_uv[1] < H): 
        print("target out")
        return []

    # 5. Run
    path = RRT_Core(start_uv, target_uv, occ["free_safe"], meta)
    if not path:
        print("path error!!!")
    
    return path

