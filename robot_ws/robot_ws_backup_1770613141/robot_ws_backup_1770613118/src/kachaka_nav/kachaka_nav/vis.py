import json
import os
import numpy as np
import matplotlib
# Force Headless Mode for Matplotlib to prevent GTK crashes
matplotlib.use('Agg') 
import matplotlib.pyplot as plt
import open3d as o3d 
from sklearn.decomposition import PCA
from scipy.spatial.transform import Rotation as R
from scipy.ndimage import binary_closing

# --- CONFIGURATION ---
FLIP_X = True            
DOWNSAMPLE = 1            # 1 = Max Detail
MAP_RESOLUTION = 0.01    # 2mm per pixel (High Def)
CONNECTIVITY_STRENGTH = 80 
# ---------------------

def level_the_floor(pts, ids, id_to_cat):
    """ Corrects floor inclination to be flat at Z=0. """
    print("   -> Detecting floor tilt...")
    floor_indices = [i for i, uid in enumerate(ids) if id_to_cat.get(uid, 'unknown') == 'floor']
    if not floor_indices: return pts
    floor_pts = pts[floor_indices]
    pca = PCA(n_components=3)
    pca.fit(floor_pts)
    normal_vector = pca.components_[2]
    target_vector = np.array([0, 0, 1])
    if np.dot(normal_vector, target_vector) < 0: normal_vector = -normal_vector
    rotation_axis = np.cross(normal_vector, target_vector)
    if np.linalg.norm(rotation_axis) < 1e-6: return pts
    rotation_axis = rotation_axis / np.linalg.norm(rotation_axis)
    angle = np.arccos(np.clip(np.dot(normal_vector, target_vector), -1.0, 1.0))
    r = R.from_rotvec(rotation_axis * angle)
    corrected_pts = np.dot(pts, r.as_matrix().T)
    new_floor_z = corrected_pts[floor_indices, 2]
    corrected_pts[:, 2] -= np.mean(new_floor_z)
    return corrected_pts

def generate_occupancy_grid(pts, ids, id_to_cat, resolution=0.05, connectivity=2):
    min_x, max_x = pts[:, 0].min(), pts[:, 0].max()
    min_y, max_y = pts[:, 1].min(), pts[:, 1].max()
    padding = 50 
    width = int(np.ceil((max_x - min_x) / resolution)) + (padding * 2)
    height = int(np.ceil((max_y - min_y) / resolution)) + (padding * 2)
    print(f"   -> Grid Dimensions: {width} x {height} pixels")
    grid_map = np.full((height, width), 127, dtype=np.uint8)
    grid_x = ((pts[:, 0] - min_x) / resolution).astype(int) + padding
    grid_y = ((pts[:, 1] - min_y) / resolution).astype(int) + padding
    grid_x = np.clip(grid_x, 0, width - 1)
    grid_y = np.clip(grid_y, 0, height - 1)
    
    floor_mask_pts = []
    obs_mask_pts = []
    for i, uid in enumerate(ids):
        cat = id_to_cat.get(uid, 'unknown')
        if cat == 'floor': floor_mask_pts.append(i)
        elif cat != 'ceiling': obs_mask_pts.append(i)

    raw_floor_grid = np.zeros((height, width), dtype=bool)
    if floor_mask_pts:
        raw_floor_grid[height - 1 - grid_y[floor_mask_pts], grid_x[floor_mask_pts]] = True

    print(f"   -> Connecting pathways (Strength: {connectivity})...")
    structure = np.ones((3, 3), dtype=bool)
    connected_floor_grid = binary_closing(raw_floor_grid, structure=structure, iterations=connectivity)

    grid_map[connected_floor_grid] = 254 
    if obs_mask_pts:
        grid_map[height - 1 - grid_y[obs_mask_pts], grid_x[obs_mask_pts]] = 0
    
    origin_x = min_x - (padding * resolution)
    origin_y = min_y - (padding * resolution)
    map_meta = {
        "W": width, "H": height,
        "x_min": float(min_x), "x_max": float(max_x),
        "z_min": float(min_y), "z_max": float(max_y),
        "resolution": resolution,
        "origin": [float(origin_x), float(origin_y), 0.0]
    }
    return grid_map, map_meta

def save_map_yaml(output_dir, image_filename, meta):
    yaml_path = os.path.join(output_dir, "map.yaml")
    yaml_content = f"""image: {image_filename}
mode: trinary
resolution: {meta['resolution']}
origin: [{meta['origin'][0]:.4f}, {meta['origin'][1]:.4f}, {meta['origin'][2]:.4f}]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
"""
    with open(yaml_path, "w") as f:
        f.write(yaml_content)
    print(f"   -> Saved ROS YAML: {yaml_path}")

def generate_semantic_colors(ids):
    """ Generates a unique color for each object ID """
    unique_ids = np.unique(ids)
    # Generate random colors for each unique ID
    # Seed ensures colors are consistent every run
    np.random.seed(42) 
    color_map = {uid: np.random.rand(3) for uid in unique_ids}
    
    # Map every point to its color
    semantic_colors = np.array([color_map[uid] for uid in ids])
    return semantic_colors

def run_pipeline(json_path, npz_path, output_dir="./output_maps/"):
    os.makedirs(output_dir, exist_ok=True)

    print("1. Loading Data...")
    with open(json_path, 'r') as f:
        meta = json.load(f)
    id_to_cat = {s['id']: s['category_name'] for s in meta['segments_info']}

    data = np.load(npz_path)
    pts = data['pts'][::DOWNSAMPLE]
    ids = data['pan'][::DOWNSAMPLE]
    rgb = data['rgb'][::DOWNSAMPLE]

    print("2. Fixing Geometry...")
    new_pts = np.zeros_like(pts)
    new_pts[:, 0] = pts[:, 0]
    new_pts[:, 1] = pts[:, 2] 
    new_pts[:, 2] = -pts[:, 1]
    pts = new_pts
    if FLIP_X: pts[:, 0] = -pts[:, 0]
    pts = level_the_floor(pts, ids, id_to_cat)
    floor_plane = pts[:, [0, 1]] 
    pca = PCA(n_components=2)
    aligned_floor = pca.fit_transform(floor_plane)
    pts[:, 0] = aligned_floor[:, 0]
    pts[:, 1] = aligned_floor[:, 1]

    # --- 2D MAP GENERATION ---
    print(f"3. Generating High-Res 2D Map...")
    grid_img, map_meta = generate_occupancy_grid(pts, ids, id_to_cat, resolution=MAP_RESOLUTION, connectivity=CONNECTIVITY_STRENGTH)
    
    image_filename = "map_occupancy.png"
    img_path = os.path.join(output_dir, image_filename)
    plt.imsave(img_path, grid_img, cmap='gray', vmin=0, vmax=255)
    print(f"   -> Saved Map Image: {img_path}")
    
    save_map_yaml(output_dir, image_filename, map_meta)
    
    json_out_path = os.path.join(output_dir, "map_meta.json")
    with open(json_out_path, "w") as f:
        json.dump(map_meta, f, indent=2)

    # --- 3D VISUALIZATION (OPEN3D) ---
    print("4. Launching 3D Viewers...")
    
    # Filter Ceiling for better view
    ceiling_ids = [k for k,v in id_to_cat.items() if v == 'ceiling']
    mask = ~np.isin(ids, ceiling_ids)
    
    vis_pts = pts[mask]
    vis_ids = ids[mask]
    vis_rgb = rgb[mask]

    # Ensure RGB is float 0-1
    if vis_rgb.max() > 1.1: vis_rgb = vis_rgb.astype(np.float64) / 255.0

    # --- VIEWER 1: REAL COLOR (RGB) ---
    print("   -> Showing RGB Map... (Close window to proceed)")
    pcd_rgb = o3d.geometry.PointCloud()
    pcd_rgb.points = o3d.utility.Vector3dVector(vis_pts)
    pcd_rgb.colors = o3d.utility.Vector3dVector(vis_rgb)
    
    axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0, origin=[0,0,0])
    o3d.visualization.draw_geometries([pcd_rgb, axes], window_name="1/2: RGB Map (Real Colors)")

    # --- VIEWER 2: SEMANTIC COLOR ---
    print("   -> Showing Semantic Map... (Close window to finish)")
    semantic_colors = generate_semantic_colors(vis_ids)
    
    pcd_sem = o3d.geometry.PointCloud()
    pcd_sem.points = o3d.utility.Vector3dVector(vis_pts)
    pcd_sem.colors = o3d.utility.Vector3dVector(semantic_colors)
    
    o3d.visualization.draw_geometries([pcd_sem, axes], window_name="2/2: Semantic Map (Categories)")
    print("Done.")

# --- RUN ---
json_file = "/home/acm118/robot_ws/data/lab/panst3r_meta.json"
npz_file = "/home/acm118/robot_ws/data/lab/panst3r_result_confge3_no_pts_local.npz"

run_pipeline(json_file, npz_file, output_dir="/home/acm118/robot_ws/data/output_maps")