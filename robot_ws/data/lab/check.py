import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import yaml
import math
import json
import os

# ==========================================
# 1. FILE PATHS
# ==========================================
NPZ_PATH = 'panst3r_result_confge3_no_pts_local.npz'
JSON_PATH = 'panst3r_meta.json' 
YAML_PATH = 'kachaka_native.yaml'
PNG_PATH = 'kachaka_native.png' 

# ==========================================
# 2. DISPLAY SETTINGS (Try changing these if map looks wrong)
# ==========================================
FLIP_IMAGE_DISPLAY = True   # <--- Set True if map is upside down
ORIGIN_SETTING = 'upper' if FLIP_IMAGE_DISPLAY else 'lower'

# ==========================================
# 3. YOUR CALIBRATION (MUST MATCH ROS NODE)
# ==========================================
CALIB_TX = 3.90         # Translation X
CALIB_TY = 0.10         # Translation Y
CALIB_YAW = 0.611      # Rotation (Radians)
CALIB_SX = 1.0          # Scale X
CALIB_SY = 1.0         # Scale Y

def load_data():
    print(f"--- Loading Data ---")
    
    # 1. Load 3D Points & Semantics
    data = np.load(NPZ_PATH)
    
    if 'means3D' in data.files: points = data['means3D']
    elif 'pts' in data.files: points = data['pts']
    elif 'xyz' in data.files: points = data['xyz']
    else: raise ValueError(f"No points found in {NPZ_PATH}")

    if 'pan' in data.files: sem_ids = data['pan']
    elif 'semantic_ids' in data.files: sem_ids = data['semantic_ids']
    else: 
        print("Warning: No semantic IDs found.")
        sem_ids = None

    raw_x = points[:, 0]
    raw_y = points[:, 2] # Z is usually forward/up in these datasets
    
    print(f"Loaded {len(raw_x)} points.")

    # 2. Load 2D Map Metadata
    with open(YAML_PATH, 'r') as f:
        meta = yaml.safe_load(f)
    
    img = mpimg.imread(PNG_PATH)
    resolution = meta['resolution']
    origin = meta['origin'] 
    
    return raw_x, raw_y, sem_ids, img, resolution, origin

def get_object_centroids(raw_x, raw_y, sem_ids):
    if sem_ids is None or not os.path.exists(JSON_PATH): return []

    with open(JSON_PATH, 'r') as f:
        meta = json.load(f)
        
    if isinstance(meta, dict) and 'segments_info' in meta:
        segments = meta['segments_info']
    elif isinstance(meta, list):
        segments = meta
    else:
        return []

    objects_to_plot = []
    for seg in segments:
        seg_id = seg['id']
        name = seg['category_name']
        mask = (sem_ids == seg_id)
        if np.sum(mask) > 0:
            center_x = np.mean(raw_x[mask])
            center_y = np.mean(raw_y[mask])
            objects_to_plot.append({'name': name, 'x': center_x, 'y': center_y})
            
    return objects_to_plot

def apply_transform(x_in, y_in):
    x_scaled = x_in * CALIB_SX
    y_scaled = y_in * CALIB_SY

    c = math.cos(CALIB_YAW)
    s = math.sin(CALIB_YAW)
    
    x_rot = x_scaled * c - y_scaled * s
    y_rot = x_scaled * s + y_scaled * c

    x_final = x_rot + CALIB_TX
    y_final = y_rot + CALIB_TY
    
    return x_final, y_final

def main():
    raw_x, raw_y, sem_ids, map_img, res, origin = load_data()

    # Transform Data
    nav_x, nav_y = apply_transform(raw_x, raw_y)
    objects = get_object_centroids(raw_x, raw_y, sem_ids)
    for obj in objects:
        tx, ty = apply_transform(obj['x'], obj['y'])
        obj['tx'] = tx
        obj['ty'] = ty

    # --- PLOT ---
    fig, ax = plt.subplots(figsize=(12, 12))
    ax.set_title(f"Check (Flipped={FLIP_IMAGE_DISPLAY})\nTx={CALIB_TX}, Ty={CALIB_TY}")

    h, w = map_img.shape[:2]
    extent = [origin[0], origin[0] + w * res, origin[1], origin[1] + h * res]
    
    # Show Map with adjusted origin
    ax.imshow(map_img, extent=extent, cmap='gray', alpha=0.5, origin=ORIGIN_SETTING)

    # Show Points (Red)
    ax.scatter(nav_x[::10], nav_y[::10], s=1, c='red', alpha=0.3, label='3D Points')

    # Show Objects (Blue)
    for obj in objects:
        if obj['name'] in ['wall', 'floor', 'ceiling']: continue
        ax.scatter(obj['tx'], obj['ty'], c='blue', s=80, edgecolors='white', zorder=10)
        ax.text(obj['tx'] + 0.1, obj['ty'] + 0.1, obj['name'], 
                color='darkblue', fontsize=9, fontweight='bold', clip_on=True)

    ax.legend(loc='upper right')
    ax.grid(True, linestyle=':', alpha=0.6)
    
    # Set Limits
    ax.set_xlim(origin[0], origin[0] + w * res)
    ax.set_ylim(origin[1], origin[1] + h * res)

    print("Displaying plot...")
    plt.show()

if __name__ == "__main__":
    main()