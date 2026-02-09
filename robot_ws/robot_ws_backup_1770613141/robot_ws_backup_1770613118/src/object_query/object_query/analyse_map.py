import numpy as np
import json
import os
from collections import defaultdict

# ==========================================
# CONFIGURATION
# ==========================================
MAP_PATH = '/mnt/HDD1/phudh/home_robot/robot_ws/data/params.npz'
SEMANTIC_PATH = '/mnt/HDD1/phudh/home_robot/robot_ws/data/semantic.json'

def analyze_redundancy():
    print(f"📂 Loading Map: {MAP_PATH}")
    print(f"📂 Loading JSON: {SEMANTIC_PATH}")

    if not os.path.exists(MAP_PATH) or not os.path.exists(SEMANTIC_PATH):
        print("❌ Error: Files not found. Check your paths.")
        return

    # 1. Load Data
    data = np.load(MAP_PATH)
    semantic_ids = data['semantic_ids'] # The raw IDs for every point in the map
    
    with open(SEMANTIC_PATH, 'r') as f:
        sem_json = json.load(f)

    # 2. Map IDs to Names
    # Structure: { 63: 'vase', 65: 'vase', 20: 'chair' }
    id_to_name = {seg['id']: seg['class'] for seg in sem_json.get('segmentation', [])}

    # 3. Verify which IDs actually exist in the 3D Map
    # (Sometimes JSON has IDs that don't appear in the scan)
    present_ids = np.unique(semantic_ids)
    
    # 4. Group by Name
    # Structure: { 'vase': [63, 65, 91], 'chair': [20] }
    objects_found = defaultdict(list)
    
    total_points = len(semantic_ids)
    
    print("-" * 60)
    print(f"{'OBJECT CLASS':<20} | {'INSTANCES':<10} | {'IDS':<20} | {'POINTS'}")
    print("-" * 60)

    for uid in present_ids:
        # Ignore -1 or undefined IDs
        if uid not in id_to_name:
            continue
            
        name = id_to_name[uid]
        point_count = np.sum(semantic_ids == uid)
        
        # Store tuple: (ID, PointCount)
        objects_found[name].append((uid, point_count))

    # 5. Print Report
    redundant_count = 0
    
    for name, instances in objects_found.items():
        count = len(instances)
        ids_str = ", ".join([str(x[0]) for x in instances])
        total_pts = sum([x[1] for x in instances])
        
        # Mark redundant objects with a star *
        prefix = "🔴" if count > 1 else "  "
        if count > 1: redundant_count += 1
        
        print(f"{prefix} {name:<18} | {count:<10} | {ids_str:<20} | {total_pts}")

    print("-" * 60)
    print(f"📊 Analysis Complete.")
    print(f"   - Total Unique Classes: {len(objects_found)}")
    print(f"   - Redundant Classes:    {redundant_count}")

if __name__ == "__main__":
    analyze_redundancy()