import yaml
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from matplotlib import transforms
import os
import sys
import numpy as np

def load_map_metadata(yaml_path):
    if not os.path.exists(yaml_path):
        print(f"Error: File not found {yaml_path}")
        sys.exit(1)
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)
    
    map_dir = os.path.dirname(os.path.abspath(yaml_path))
    # Handle absolute vs relative image paths
    if os.path.isabs(data['image']):
        image_path = data['image']
    else:
        image_path = os.path.join(map_dir, data['image'])

    if not os.path.exists(image_path):
        print(f"Error: Image {image_path} not found.")
        sys.exit(1)

    return {
        'path': image_path,
        'res': data['resolution'],
        'origin': data['origin'] # [x, y, yaw]
    }

def plot_map(ax, meta, cmap, alpha, label):
    # 1. Load Image
    img = mpimg.imread(meta['path'])
    h, w = img.shape[:2]
    
    # 2. Extract ROS Parameters
    res = meta['res']
    origin_x = meta['origin'][0]
    origin_y = meta['origin'][1]
    yaw = meta['origin'][2]

    # 3. Create the Transform
    # ROS Logic: The image is scaled, then rotated, then placed at (x,y).
    # The 'origin' in YAML refers to the BOTTOM-LEFT corner of the image.
    
    # transform_data: Maps pixels (0..W, 0..H) to Physical Meters
    tr = (transforms.Affine2D()
          .scale(res)              # 1. Scale pixels to meters
          .rotate(yaw)             # 2. Rotate around (0,0)
          .translate(origin_x, origin_y) # 3. Move to global position
          + ax.transData)          # 4. Add to plot's data coordinate system

    # 4. Draw the Image
    # origin='lower' puts pixel [0,0] at the bottom-left, which matches ROS grids
    im = ax.imshow(img, cmap=cmap, alpha=alpha, origin='lower', transform=tr)
    
    # 5. Manually calculate bounds to update the plot view
    # (imshow with transform doesn't auto-update plot limits correctly)
    corners = np.array([
        [0, 0], [w, 0], [w, h], [0, h]
    ])
    # Apply the same logic to corners to find limits
    # x_new = x*res*cos(yaw) - y*res*sin(yaw) + origin_x
    c, s = np.cos(yaw), np.sin(yaw)
    
    # Transform all 4 corners manually
    x_corn = (corners[:,0] * res * c) - (corners[:,1] * res * s) + origin_x
    y_corn = (corners[:,0] * res * s) + (corners[:,1] * res * c) + origin_y
    
    ax.plot(origin_x, origin_y, 'x', markersize=10, label=f"{label} Origin")
    
    return min(x_corn), max(x_corn), min(y_corn), max(y_corn)

def main():
    if len(sys.argv) < 3:
        print("Usage: python3 verify_map_v2.py <kachaka.yaml> <my_map.yaml>")
        return

    # Setup Plot
    fig, ax = plt.subplots(figsize=(10, 10))
    ax.set_facecolor('#202020')
    ax.set_title("True ROS Alignment Verification (With Rotation)", color='white')
    
    # Plot Map 1
    print(f"Plotting Map 1: {sys.argv[1]}")
    m1 = load_map_metadata(sys.argv[1])
    x1a, x1b, y1a, y1b = plot_map(ax, m1, 'gray', 1.0, "Kachaka")

    # Plot Map 2
    print(f"Plotting Map 2: {sys.argv[2]}")
    m2 = load_map_metadata(sys.argv[2])
    x2a, x2b, y2a, y2b = plot_map(ax, m2, 'plasma', 0.6, "User")

    # Set View Limits (Zoom to fit both)
    ax.set_xlim(min(x1a, x2a)-1, max(x1b, x2b)+1)
    ax.set_ylim(min(y1a, y2a)-1, max(y1b, y2b)+1)
    
    ax.grid(True, linestyle='--', alpha=0.3)
    ax.legend()
    plt.show()

if __name__ == "__main__":
    main()