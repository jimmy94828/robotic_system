import numpy as np
from plyfile import PlyData, PlyElement

def convert_slam_npz_to_ply(npz_path, ply_path):
    print(f"Loading {npz_path}...")
    data = np.load(npz_path)
    
    # --- 1. Extract Data ---
    xyz = data['means3D']
    opacities = data['logit_opacities']
    scales = data['log_scales']
    rots = data['unnorm_rotations']
    rgb = data['rgb_colors']

    count = xyz.shape[0]
    print(f"Found {count} points.")
    print(f"Scales shape: {scales.shape}")
    print(f"RGB shape: {rgb.shape}")

    # --- 2. Handle Scales (The Fix) ---
    # Check if scales are (N, 1) or (N, 3)
    if scales.ndim == 2 and scales.shape[1] == 1:
        print("Detected Isotropic scaling (N, 1). Broadcasting to 3 dimensions...")
        scale_0 = scales[:, 0]
        scale_1 = scales[:, 0]
        scale_2 = scales[:, 0]
    elif scales.ndim == 1:
        # Handle case where it is just a flat 1D array
        print("Detected flat scaling array. Broadcasting...")
        scale_0 = scales
        scale_1 = scales
        scale_2 = scales
    else:
        # Standard Anisotropic (N, 3)
        scale_0 = scales[:, 0]
        scale_1 = scales[:, 1]
        scale_2 = scales[:, 2]

    # --- 3. Handle Colors ---
    # Convert RGB to Spherical Harmonics 0 (DC)
    # SH_0 = (RGB - 0.5) / 0.282
    SH_C0 = 0.28209479177387814
    if rgb.shape[1] == 3:
        f_dc = (rgb - 0.5) / SH_C0
        f_dc_0 = f_dc[:, 0]
        f_dc_1 = f_dc[:, 1]
        f_dc_2 = f_dc[:, 2]
    else:
        # Fallback for grayscale or weird shapes
        print("Warning: RGB not (N,3). Filling with defaults.")
        f_dc_0 = np.zeros(count)
        f_dc_1 = np.zeros(count)
        f_dc_2 = np.zeros(count)

    # --- 4. Create Structured Array ---
    dtype_full = [
        ('x', 'f4'), ('y', 'f4'), ('z', 'f4'),
        ('nx', 'f4'), ('ny', 'f4'), ('nz', 'f4'),
        ('f_dc_0', 'f4'), ('f_dc_1', 'f4'), ('f_dc_2', 'f4'),
        ('opacity', 'f4'),
        ('scale_0', 'f4'), ('scale_1', 'f4'), ('scale_2', 'f4'),
        ('rot_0', 'f4'), ('rot_1', 'f4'), ('rot_2', 'f4'), ('rot_3', 'f4')
    ]
    
    # Add dummy rest harmonics
    for i in range(45):
        dtype_full.append((f'f_rest_{i}', 'f4'))

    elements = np.empty(count, dtype=dtype_full)

    # --- 5. Fill Data ---
    elements['x'] = xyz[:, 0]
    elements['y'] = xyz[:, 1]
    elements['z'] = xyz[:, 2]
    elements['nx'] = 0
    elements['ny'] = 0
    elements['nz'] = 0

    elements['f_dc_0'] = f_dc_0
    elements['f_dc_1'] = f_dc_1
    elements['f_dc_2'] = f_dc_2

    # Fill rest harmonics with zeros
    for i in range(45):
        elements[f'f_rest_{i}'] = 0

    elements['opacity'] = opacities.flatten()
    
    # Use the prepared scale variables
    elements['scale_0'] = scale_0
    elements['scale_1'] = scale_1
    elements['scale_2'] = scale_2
    
    # Rotations
    elements['rot_0'] = rots[:, 0]
    elements['rot_1'] = rots[:, 1]
    elements['rot_2'] = rots[:, 2]
    elements['rot_3'] = rots[:, 3]

    # --- 6. Save ---
    el = PlyElement.describe(elements, 'vertex')
    PlyData([el]).write(ply_path)
    print(f"Success! Saved to {ply_path}")

if __name__ == "__main__":
    # REPLACE THIS
    input_filename = "/mnt/HDD1/phudh/home_robot/robot_ws/data/params.npz" 
    output_filename = "/mnt/HDD1/phudh/home_robot/robot_ws/data/params.ply"
    
    try:
        convert_slam_npz_to_ply(input_filename, output_filename)
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()