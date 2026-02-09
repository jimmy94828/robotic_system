import yaml
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button
import matplotlib.image as mpimg
from matplotlib import transforms
from PIL import Image
import sys
import os
import math

def load_map_metadata(yaml_path):
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)
    map_dir = os.path.dirname(os.path.abspath(yaml_path))
    if os.path.isabs(data['image']):
        image_path = data['image']
    else:
        image_path = os.path.join(map_dir, data['image'])
        
    return {
        'path': image_path,
        'res': data['resolution'],
        'origin': data['origin'], # [x, y, yaw]
        'dir': map_dir
    }

def main():
    if len(sys.argv) < 3:
        print("Usage: python3 align_tool_v3.py <kachaka.yaml> <my_map.yaml>")
        return

    # 1. Load Maps
    meta_ref = load_map_metadata(sys.argv[1])
    meta_mov = load_map_metadata(sys.argv[2])

    img_ref = mpimg.imread(meta_ref['path'])
    # Load Movable Map as specific type for manipulation
    img_mov_original = mpimg.imread(meta_mov['path'])
    
    # State Container to track flips
    class MapState:
        img = img_mov_original.copy()
        flip_v = False
        flip_h = False

    state = MapState()

    # 2. Setup Plot
    fig, ax = plt.subplots(figsize=(11, 9))
    plt.subplots_adjust(left=0.1, bottom=0.35) 
    ax.set_facecolor('#202020')
    ax.set_title("Align Tool: Drag Sliders or Click Flip Buttons")

    # 3. Draw Reference (Kachaka - Grey)
    ref_tr = (transforms.Affine2D()
              .scale(meta_ref['res'])
              .rotate(meta_ref['origin'][2])
              .translate(meta_ref['origin'][0], meta_ref['origin'][1])
              + ax.transData)
    ax.imshow(img_ref, cmap='gray', origin='lower', transform=ref_tr, alpha=0.9)

    # 4. Draw User Map (Plasma)
    mov_layer = ax.imshow(state.img, cmap='plasma', origin='lower', alpha=0.6)

    # 5. Controls
    # Sliders
    ax_res = plt.axes([0.15, 0.20, 0.75, 0.03])
    s_res = Slider(ax_res, 'Scale', 0.001, 0.1, valinit=meta_mov['res'], valfmt='%0.5f')

    ax_x = plt.axes([0.15, 0.15, 0.75, 0.03])
    s_x = Slider(ax_x, 'X Pos', -20.0, 20.0, valinit=meta_mov['origin'][0])

    ax_y = plt.axes([0.15, 0.10, 0.75, 0.03])
    s_y = Slider(ax_y, 'Y Pos', -20.0, 20.0, valinit=meta_mov['origin'][1])

    ax_rot = plt.axes([0.15, 0.05, 0.75, 0.03])
    s_rot = Slider(ax_rot, 'Rotation', -math.pi, math.pi, valinit=meta_mov['origin'][2])

    # Buttons (Flip & Save)
    ax_flipv = plt.axes([0.05, 0.25, 0.1, 0.04])
    b_flipv = Button(ax_flipv, 'Flip Vertical')

    ax_fliph = plt.axes([0.16, 0.25, 0.1, 0.04])
    b_fliph = Button(ax_fliph, 'Flip Horizontal')

    ax_save = plt.axes([0.75, 0.25, 0.15, 0.04])
    b_save = Button(ax_save, 'Save New Map', color='green', hovercolor='lightgreen')

    # 6. Functions
    def update_transform(val):
        tr = (transforms.Affine2D()
              .scale(s_res.val)
              .rotate(s_rot.val)
              .translate(s_x.val, s_y.val)
              + ax.transData)
        mov_layer.set_transform(tr)
        fig.canvas.draw_idle()

    def do_flip_v(event):
        state.img = np.flipud(state.img) # Flip Up/Down
        state.flip_v = not state.flip_v
        mov_layer.set_data(state.img)
        fig.canvas.draw_idle()

    def do_flip_h(event):
        state.img = np.fliplr(state.img) # Flip Left/Right
        state.flip_h = not state.flip_h
        mov_layer.set_data(state.img)
        fig.canvas.draw_idle()

    def save_and_print(event):
        # 1. Save the Modified Image
        # We must save it because flipped maps require new image files in ROS
        new_img_name = "my_map_aligned.png"
        new_img_path = os.path.join(meta_mov['dir'], new_img_name)
        
        # Save using matplotlib (preserves flip)
        plt.imsave(new_img_path, state.img, cmap='gray', origin='lower')
        
        print("\n" + "="*60)
        print(f"SUCCESS! New map image saved to: {new_img_name}")
        print("="*60)
        print("   COPY THIS EXACTLY TO YOUR YAML FILE:")
        print("="*60)
        print(f"image: {new_img_name}")
        print(f"resolution: {s_res.val:.6f}")
        print(f"origin: [{s_x.val:.4f}, {s_y.val:.4f}, {s_rot.val:.4f}]")
        print("="*60)
        print("Note: I automatically updated the 'image' line to use the new flipped file.")

    # Link Controls
    s_res.on_changed(update_transform)
    s_x.on_changed(update_transform)
    s_y.on_changed(update_transform)
    s_rot.on_changed(update_transform)
    b_flipv.on_clicked(do_flip_v)
    b_fliph.on_clicked(do_flip_h)
    b_save.on_clicked(save_and_print)

    # Initial Draw
    update_transform(0)
    
    # Zoom out initially
    ax.set_xlim(-15, 15)
    ax.set_ylim(-15, 15)

    plt.show()

if __name__ == "__main__":
    main()