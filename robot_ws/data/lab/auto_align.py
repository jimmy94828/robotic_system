import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import yaml

# --- CONFIG ---
#NPZ_PATH = '/home/acm118/robot_ws/data/Util/Final_SEM_GS_converted.npz'
NPZ_PATH = '/home/acm118/robot_ws/data/Util/Final_GS_converted.npz'
YAML_PATH = 'kachaka_native.yaml'
PNG_PATH = 'kachaka_native.png' 

# --- DISPLAY CONFIG ---
FLIP_IMAGE_DISPLAY = True   

class ManualAligner:
    def __init__(self):
        # === 1. UNBIND DEFAULT MATPLOTLIB KEYS ===
        # 'q' defaults to Quit, 'h' defaults to Home (Reset View)
        # We must remove these so your custom controls work.
        for key in ['q', 'h']:
            if key in plt.rcParams['keymap.quit']:
                plt.rcParams['keymap.quit'].remove(key)
            if key in plt.rcParams['keymap.home']:
                plt.rcParams['keymap.home'].remove(key)

        self.tx = 0.0   # Translation X
        self.ty = 0.0   # Translation Y
        self.rot = 0.0  # Rotation (Degrees)
        self.sx = 1.0   # Scale X 
        self.sy = 1.0   # Scale Y 
        
        self.colors = None 
        
        self.load_data()
        self.init_plot()

    def load_data(self):
        data = np.load(NPZ_PATH)
        
        # Load Points
        if 'means3D' in data.files: self.pts = data['means3D']
        elif 'pts' in data.files: self.pts = data['pts']
        elif 'xyz' in data.files: self.pts = data['xyz']
        else: raise ValueError(f"No points found in NPZ.")
            
        print(f"Loaded {len(self.pts)} points.")

        # Load Colors
        color_keys = ['rgb', 'colors', 'color', 'rgb_colors']
        for key in color_keys:
            if key in data.files:
                print(f"Found color data using key: '{key}'")
                raw_colors = data[key]
                if raw_colors.shape[1] == 3:
                    if raw_colors.max() > 1.1:
                        self.colors = raw_colors.astype(np.float32) / 255.0
                    else:
                        self.colors = raw_colors
                    break
        
        if self.colors is None: print("No color data found. Defaulting to Red.")

        # Project 2D
        self.pts_x_raw = self.pts[:, 0]
        self.pts_y_raw = self.pts[:, 2] 

        # Load Map
        with open(YAML_PATH, 'r') as f: meta = yaml.safe_load(f)
        self.img = mpimg.imread(PNG_PATH)
        self.res = meta['resolution']
        self.origin = meta['origin'] 

    def init_plot(self):
        self.fig, self.ax = plt.subplots(figsize=(12, 12))
        self.update_plot()
        
        print("\n=== CONTROLS ===")
        print("Arrow Keys   : Move Translation")
        print("Q / E        : Rotate Left / Right")
        print("H            : Flip Horizontally")
        print("V            : Flip Vertically")
        print("Shift+Arrows : Move Faster")
        print("================\n")
        
        self.fig.canvas.mpl_connect('key_press_event', self.on_key)
        plt.show()

    def update_plot(self):
        self.ax.clear()
        flip_text = f"FlipX: {self.sx}, FlipY: {self.sy}"
        self.ax.set_title(f"Align Map (Flipped={FLIP_IMAGE_DISPLAY})\nTx: {self.tx:.2f}, Ty: {self.ty:.2f}, Rot: {self.rot:.1f}°\n{flip_text}")
        
        h, w = self.img.shape[:2]
        extent = [self.origin[0], self.origin[0] + w * self.res,
                  self.origin[1], self.origin[1] + h * self.res]
        
        origin_setting = 'upper' if FLIP_IMAGE_DISPLAY else 'lower'
        self.ax.imshow(self.img, extent=extent, cmap='gray', alpha=0.5, origin=origin_setting)
        
        # Transform
        x_flipped = self.pts_x_raw * self.sx
        y_flipped = self.pts_y_raw * self.sy
        
        theta = np.radians(self.rot)
        c_val, s_val = np.cos(theta), np.sin(theta)
        
        x_rot = x_flipped * c_val - y_flipped * s_val
        y_rot = x_flipped * s_val + y_flipped * c_val
        
        x_final = x_rot + self.tx
        y_final = y_rot + self.ty
        
        # Color & Plot
        plot_c = self.colors[::10] if self.colors is not None else 'red'
        self.ax.scatter(x_final[::10], y_final[::10], s=1, c=plot_c, alpha=0.6)
        
        self.ax.set_aspect('equal')
        self.ax.grid(True)
        self.fig.canvas.draw()

    def on_key(self, event):
        step = 0.5 if event.key.isupper() else 0.1
        rot_step = 1.0
        
        if event.key == 'right': self.tx += step
        elif event.key == 'left': self.tx -= step
        elif event.key == 'up': self.ty += step
        elif event.key == 'down': self.ty -= step
        elif event.key == 'e': self.rot -= rot_step
        elif event.key == 'q': self.rot += rot_step
        elif event.key == 'h': self.sx *= -1.0
        elif event.key == 'v': self.sy *= -1.0
        
        self.update_plot()
        print(f"BRIDGE CONFIG -> tx: {self.tx:.3f}, ty: {self.ty:.3f}, rot: {np.radians(self.rot):.3f}, sx: {self.sx}, sy: {self.sy}")

if __name__ == "__main__":
    ManualAligner()