#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Collector v3 (ROS2 tf2 + RealSense SDK API) with IR1/IR2 + Left/Right Depth
---------------------------------------------------------------------------
1) ROS2 tf2: world_frame -> base_footprint
2) RealSense via pyrealsense2 (NO realsense-ros):
   - Color (RGB)
   - Depth (Z16)
   - IR-left  (infrared index=1, Y8)
   - IR-right (infrared index=2, Y8)
3) Capture key:
   - First: query base_footprint pose
   - Then: grab one frameset (RGB/Depth/IR L/R)
4) Output folders by modality, filenames by index 000/001/...

WSL2 note:
- Using IR streams increases USB bandwidth requirement, so we try multiple profiles automatically.
"""

import json
import math
import time
import threading
import argparse
from pathlib import Path
from typing import Optional, Tuple, Dict, Any

import numpy as np
import cv2

# ---------- ROS2 ----------
import rclpy
from rclpy.node import Node
from rclpy.time import Time as RosTime
from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException

# ---------- RealSense ----------
try:
    import pyrealsense2 as rs
except Exception:
    rs = None


# =========================
# Utils
# =========================
def quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def depth_to_gray_u8(depth_m: np.ndarray, max_depth_m: float) -> np.ndarray:
    """
    Convert depth(meters) -> grayscale uint8.
    Near -> white, Far -> black, invalid(0) -> 0
    """
    d = depth_m.copy()
    valid = d > 0
    d = np.clip(d, 0.0, max_depth_m)
    gray = np.zeros_like(d, dtype=np.uint8)
    if np.any(valid):
        norm = np.zeros_like(d, dtype=np.float32)
        norm[valid] = d[valid] / max_depth_m
        gray[valid] = ((1.0 - norm[valid]) * 255.0).astype(np.uint8)
    return gray


def zbuffer_project_depth_to_ir2(
    depth_m: np.ndarray,
    depth_intr: "rs.intrinsics",
    ir2_intr: "rs.intrinsics",
    extr_depth_to_ir2: "rs.extrinsics",
) -> np.ndarray:
    """
    Project depth map (depth camera frame) into IR-right camera image plane
    to build a right-view depth map (meters). invalid=0.
    """
    h, w = depth_m.shape

    u = np.arange(w, dtype=np.float32)
    v = np.arange(h, dtype=np.float32)
    uu, vv = np.meshgrid(u, v)

    z = depth_m.reshape(-1).astype(np.float32)
    valid = z > 0
    if not np.any(valid):
        return np.zeros((ir2_intr.height, ir2_intr.width), dtype=np.float32)

    uu = uu.reshape(-1)[valid]
    vv = vv.reshape(-1)[valid]
    z = z[valid]

    # back-project to 3D in depth camera coordinates
    X = (uu - depth_intr.ppx) / depth_intr.fx * z
    Y = (vv - depth_intr.ppy) / depth_intr.fy * z

    # transform to IR2 coordinates: p2 = R*p + t
    Rm = np.array(extr_depth_to_ir2.rotation, dtype=np.float32).reshape(3, 3)
    t = np.array(extr_depth_to_ir2.translation, dtype=np.float32).reshape(3,)

    pts = np.stack([X, Y, z], axis=0)      # (3, N)
    pts2 = (Rm @ pts).T + t                # (N, 3)

    X2, Y2, Z2 = pts2[:, 0], pts2[:, 1], pts2[:, 2]
    ok = Z2 > 0
    if not np.any(ok):
        return np.zeros((ir2_intr.height, ir2_intr.width), dtype=np.float32)

    X2, Y2, Z2 = X2[ok], Y2[ok], Z2[ok]

    # project to IR2 pixels
    u2 = (X2 / Z2) * ir2_intr.fx + ir2_intr.ppx
    v2 = (Y2 / Z2) * ir2_intr.fy + ir2_intr.ppy

    u2i = np.round(u2).astype(np.int32)
    v2i = np.round(v2).astype(np.int32)

    W2, H2 = ir2_intr.width, ir2_intr.height
    inside = (u2i >= 0) & (u2i < W2) & (v2i >= 0) & (v2i < H2)
    u2i, v2i, Z2 = u2i[inside], v2i[inside], Z2[inside]

    out = np.full((H2, W2), np.inf, dtype=np.float32)
    idx = v2i * W2 + u2i
    out_flat = out.reshape(-1)
    np.minimum.at(out_flat, idx, Z2.astype(np.float32))
    out = out_flat.reshape(H2, W2)
    out[np.isinf(out)] = 0.0
    return out


# =========================
# ROS2 TF client
# =========================
class PoseTfClient(Node):
    def __init__(self, world_frame: str, base_frame: str):
        super().__init__("pose_tf_client")
        self.world_frame = world_frame
        self.base_frame = base_frame
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def get_base_pose(self) -> Dict[str, Any]:
        try:
            tf = self.tf_buffer.lookup_transform(self.world_frame, self.base_frame, RosTime())
        except TransformException as e:
            raise RuntimeError(f"TF lookup failed: {self.world_frame} -> {self.base_frame} | {e}")

        t = tf.transform.translation
        q = tf.transform.rotation
        yaw = quat_to_yaw(q.x, q.y, q.z, q.w)

        return {
            "world_frame": self.world_frame,
            "base_frame": self.base_frame,
            "stamp_ros": {"sec": int(tf.header.stamp.sec), "nanosec": int(tf.header.stamp.nanosec)},
            "translation": {"x": float(t.x), "y": float(t.y), "z": float(t.z)},
            "rotation_xyzw": {"x": float(q.x), "y": float(q.y), "z": float(q.z), "w": float(q.w)},
            "yaw_rad": float(yaw),
        }


# =========================
# RealSense grabber (Color+Depth+IR1+IR2) with fallback profiles
# =========================
class RealSenseGrabber:
    def __init__(self, serial: Optional[str] = None):
        if rs is None:
            raise RuntimeError("pyrealsense2 not available. Install librealsense + pyrealsense2.")

        self.serial = serial
        self._lock = threading.Lock()
        self._running = False
        self._thread: Optional[threading.Thread] = None

        self._pipeline = rs.pipeline()
        self.profile = None

        # Selected settings
        self.cw = self.ch = self.cfps = None
        self.dw = self.dh = self.dfps = None

        # Intr/extr
        self.depth_scale = None
        self.color_stream = None
        self.depth_stream = None
        self.ir1_stream = None
        self.ir2_stream = None
        self.depth_intr = None
        self.ir2_intr = None
        self.extr_depth_to_ir2 = None

        self.latest: Optional[Dict[str, np.ndarray]] = None
        self.latest_meta: Optional[Dict[str, Any]] = None

    def start(self, prefer_color=(640, 480, 15), prefer_depth=(424, 240, 15)):
        """
        Try multiple combinations until start succeeds.
        WSL2 建議：color fps 15，depth/IR 424x240 15 或更低。
        """
        candidates = [
            # (cw,ch,cfps, dw,dh,dfps)
            (640, 480, 30, 640, 480, 30),
            (640, 480, 15, 424, 240, 15),
            (640, 480, 30, 424, 240, 30),
            (640, 480, 15, 424, 240, 15),
            (424, 240, 30, 424, 240, 30),
            (424, 240, 15, 424, 240, 15),
            (320, 240, 30, 320, 240, 30),
            (320, 240, 15, 320, 240, 15),
        ]

        # put preferred first
        pref = (prefer_color[0], prefer_color[1], prefer_color[2], prefer_depth[0], prefer_depth[1], prefer_depth[2])
        if pref in candidates:
            candidates.remove(pref)
        candidates.insert(0, pref)

        last_err = None
        for (cw, ch, cfps, dw, dh, dfps) in candidates:
            try:
                # recreate pipeline each attempt
                self._pipeline = rs.pipeline()
                cfg = rs.config()
                if self.serial:
                    cfg.enable_device(self.serial)

                # Color
                cfg.enable_stream(rs.stream.color, cw, ch, rs.format.rgb8, cfps)
                # Depth
                cfg.enable_stream(rs.stream.depth, dw, dh, rs.format.z16, dfps)
                # IR L/R
                cfg.enable_stream(rs.stream.infrared, 1, dw, dh, rs.format.y8, dfps)
                cfg.enable_stream(rs.stream.infrared, 2, dw, dh, rs.format.y8, dfps)

                profile = self._pipeline.start(cfg)
                self.profile = profile

                # save selected
                self.cw, self.ch, self.cfps = cw, ch, cfps
                self.dw, self.dh, self.dfps = dw, dh, dfps

                # intr/extr
                depth_sensor = profile.get_device().first_depth_sensor()
                self.depth_scale = float(depth_sensor.get_depth_scale())

                self.color_stream = profile.get_stream(rs.stream.color).as_video_stream_profile()
                self.depth_stream = profile.get_stream(rs.stream.depth).as_video_stream_profile()
                self.ir1_stream = profile.get_stream(rs.stream.infrared, 1).as_video_stream_profile()
                self.ir2_stream = profile.get_stream(rs.stream.infrared, 2).as_video_stream_profile()

                self.depth_intr = self.depth_stream.get_intrinsics()
                self.ir2_intr = self.ir2_stream.get_intrinsics()
                self.extr_depth_to_ir2 = self.depth_stream.get_extrinsics_to(self.ir2_stream)

                print(f"[RealSense] started OK: "
                      f"color={cw}x{ch}@{cfps}, depth/ir={dw}x{dh}@{dfps}")
                break

            except Exception as e:
                last_err = e
                print(f"[RealSense] start failed: color={cw}x{ch}@{cfps}, depth/ir={dw}x{dh}@{dfps} -> {e}")

        if self.profile is None:
            raise RuntimeError(f"RealSense start failed for all candidates. Last error: {last_err}")

        # start background thread
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        try:
            self._pipeline.stop()
        except Exception:
            pass

    def _loop(self):
        while self._running:
            frames = self._pipeline.wait_for_frames()

            color = frames.get_color_frame()
            depth = frames.get_depth_frame()
            ir1 = frames.get_infrared_frame(1)
            ir2 = frames.get_infrared_frame(2)

            if not color or not depth or not ir1 or not ir2:
                continue

            rgb = np.asanyarray(color.get_data())  # RGB8
            rgb_bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

            depth_raw = np.asanyarray(depth.get_data()).astype(np.uint16)  # Z16
            depth_m = depth_raw.astype(np.float32) * self.depth_scale

            ir_left = np.asanyarray(ir1.get_data())   # Y8
            ir_right = np.asanyarray(ir2.get_data())  # Y8

            out = {
                "rgb_bgr": rgb_bgr,
                "depth_raw": depth_raw,
                "depth_m": depth_m,
                "ir_left": ir_left,
                "ir_right": ir_right,
            }

            meta = {
                "ts_system": time.time(),
                "depth_scale": self.depth_scale,
                "color_profile": {"w": self.cw, "h": self.ch, "fps": self.cfps},
                "depth_ir_profile": {"w": self.dw, "h": self.dh, "fps": self.dfps},
            }

            with self._lock:
                self.latest = out
                self.latest_meta = meta

    def get_latest_copy(self) -> Tuple[Dict[str, np.ndarray], Dict[str, Any]]:
        with self._lock:
            if self.latest is None:
                raise RuntimeError("No frames received yet.")
            snap = {k: v.copy() for k, v in self.latest.items()}
            meta = dict(self.latest_meta) if self.latest_meta else {}
        return snap, meta

    def get_intrinsics_extrinsics(self) -> Dict[str, Any]:
        def intr_to_dict(intr: "rs.intrinsics") -> Dict[str, Any]:
            return {
                "width": intr.width,
                "height": intr.height,
                "fx": intr.fx,
                "fy": intr.fy,
                "ppx": intr.ppx,
                "ppy": intr.ppy,
                "model": int(intr.model),
                "coeffs": list(intr.coeffs),
            }

        info = {
            "depth_intr": intr_to_dict(self.depth_intr),
            "ir_right_intr": intr_to_dict(self.ir2_intr),
            "color_intr": intr_to_dict(self.color_stream.get_intrinsics()),
            "extr_depth_to_ir_right": {
                "rotation": list(self.extr_depth_to_ir2.rotation),
                "translation": list(self.extr_depth_to_ir2.translation),
            },
        }
        return info


# =========================
# IO: folders
# =========================
def make_output_folders(out_dir: Path) -> Dict[str, Path]:
    folders = {
        "rgb": out_dir / "rgb",
        "ir_left": out_dir / "ir_left",
        "ir_right": out_dir / "ir_right",
        "depth_left_gray": out_dir / "depth_left_gray",
        "depth_right_gray": out_dir / "depth_right_gray",
        "depth_left_raw_mm": out_dir / "depth_left_raw_mm",
        "depth_right_raw_mm": out_dir / "depth_right_raw_mm",
        "meta": out_dir / "meta",
    }
    for p in folders.values():
        p.mkdir(parents=True, exist_ok=True)
    return folders


def save_sample(
    folders: Dict[str, Path],
    idx: int,
    pose: Dict[str, Any],
    rs_frames: Dict[str, np.ndarray],
    rs_meta: Dict[str, Any],
    rs_intr_extr: Dict[str, Any],
    max_depth_m: float,
):
    img_name = f"{idx:03d}.png"
    json_name = f"{idx:03d}.json"

    meta = {
        "idx": idx,
        "ts_system_pose_then_image": time.time(),
        "pose": pose,
        "realsense_meta": rs_meta,
        "realsense_intr_extr": rs_intr_extr,
        "notes": {"order": "pose first, then frameset snapshot", "right_depth": "projected into IR-right view"},
    }
    (folders["meta"] / json_name).write_text(json.dumps(meta, indent=2, ensure_ascii=False), encoding="utf-8")

    # rgb / ir
    cv2.imwrite(str(folders["rgb"] / img_name), rs_frames["rgb_bgr"])
    cv2.imwrite(str(folders["ir_left"] / img_name), rs_frames["ir_left"])
    cv2.imwrite(str(folders["ir_right"] / img_name), rs_frames["ir_right"])

    # depth-left
    depth_raw = rs_frames["depth_raw"]
    depth_m = rs_frames["depth_m"]
    cv2.imwrite(str(folders["depth_left_raw_mm"] / img_name), depth_raw)
    cv2.imwrite(str(folders["depth_left_gray"] / img_name), depth_to_gray_u8(depth_m, max_depth_m))

    # depth-right
    depth_right_m = rs_frames["depth_right_m"]
    cv2.imwrite(str(folders["depth_right_gray"] / img_name), depth_to_gray_u8(depth_right_m, max_depth_m))
    depth_right_mm = np.clip(depth_right_m * 1000.0, 0, 65535).astype(np.uint16)
    cv2.imwrite(str(folders["depth_right_raw_mm"] / img_name), depth_right_mm)


# =========================
# Main
# =========================
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", type=str, default="dataset_out", help="Output directory")
    parser.add_argument("--world-frame", type=str, default="map", help="TF world frame (map or odom)")
    parser.add_argument("--base-frame", type=str, default="base_footprint", help="Base frame")

    # preferred profiles (WSL2 建議先這樣)
    parser.add_argument("--pref-color", type=str, default="640,480,15", help="preferred color w,h,fps")
    parser.add_argument("--pref-depth", type=str, default="640,480,15", help="preferred depth/ir w,h,fps")

    parser.add_argument("--max-depth", type=float, default=3.0, help="Max depth (m) for gray rendering")
    parser.add_argument("--no-gui", action="store_true", help="CLI capture (Enter) instead of GUI key press")
    parser.add_argument("--serial", type=str, default=None, help="RealSense serial (optional)")
    args = parser.parse_args()

    def parse_triplet(s: str):
        a = [int(x.strip()) for x in s.split(",")]
        if len(a) != 3:
            raise ValueError("triplet must be w,h,fps")
        return (a[0], a[1], a[2])

    pref_color = parse_triplet(args.pref_color)
    pref_depth = parse_triplet(args.pref_depth)

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    folders = make_output_folders(out_dir)

    poses_csv = out_dir / "poses.csv"
    if not poses_csv.exists():
        poses_csv.write_text(
            "idx,ts_system,world_frame,base_frame,x,y,z,qx,qy,qz,qw,yaw_rad\n",
            encoding="utf-8",
        )

    # ROS2 init
    rclpy.init()
    node = PoseTfClient(world_frame=args.world_frame, base_frame=args.base_frame)

    # RealSense
    grabber = RealSenseGrabber(serial=args.serial)
    grabber.start(prefer_color=pref_color, prefer_depth=pref_depth)

    # ROS spin
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    idx = 0
    running = True

    def do_capture():
        nonlocal idx

        # 1) pose first
        pose = node.get_base_pose()

        # 2) then frameset snapshot
        rs_frames, rs_meta = grabber.get_latest_copy()
        rs_intr_extr = grabber.get_intrinsics_extrinsics()

        # 3) right depth from projection into IR-right geometry
        rs_frames["depth_right_m"] = zbuffer_project_depth_to_ir2(
            depth_m=rs_frames["depth_m"],
            depth_intr=grabber.depth_intr,
            ir2_intr=grabber.ir2_intr,
            extr_depth_to_ir2=grabber.extr_depth_to_ir2,
        )

        # 4) save
        save_sample(
            folders=folders,
            idx=idx,
            pose=pose,
            rs_frames=rs_frames,
            rs_meta=rs_meta,
            rs_intr_extr=rs_intr_extr,
            max_depth_m=args.max_depth,
        )

        # poses.csv
        t = pose["translation"]
        q = pose["rotation_xyzw"]
        line = (
            f'{idx},{time.time()},{pose["world_frame"]},{pose["base_frame"]},'
            f'{t["x"]},{t["y"]},{t["z"]},{q["x"]},{q["y"]},{q["z"]},{q["w"]},{pose["yaw_rad"]}\n'
        )
        with poses_csv.open("a", encoding="utf-8") as f:
            f.write(line)

        print(f"[CAPTURE] saved idx={idx:03d}")
        idx += 1

    if args.no_gui:
        print("CLI 模式：按 Enter capture；輸入 q 後 Enter 離開。")
        while running:
            s = input()
            if s.strip().lower() == "q":
                running = False
                break
            try:
                do_capture()
            except Exception as e:
                print(f"[ERROR] capture failed: {e}")
    else:
        import matplotlib
        matplotlib.use("TkAgg")
        import matplotlib.pyplot as plt

        plt.ion()
        fig, axs = plt.subplots(1, 3, figsize=(13, 4))
        fig.canvas.manager.set_window_title("Preview (press 'c' capture, 'q' quit)")

        ax1, ax2, ax3 = axs
        im1 = ax1.imshow(np.zeros((pref_color[1], pref_color[0], 3), dtype=np.uint8))
        ax1.set_title("RGB")
        ax1.axis("off")

        im2 = ax2.imshow(np.zeros((pref_depth[1], pref_depth[0]), dtype=np.uint8), cmap="gray", vmin=0, vmax=255)
        ax2.set_title("IR-left")
        ax2.axis("off")

        im3 = ax3.imshow(np.zeros((pref_depth[1], pref_depth[0]), dtype=np.uint8), cmap="gray", vmin=0, vmax=255)
        ax3.set_title("Depth-left(gray)")
        ax3.axis("off")

        def on_key(event):
            nonlocal running
            if event.key == "c":
                try:
                    do_capture()
                except Exception as e:
                    print(f"[ERROR] capture failed: {e}")
            elif event.key == "q":
                running = False

        fig.canvas.mpl_connect("key_press_event", on_key)

        while running and plt.fignum_exists(fig.number):
            try:
                rs_frames, _ = grabber.get_latest_copy()
                rgb = rs_frames["rgb_bgr"]
                irl = rs_frames["ir_left"]
                dgl = depth_to_gray_u8(rs_frames["depth_m"], args.max_depth)

                im1.set_data(cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB))
                im2.set_data(irl)
                im3.set_data(dgl)

                fig.canvas.draw()
                fig.canvas.flush_events()
                time.sleep(0.03)
            except Exception:
                time.sleep(0.05)

        plt.close(fig)

    grabber.stop()
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
