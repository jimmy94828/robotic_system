#!/usr/bin/env python3
from __future__ import annotations

import argparse
import re
from pathlib import Path
from typing import Dict, List, Tuple

'''
get timestamp of the camera poses from SLAM
usage:
python3 get_timestamps.py \
  --poses_in  /home/acm118/robot_ws/data/Util/pred_trajectory.txt \
  --img_dir   /home/acm118/robot_ws/data/Util/alignment2/camera_timestamp \
  --poses_out /home/acm118/robot_ws/data/Util/timestamp_pose.txt \
  --out_unit ns 
'''


IMG_EXTS = {".png", ".jpg", ".jpeg", ".bmp", ".tif", ".tiff", ".webp"}

# 你的檔名範例：
# camera_camera_color_image_raw_1770106621931724854_0000.png
# -> stem: camera_camera_color_image_raw_1770106621931724854_0000
# 取出 ts(ns)=1770106621931724854, idx=0000
RE_STEM = re.compile(r"^(?P<prefix>.+)_(?P<ts>\d{10,})_(?P<idx>\d+)$")

def parse_filename(p: Path) -> Tuple[int, int]:
    m = RE_STEM.match(p.stem)
    if not m:
        raise RuntimeError(f"Filename does not match '*_<timestamp>_<index>.*': {p.name}")
    ts_ns = int(m.group("ts"))
    idx = int(m.group("idx"))
    return idx, ts_ns

def load_index_to_ts(img_dir: Path) -> Dict[int, int]:
    files = [p for p in img_dir.iterdir()
             if p.is_file() and p.suffix.lower() in IMG_EXTS]
    if not files:
        raise RuntimeError(f"No images found in: {img_dir}")

    idx2ts: Dict[int, int] = {}
    dups = []
    for p in files:
        idx, ts_ns = parse_filename(p)
        if idx in idx2ts and idx2ts[idx] != ts_ns:
            dups.append((idx, idx2ts[idx], ts_ns, p.name))
        idx2ts[idx] = ts_ns

    if dups:
        ex = dups[0]
        raise RuntimeError(f"Duplicate idx with different timestamps found. Example: idx={ex[0]}, {ex[1]} vs {ex[2]} ({ex[3]})")

    return idx2ts

def is_int_like(s: str) -> bool:
    try:
        v = float(s)
        return abs(v - round(v)) < 1e-6
    except ValueError:
        return False

def main():
    ap = argparse.ArgumentParser(
        description="Replace pose txt timestamps using timestamps parsed from image filenames."
    )
    ap.add_argument("--poses_in", required=True, help="Input pose txt: timestamp tx ty tz qx qy qz qw")
    ap.add_argument("--img_dir", required=True, help="Image folder with '*_<timestamp>_<index>.png'")
    ap.add_argument("--poses_out", required=True, help="Output pose txt with corrected timestamps")
    ap.add_argument("--out_unit", choices=["sec", "ns"], default="sec",
                    help="Output timestamp as seconds(float) or nanoseconds(int). Default: sec")
    ap.add_argument("--idx_offset", type=int, default=0,
                    help="If pose index and filename index differ by a constant offset, set it here. "
                         "Example: pose has 0..449 but filenames are 1..450 -> idx_offset=1")
    ap.add_argument("--use_pose_index_column", action="store_true",
                    help="Use the existing first column as index (0,1,2...) to lookup filenames. "
                         "If not set, use line order (1st pose line -> smallest idx image, etc.)")
    args = ap.parse_args()

    poses_in = Path(args.poses_in)
    img_dir = Path(args.img_dir)
    poses_out = Path(args.poses_out)

    if not poses_in.exists():
        raise FileNotFoundError(f"poses_in not found: {poses_in}")
    if not img_dir.is_dir():
        raise FileNotFoundError(f"img_dir not found: {img_dir}")

    idx2ts = load_index_to_ts(img_dir)
    sorted_indices = sorted(idx2ts.keys())

    # For "line order" mode: map pose line i -> sorted_indices[i]
    def ts_for_line(line_idx_0based: int) -> int:
        if line_idx_0based >= len(sorted_indices):
            raise RuntimeError(f"Pose lines exceed image count: line={line_idx_0based+1}, images={len(sorted_indices)}")
        return idx2ts[sorted_indices[line_idx_0based]]

    out_lines: List[str] = []
    pose_line_idx = 0

    for raw in poses_in.read_text(encoding="utf-8").splitlines():
        s = raw.strip()
        if not s or s.startswith("#"):
            out_lines.append(raw)
            continue

        parts = s.split()
        if len(parts) < 8:
            out_lines.append(raw)
            continue

        if args.use_pose_index_column:
            # first column is index-like (0,1,2...)
            if not is_int_like(parts[0]):
                raise RuntimeError(f"First column is not index-like: '{parts[0]}' (line {pose_line_idx+1})")
            idx = int(round(float(parts[0]))) + args.idx_offset
            if idx not in idx2ts:
                raise RuntimeError(f"Index {idx} not found in image filenames (line {pose_line_idx+1}). "
                                   f"Try adjusting --idx_offset.")
            ts_ns = idx2ts[idx]
        else:
            # use line order
            ts_ns = ts_for_line(pose_line_idx)

        if args.out_unit == "sec":
            new_ts = f"{ts_ns / 1e9:.6f}"
        else:
            new_ts = str(ts_ns)

        out_lines.append(" ".join([new_ts] + parts[1:8]))
        pose_line_idx += 1

    poses_out.write_text("\n".join(out_lines) + "\n", encoding="utf-8")

    print(f"Done. Wrote: {poses_out}")
    print(f"Pose lines updated: {pose_line_idx}")
    print(f"Images indexed: {len(sorted_indices)}")
    print(f"First 3 indices: {sorted_indices[:3]}")
    if sorted_indices:
        i0 = sorted_indices[0]
        print(f"Example: idx={i0} -> ts_ns={idx2ts[i0]} (sec={idx2ts[i0]/1e9:.6f})")

if __name__ == "__main__":
    main()
