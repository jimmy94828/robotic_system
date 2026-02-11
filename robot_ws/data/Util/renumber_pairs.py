#!/usr/bin/env python3
from __future__ import annotations

import argparse
import re
from pathlib import Path
from typing import Dict, List, Tuple

IMG_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff", ".webp"}

'''
renumber the sampled image data:
run:
python3 renumber_pairs.py \
  --rgb_dir /home/acm118/robot_ws/data/Util/alignment2/rgb \
  --depth_dir /home/acm118/robot_ws/data/Util/alignment2/depth \
  --n 450
'''


def list_images(folder: Path) -> List[Path]:
    files = [p for p in folder.iterdir() if p.is_file() and p.suffix.lower() in IMG_EXTS]
    files.sort()
    return files


def trailing_digits_key(p: Path) -> int:
    """
    Extract trailing digits from filename stem.
    Example: camera_camera_color_image_raw_0007.png -> 7
    """
    m = re.search(r"(\d+)$", p.stem)
    if not m:
        raise RuntimeError(f"Cannot extract trailing digits from: {p.name}")
    return int(m.group(1))


def build_key_map(files: List[Path]) -> Dict[int, Path]:
    m: Dict[int, Path] = {}
    dup = []
    for f in files:
        k = trailing_digits_key(f)
        if k in m:
            dup.append(k)
        m[k] = f
    if dup:
        raise RuntimeError(f"Duplicate trailing-digit keys found: {sorted(set(dup))[:10]} (showing up to 10)")
    return m


def main():
    ap = argparse.ArgumentParser(description="Renumber paired RGB/Depth images to 001..N while keeping correspondence.")
    ap.add_argument("--rgb_dir", required=True, help="RGB folder (already downsampled)")
    ap.add_argument("--depth_dir", required=True, help="Depth folder (already downsampled)")
    ap.add_argument("--n", type=int, required=True, help="Expected number of pairs, e.g., 450")
    ap.add_argument("--dry_run", action="store_true", help="Print plan only, do not rename")
    args = ap.parse_args()

    rgb_dir = Path(args.rgb_dir)
    depth_dir = Path(args.depth_dir)

    if not rgb_dir.is_dir():
        raise FileNotFoundError(f"RGB dir not found: {rgb_dir}")
    if not depth_dir.is_dir():
        raise FileNotFoundError(f"Depth dir not found: {depth_dir}")

    rgb_files = list_images(rgb_dir)
    depth_files = list_images(depth_dir)

    if len(rgb_files) != args.n or len(depth_files) != args.n:
        raise RuntimeError(f"Count mismatch: rgb={len(rgb_files)}, depth={len(depth_files)}, expected n={args.n}")

    rgb_map = build_key_map(rgb_files)
    depth_map = build_key_map(depth_files)

    keys_rgb = set(rgb_map.keys())
    keys_depth = set(depth_map.keys())
    common = sorted(keys_rgb & keys_depth)

    if len(common) != args.n:
        missing_rgb = sorted(keys_depth - keys_rgb)
        missing_depth = sorted(keys_rgb - keys_depth)
        raise RuntimeError(
            f"Pairing mismatch by trailing digits.\n"
            f"common={len(common)} expected={args.n}\n"
            f"missing in rgb (show 10): {missing_rgb[:10]}\n"
            f"missing in depth (show 10): {missing_depth[:10]}"
        )

    # Build ordered pairs (by original trailing index order)
    pairs: List[Tuple[Path, Path]] = [(rgb_map[k], depth_map[k]) for k in common]

    # Plan final names: 001..N, keep original extension separately for rgb/depth
    plan = []
    for i, (rp, dp) in enumerate(pairs, start=1):
        rgb_new = rgb_dir / f"{i:03d}{rp.suffix.lower()}"
        depth_new = depth_dir / f"{i:03d}{dp.suffix.lower()}"
        plan.append((rp, rgb_new, dp, depth_new))

    # Print preview
    print(f"Will renumber {args.n} pairs.")
    for row in plan[:10]:
        rp, rgb_new, dp, depth_new = row
        print(f"RGB:   {rp.name} -> {rgb_new.name}")
        print(f"Depth: {dp.name} -> {depth_new.name}")
    if args.n > 10:
        print("...")

    if args.dry_run:
        print("Dry run: no files renamed.")
        return

    # Two-phase rename to avoid collisions:
    # Phase 1: rename all to temporary unique names
    tmp_rgb = []
    tmp_depth = []
    for idx, (rp, _, dp, _) in enumerate(plan, start=1):
        rp_tmp = rgb_dir / f"__tmp__{idx:04d}__{rp.name}"
        dp_tmp = depth_dir / f"__tmp__{idx:04d}__{dp.name}"
        rp.rename(rp_tmp)
        dp.rename(dp_tmp)
        tmp_rgb.append(rp_tmp)
        tmp_depth.append(dp_tmp)

    # Phase 2: rename temporary files to final 001..N
    for i in range(args.n):
        rp_tmp = tmp_rgb[i]
        dp_tmp = tmp_depth[i]
        rgb_final = rgb_dir / f"{i+1:03d}{rp_tmp.suffix.lower()}"
        depth_final = depth_dir / f"{i+1:03d}{dp_tmp.suffix.lower()}"

        # Note: rp_tmp.suffix is still original extension; tmp name doesn't change it.
        rp_tmp.rename(rgb_final)
        dp_tmp.rename(depth_final)

    print("Done renumbering.")


if __name__ == "__main__":
    main()
