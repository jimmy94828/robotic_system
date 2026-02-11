#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import random
import shutil
import re
from pathlib import Path
from typing import Dict, List, Tuple

IMG_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff", ".webp"}

'''
sample images from the collected data
run:
python3 downsample.py \
  --rgb_dir /home/acm118/robot_ws/data/Util/alignment2/camera_camera_color_image_raw \
  --depth_dir /home/acm118/robot_ws/data/Util/alignment2/camera_camera_depth_image_rect_raw \
  --out_rgb /home/acm118/robot_ws/data/Util/alignment2/rgb \
  --out_depth /home/acm118/robot_ws/data/Util/alignment2/depth \
  --n 450 --mode uniform --key digits
'''

def list_images(root: Path, recursive: bool) -> List[Path]:
    it = root.rglob("*") if recursive else root.iterdir()
    files = [p for p in it if p.is_file() and p.suffix.lower() in IMG_EXTS]
    files.sort()
    return files


def make_key(p: Path, root: Path, key_mode: str):
    """
    key_mode:
      - "rel": relative path without suffix
      - "stem": filename stem
      - "digits": trailing digits in stem (e.g., *_000123 -> 123)
    """
    if key_mode == "digits":
        m = re.search(r"(\d+)$", p.stem)
        if not m:
            raise RuntimeError(f"Cannot extract trailing digits from: {p.name}")
        return int(m.group(1))  # 用 int 讓排序是數字順序（uniform 才會真的均勻）
    if key_mode == "stem":
        return p.stem
    # "rel"
    rel = p.relative_to(root)
    return str(rel.with_suffix(""))



def uniform_indices(m: int, n: int) -> List[int]:
    if n <= 1:
        return [0]
    return [round(i * (m - 1) / (n - 1)) for i in range(n)]


def main():
    ap = argparse.ArgumentParser(
        description="Downsample paired RGB/Depth folders while keeping 1-1 correspondence."
    )
    ap.add_argument("--rgb_dir", required=True, help="RGB folder path")
    ap.add_argument("--depth_dir", required=True, help="Depth folder path")
    ap.add_argument("--out_rgb", required=True, help="Output RGB folder")
    ap.add_argument("--out_depth", required=True, help="Output Depth folder")
    ap.add_argument("--n", type=int, default=450, help="Number of pairs to sample (e.g., 400~500)")
    ap.add_argument("--mode", choices=["random", "uniform"], default="random",
                    help="random: random subset; uniform: evenly spaced across sorted keys")
    ap.add_argument("--seed", type=int, default=42, help="Random seed (random mode only)")
    ap.add_argument("--recursive", action="store_true", help="Include subfolders")
    ap.add_argument("--key", choices=["rel", "stem", "digits"], default="digits",
                help="How to match rgb<->depth: digits (recommended), rel, or stem")
    ap.add_argument("--overwrite", action="store_true",
                    help="Allow output folders to exist (files may be added/overwritten by name)")
    ap.add_argument("--dry_run", action="store_true", help="Do not copy, only print summary")

    args = ap.parse_args()

    rgb_root = Path(args.rgb_dir)
    depth_root = Path(args.depth_dir)
    out_rgb = Path(args.out_rgb)
    out_depth = Path(args.out_depth)

    if not rgb_root.is_dir():
        raise FileNotFoundError(f"RGB dir not found: {rgb_root}")
    if not depth_root.is_dir():
        raise FileNotFoundError(f"Depth dir not found: {depth_root}")

    rgb_files = list_images(rgb_root, args.recursive)
    depth_files = list_images(depth_root, args.recursive)

    if not rgb_files:
        raise RuntimeError(f"No RGB images found in: {rgb_root}")
    if not depth_files:
        raise RuntimeError(f"No Depth images found in: {depth_root}")

    # Build depth lookup by key
    depth_map: Dict[str, Path] = {}
    dup_depth_keys = set()
    for p in depth_files:
        k = make_key(p, depth_root, args.key)
        if k in depth_map:
            dup_depth_keys.add(k)
        depth_map[k] = p
    if dup_depth_keys:
        raise RuntimeError(
            f"Depth has duplicate keys ({len(dup_depth_keys)}). "
            f"Use --key rel and/or avoid duplicate filenames. Example: {next(iter(dup_depth_keys))}"
        )

    # Pair by key (and check missing)
    pairs: List[Tuple[str, Path, Path]] = []
    missing_in_depth = []
    dup_rgb_keys = set()
    seen_rgb_keys = set()

    for rp in rgb_files:
        k = make_key(rp, rgb_root, args.key)
        if k in seen_rgb_keys:
            dup_rgb_keys.add(k)
            continue
        seen_rgb_keys.add(k)

        dp = depth_map.get(k)
        if dp is None:
            missing_in_depth.append(k)
            continue
        pairs.append((k, rp, dp))

    if dup_rgb_keys:
        raise RuntimeError(
            f"RGB has duplicate keys ({len(dup_rgb_keys)}). "
            f"Use --key rel and/or avoid duplicate filenames. Example: {next(iter(dup_rgb_keys))}"
        )

    if missing_in_depth:
        preview = "\n".join(missing_in_depth[:10])
        raise RuntimeError(
            f"Found {len(missing_in_depth)} RGB files without matching Depth key.\n"
            f"First 10 missing keys:\n{preview}"
        )

    if len(pairs) == 0:
        raise RuntimeError("No paired files found. Check folder structure / filenames / --key option.")

    if args.n < 1:
        raise ValueError("--n must be >= 1")
    if args.n > len(pairs):
        raise ValueError(f"--n ({args.n}) is larger than available pairs ({len(pairs)})")

    # Sort pairs by key for reproducibility / uniform mode
    pairs.sort(key=lambda x: x[0])

    if args.mode == "random":
        rng = random.Random(args.seed)
        selected = rng.sample(pairs, k=args.n)
    else:
        idxs = uniform_indices(len(pairs), args.n)
        # remove duplicates if rounding causes repeats, then fill
        used = set()
        selected = []
        for idx in idxs:
            j = idx
            while j < len(pairs) and j in used:
                j += 1
            if j >= len(pairs):
                j = idx
                while j >= 0 and j in used:
                    j -= 1
            if 0 <= j < len(pairs) and j not in used:
                used.add(j)
                selected.append(pairs[j])
        # fill remaining
        if len(selected) < args.n:
            for j in range(len(pairs)):
                if j not in used:
                    used.add(j)
                    selected.append(pairs[j])
                if len(selected) == args.n:
                    break
        selected = selected[:args.n]

    print(f"RGB files:   {len(rgb_files)}")
    print(f"Depth files: {len(depth_files)}")
    print(f"Paired:      {len(pairs)}")
    print(f"Selected:    {len(selected)} (mode={args.mode})")

    if args.dry_run:
        print("Dry run: no files copied.")
        return

    # Prepare output dirs
    if (out_rgb.exists() or out_depth.exists()) and not args.overwrite:
        raise FileExistsError("Output folder exists. Use --overwrite to allow.")
    out_rgb.mkdir(parents=True, exist_ok=True)
    out_depth.mkdir(parents=True, exist_ok=True)

    # Copy while preserving relative structure (if recursive) or flat (if not recursive)
    manifest_path = out_rgb.parent / "selected_pairs.csv"
    with manifest_path.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["key", "rgb_src", "depth_src", "rgb_dst", "depth_dst"])

        for k, rp, dp in selected:
            if args.recursive:
                rgb_rel = rp.relative_to(rgb_root)
                depth_rel = dp.relative_to(depth_root)
                rgb_dst = out_rgb / rgb_rel
                depth_dst = out_depth / depth_rel
            else:
                rgb_dst = out_rgb / rp.name
                depth_dst = out_depth / dp.name

            rgb_dst.parent.mkdir(parents=True, exist_ok=True)
            depth_dst.parent.mkdir(parents=True, exist_ok=True)

            shutil.copy2(rp, rgb_dst)
            shutil.copy2(dp, depth_dst)

            w.writerow([k, str(rp), str(dp), str(rgb_dst), str(depth_dst)])

    print(f"Done. Output:")
    print(f"  RGB:   {out_rgb}")
    print(f"  Depth: {out_depth}")
    print(f"Manifest: {manifest_path}")


if __name__ == "__main__":
    main()
