#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import re
import shutil
from pathlib import Path
from typing import Dict, List, Tuple, Set

IMG_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff", ".webp"}
'''
sample camera image from collected data
run:
python3 sample_timestamp.py \
  --csv /home/acm118/robot_ws/data/Util/alignment2/selected_pairs.csv \
  --camera_dir /home/acm118/robot_ws/data/Util/alignment2/camera_camera_color_image_raw \
  --out_dir /home/acm118/robot_ws/data/Util/alignment2/extracted_camera \
  --key_col key \
  --rename seq
'''

def extract_trailing_int(s: str) -> int:
    """
    Extract trailing digits as int.
    Examples:
      "camera_camera_color_image_raw_0007" -> 7
      "7" -> 7
    """
    s = str(s).strip()
    if s.isdigit():
        return int(s)
    m = re.search(r"(\d+)$", s)
    if not m:
        raise ValueError(f"Cannot extract trailing digits from key='{s}'")
    return int(m.group(1))


def list_images(root: Path, recursive: bool) -> List[Path]:
    it = root.rglob("*") if recursive else root.iterdir()
    files = [p for p in it if p.is_file() and p.suffix.lower() in IMG_EXTS]
    files.sort()
    return files


def build_index_map(files: List[Path]) -> Dict[int, Path]:
    """
    Map trailing index -> image path. Detect duplicates.
    """
    mp: Dict[int, Path] = {}
    dup: List[Tuple[int, Path, Path]] = []
    for p in files:
        idx = extract_trailing_int(p.stem)
        if idx in mp:
            dup.append((idx, mp[idx], p))
        else:
            mp[idx] = p
    if dup:
        # show only first few duplicates to help debug
        msg = "\n".join([f"idx={i}: {a.name} vs {b.name}" for i, a, b in dup[:10]])
        raise RuntimeError(
            f"Duplicate trailing indices found in camera_dir (showing up to 10):\n{msg}\n"
            f"Fix: ensure unique filenames per index, or include subfolder info and change pairing logic."
        )
    return mp


def read_keys_from_csv(csv_path: Path, key_col: str) -> List[int]:
    """
    Read key column values and return unique indices (preserve sorted order later).
    """
    if not csv_path.exists():
        raise FileNotFoundError(f"CSV not found: {csv_path}")

    keys: List[int] = []
    with csv_path.open("r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None or key_col not in reader.fieldnames:
            raise RuntimeError(f"CSV missing column '{key_col}'. Found columns: {reader.fieldnames}")
        for row in reader:
            keys.append(extract_trailing_int(row[key_col]))

    # remove duplicates but keep order (stable)
    seen: Set[int] = set()
    uniq = []
    for k in keys:
        if k not in seen:
            seen.add(k)
            uniq.append(k)
    return uniq


def main():
    ap = argparse.ArgumentParser(
        description="Select camera images by CSV key column (trailing index) and copy to a new folder."
    )
    ap.add_argument("--csv", required=True, help="Path to selected_pairs.csv")
    ap.add_argument("--camera_dir", required=True, help="Source camera image folder to pick from")
    ap.add_argument("--out_dir", required=True, help="Output folder")
    ap.add_argument("--key_col", default="key", help="CSV column name for key (default: key)")
    ap.add_argument("--recursive", action="store_true", help="Scan camera_dir recursively")
    ap.add_argument("--rename", choices=["keep", "key", "seq"], default="seq",
                    help="Output naming: keep(original name) / key(0007.ext) / seq(001..N.ext)")
    ap.add_argument("--start", type=int, default=1, help="Start number for seq rename (default: 1)")
    ap.add_argument("--digits", type=int, default=3,
                    help="Zero padding digits for seq/key rename (default: 3 -> 001)")
    ap.add_argument("--dry_run", action="store_true", help="Print plan only, do not copy")
    args = ap.parse_args()

    csv_path = Path(args.csv)
    camera_dir = Path(args.camera_dir)
    out_dir = Path(args.out_dir)

    if not camera_dir.is_dir():
        raise FileNotFoundError(f"camera_dir not found: {camera_dir}")

    keys = read_keys_from_csv(csv_path, args.key_col)
    if not keys:
        raise RuntimeError("No keys found in CSV.")

    cam_files = list_images(camera_dir, args.recursive)
    if not cam_files:
        raise RuntimeError(f"No images found in camera_dir: {camera_dir}")

    idx_map = build_index_map(cam_files)

    missing = [k for k in keys if k not in idx_map]
    found = [k for k in keys if k in idx_map]

    print(f"CSV keys (unique): {len(keys)}")
    print(f"Camera images scanned: {len(cam_files)}")
    print(f"Matched: {len(found)}")
    print(f"Missing: {len(missing)}")
    if missing:
        print("First 20 missing indices:", missing[:20])

    # build copy plan in the same order as CSV
    plan = []
    seq_no = args.start
    for k in keys:
        if k not in idx_map:
            continue
        src = idx_map[k]
        if args.rename == "keep":
            dst_name = src.name
        elif args.rename == "key":
            dst_name = f"{k:0{args.digits}d}{src.suffix.lower()}"
        else:  # seq
            dst_name = f"{seq_no:0{args.digits}d}{src.suffix.lower()}"
            seq_no += 1
        dst = out_dir / dst_name
        plan.append((k, src, dst))

    # preview
    print("\nPreview (first 10):")
    for k, src, dst in plan[:10]:
        print(f"key={k}  {src.name}  ->  {dst.name}")
    if args.dry_run:
        print("\nDry run: no files copied.")
        return

    out_dir.mkdir(parents=True, exist_ok=True)

    # copy
    for _, src, dst in plan:
        shutil.copy2(src, dst)

    # write manifest
    manifest = out_dir / "extracted_manifest.csv"
    with manifest.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["key", "src", "dst"])
        for k, src, dst in plan:
            w.writerow([k, str(src), str(dst)])

    print(f"\nDone. Copied {len(plan)} files to: {out_dir}")
    print(f"Manifest: {manifest}")


if __name__ == "__main__":
    main()
