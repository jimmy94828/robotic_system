#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Convert a folder of one-line pose .txt files into pose2d.csv.

Expected filename examples:
  000000_1776216000.123456_pose.txt
  12_1776216000_pose.txt
  12_pose.txt

Supported one-line content examples:
  x y yaw
  timestamp x y yaw
  idx timestamp x y yaw
  x y z qx qy qz qw
  timestamp x y z qx qy qz qw
  {"x": 1.2, "y": 3.4, "yaw_rad": 0.5}
  x: 1.2, y: 3.4, yaw: 0.5

Output columns by default:
  idx,stamp,x,y,yaw_rag

Note: map_alignment_v3.py accepts yaw or yaw_rad by default, not yaw_rag.
If you want to feed this CSV directly to the unmodified map_alignment_v3.py, use --output_yaw_col yaw_rad
or modify map_alignment_v3.py to accept yaw_rag as an alias.
"""

from __future__ import annotations

import argparse
import json
import math
import re
from pathlib import Path
from typing import Optional, Tuple

import pandas as pd

FLOAT_RE = re.compile(r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?")
KEYVAL_RE = re.compile(
    r"(?P<key>idx|index|timestamp|t_sec|time|x|y|z|yaw|yaw_rad|yaw_deg|theta|qx|qy|qz|qw)"
    r"\s*[:=]\s*"
    r"(?P<val>[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?)",
    re.IGNORECASE,
)


def parse_filename(path: Path) -> Tuple[int, Optional[float]]:
    """
    Parse filename like:
      <idx>_<timestamp>_pose.txt
      <idx>_pose.txt
    Return: (idx, timestamp_or_None)
    """
    stem = path.name
    # Remove final .txt only, preserving possible dots inside timestamp.
    if stem.lower().endswith(".txt"):
        stem = stem[:-4]

    parts = stem.split("_")
    if not parts:
        raise ValueError(f"Cannot parse filename: {path.name}")

    # First token must be idx.
    try:
        idx = int(parts[0])
    except ValueError as exc:
        raise ValueError(
            f"Filename must start with integer idx, e.g. 000001_1776216000.123_pose.txt; got {path.name}"
        ) from exc

    timestamp = None
    if len(parts) >= 2:
        try:
            timestamp = float(parts[1])
        except ValueError:
            timestamp = None

    return idx, timestamp


def yaw_from_quat(qx: float, qy: float, qz: float, qw: float) -> float:
    """Return yaw angle in radians from quaternion x,y,z,w."""
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 0:
        raise ValueError("Quaternion norm is zero.")
    qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm

    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle_rad(a: float) -> float:
    """Normalize angle to [-pi, pi)."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def maybe_deg_to_rad(yaw: float, yaw_unit: str) -> float:
    if yaw_unit == "deg":
        yaw = math.radians(yaw)
    return normalize_angle_rad(yaw)


def parse_json_line(line: str) -> Optional[Tuple[float, float, float]]:
    try:
        obj = json.loads(line)
    except Exception:
        return None
    if not isinstance(obj, dict):
        return None

    lower = {str(k).lower(): v for k, v in obj.items()}
    if "x" in lower and "y" in lower:
        x = float(lower["x"])
        y = float(lower["y"])
        if "yaw_rad" in lower:
            yaw = float(lower["yaw_rad"])
        elif "yaw" in lower:
            yaw = float(lower["yaw"])
        elif "theta" in lower:
            yaw = float(lower["theta"])
        elif all(k in lower for k in ["qx", "qy", "qz", "qw"]):
            yaw = yaw_from_quat(
                float(lower["qx"]), float(lower["qy"]), float(lower["qz"]), float(lower["qw"])
            )
        else:
            return None
        return x, y, yaw
    return None


def parse_keyval_line(line: str) -> Optional[Tuple[float, float, float]]:
    pairs = {m.group("key").lower(): float(m.group("val")) for m in KEYVAL_RE.finditer(line)}
    if not pairs or "x" not in pairs or "y" not in pairs:
        return None

    x = float(pairs["x"])
    y = float(pairs["y"])
    if "yaw_rad" in pairs:
        yaw = float(pairs["yaw_rad"])
    elif "yaw" in pairs:
        yaw = float(pairs["yaw"])
    elif "theta" in pairs:
        yaw = float(pairs["theta"])
    elif all(k in pairs for k in ["qx", "qy", "qz", "qw"]):
        yaw = yaw_from_quat(pairs["qx"], pairs["qy"], pairs["qz"], pairs["qw"])
    else:
        return None
    return x, y, yaw


def parse_numeric_line(line: str, line_mode: str) -> Tuple[float, float, float]:
    vals = [float(x) for x in FLOAT_RE.findall(line)]
    if not vals:
        raise ValueError("No numeric value found in line.")

    if line_mode == "xyyaw":
        if len(vals) < 3:
            raise ValueError("xyyaw mode requires at least 3 values: x y yaw")
        return vals[0], vals[1], vals[2]

    if line_mode == "timestamp_xyyaw":
        if len(vals) < 4:
            raise ValueError("timestamp_xyyaw mode requires at least 4 values: timestamp x y yaw")
        return vals[1], vals[2], vals[3]

    if line_mode == "idx_timestamp_xyyaw":
        if len(vals) < 5:
            raise ValueError("idx_timestamp_xyyaw mode requires at least 5 values: idx timestamp x y yaw")
        return vals[2], vals[3], vals[4]

    if line_mode == "xyz_quat":
        if len(vals) < 7:
            raise ValueError("xyz_quat mode requires at least 7 values: x y z qx qy qz qw")
        return vals[0], vals[1], yaw_from_quat(vals[3], vals[4], vals[5], vals[6])

    if line_mode == "timestamp_xyz_quat":
        if len(vals) < 8:
            raise ValueError("timestamp_xyz_quat mode requires at least 8 values: timestamp x y z qx qy qz qw")
        return vals[1], vals[2], yaw_from_quat(vals[4], vals[5], vals[6], vals[7])

    if line_mode != "auto":
        raise ValueError(f"Unknown line_mode: {line_mode}")

    # Auto mode: common cases.
    if len(vals) == 3:
        # x y yaw
        return vals[0], vals[1], vals[2]

    if len(vals) == 4:
        # Most likely: timestamp x y yaw.
        # If first value is not timestamp, this still works for many logs whose first value is a sequence id.
        return vals[1], vals[2], vals[3]

    if len(vals) == 5:
        # Most likely: idx timestamp x y yaw.
        return vals[2], vals[3], vals[4]

    if len(vals) == 7:
        # x y z qx qy qz qw
        return vals[0], vals[1], yaw_from_quat(vals[3], vals[4], vals[5], vals[6])

    if len(vals) >= 8:
        # timestamp x y z qx qy qz qw, or idx x y z qx qy qz qw.
        return vals[1], vals[2], yaw_from_quat(vals[4], vals[5], vals[6], vals[7])

    raise ValueError(f"Cannot infer pose format from {len(vals)} values: {vals}")


def parse_pose_line(line: str, line_mode: str, yaw_unit: str) -> Tuple[float, float, float]:
    line = line.strip()
    if not line:
        raise ValueError("Empty line.")

    parsed = parse_json_line(line)
    if parsed is None:
        parsed = parse_keyval_line(line)
    if parsed is None:
        parsed = parse_numeric_line(line, line_mode=line_mode)

    x, y, yaw = parsed
    yaw = maybe_deg_to_rad(yaw, yaw_unit=yaw_unit)
    return float(x), float(y), float(yaw)


def convert_folder(
    input_dir: Path,
    output_csv: Path,
    pattern: str,
    line_mode: str,
    yaw_unit: str,
    output_yaw_col: str,
    timestamp_col: str,
    include_stamp: bool,
    strict: bool,
) -> pd.DataFrame:
    files = sorted(input_dir.glob(pattern))
    if not files:
        raise FileNotFoundError(f"No files found: {input_dir / pattern}")

    rows = []
    errors = []
    for txt_path in files:
        try:
            idx, timestamp = parse_filename(txt_path)
            lines = txt_path.read_text(encoding="utf-8", errors="replace").strip().splitlines()
            if len([ln for ln in lines if ln.strip()]) != 1:
                raise ValueError(f"Expected exactly one non-empty line, got {len(lines)} lines.")
            x, y, yaw = parse_pose_line(lines[0], line_mode=line_mode, yaw_unit=yaw_unit)

            row = {"idx": idx, "x": x, "y": y, output_yaw_col: yaw}
            if include_stamp:
                row = {"idx": idx, timestamp_col: timestamp, "x": x, "y": y, output_yaw_col: yaw}
            rows.append(row)
        except Exception as exc:
            msg = f"{txt_path.name}: {exc}"
            if strict:
                raise RuntimeError(msg) from exc
            errors.append(msg)

    if not rows:
        detail = "\n".join(errors[:10])
        raise RuntimeError(f"No valid pose file parsed. First errors:\n{detail}")

    df = pd.DataFrame(rows).sort_values("idx").reset_index(drop=True)

    # Validate duplicate indices because map_alignment_v3.py merges trajectories by idx.
    dup = df[df["idx"].duplicated()]["idx"].tolist()
    if dup:
        raise ValueError(f"Duplicate idx found: {dup[:20]}")

    output_csv.parent.mkdir(parents=True, exist_ok=True)
    df.to_csv(output_csv, index=False)

    print(f"[write] {output_csv}")
    print(f"[rows] {len(df)}")
    print(f"[columns] {list(df.columns)}")
    if errors:
        print(f"[warning] skipped {len(errors)} file(s). First 10 errors:")
        for e in errors[:10]:
            print(f"  - {e}")

    return df


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--input_dir", required=True, type=Path, help="Folder containing *_pose.txt files.")
    ap.add_argument("--output_csv", default=Path("./input/pose2d.csv"), type=Path)
    ap.add_argument("--pattern", default="*.txt", help="Glob pattern, e.g. '*_pose.txt'.")
    ap.add_argument(
        "--line_mode",
        default="auto",
        choices=[
            "auto",
            "xyyaw",
            "timestamp_xyyaw",
            "idx_timestamp_xyyaw",
            "xyz_quat",
            "timestamp_xyz_quat",
        ],
        help="How to interpret the single line inside each txt.",
    )
    ap.add_argument("--yaw_unit", default="rad", choices=["rad", "deg"], help="Unit of yaw if line directly stores yaw.")
    ap.add_argument(
        "--output_yaw_col",
        default="yaw_rag",
        choices=["yaw_rag", "yaw_rad", "yaw"],
        help="Yaw column name. Default: yaw_rag, as requested. Use yaw_rad for original map_alignment_v3.py.",
    )
    ap.add_argument(
        "--timestamp_col",
        default="stamp",
        choices=["stamp", "timestamp", "t_sec"],
        help="Timestamp column name. Default: stamp.",
    )
    ap.add_argument("--include_stamp", action="store_true", default=True, help="Write timestamp from filename. Default: enabled.")
    ap.add_argument("--no_stamp", dest="include_stamp", action="store_false", help="Do not write timestamp column.")
    ap.add_argument("--strict", action="store_true", help="Stop immediately when any file cannot be parsed.")
    args = ap.parse_args()

    convert_folder(
        input_dir=args.input_dir,
        output_csv=args.output_csv,
        pattern=args.pattern,
        line_mode=args.line_mode,
        yaw_unit=args.yaw_unit,
        output_yaw_col=args.output_yaw_col,
        timestamp_col=args.timestamp_col,
        include_stamp=args.include_stamp,
        strict=args.strict,
    )


if __name__ == "__main__":
    main()
