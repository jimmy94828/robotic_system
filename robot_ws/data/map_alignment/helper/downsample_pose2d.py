#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Downsample pose2d.csv by taking one row every N rows.

Default behavior:
  input : ./input/pose2d.csv
  output: ./input/pose2d_downsampled.csv
  step  : 5

Example:
  python downsample_pose2d.py \
    --input_csv ./input/pose2d.csv \
    --output_csv ./input/pose2d_downsampled.csv \
    --step 5

This preserves all columns, e.g.:
  idx,stamp,x,y,yaw_rag
or:
  idx,x,y,yaw_rad
"""

from __future__ import annotations

import argparse
from pathlib import Path

import pandas as pd


def downsample_pose2d(
    input_csv: Path,
    output_csv: Path,
    step: int = 5,
    start: int = 0,
    reset_row_index: bool = True,
    reset_idx_col: bool = False,
) -> pd.DataFrame:
    if step <= 0:
        raise ValueError(f"step must be positive, got {step}")
    if start < 0:
        raise ValueError(f"start must be >= 0, got {start}")
    if not input_csv.exists():
        raise FileNotFoundError(f"Input CSV not found: {input_csv}")

    df = pd.read_csv(input_csv)
    if len(df) == 0:
        raise ValueError(f"Input CSV is empty: {input_csv}")

    # Take rows: start, start+step, start+2*step, ...
    df_down = df.iloc[start::step].copy()

    if reset_row_index:
        df_down = df_down.reset_index(drop=True)

    # Usually for map alignment you should NOT reset idx, because idx may be used
    # as the original frame id. Enable this only if you explicitly want new idx=0..N-1.
    if reset_idx_col:
        if "idx" not in df_down.columns:
            raise ValueError("Cannot reset idx column because input CSV has no 'idx' column.")
        df_down["idx"] = range(len(df_down))

    output_csv.parent.mkdir(parents=True, exist_ok=True)
    df_down.to_csv(output_csv, index=False)

    print(f"[read]  {input_csv}")
    print(f"[write] {output_csv}")
    print(f"[step]  every {step} rows, start={start}")
    print(f"[rows]  {len(df)} -> {len(df_down)}")
    print(f"[cols]  {list(df_down.columns)}")
    print(df_down.head())

    return df_down


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--input_csv", type=Path, default=Path("./input/pose2d.csv"))
    ap.add_argument("--output_csv", type=Path, default=Path("./input/pose2d_downsampled.csv"))
    ap.add_argument("--step", type=int, default=5, help="Take one row every N rows. Default: 5")
    ap.add_argument("--start", type=int, default=0, help="Start row position. Default: 0")
    ap.add_argument(
        "--no_reset_row_index",
        action="store_true",
        help="Do not reset pandas row index after downsampling. CSV output itself is unaffected unless index=True, which this script never uses.",
    )
    ap.add_argument(
        "--reset_idx_col",
        action="store_true",
        help="Reset idx column to 0..N-1. Not recommended if idx is original frame id used for matching.",
    )
    args = ap.parse_args()

    downsample_pose2d(
        input_csv=args.input_csv,
        output_csv=args.output_csv,
        step=args.step,
        start=args.start,
        reset_row_index=not args.no_reset_row_index,
        reset_idx_col=args.reset_idx_col,
    )


if __name__ == "__main__":
    main()
