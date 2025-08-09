#!/usr/bin/env python3
"""
flip_resampled_handedness.py

Convert resampled CSVs from right-handed to left-handed coordinates by mirroring one axis.
Targets files directly inside each recording folder:
  - CentC_resampled_*Hz.csv
  - HeadR_resampled_*Hz.csv
  - realsense_resampled_*Hz.csv

Skips 'Raw/' and videos. Columns expected (if present): t_us, t_sec, ax, ay, az, gx, gy, gz.
Polar vectors (a*, v*): v' = S v
Axial vectors (g*, m*): w' = -S w     (correct for reflection)

Usage:
  python flip_resampled_handedness.py --root "." --axis y --suffix "_LH"
  # overwrite in-place:
  python flip_resampled_handedness.py --root "." --axis y --inplace
"""
import os
import re
import argparse
import pandas as pd

REC_DIR_RE = re.compile(r"^recording_\d{8}_\d{6}_.+")
RESAMP_RE  = re.compile(r"^(CentC|HeadR|realsense)_resampled_\d+Hz\.csv$", re.IGNORECASE)

def scales_for_axis(axis: str):
    axis = axis.lower()
    if axis == "x":
        polar = (-1,  1,  1)
        axial = ( 1, -1, -1)
    elif axis == "y":
        polar = ( 1, -1,  1)
        axial = ( 1, -1,  1)
    elif axis == "z":
        polar = ( 1,  1, -1)
        axial = (-1, -1,  1)
    else:
        raise ValueError("axis must be one of: x, y, z")
    return polar, axial

def flip_df(df: pd.DataFrame, polar_scale, axial_scale) -> bool:
    changed = False
    # Polar
    if all(c in df.columns for c in ("ax","ay","az")):
        df["ax"] *= polar_scale[0]
        df["ay"] *= polar_scale[1]
        df["az"] *= polar_scale[2]
        changed = True
    if all(c in df.columns for c in ("vx","vy","vz")):
        df["vx"] *= polar_scale[0]
        df["vy"] *= polar_scale[1]
        df["vz"] *= polar_scale[2]
        changed = True
    # Axial
    if all(c in df.columns for c in ("gx","gy","gz")):
        df["gx"] *= axial_scale[0]
        df["gy"] *= axial_scale[1]
        df["gz"] *= axial_scale[2]
        changed = True
    if all(c in df.columns for c in ("mx","my","mz")):
        df["mx"] *= axial_scale[0]
        df["my"] *= axial_scale[1]
        df["mz"] *= axial_scale[2]
        changed = True
    return changed

def process_file(path, axis, inplace=False, suffix="_LH"):
    df = pd.read_csv(path)
    polar_scale, axial_scale = scales_for_axis(axis)
    changed = flip_df(df, polar_scale, axial_scale)
    if not changed:
        return False, None
    out_path = path if inplace else f"{os.path.splitext(path)[0]}{suffix}.csv"
    df.to_csv(out_path, index=False)
    return True, out_path

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", required=True, help="Root containing date folders like 0804, 0805")
    ap.add_argument("--axis", default="y", choices=["x","y","z"], help="Axis to mirror (default: y)")
    ap.add_argument("--inplace", action="store_true", help="Overwrite files instead of writing a suffixed copy")
    ap.add_argument("--suffix", default="_LH", help="Suffix when not using --inplace (default: _LH)")
    args = ap.parse_args()

    total, converted = 0, 0
    root = os.path.abspath(args.root)

    # Walk date folders (e.g., 0804, 0805)
    for d1 in sorted(os.listdir(root)):
        date_dir = os.path.join(root, d1)
        if not os.path.isdir(date_dir):
            continue
        # Only dive one level: recording_* folders
        for d2 in sorted(os.listdir(date_dir)):
            rec_dir = os.path.join(date_dir, d2)
            if not (os.path.isdir(rec_dir) and REC_DIR_RE.match(d2)):
                continue
            # Process resampled CSVs directly in the recording folder
            for fname in sorted(os.listdir(rec_dir)):
                if not fname.lower().endswith(".csv"):
                    continue
                if not RESAMP_RE.match(fname):
                    continue
                # Skip anything inside Raw/ (we are only looking at top level anyway)
                src = os.path.join(rec_dir, fname)
                total += 1
                try:
                    did, outp = process_file(src, args.axis, inplace=args.inplace, suffix=args.suffix)
                    if did:
                        converted += 1
                        print(f"[OK] {src} -> {outp}")
                    else:
                        print(f"[SKIP] {src}: required columns not found")
                except Exception as e:
                    print(f"[WARN] {src}: {e}")

    print(f"\nDone. Checked {total} resampled CSVs, converted {converted}.")

if __name__ == "__main__":
    main()
