#!/usr/bin/env python3
"""
postprocess_imu.py

1. Reads raw IMU CSV (auto-detected in --path).
2. Bias‐corrects accelerations (removes the pre‐calibrated resting vector).
3. Rotates IMU→Car frame.
4. (Optional) fine calibrates so gravity aligns to (0,0,-1).
5. Converts units to m/s² and rad/s.
6. Writes out the result with “_Rot” suffix.
"""

import os
import sys
import glob
import argparse
import subprocess
import numpy as np
import pandas as pd

# Number of initial samples to average for bias removal (unused in this fixed-bias example)
BIAS_SAMPLES = 10  


def compute_calib_rotation(v0, v_target=(0, 0, -1), eps=1e-6):
    """
    Compute the rotation matrix that maps v0 to v_target using Rodrigues' formula.
    If v0 is already (anti-)parallel to v_target, returns identity.
    """
    v0_u = v0 / np.linalg.norm(v0)
    vt_u = np.array(v_target) / np.linalg.norm(v_target)

    axis = np.cross(v0_u, vt_u)
    axis_norm = np.linalg.norm(axis)
    if axis_norm < eps:
        return np.eye(3)

    axis /= axis_norm
    phi = np.arccos(np.clip(np.dot(v0_u, vt_u), -1.0, 1.0))
    ux, uy, uz = axis
    K = np.array([
        [ 0,  -uz,  uy],
        [ uz,   0, -ux],
        [-uy,  ux,   0]
    ])
    R = (
        np.eye(3) * np.cos(phi)
        + (1 - np.cos(phi)) * np.outer(axis, axis)
        + K * np.sin(phi)
    )
    return R


def main():
    parser = argparse.ArgumentParser(description="Post-process IMU CSV in a given folder (HeadR_serial_*.csv → _Rot.csv)")
    parser.add_argument("--path", "-p", required=True)
    parser.add_argument("--show", "-s", action='store_true')
    args = parser.parse_args()
    folder = args.path

    # find the first matching CSV
    pattern = os.path.join(folder, "HeadR_serial_*.csv")
    files = sorted(glob.glob(pattern))
    if not files:
        raise SystemExit(f"No matching CSV files found in {folder!r}")
    input_csv = files[0]

    # build output filename
    base = os.path.basename(input_csv)
    name, ext = os.path.splitext(base)
    output_csv = os.path.join(folder, f"{name}_Rot{ext}")

    # 1. Load
    df = pd.read_csv(input_csv)

    # 2. Bias removal (fixed bias here; to use dynamic, replace with df[['ax','ay','az']].iloc[:BIAS_SAMPLES].mean())
    bias = np.array([0.0, 10.0, -10.0])
    acc_raw = df[['ax','ay','az']].to_numpy()
    acc_bc  = acc_raw - bias

    # 3. IMU→Car rotation
    # IMU X→left = –Y_car;   Y→up =  Z_car;   Z→back = –X_car
    R_imu2car = np.array([
        [ 0,  0, -1],
        [-1,  0,  0],
        [ 0,  1,  0],
    ])
    gyro_raw = df[['gx','gy','gz']].to_numpy()

    acc_car  = acc_bc   @ R_imu2car.T
    gyro_car = gyro_raw @ R_imu2car.T

    # 4. (Optional) fine‐tune so gravity points to (0,0,-1)
    # Uncomment the next block if you want a dynamic calibration:
    # first_acc = acc_car[0]
    # R_calib   = compute_calib_rotation(first_acc)
    # acc_car   = acc_car   @ R_calib.T
    # gyro_car  = gyro_car  @ R_calib.T

    # 5. Unit conversion
    # acc_car  *= 9.80665          # g → m/s²
    # gyro_car *= np.pi / 180.0    # °/s → rad/s

    # 6. Save
    df[['ax','ay','az']] = acc_car
    df[['gx','gy','gz']] = gyro_car
    df.to_csv(output_csv, index=False)

    print(f"Done! Processed data written to: {output_csv}")
    if args.show:
        print(f"plotting")
        subprocess.run([sys.executable, "show.py", "-p", folder])

if __name__ == "__main__":
    main()
