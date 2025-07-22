#!/usr/bin/env python3
import argparse, os, re, glob
import numpy as np
import pandas as pd
import cv2
from tqdm import tqdm

def parse_args():
    ap = argparse.ArgumentParser()
    ap.add_argument("--path", required=True, help="Recording directory produced by RS_capture.py")
    ap.add_argument("--target_hz", type=int, default=100)
    ap.add_argument("--make_video", type=int, default=1, help="1=create resampled ZOH video, 0=skip")
    ap.add_argument("--video_rate", type=int, default=100, help="Output video FPS (ZOH)")
    ap.add_argument("--frame_glob", default="frames/frame_*.jpg")
    return ap.parse_args()

def load_frames(record_dir, frame_glob):
    pattern = os.path.join(record_dir, frame_glob)
    files = glob.glob(pattern)
    if not files:
        raise RuntimeError("No frame JPEGs found.")
    # Expect names like frame_<rel_us>.jpg
    def extract_us(path):
        m = re.search(r"frame_(\d+)\.jpg$", path)
        if not m:
            raise ValueError(f"Bad frame name: {path}")
        return int(m.group(1))
    frames = [(extract_us(f), f) for f in files]
    frames.sort()
    rel_us = np.array([t for t,_ in frames], dtype=np.int64)
    paths  = [p for _,p in frames]
    return rel_us, paths

def load_imu_csv(raw_csv_path):
    df = pd.read_csv(raw_csv_path)
    # columns: t_accel_us, t_color_us, ax..gz
    return df

def resample_imu(df, target_hz):
    imu_us = df["t_accel_us"].to_numpy(dtype=np.int64)
    if imu_us.size == 0:
        raise RuntimeError("Empty IMU data.")
    t_end_us = imu_us[-1]
    print(t_end_us)
    dt_us    = int(1_000_000 / target_hz)
    uniform_us = np.arange(0, t_end_us + 1, dt_us, dtype=np.int64)

    out = {"t_us": uniform_us, "t_sec": uniform_us / 1e6}
    channels = ["ax","ay","az","gx","gy","gz"]
    for ch in channels:
        out[ch] = np.interp(uniform_us, imu_us, df[ch].to_numpy())
    res_df = pd.DataFrame(out)
    return res_df

def zoh_video(frames_us, frame_paths, target_rate, out_path):
    dt_us = int(1_000_000 / target_rate)
    t_end = frames_us[-1]
    grid  = np.arange(0, t_end + 1, dt_us, dtype=np.int64)

    # read first frame for size
    first_img = cv2.imread(frame_paths[0])
    if first_img is None:
        raise RuntimeError("Failed to read first frame.")
    h, w = first_img.shape[:2]
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    vw = cv2.VideoWriter(out_path, fourcc, float(target_rate), (w, h), isColor=True)

    idx = 0
    for t in tqdm(grid, desc="ZOH video", unit="frm"):
        # advance while next frame timestamp <= t
        while idx + 1 < len(frames_us) and frames_us[idx+1] <= t:
            idx += 1
        img = cv2.imread(frame_paths[idx])
        if img is None:
            raise RuntimeError(f"Failed read {frame_paths[idx]}")
        vw.write(img)
    vw.release()

def main():
    args = parse_args()
    record_dir = args.path
    raw_imu_csv = os.path.join(record_dir, "imu_raw.csv")

    if not os.path.isfile(raw_imu_csv):
        raise RuntimeError(f"Cannot find IMU CSV: {raw_imu_csv}")

    print("[INFO] Loading frames...")
    frames_us, frame_paths = load_frames(record_dir, args.frame_glob)

    print("[INFO] Loading IMU CSV...")
    imu_df = load_imu_csv(raw_imu_csv)

    print("[INFO] Resampling IMU to uniform grid...")
    res_df = resample_imu(imu_df, args.target_hz)
    imu_out_csv = os.path.join(record_dir, f"imu_resampled_{args.target_hz}Hz.csv")
    res_df.to_csv(imu_out_csv, index=False)
    print(f"[INFO] IMU resampled CSV: {imu_out_csv} ({len(res_df)} rows)")

    print(f"[REPORT] Raw video span: {frames_us[-1]/1e6:.3f}s | "f"Resampled IMU span: {res_df['t_sec'].iloc[-1]:.3f}s")

    if args.make_video:
        vid_out = os.path.join(record_dir, f"video_zoh_{args.video_rate}Hz.avi")
        print("[INFO] Building ZOH video...")
        zoh_video(frames_us, frame_paths, args.video_rate, vid_out)
        print(f"[INFO] ZOH video written: {vid_out}")

    # Simple report
    print(f"[REPORT] Raw video span: {frames_us[-1]/1e6:.3f}s | "f"Resampled IMU span: {res_df['t_sec'].iloc[-1]:.3f}s")

if __name__ == "__main__":
    main()