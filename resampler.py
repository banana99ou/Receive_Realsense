#!/usr/bin/env python3
"""
!this code must be at same directory as recording folder
takes csv files and Jpeg files labeled with timestamp in microseconds 
to 100Hz fixed timestep file and 100fps video file
"""
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
     # find all IMU CSV sources: RealSense + any serial‐IMU logs
    imu_sources = []
    rs_csv = os.path.join(record_dir, "imu_raw.csv")
    if os.path.isfile(rs_csv):
        imu_sources.append(("realsense", rs_csv))

    import glob
    # gather all serial CSVs (raw + rotated)
    all_serial = glob.glob(os.path.join(record_dir, "*_serial_*.csv"))

    # separate rotated vs raw
    rot_files = [p for p in all_serial if p.endswith("_Rot.csv")]
    raw_files = [p for p in all_serial if not p.endswith("_Rot.csv")]

    # find which base‐names have a rotated version
    rot_bases = {
        os.path.splitext(os.path.basename(p))[0].rsplit("_Rot", 1)[0]
        for p in rot_files
    }

    # prefer rotated; only include raw if no Rot exists
    chosen = []
    chosen.extend(rot_files)
    for p in raw_files:
        base = os.path.splitext(os.path.basename(p))[0]
        if base not in rot_bases:
            chosen.append(p)

    # finally, build imu_sources from chosen files
    for p in chosen:
        tag = os.path.basename(p).split("_")[0]
        imu_sources.append((tag, p))

    # if not imu_sources:
    #     raise RuntimeError(f"No IMU CSV files found in {record_dir}")

    print("[INFO] Loading frames...")
    frames_us, frame_paths = load_frames(record_dir, args.frame_glob)

    # process each IMU source
    for tag, path in imu_sources:
        print(f"[INFO] Loading IMU CSV ({tag}): {path}")
        df = pd.read_csv(path)

        # normalize columns
        if "t_accel_us" not in df.columns and "t_rel" in df.columns:
            df = df.rename(columns={"t_rel":"t_accel_us"})
            df["t_accel_us"] = (df["t_accel_us"].to_numpy()*1e6).round().astype(np.int64)
        # now should have a numeric t_accel_us and ax…gz

        print(f"[INFO] Resampling {tag} IMU to {args.target_hz}Hz...")
        res_df = resample_imu(df, args.target_hz)
        out_csv = os.path.join(record_dir, f"{tag}_resampled_{args.target_hz}Hz.csv")
        res_df.to_csv(out_csv, index=False)
        print(f"[INFO] → {tag} resampled CSV: {out_csv} ({len(res_df)} rows)")

    # final report
    video_span = frames_us[-1] / 1e6
    imu_spans  = ", ".join(
        f"{tag}={pd.read_csv(os.path.join(record_dir, f'{tag}_resampled_{args.target_hz}Hz.csv'))['t_sec'].iloc[-1]:.3f}s"
        for tag, _ in imu_sources
    )
    print(f"[REPORT] Video span: {video_span:.3f}s | IMU spans: {imu_spans}")

    if args.make_video:
        vid_out = os.path.join(record_dir, f"video_zoh_{args.video_rate}Hz.avi")
        print("[INFO] Building ZOH video...")
        zoh_video(frames_us, frame_paths, args.video_rate, vid_out)
        print(f"[INFO] ZOH video written: {vid_out}")

    # Simple report
    print(f"[REPORT] Raw video span: {frames_us[-1]/1e6:.3f}s | "f"Resampled IMU span: {res_df['t_sec'].iloc[-1]:.3f}s")

if __name__ == "__main__":
    main()