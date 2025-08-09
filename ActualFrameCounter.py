import os
import re
import csv
from pathlib import Path
import argparse

def extract_timestamps_from_filenames(directory, output_csv):
    # Match filenames like frame_123456789.jpg
    pattern = re.compile(r'^frame_(\d+)\.jpg$')
    timestamps = []
    ts_prev = 0
    missing_frames = 0

    for filename in os.listdir(directory):
        match = pattern.match(filename)
        if match:
            ts = int(match.group(1))
            timestamps.append(ts)

    # Sort timestamps
    timestamps.sort()

    # Save to CSV
    dir = Path(directory)
    csv_path = os.path.join(dir.parent, output_csv)
    with open(csv_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['t_us', 'timestep'])
        for ts in timestamps:
            ts_delta = (ts-ts_prev)/1000000
            if ts_delta < 0.033333:
                ts_delta = 0
            else:
                print(f"[Warning!] ⚠️ ts over 0.03 by {ts_delta/(1/60)-1:.3f}frame @ {ts/1000000}s")
                missing_frames += (ts_delta/(1/60)-1)
            writer.writerow([ts, ts_delta])
            ts_prev = ts

    print(f"Saved {len(timestamps)} timestamps to {csv_path}. missing frames: {missing_frames:.2f} @ {missing_frames/len(timestamps)*100:.3f}%")

# Example usage
if __name__ == '__main__':
    args = argparse.ArgumentParser()
    args.add_argument("--path", "-p", required=True)
    args = args.parse_args()

    # folder_path = r'C:\Users\Work\code\Receive_Realsense\recording_20250729_171045_594\frames'
    folder_path = os.path.join(args.path, "frames")
    extract_timestamps_from_filenames(folder_path, 'sorted_timestamps.csv')