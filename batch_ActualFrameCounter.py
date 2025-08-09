#!/usr/bin/env python3
"""
batch_process.py

Recursively finds all `recording_YYYYMMDD_HHMMSS_*` folders under a given root,
runs `rotate-idk.py -p <record_dir>` first, then calls `resampler.py` for each.

Usage:
    python batch_process.py [--root /path/to/data]
"""
import os
import sys
import argparse
import subprocess

def main():
    parser = argparse.ArgumentParser(description="Batch run rotate-idk.py and resampler.py")
    parser.add_argument(
        "--path", "-p",
        default=".",
        help="Root directory containing date folders (e.g. '0804', '0805', …). Default: current directory."
    )
    args = parser.parse_args()
    root_dir = os.path.abspath(args.path)

    for date_folder in os.listdir(root_dir):
        date_path = os.path.join(root_dir, date_folder)
        if not os.path.isdir(date_path):
            continue

        for entry in os.listdir(date_path):
            if not entry.startswith("recording_"):
                continue
            record_dir = os.path.join(date_path, entry)
            if not os.path.isdir(record_dir):
                continue

            print(f"\n=== Processing {record_dir} ===")

            # 1) Run idk.py
            print("→ Running idk.py …")
            subprocess.run(
                [sys.executable, "ActualFrameCounter.py", "-p", record_dir],
                check=True
            )

    print("\nAll recordings processed.")

if __name__ == "__main__":
    main()
