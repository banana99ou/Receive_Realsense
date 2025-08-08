#!/usr/bin/env python3
"""
Overlay ax/ay/az from every CSV in one folder, with a clickable legend
to toggle each run on/off.  Understands these layouts:

1) t_rel,ax,ay,az,gx,gy,gz,grav_x,grav_y,grav_z,crc
2) t_sec or t_s,ax,ay,az,gx,gy,gz
3) t_us (µs),... or t_accel_us,ax,ay,az,gx,gy,gz  (convert to seconds)
4) t_color_us (µs) if accel-only timestamp missing
"""

import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path
import argparse

# ── CONFIGURE HERE ───────────────────────────────────────────────────────────
FOLDER    = Path("./Experiment data/0805/recording_20250805_104117_893")
ACC_COLS  = ["ax", "ay", "az", "gx", "gy", "gz"]   # which accelerometer channels to draw
FIGSIZE   = (12, 8)
# ─────────────────────────────────────────────────────────────────────────────


def extract_time_series(df: pd.DataFrame) -> pd.Series:
    """
    Return a time Series in *seconds* starting at 0 for the file.
    Priority:
      1) t_rel (already zero-based)
      2) t_sec or t_s  (seconds, normalize to zero)
      3) t_us or t_accel_us (microseconds → seconds, normalize)
      4) t_color_us (microseconds → seconds, normalize)
    """
    if "t_rel" in df.columns:
        return df["t_rel"].astype(float)

    for col in ("t_sec", "t_s"):
        if col in df.columns:
            ts = df[col].astype(float)
            return ts - ts.iloc[0]

    for col in ("t_us", "t_accel_us"):
        if col in df.columns:
            ts = df[col].astype(float) / 1e6
            return ts - ts.iloc[0]

    if "t_color_us" in df.columns:
        ts = df["t_color_us"].astype(float) / 1e6
        return ts - ts.iloc[0]

    raise KeyError(
        "No recognized time column (t_rel, t_sec, t_s, t_us, "
        "t_accel_us, or t_color_us) in CSV."
    )


def main() -> None:
    args = argparse.ArgumentParser()
    args.add_argument("--path")
    args= args.parse_args()

    FOLDER = Path(args.path)

    csv_files = sorted(FOLDER.glob("*.csv"))
    if not csv_files:
        raise FileNotFoundError(f"No CSV files found in {FOLDER}")

    # one subplot per axis, shared X axis
    fig, axes = plt.subplots(
        nrows=len(ACC_COLS),
        ncols=1,
        sharex=True,
        figsize=FIGSIZE,
        layout="constrained"
    )

    run_lines: dict[str, list] = {}

    for csv_path in csv_files:
        df   = pd.read_csv(csv_path)
        time = extract_time_series(df)

        label = csv_path.stem
        run_lines[label] = []

        for ax, col in zip(axes, ACC_COLS):
            if col not in df.columns:
                print(f"⚠️  Column '{col}' missing in {csv_path.name}; skipping.")
                continue
            line, = ax.plot(time, df[col].astype(float), label=label, lw=1.0)
            run_lines[label].append(line)

    # cosmetics
    for ax, col in zip(axes, ACC_COLS):
        ax.set_ylabel(col)
        ax.grid(True, alpha=0.3)
    axes[0].set_title(f"Accelerometer comparison across {len(csv_files)} run(s)")
    axes[-1].set_xlabel("Time [s]")

    # clickable legend
    handles = [run_lines[name][0] for name in run_lines if run_lines[name]]
    labels  = [name for name in run_lines if run_lines[name]]
    leg = axes[0].legend(
        handles, labels,
        loc="upper left", bbox_to_anchor=(1.02, 1.0),
        title="Click to toggle", frameon=False
    )

    for legline in leg.get_lines():
        legline.set_picker(True)
        legline.set_pickradius(5)

    legend_map = {
        legline: run_lines[label]
        for legline, label in zip(leg.get_lines(), labels)
    }

    def on_pick(event):
        legline = event.artist
        lines   = legend_map[legline]
        visible = not lines[0].get_visible()
        for ln in lines:
            ln.set_visible(visible)
        legline.set_alpha(1.0 if visible else 0.2)
        fig.canvas.draw_idle()

    fig.canvas.mpl_connect("pick_event", on_pick)
    plt.show()


if __name__ == "__main__":
    main()
