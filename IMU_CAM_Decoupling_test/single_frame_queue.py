#!/usr/bin/env python3
import pyrealsense2 as rs
import numpy as np
import cv2
import matplotlib.pyplot as plt
from collections import deque

# — RealSense + queue setup —
IMG_W, IMG_H = 320, 240
ACC_RATE, GYRO_RATE, COLOR_RATE = 100, 200, 60

q    = rs.frame_queue(16)
pipe = rs.pipeline()
cfg  = rs.config()
cfg.enable_stream(rs.stream.color, IMG_W, IMG_H, rs.format.bgr8, COLOR_RATE)
cfg.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, ACC_RATE)
cfg.enable_stream(rs.stream.gyro,  rs.format.motion_xyz32f, GYRO_RATE)
pipe.start(cfg, q)

# — Buffers for last N accel samples —
buffer_len = 200
times  = deque(maxlen=buffer_len)
ax_buf = deque(maxlen=buffer_len)
ay_buf = deque(maxlen=buffer_len)
az_buf = deque(maxlen=buffer_len)

# — Matplotlib real-time plot setup —
plt.ion()
fig, ax = plt.subplots(figsize=(6,3))
l_ax, = ax.plot([], [], label='ax')
l_ay, = ax.plot([], [], label='ay')
l_az, = ax.plot([], [], label='az')
ax.legend(loc='upper right')
ax.set_xlabel('Time (s)')
ax.set_ylabel('Accel (m/s²)')
ax.set_ylim(-16, 16)
ax.set_xlim(0, 1)
plt.show()

t0 = None

try:
    while True:
        frame = q.wait_for_frame()   # blocks until any frame arrives
        prof  = frame.get_profile()
        stype = prof.stream_type()

        if stype == rs.stream.color:
            # — reshape raw buffer into H×W×3 image —
            raw = frame.get_data()                 # BufData
            arr = np.frombuffer(raw, dtype=np.uint8)
            img = arr.reshape((IMG_H, IMG_W, 3))
            cv2.imshow("RGB Test", img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        elif stype == rs.stream.accel:
            md = frame.as_motion_frame().get_motion_data()
            ts = frame.get_timestamp() * 1e-3      # ms→s
            if t0 is None:
                t0 = ts
            rel = ts - t0

            # times.append(rel)
            # ax_buf.append(md.x)
            # ay_buf.append(md.y)
            # az_buf.append(md.z)

            # update plot
            # l_ax.set_data(times, ax_buf)
            # l_ay.set_data(times, ay_buf)
            # l_az.set_data(times, az_buf)
            # ax.set_xlim(max(0, rel-1), rel)
            # fig.canvas.draw()
            # fig.canvas.flush_events()

        # (you can likewise handle gyro if desired)

finally:
    pipe.stop()
    cv2.destroyAllWindows()
