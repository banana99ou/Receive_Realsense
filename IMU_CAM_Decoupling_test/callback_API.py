import pyrealsense2 as rs
import time

# 1) prepare counters
fps = {'Accel': 0, 'Gyro': 0, 'Color': 0}
t0  = time.time()

def frame_callback(frame):
    global t0, fps
    prof = frame.get_profile()
    name = prof.stream_name()       # "Accel", "Gyro" or "Color"
    fps[name] += 1

    now = time.time()
    if now - t0 >= 1.0:
        # print and reset every second
        print({k: fps[k]/(now-t0) for k in fps})
        t0 = now
        fps = dict.fromkeys(fps, 0)

pipe = rs.pipeline()
cfg  = rs.config()
cfg.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 100)
cfg.enable_stream(rs.stream.gyro,  rs.format.motion_xyz32f, 200)
cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 60)
pipe.start(cfg, frame_callback)

try:
    while True:
        time.sleep(1)
finally:
    pipe.stop()

