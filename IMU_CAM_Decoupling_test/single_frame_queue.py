import pyrealsense2 as rs
import time

q = rs.frame_queue(16)
pipe = rs.pipeline()
cfg = rs.config()
cfg.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 100)
cfg.enable_stream(rs.stream.gyro,  rs.format.motion_xyz32f, 200)
cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 60)
pipe.start(cfg, q)  # queue-based

fps = {'Accel':0, 'Gyro':0, 'Color':0}
t0  = time.time()

try:
    while True:
        frame = q.wait_for_frame()
        prof  = frame.get_profile()
        stype = prof.stream_type()
        name  = prof.stream_name()

        fps[name] += 1

        now = time.time()
        if now - t0 >= 1.0:
            print({k: fps[k]/(now-t0) for k in fps})
            t0 = now
            fps = dict.fromkeys(fps, 0)
finally:
    pipe.stop()

