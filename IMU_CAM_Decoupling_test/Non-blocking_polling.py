import pyrealsense2 as rs
import time

pipe = rs.pipeline()
cfg  = rs.config()
cfg.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 100)
cfg.enable_stream(rs.stream.gyro,  rs.format.motion_xyz32f, 200)
cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 60)
pipe.start(cfg)  # no callback → poll_for_frames OK

fps = {'ACCEL':0, 'GYRO':0, 'COLOR':0}
t0  = time.time()

try:
    while True:
        frames = pipe.poll_for_frames()
        if not frames:
            time.sleep(0.001)
            continue

        if frames.first_or_default(rs.stream.accel):
            fps['ACCEL'] += 1
        if frames.first_or_default(rs.stream.gyro):
            fps['GYRO']  += 1
        if frames.first_or_default(rs.stream.color):
            fps['COLOR'] += 1

        now = time.time()
        if now - t0 >= 1.0:
            print({k: fps[k]/(now-t0) for k in fps})
            t0 = now
            fps = dict.fromkeys(fps, 0)
finally:
    pipe.stop()

