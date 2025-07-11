import pyrealsense2 as rs
import numpy as np
import cv2
import time

def main():
    # 1) Configure pipeline
    pipeline = rs.pipeline()
    cfg = rs.config()
    # Color stream: 640×480 @ 60Hz
    cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 60)
    # IMU streams: accel & gyro
    cfg.enable_stream(rs.stream.accel)
    cfg.enable_stream(rs.stream.gyro)
    
    # 2) Start streaming
    profile = pipeline.start(cfg)
    
    try:
        print("Streaming color @640×480 @ 60 Hz + IMU; press ESC to exit")
        prev = time.time()
        frame_count = 0

        while True:
            frames = pipeline.wait_for_frames()
            
            # --- Color ---
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue
            color = np.asanyarray(color_frame.get_data())
            t_color = color_frame.get_timestamp()
            
            # --- IMU ---
            accel_frame = frames.first_or_default(rs.stream.accel)
            gyro_frame  = frames.first_or_default(rs.stream.gyro)
            if not accel_frame or not gyro_frame:
                continue
            
            # extract vector objects and read x,y,z
            a = accel_frame.as_motion_frame().get_motion_data()
            g = gyro_frame.as_motion_frame().get_motion_data()
            t_accel = accel_frame.get_timestamp()
            t_gyro  = gyro_frame.get_timestamp()
            ax, ay, az = a.x, a.y, a.z
            gx, gy, gz = g.x, g.y, g.z
            
            # --- Display ---
            cv2.putText(color,
                        f'FPS: {frame_count/(time.time()-prev):.1f}',
                        (10,30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
            cv2.imshow('RGB 640x480@60', color)
            
            # --- Print timestamps & data ---
            print(f"[Color] ts={t_color:.2f} ms; "
                  f"[Accel] ts={t_accel:.2f} ms: ax,ay,az=({ax:.3f},{ay:.3f},{az:.3f}); "
                  f"[Gyro] ts={t_gyro:.2f} ms: gx,gy,gz=({gx:.3f},{gy:.3f},{gz:.3f})")
            
            frame_count += 1
            if time.time() - prev >= 1.0:
                frame_count = 0
                prev = time.time()
            
            if cv2.waitKey(1) == 27:
                break

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
