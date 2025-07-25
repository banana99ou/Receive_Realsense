#!/usr/bin/env python3
import threading
import time
import struct
import socket
import datetime
import csv
import os
import sys
import subprocess
import numpy as np
import cv2
import pyrealsense2 as rs

# ================= User Config =================
SIMULINK_IP = "127.0.0.1"
IMU_PORT    = 5005
CAM_PORT    = 5006
IMG_W, IMG_H = 320, 240
JPEG_QUALITY = 90
IMU_RATE_REQ = 100   # requested accel Hz
GYRO_RATE_REQ= 200
COLOR_RATE   = 60
UDP_IMU_RATE = 100.0
UDP_CAM_RATE = 60.0
# =================================================

class SharedPacket:
    """Holds the latest sensor packet with a lock."""
    def __init__(self):
        self.lock = threading.Lock()
        self.data = None
        # data = (t_accel_us, t_enqueue, imu_dict, jpeg_bytes, t_color_us)

class SimulinkUDPSender(threading.Thread):
    """
    Reads from shared_packet.data and sends either IMU or camera
    over UDP at a given rate.
    """
    def __init__(self, stream_type, shared, ip, port, sample_rate, udp_chunk_size=8192):
        super().__init__(daemon=True)
        assert stream_type in ("imu", "cam")
        self.stream = stream_type
        self.shared = shared
        self.ip, self.port = ip, port
        self.period = 1.0 / sample_rate
        self.chunk  = udp_chunk_size
        self.sock   = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def run(self):
        while True:
            with self.shared.lock:
                pkt = self.shared.data
            if not pkt:
                time.sleep(0.001)
                continue
            rel_accel_us, t_enqueue, imu, jpeg, rel_color_us = pkt

            if self.stream == "imu":
                # pack just the 6 floats
                payload = struct.pack(
                    "<6f",
                    imu["ax"], imu["ay"], imu["az"],
                    imu["gx"], imu["gy"], imu["gz"]
                )
                self.sock.sendto(payload, (self.ip, self.port))

            else:  # camera
                # skip if jpeg is missing
                if not jpeg:
                    time.sleep(self.period)
                    continue

                # chunked grayscale transport
                arr = np.frombuffer(jpeg, dtype=np.uint8)
                img = cv2.imdecode(arr, cv2.IMREAD_GRAYSCALE)
                img = cv2.resize(img, (IMG_W, IMG_H))
                frame_bytes = img.T.tobytes()
                total = len(frame_bytes)
                n_chunks = (total + self.chunk - 1) // self.chunk
                frame_id = rel_color_us & 0xFFFFFFFF
                for chunkID in range(n_chunks):
                    offset   = chunkID * self.chunk
                    chunk = frame_bytes[offset:offset+self.chunk]
                    header   = struct.pack("<IHH", frame_id, chunkID, n_chunks)
                    self.sock.sendto(header + chunk, (self.ip, self.port))
            
            time.sleep(self.period)

def ensure_dir(p):
    os.makedirs(p, exist_ok=True)
    return p

def main():
    # ----- Output directory -----
    session_ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
    record_dir = f"record_{session_ts}"
    frames_dir = ensure_dir(os.path.join(record_dir, "frames"))
    raw_imu_csv_path = os.path.join(record_dir, "imu_raw.csv")
    print(f"[INFO] Recording into: {record_dir}")

    # ----- RealSense setup (single queue) -----
    q    = rs.frame_queue(32)
    pipe = rs.pipeline()
    cfg  = rs.config()
    cfg.enable_stream(rs.stream.color, IMG_W, IMG_H, rs.format.bgr8, COLOR_RATE)
    cfg.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, IMU_RATE_REQ)
    cfg.enable_stream(rs.stream.gyro,  rs.format.motion_xyz32f, GYRO_RATE_REQ)
    pipe.start(cfg, q)

    # 2) Prepare files
    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
    csv_file   = open(raw_imu_csv_path, "w", newline='')
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow([
        't_accel_us','t_color_us',
        'ax','ay','az','gx','gy','gz'
    ])

    # add counters for video frames
    video_frame_count = 0
    first_video_time = None
    last_video_time  = None
    # ----- Shared + UDP senders -----
    shared     = SharedPacket()
    imu_sender = SimulinkUDPSender("imu", shared, SIMULINK_IP, IMU_PORT, UDP_IMU_RATE)
    cam_sender = SimulinkUDPSender("cam", shared, SIMULINK_IP, CAM_PORT, UDP_CAM_RATE)
    imu_sender.start()
    cam_sender.start()

    # ----- IMU buffering (faster than per-line CSV writes) -----
    imu_rows = []  # each: (rel_accel_us, rel_color_us, ax, ay, az, gx, gy, gz)

    t0_color = None
    t0_accel = None

    last_gyro = {"gx":0.0, "gy":0.0, "gz":0.0}
    last_jpeg = b""
    last_color_us = None

    frame_count = 0
    last_fps_print = time.time()
    fps_counter = 0

    try:
        while True:
            frame = q.wait_for_frame()
            prof  = frame.get_profile()
            Stream_Type = prof.stream_type()

            # —— Color frame —— #
            if Stream_Type == rs.stream.color:
                ts_ms = frame.get_timestamp()  # ms
                if t0_color is None:
                    t0_color = ts_ms
                    # enforce accel zero alignment on first color frame:
                    if t0_accel is None:
                        t0_accel = None  # force accel branch to set when first accel after color arrives
                rel_color_us = int((ts_ms - t0_color) * 1000)

                raw = frame.get_data()
                arr = np.frombuffer(raw, dtype=np.uint8)
                img = arr.reshape((IMG_H, IMG_W, 3))
                # Save JPEG frame with timestamp in filename
                fname = os.path.join(frames_dir, f"frame_{rel_color_us}.jpg")
                cv2.imwrite(fname, img, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])

                # Prepare grayscale JPEG for UDP sender (compress once)
                gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                ok, enc = cv2.imencode(".jpg", gray, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
                if ok:
                    last_jpeg = enc.tobytes()
                    last_color_us = rel_color_us

                # Simple preview
                fps_counter += 1
                now = time.time()
                if now - last_fps_print >= 1.0:
                    cv2.putText(img, f"FPS~{fps_counter/(now-last_fps_print):.1f}", (5,25), cv2.FONT_HERSHEY_SIMPLEX, 0.7,(0,255,0),2)
                    last_fps_print = now
                    fps_counter = 0
                cv2.imshow("RGB", img)
                if (cv2.waitKey(1) & 0xFF) == ord('q'):
                    break
                frame_count += 1

            elif Stream_Type == rs.stream.accel:
                ts_ms = frame.get_timestamp()
                if last_jpeg is not None and t0_accel is None:
                    t0_accel = ts_ms
                rel_accel_us = int((ts_ms - t0_accel) * 1000)

                if not last_color_us or not last_jpeg:
                    continue  # wait until we have at least one frame

                md = frame.as_motion_frame().get_motion_data()
                imu = {
                    "ax": md.x, "ay": md.y, "az": md.z,
                    **last_gyro
                }
                t_enqueue = time.perf_counter()

                # Update shared packet for UDP threads
                with shared.lock:
                    shared.data = (rel_accel_us, t_enqueue, imu, last_jpeg, last_color_us)

                # write CSV
                csv_writer.writerow((
                    rel_accel_us, last_color_us,
                    imu["ax"], imu["ay"], imu["az"],
                    imu["gx"], imu["gy"], imu["gz"]
                ))

            elif Stream_Type == rs.stream.gyro:
                md = frame.as_motion_frame().get_motion_data()
                last_gyro = {"gx": md.x, "gy": md.y, "gz": md.z}

    except KeyboardInterrupt:
        pass

    finally:
        pipe.stop()
        cv2.destroyAllWindows()

        # ----- Auto-run resampler -----
        try:
            print("[INFO] Running resampler...")
            subprocess.run([sys.executable, "resampler.py",
                            "--path", record_dir,
                            "--target_hz", "100",
                            "--make_video", "1",
                            "--video_rate", "100"],
                           check=True)
        except Exception as e:
            print(f"[WARN] Resampler failed: {e}")

        print("[INFO] Done.")

if __name__ == "__main__":
    main()