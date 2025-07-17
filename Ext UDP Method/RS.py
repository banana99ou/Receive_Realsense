#!/usr/bin/env python3
import threading
import time
import math
import struct
import socket
import datetime
import csv

import numpy as np
import cv2
import pyrealsense2 as rs

# —————— Your Simulink target IP/ports ——————
SIMULINK_IP    = "127.0.0.1"
IMU_PORT       = 5005
CAM_PORT       = 5006
# ————————————————————————————————————————

IMG_W = 320#640
IMG_H = 240#480

class SharedPacket:
    """Holds the latest sensor packet with a lock."""
    def __init__(self):
        self.lock = threading.Lock()
        self.data = None  # (t_res_us, t_enqueue, imu_dict, jpeg_bytes, t_color_us)

class SimulinkUDPSender(threading.Thread):
    """
    Reads from shared_packet.data and sends either IMU or camera
    over UDP at a given rate.
    """
    def __init__(self, stream_type, shared, ip, port,
                 sample_rate=100.0, udp_chunk_size=9600):#8192):
        super().__init__(daemon=True)
        assert stream_type in ("imu", "cam")
        self.stream = stream_type
        self.shared = shared
        self.ip, self.port = ip, port
        self.rate = sample_rate
        self.chunk = udp_chunk_size
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # IMU pack: [t_res_s(double), t_enqueue(double), t_send(double),
        #            ax,ay,az,gx,gy,gz] as 9 floats
        if self.stream == "cam":
            # we'll send at 25 Hz by default, chunked JPEG
            self.rate = 60

    def run(self):
        period = 1.0/self.rate
        next_ts = time.monotonic()
        frame_id = 0

        while True:
            with self.shared.lock:
                pkt = self.shared.data
            if pkt is None:
                time.sleep(0.001)
                continue

            if self.stream == "imu":
                t_res_us, t_enqueue, imu, _, _ = pkt
                t_res_s = t_res_us * 1e-6
                t_send = time.perf_counter()
                payload = struct.pack(
                    "<6f",#"<2d d 6f",
                    # t_res_s, t_enqueue,
                    # t_send,
                    imu["ax"], imu["ay"], imu["az"],
                    imu["gx"], imu["gy"], imu["gz"]
                )
                self.sock.sendto(payload, (self.ip, self.port))
                tmp = imu["ax"]
                print(f"{tmp} {len(payload)} {next_ts}")

            else:  # camera
                # unpack the JPEG we got
                _, _, _, jpeg_bytes, _ = pkt

                # 1) decode JPEG → grayscale
                arr = np.frombuffer(jpeg_bytes, dtype=np.uint8)
                img = cv2.imdecode(arr, cv2.IMREAD_GRAYSCALE)

                # 2) resize to exactly what the S-Function wants:
                #    note: cv2.resize takes (width, height)
                img = cv2.resize(img, (IMG_W, IMG_H))   # e.g. (320,240)

                # 3) flatten to raw bytes
                frame_bytes = img.T.tobytes()

                # 4) chunk exactly as before, but now with raw pixels
                total    = len(frame_bytes)  # should be IMG_W*IMG_H
                n_chunks = math.ceil(total/self.chunk)
                for cid in range(n_chunks):
                    off   = cid*self.chunk
                    chunk = frame_bytes[off:off+self.chunk]
                    hdr   = struct.pack("<IHH", frame_id, cid, n_chunks)
                    self.sock.sendto(hdr + chunk, (self.ip, self.port))
                frame_id = (frame_id+1) & 0xFFFFFFFF


def main():
    # 1) Setup RealSense
    pipe = rs.pipeline()
    cfg  = rs.config()
    cfg.enable_stream(rs.stream.color, IMG_W, IMG_H, rs.format.bgr8, 60)
    cfg.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 100)
    cfg.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 200)
    pipe.start(cfg)

    # 2) Prepare file names with start timestamp (down to milliseconds)
    start_time = datetime.datetime.now()
    ts_str = start_time.strftime("%Y%m%d_%H%M%S_%f")[:-3]  # e.g. "20250717_161234_123"
    csv_path   = f"data_{ts_str}.csv"
    video_path = f"video_{ts_str}.avi"

    # 3) Open CSV and write header
    csv_file   = open(csv_path, mode='w', newline='')
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow([
        't_accel_us', 't_color_us',
        'ax','ay','az','gx','gy','gz'
    ])

    # 4) Open VideoWriter (XVID @ 60 FPS, BGR color)
    fourcc      = cv2.VideoWriter_fourcc(*'XVID')
    video_writer = cv2.VideoWriter(
        video_path, fourcc, 60.0,
        (IMG_W, IMG_H), isColor=True
    )

    shared = SharedPacket()
    # start sender threads
    imu_sender = SimulinkUDPSender("imu", shared, SIMULINK_IP, IMU_PORT, sample_rate=60.0)
    cam_sender = SimulinkUDPSender("cam", shared, SIMULINK_IP, CAM_PORT, sample_rate=25.0)
    imu_sender.start()
    cam_sender.start()

    try:
        print(f"Running.CTRL+C to exit. Logging to {csv_path} & {video_path}")
        start_t = time.time()
        frame_ct = 0

        t_color_0 = None
        t_accel_0 = None

        while True:
            frames = pipe.wait_for_frames()

            # color
            cf = frames.get_color_frame()
            if not cf:
                continue
            color = np.asanyarray(cf.get_data())
            t_color = cf.get_timestamp()   # to ms
            # encode to JPEG
            ok, enc = cv2.imencode('.jpg', color, [cv2.IMWRITE_JPEG_QUALITY, 90])
            if not ok:
                continue
            jpeg = enc.tobytes()

            # imu
            af = frames.first_or_default(rs.stream.accel)
            gf = frames.first_or_default(rs.stream.gyro)
            if not af or not gf:
                continue
            a = af.as_motion_frame().get_motion_data()
            g = gf.as_motion_frame().get_motion_data()
            imu = {
                "ax": a.x, "ay": a.y, "az": a.z,
                "gx": g.x, "gy": g.y, "gz": g.z
            }
            t_accel = af.get_timestamp() # to ms
            # use t_color for camera timestamp, but you can also use t_accel if desired
            t_enqueue = time.perf_counter()

            if t_color_0 is None:
                t_color_0 = t_color
                t_accel_0 = t_accel

            # update shared packet
            with shared.lock:
                shared.data = (
                    t_accel,      # t_res_us (we use accel timestamp)
                    t_enqueue,    # enqueue time (perf_counter)
                    imu,
                    jpeg,
                    t_color       # raw color timestamp μs
                )

            # --- write to CSV: use sensor timestamps ---
            csv_writer.writerow([
                int(t_accel - t_accel_0), int(t_color - t_color_0),
                imu["ax"], imu["ay"], imu["az"],
                imu["gx"], imu["gy"], imu["gz"]
            ])

            # --- write frame to video (BGR color) ---
            # Note: VideoWriter expects (width,height) same as IMG_W,IMG_H
            #       if your capture is larger, resize first.
            small = cv2.resize(color, (IMG_W, IMG_H))
            video_writer.write(small)

            # (optional) display
            cv2.putText(color, f"FPS {frame_ct/(time.time()-start_t):.1f}",
                        (10,30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0),2)
            cv2.imshow("D435i RGB@60", color)
            frame_ct += 1
            if time.time() - start_t >= 1.0:
                frame_ct = 0
                start_t = time.time()

            if cv2.waitKey(1) == 27:
                break

    except KeyboardInterrupt:
        pass
    finally:
        # clean up
        pipe.stop()
        cv2.destroyAllWindows()
        csv_file.close()
        video_writer.release()
        print("Exiting.")

if __name__ == "__main__":
    main()