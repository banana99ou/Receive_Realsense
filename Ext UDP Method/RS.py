#!/usr/bin/env python3
import threading
import time
import struct
import socket
import datetime
import csv

import numpy as np
import cv2
import pyrealsense2 as rs

# —————— Your Simulink target IP/ports ——————
SIMULINK_IP = "127.0.0.1"
IMU_PORT    = 5005
CAM_PORT    = 5006
# ————————————————————————————————————————

IMG_W, IMG_H = 320, 240

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
    def __init__(self, stream_type, shared, ip, port,
                 sample_rate=100.0, udp_chunk_size=8192):
        super().__init__(daemon=True)
        assert stream_type in ("imu", "cam")
        self.stream = stream_type
        self.shared = shared
        self.ip, self.port = ip, port
        self.rate  = sample_rate
        self.chunk = udp_chunk_size
        self.sock  = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        if self.stream == "cam":
            self.rate = 60

    def run(self):
        period = 1.0 / self.rate
        while True:
            with self.shared.lock:
                pkt = self.shared.data
            if pkt is None:
                time.sleep(0.001)
                continue

            t_accel_us, t_enqueue, imu, jpeg, t_color_us = pkt

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
                    time.sleep(period)
                    continue

                # chunked grayscale transport
                arr = np.frombuffer(jpeg, dtype=np.uint8)
                img = cv2.imdecode(arr, cv2.IMREAD_GRAYSCALE)
                img = cv2.resize(img, (IMG_W, IMG_H))
                frame_bytes = img.T.tobytes()

                total    = len(frame_bytes)
                n_chunks = (total + self.chunk - 1)//self.chunk
                frame_id = int(t_color_us) & 0xFFFFFFFF

                for cid in range(n_chunks):
                    off   = cid*self.chunk
                    chunk = frame_bytes[off:off+self.chunk]
                    hdr   = struct.pack("<IHH", frame_id, cid, n_chunks)
                    self.sock.sendto(hdr + chunk, (self.ip, self.port))

            time.sleep(period)


def main():
    # 1) RealSense + single queue
    q    = rs.frame_queue(16)
    pipe = rs.pipeline()
    cfg  = rs.config()
    cfg.enable_stream(rs.stream.color, IMG_W, IMG_H, rs.format.bgr8, 60)
    cfg.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 100)
    cfg.enable_stream(rs.stream.gyro,  rs.format.motion_xyz32f, 200)
    pipe.start(cfg, q)

    # 2) Prepare files
    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
    csv_file   = open(f"data_{ts}.csv",   "w", newline='')
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow([
        't_accel_us','t_color_us',
        'ax','ay','az','gx','gy','gz'
    ])

    fourcc       = cv2.VideoWriter_fourcc(*'XVID')
    video_writer = cv2.VideoWriter(
        f"video_{ts}.avi", fourcc, 60.0,
        (IMG_W, IMG_H), isColor=True
    )
    # add counters for video frames
    video_frame_count = 0
    first_video_time = None
    last_video_time  = None

    # 3) SharedPacket + sender threads
    shared     = SharedPacket()
    imu_sender = SimulinkUDPSender("imu", shared, SIMULINK_IP, IMU_PORT, sample_rate=100.0)
    cam_sender = SimulinkUDPSender("cam", shared, SIMULINK_IP, CAM_PORT, sample_rate=60.0)
    imu_sender.start()
    cam_sender.start()

    # 4) State for zeroing & buffering
    t0_accel  = None
    t0_color  = None
    last_gyro = {"gx":0.0, "gy":0.0, "gz":0.0}
    last_jpeg = None
    last_c_us = None

    try:
        while True:
            frame = q.wait_for_frame()
            prof  = frame.get_profile()
            stype = prof.stream_type()

            # —— Color frame —— #
            if stype == rs.stream.color:
                raw = frame.get_data()
                arr = np.frombuffer(raw, dtype=np.uint8)
                img = arr.reshape((IMG_H, IMG_W, 3))

                ts_c = frame.get_timestamp()  # ms
                if t0_color is None:
                    t0_color = ts_c
                rel_c_us = int((ts_c - t0_color) * 1000)

                # prepare for UDP
                gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                ok, enc = cv2.imencode('.jpg', gray, [cv2.IMWRITE_JPEG_QUALITY, 90])
                if ok:
                    last_jpeg = enc.tobytes()
                    last_c_us = rel_c_us

                # count this frame
                now = time.time()
                if first_video_time is None:
                    first_video_time = now
                video_frame_count += 1
                last_video_time = now

                # write video
                video_writer.write(img)

                # optional display
                cv2.imshow("RGB", img)
                if cv2.waitKey(1) & 0xFF == 27:
                    break

            # —— Accel frame —— #
            elif stype == rs.stream.accel:
                md = frame.as_motion_frame().get_motion_data()
                ts_a = frame.get_timestamp()  # ms
                # if t0_accel is None:
                #     t0_accel = ts_a
                if last_jpeg is not None and t0_accel is None:
                    t0_accel = ts_a

                # skip until we have at least one JPEG+color-ts
                if last_jpeg is None or last_c_us is None:
                    continue
                
                rel_a_us = int((ts_a - t0_accel) * 1000)

                # build imu dict including last gyro
                imu = {
                    "ax": md.x, "ay": md.y, "az": md.z,
                    **last_gyro
                }
                t_enqueue = time.perf_counter()

                # update shared packet
                with shared.lock:
                    shared.data = (
                        rel_a_us, t_enqueue, imu,
                        last_jpeg, last_c_us
                    )

                # write CSV
                csv_writer.writerow([
                    rel_a_us, last_c_us,
                    imu["ax"], imu["ay"], imu["az"],
                    imu["gx"], imu["gy"], imu["gz"]
                ])

            # —— Gyro frame —— #
            elif stype == rs.stream.gyro:
                md = frame.as_motion_frame().get_motion_data()
                last_gyro = {"gx":md.x, "gy":md.y, "gz":md.z}

    except KeyboardInterrupt:
        pass

    finally:
        pipe.stop()
        cv2.destroyAllWindows()
        csv_file.close()
        video_writer.release()
        if first_video_time and last_video_time:
            real_dur = last_video_time - first_video_time
            real_fps = video_frame_count / real_dur
            print(f"VIDEO: wrote {video_frame_count} frames over {real_dur:.3f}s → {real_fps:.2f} FPS")
        print("Exiting.")

if __name__ == "__main__":
    main()
