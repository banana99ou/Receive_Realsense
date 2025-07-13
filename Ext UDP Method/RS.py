#!/usr/bin/env python3
import threading
import time
import math
import struct
import socket

import numpy as np
import cv2
import pyrealsense2 as rs


# —————— Your Simulink target IP/ports ——————
SIMULINK_IP    = "127.0.0.1"
IMU_PORT       = 5005
CAM_PORT       = 5006
# ————————————————————————————————————————

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
                 sample_rate=100.0, udp_chunk_size=8192):
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
        self.imu_fmt = "<2d d 6f"
        if self.stream == "cam":
            # we'll send at 25 Hz by default, chunked JPEG
            self.rate = 25.0

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
                    self.imu_fmt,
                    t_res_s, t_enqueue,
                    t_send,
                    imu["ax"], imu["ay"], imu["az"],
                    imu["gx"], imu["gy"], imu["gz"]
                )
                self.sock.sendto(payload, (self.ip, self.port))

            else:  # camera
                _, _, _, jpeg_bytes, _ = pkt
                total = len(jpeg_bytes)
                n_chunks = math.ceil(total/self.chunk)
                for cid in range(n_chunks):
                    off = cid*self.chunk
                    chunk = jpeg_bytes[off:off+self.chunk]
                    # header: [frame_id(uint32), chunk_id(uint16), total_chunks(uint16)]
                    hdr = struct.pack("<IHH", frame_id, cid, n_chunks)
                    self.sock.sendto(hdr + chunk, (self.ip, self.port))
                frame_id = (frame_id+1) & 0xFFFFFFFF

            # pacing
            now = time.monotonic()
            if now < next_ts:
                time.sleep(next_ts - now)
            next_ts += period


def main():
    # 1) Setup RealSense
    pipe = rs.pipeline()
    cfg  = rs.config()
    cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 60)
    cfg.enable_stream(rs.stream.accel)
    cfg.enable_stream(rs.stream.gyro)
    pipe.start(cfg)

    shared = SharedPacket()
    # start sender threads
    imu_sender = SimulinkUDPSender("imu", shared, SIMULINK_IP, IMU_PORT, sample_rate=100.0)
    cam_sender = SimulinkUDPSender("cam", shared, SIMULINK_IP, CAM_PORT, sample_rate=25.0)
    imu_sender.start()
    cam_sender.start()

    try:
        print("Running. CTRL+C to exit.")
        start_t = time.time()
        frame_ct = 0

        while True:
            frames = pipe.wait_for_frames()

            # color
            cf = frames.get_color_frame()
            if not cf:
                continue
            color = np.asanyarray(cf.get_data())
            t_color = cf.get_timestamp() * 1000.0  # to microseconds
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
            t_accel = af.get_timestamp()*1000.0  # µs
            # use t_color for camera timestamp, but you can also use t_accel if desired
            t_enqueue = time.perf_counter()

            # update shared packet
            with shared.lock:
                shared.data = (
                    t_accel,      # t_res_us (we use accel timestamp)
                    t_enqueue,    # enqueue time (perf_counter)
                    imu,
                    jpeg,
                    t_color       # raw color timestamp μs
                )

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
        pipe.stop()
        cv2.destroyAllWindows()
        print("Exiting.")

if __name__ == "__main__":
    main()
