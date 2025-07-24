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
import argparse
import numpy as np
import cv2
import serial, zlib
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
UDP_IMU_RATE = 99.0
UDP_CAM_RATE = 59.0
HB_TIMEOUT = 0.5 #2.0 / UDP_IMU_RATE
# =================================================

class SerialIMUThread(threading.Thread):
    """Read lines from Serial, parse CSV+CRC, write to its own imu_serial.csv."""
    def __init__(self, port, baudrate, out_dir, last_hb, hb_timeout, tag="imu1", max_gap=1.0):
        super().__init__(daemon=True)
        self.port        = port
        self.baudrate    = baudrate
        self.max_gap     = max_gap
        self.out_dir     = out_dir
        self.tag         = tag
        self.last_hb     = last_hb       # (shared list)
        self.hb_timeout  = hb_timeout

    def run(self):
        ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
        # open csv
        fname = os.path.join(self.out_dir, f"{self.tag}_serial_{datetime.datetime.now():%Y%m%d_%H%M%S}_{self.port}.csv")
        with open(fname, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['t_us','t_s','t_rel', 'ax','ay','az', 'gx','gy','gz','crc'])

            t0 = None
            last_recv = time.time()
            print(f"[SERIAL {self.tag}] Logging to {fname}")

            while True:
                # check heartbeat timeout
                hb = last_hb[0]
                if hb is not None and (time.perf_counter() - hb) > HB_TIMEOUT:
                    print(f"[INFO] no heartbeat for {HB_TIMEOUT:.3f}s → exiting")
                    break

                # read line
                line = ser.readline().decode(errors='ignore').strip()
                if not line:
                    if time.time() - last_recv > self.max_gap:
                        print(f"[SERIAL {self.tag}] ⚠️  No data for {self.max_gap}s")
                        last_recv = time.time()
                    continue
                last_recv = time.time()

                parts = line.split(',')
                if len(parts) != 11:
                    print(f"[SERIAL {self.tag}] ⚠️ Bad field count: {line}")
                    continue

                # verify CRC
                data_str = ','.join(parts[:-1])
                recv_crc = int(parts[-1], 16)
                calc_crc = zlib.crc32(data_str.encode()) & 0xFFFFFFFF
                if calc_crc != recv_crc:
                    print(f"[SERIAL {self.tag}] ❌ CRC mismatch: {parts[-1]} vs {calc_crc:08X}")
                    continue

                # do not write with no heartbeat
                if hb is None:
                    continue

                t_us = int(parts[0])
                t_s  = t_us/1e6
                if t0 is None:
                    t0 = t_s
                t_rel = t_s - t0

                ax, ay, az = map(float, parts[1:4])
                gx, gy, gz = map(float, parts[4:7])

                # convert unit
                ax = 9.81 * ax
                ay = 9.81 * ay
                az = 9.81 * az
                gx = np.deg2rad(gx)
                gy = np.deg2rad(gy)
                gz = np.deg2rad(gz)

                crc_hex    = parts[10]

                vals = list(map(float, parts[1:-1]))
                writer.writerow([
                    t_us,
                    f"{t_s:.6f}",
                    f"{t_rel:.6f}",
                    f"{ax:.6f}", f"{ay:.6f}", f"{az:.6f}",
                    f"{gx:.6f}", f"{gy:.6f}", f"{gz:.6f}",
                    crc_hex
                ])
                if int(t_rel*IMU_RATE_REQ) % 1000 == 0:
                    print(f"[SERIAL {self.tag}] {self.tag} {int(t_rel*IMU_RATE_REQ)} samples in, last accel={ax:.3f}")

                f.flush()
                os.fsync(f.fileno())

        ser.close()

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
    def __init__(self, stream_type, shared, ip, port, sample_rate, udp_chunk_size=10000):#9600):#8192):#
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

def start_heartbeat_listener(ip, port, last_hb):
    """
    Binds to (ip,port) and whenever a UDP packet arrives,
    updates last_hb[0] to now(). Runs forever as daemon.
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((ip, port))
    # No timeout needed: blocking here is fine, it's in its own thread
    while True:
        try:
            sock.recvfrom(16)
            last_hb[0] = time.perf_counter()
        except Exception:
            break  # on any socket error, thread will exit

def ensure_dir(p):
    os.makedirs(p, exist_ok=True)
    return p

def main():
    args = argparse.ArgumentParser()
    args.add_argument("--portA", default="COM4")
    args.add_argument("--portB", default="COM6")
    args= args.parse_args()

    # ----- Output directory -----
    session_ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
    record_dir = f"recording_{session_ts}"
    frames_dir = ensure_dir(os.path.join(record_dir, "frames"))
    raw_imu_csv_path = os.path.join(record_dir, "imu_raw.csv")
    print(f"[INFO] Recording into: {record_dir}")

    START_PORT = 6000
    HB_TIMEOUT = 0.5 #2.0 / UDP_IMU_RATE
    last_hb = [None]
    hb_thread = threading.Thread(
        target=start_heartbeat_listener,
        args=(SIMULINK_IP, START_PORT, last_hb),
        daemon=True
    )
    hb_thread.start()

    # ----- START SERIAL IMU THREADS -----
    SERIAL_IMUS = [
        {"port": args.portA, "baudrate": 115200, "tag": "Center_Console"},
        {"port": args.portB, "baudrate": 115200, "tag": "Headrest"},
    ]

    serial_threads = []
    for cfg_ser in SERIAL_IMUS:
        t = SerialIMUThread(
            port       = cfg_ser["port"],
            baudrate   = cfg_ser["baudrate"],
            out_dir    = record_dir,
            tag        = cfg_ser["tag"],
            max_gap    = 1.0,
            last_hb    = last_hb,
            hb_timeout = HB_TIMEOUT
        )
        t.start()
        serial_threads.append(t)

    # ----- RealSense setup (single queue) -----
    q    = rs.frame_queue(32)
    pipe = rs.pipeline()
    cfg  = rs.config()
    cfg.enable_stream(rs.stream.color, IMG_W, IMG_H, rs.format.bgr8, COLOR_RATE)
    cfg.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, IMU_RATE_REQ)
    cfg.enable_stream(rs.stream.gyro,  rs.format.motion_xyz32f, GYRO_RATE_REQ)
    pipe.start(cfg, q)

    # ----- Prepare files -----
    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
    csv_file   = open(raw_imu_csv_path, "w", newline='')
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow([
        't_accel_us','t_color_us',
        'ax','ay','az','gx','gy','gz'
    ])

    # ----- Shared + UDP senders -----
    shared     = SharedPacket()
    imu_sender = SimulinkUDPSender("imu", shared, SIMULINK_IP, IMU_PORT, UDP_IMU_RATE)
    cam_sender = SimulinkUDPSender("cam", shared, SIMULINK_IP, CAM_PORT, UDP_CAM_RATE)
    imu_sender.start()
    cam_sender.start()

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
            # heartbeat-timeout check at top of loop
            hb = last_hb[0]
            if hb is not None and (time.perf_counter() - hb) > HB_TIMEOUT:
                print(f"[INFO] no heartbeat for {HB_TIMEOUT:.3f}s → exiting")
                break

            frame = q.wait_for_frame()
            prof  = frame.get_profile()
            Stream_Type = prof.stream_type()

            # —— Color frame —— #
            if Stream_Type == rs.stream.color:
                ts_ms = frame.get_timestamp()  # ms
                if hb is not None and (t0_color is None):
                    t0_color = ts_ms
                    print("[INFO] first heartbeat received!")
                    # t0_accel = None

                raw = frame.get_data()
                arr = np.frombuffer(raw, dtype=np.uint8)
                img = arr.reshape((IMG_H, IMG_W, 3))
                # Save JPEG frame with timestamp in filename
                if t0_color is not None:
                    rel_color_us = int((ts_ms - t0_color) * 1000)
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
                cv2.putText(img, f"FPS~{fps_counter/(now-last_fps_print):.1f}", (5,25), cv2.FONT_HERSHEY_SIMPLEX, 0.7,(0,255,0),2)
                if now - last_fps_print >= 0.5:
                    last_fps_print = now
                    fps_counter = 0
                cv2.imshow("RGB", img)
                if (cv2.waitKey(1) & 0xFF) == ord('q'):
                    break
                frame_count += 1

            if Stream_Type == rs.stream.accel:
                ts_ms = frame.get_timestamp()
                # if not last_color_us or not last_jpeg:
                #     continue  # wait until we have at least one frame
                if hb is not None and (t0_accel is None) and (t0_color is not None):
                    t0_accel = ts_ms

                if t0_accel is not None:
                    rel_accel_us = int((ts_ms - t0_accel) * 1000)

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
        csv_file.close()

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