import cv2
import numpy as np
import socket
import time
import argparse

def run_camera_tcp_sender(host: str, port: int,
                          cam_index: int = 0,
                          width: int = 320,
                          height: int = 240,
                          fps: float = 30.0):
    """
    Capture from camera, convert to grayscale, resize to (height, width),
    flatten column-major, and send raw bytes over TCP to host:port at 'fps'.
    """
    # --- Set up camera capture ---
    cap = cv2.VideoCapture(cam_index, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open camera #{cam_index}")

    # --- Set up TCP client socket ---
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    sock.connect((host, port))
    print(f"[+] Connected to {host}:{port} — sending {width}×{height} @ {fps} FPS")

    period = 1.0 / fps
    next_send = time.monotonic()

    try:
        while True:
            # 1) Grab frame
            ret, frame = cap.read()
            if not ret:
                print("[!] Frame grab failed, retrying...")
                continue

            # 2) Convert & resize
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            gray = cv2.resize(gray, (width, height), interpolation=cv2.INTER_LINEAR)

            # 3) Flatten column-major order
            col_major = gray.T  # shape is now (width, height)
            payload = col_major.tobytes()  # exactly width*height bytes

            # 4) Send
            sock.sendall(payload)

            # 5) Pace the loop
            now = time.monotonic()
            sleep_time = next_send + period - now
            if sleep_time > 0:
                time.sleep(sleep_time)
                next_send += period
            else:
                # Behind schedule; skip sleeping, but resync next_send
                next_send = now

    except KeyboardInterrupt:
        print("\n[+] Stopping camera sender.")
    finally:
        cap.release()
        sock.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Soft-RT camera → TCP sender for Simulink Desktop Real-Time")
    parser.add_argument("--host", default="127.0.0.1", help="TCP server host (Simulink listens here)")
    parser.add_argument("--port", type=int, default=36880, help="TCP server port")
    parser.add_argument("--cam", type=int, default=0, help="OpenCV camera index")
    parser.add_argument("--width", type=int, default=320, help="Resize width")
    parser.add_argument("--height", type=int, default=240, help="Resize height")
    parser.add_argument("--fps", type=float, default=30.0, help="Frames per second to send")
    args = parser.parse_args()

    run_camera_tcp_sender(
        host=args.host,
        port=args.port,
        cam_index=args.cam,
        width=args.width,
        height=args.height,
        fps=args.fps
    )
