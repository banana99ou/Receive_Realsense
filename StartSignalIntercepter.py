#!/usr/bin/env python3
import socket

# Adjust these to match your Simulink UDP Send block
LISTEN_IP   = "0.0.0.0"   # or "127.0.0.1" if Simulink sends to localhost
LISTEN_PORT = 9000
BUFFER_SIZE = 8192        # enough to hold your payload

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((LISTEN_IP, LISTEN_PORT))
    print(f"[+] Listening for one UDP packet on {LISTEN_IP}:{LISTEN_PORT} (timeout 10 s)...")
    sock.settimeout(10.0)
    try:
        data, addr = sock.recvfrom(BUFFER_SIZE)
        print(f"[+] Received {len(data)} bytes from {addr}:")
        # raw bytes
        print(data)
        # if it’s numeric payload, you can unpack it, e.g.:
        # import struct
        # vals = struct.unpack('<6f', data)
        # print("Parsed floats:", vals)
    except socket.timeout:
        print("[-] Timeout: no packet received within 10 seconds.")
    finally:
        sock.close()

if __name__ == "__main__":
    main()
