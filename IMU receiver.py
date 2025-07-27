import serial, csv, zlib, os, time
from datetime import datetime
import logging
import argparse

# adjust these
PORT      = '/dev/cu.usbserial-0001'
BAUDRATE  = 115200
MAX_GAP   = 1.0   # seconds to warn on no data

def main():
    parser = argparse.ArgumentParser(description="Stand Alone IMU reciever")
    parser.add_argument("--port", default="/dev/cu.usbserial-0001")
    parser.add_argument("--baudrate", default="1000000")
    parser.add_argument("--verbose", action="store_true", help="Log debug messages to console")
    args = parser.parse_args()

    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO, format="[%(levelname)s] %(message)s")

    ser = serial.Serial(args.port, args.baudrate, timeout=0.1)
    now = datetime.now()
    port_tag = os.path.basename(args.port).replace(":", "_").replace("/", "_")
    fname = f"{datetime.now():%Y%m%d_%H%M%S}_{now.microsecond//1000:03d}_{port_tag}.csv"
    f = open(fname, 'w', newline='')
    writer = csv.writer(f)
    writer.writerow(['t_us','t_s','t_rel','ax','ay','az', 'gx','gy','gz', 'grav_x','grav_y','grav_z', 'crc'])
    f.flush(); os.fsync(f.fileno())

    t0 = None
    last_recv = time.time()

    print(f"Logging to {fname}")
    try:
        while True:
            raw = ser.readline().decode(errors='ignore').strip()
            logging.debug(f"raw: {raw}")
            if not raw:
                if time.time() - last_recv > MAX_GAP:
                    print(f"⚠️  No data for {MAX_GAP}s")
                    last_recv = time.time()
                continue

            last_recv = time.time()
            parts = raw.split(',')
            if len(parts) != 11:
                logging.warning(f"⚠️  Bad field count ({len(parts)}): {raw}")
                continue

            # split data vs CRC
            data_str = ','.join(parts[:-1])
            recv_crc = int(parts[-1], 16)
            calc_crc = zlib.crc32(data_str.encode()) & 0xFFFFFFFF

            if calc_crc != recv_crc:
                logging.warning(f"❌  CRC mismatch: got {parts[-1]} vs {calc_crc:08X}")
                continue

            # parse values
            t_us      = int(parts[0])
            t_s       = t_us / 1e6
            if t0 is None: t0 = t_s
            t_rel     = t_s - t0
            vals = list(map(float, parts[1:-1]))

            # write row + confirm offset
            writer.writerow([t_us,
                             f"{t_s:.6f}",
                             f"{t_rel:.6f}",
                             *["{:.6f}".format(v) for v in vals],
                             parts[-1]])
            f.flush(); os.fsync(f.fileno())
            pos = f.tell()
            logging.info(f"✅  Wrote row at byte offset {pos}")

    except KeyboardInterrupt:
        print("Stopping. File closed.")
        f.close()

if __name__ == '__main__':
    main()