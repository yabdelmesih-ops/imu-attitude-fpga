"""
Read attitude telemetry from the FPGA over USB-UART, print it, log it to CSV,
and optionally show a live plot.

  pip install pyserial matplotlib
  python3 tools/view_serial.py --port /dev/ttyUSB1          # Linux
  python3 tools/view_serial.py --port COM5 --plot           # Windows
  python3 tools/view_serial.py --port /dev/tty.usbserial-XXXX --csv run1.csv

Frame (8 bytes): A5 5A rollH rollL pitchH pitchL status chk
  angle deg = int16 * 360 / 65536, status = {seq[5:0], imu_running, cal_done},
  chk = XOR of bytes 2..6
"""
import argparse
import collections
import struct
import sys
import time


def frames(ser):
    """Yield (roll_deg, pitch_deg, status) from a byte stream, resyncing on errors."""
    buf = bytearray()
    while True:
        buf += ser.read(max(1, ser.in_waiting))
        while len(buf) >= 8:
            if buf[0] != 0xA5 or buf[1] != 0x5A:
                del buf[0]
                continue
            f = bytes(buf[:8])
            if f[2] ^ f[3] ^ f[4] ^ f[5] ^ f[6] != f[7]:
                del buf[0]
                continue
            del buf[:8]
            roll, pitch = struct.unpack(">hh", f[2:6])
            yield roll * 360.0 / 65536, pitch * 360.0 / 65536, f[6]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--csv", help="log to this CSV file")
    ap.add_argument("--plot", action="store_true", help="live plot (last 10 s)")
    args = ap.parse_args()

    import serial
    ser = serial.Serial(args.port, args.baud, timeout=0.1)
    log = open(args.csv, "w") if args.csv else None
    if log:
        log.write("t,roll_deg,pitch_deg,cal_done,seq\n")

    if args.plot:
        import matplotlib.pyplot as plt
        plt.ion()
        fig, ax = plt.subplots(figsize=(9, 4))
        n = 1000                                     # 10 s at 100 Hz
        rq, pq = collections.deque([0.0] * n, n), collections.deque([0.0] * n, n)
        lr, = ax.plot(rq, label="roll", color="#2a78d6")
        lp, = ax.plot(pq, label="pitch", color="#eb6834", ls="--")
        ax.set_ylim(-180, 180)
        ax.set_ylabel("deg")
        ax.legend(loc="upper left", frameon=False)
        ax.grid(alpha=0.3)

    t0, last_seq, drops, k = time.time(), None, 0, 0
    for roll, pitch, status in frames(ser):
        seq, cal = status >> 2, status & 1
        if last_seq is not None and seq != (last_seq + 1) & 63:
            drops += 1
        last_seq = seq
        t = time.time() - t0
        if log:
            log.write(f"{t:.3f},{roll:.4f},{pitch:.4f},{cal},{seq}\n")
        k += 1
        if k % 10 == 0:
            state = "ok " if cal else "CAL"
            sys.stdout.write(f"\r[{state}] roll {roll:8.2f}  pitch {pitch:8.2f}  drops {drops}   ")
            sys.stdout.flush()
        if args.plot:
            rq.append(roll)
            pq.append(pitch)
            if k % 5 == 0:
                lr.set_ydata(rq)
                lp.set_ydata(pq)
                plt.pause(0.001)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print()
