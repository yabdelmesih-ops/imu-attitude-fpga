"""
Hardware-in-the-loop run: stream recorded IMU samples to the FPGA over USB-serial
and check every returned roll/pitch against the bit-exact Python model.

Board build: boards/alchitry_au/top_alchitry_au_hil.sv (+ alchitry_au_hil.xdc).

  pip install pyserial numpy matplotlib
  python tools/hil_run.py --port COM5                  # Windows
  python tools/hil_run.py --port /dev/ttyUSB1          # Linux / macOS
  python tools/hil_run.py --selftest                   # no board: fake FPGA built from the model

Protocol (see rtl/hil_rx.sv and rtl/hil_tx.sv):
  PC -> FPGA  sample : C3 3C ax ay az gx gy (int16, big-endian) chk   chk = XOR of the 10 data bytes
              reset  : C3 E1 1E
  FPGA -> PC  result : A5 5A seq roll(u32) pitch(u32) chk             chk = XOR of bytes 2..10
"""
import argparse
import csv
import os
import struct
import subprocess
import sys
import time

ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
VEC = os.path.join(ROOT, "sim", "vectors")
sys.path.insert(0, os.path.join(ROOT, "model"))

RESET_FRAME = bytes([0xC3, 0xE1, 0x1E])
RESULT_LEN = 12


# ---------------------------------------------------------------- frames
def sample_frame(ax, ay, az, gx, gy):
    data = struct.pack(">hhhhh", ax, ay, az, gx, gy)
    chk = 0
    for b in data:
        chk ^= b
    return bytes([0xC3, 0x3C]) + data + bytes([chk])


def result_frame(seq, roll, pitch):
    body = bytes([seq & 0xFF]) + struct.pack(">II", roll & 0xFFFFFFFF, pitch & 0xFFFFFFFF)
    chk = 0
    for b in body:
        chk ^= b
    return bytes([0xA5, 0x5A]) + body + bytes([chk])


def parse_results(buf):
    """Pull complete, checksum-valid result frames out of buf (a bytearray).
    Returns a list of (seq, roll, pitch) and the number of bytes skipped while
    resynchronizing. Consumed bytes are removed from buf."""
    out, skipped = [], 0
    while len(buf) >= RESULT_LEN:
        if buf[0] != 0xA5 or buf[1] != 0x5A:
            del buf[0]
            skipped += 1
            continue
        f = bytes(buf[:RESULT_LEN])
        chk = 0
        for b in f[2:11]:
            chk ^= b
        if chk != f[11]:
            del buf[0]
            skipped += 1
            continue
        del buf[:RESULT_LEN]
        roll, pitch = struct.unpack(">II", f[3:11])
        out.append((f[2], roll, pitch))
    return out, skipped


# ---------------------------------------------------------------- vectors
def load_vectors(limit=None):
    stim_path = os.path.join(VEC, "imu_stimulus.hex")
    exp_path = os.path.join(VEC, "imu_expected.hex")
    if not (os.path.exists(stim_path) and os.path.exists(exp_path)):
        print("test vectors not found; generating them with model/gen_stimulus.py ...")
        subprocess.run([sys.executable, os.path.join(ROOT, "model", "gen_stimulus.py")], check=True)

    def s16(h):
        v = int(h, 16)
        return v - 0x10000 if v >= 0x8000 else v

    stim = [[s16(h) for h in line.split()] for line in open(stim_path) if line.strip()]
    exp = [[int(h, 16) for h in line.split()[:2]] for line in open(exp_path) if line.strip()]
    n = len(stim) if limit is None else min(limit, len(stim))
    return stim[:n], exp[:n]


def bam_deg(v):
    v &= 0xFFFFFFFF
    if v >= 0x80000000:
        v -= 0x100000000
    return v * 360.0 / 2**32


# ---------------------------------------------------------------- fake FPGA
class FakeFPGA:
    """Serial-port stand-in that behaves like the HIL bitstream, built from the
    bit-exact model. Used by --selftest to check this script without a board."""

    def __init__(self, corrupt_every=0):
        import fixed_model as fm
        self.fm = fm
        self.core = fm.AttitudeCore()
        self.seq = 0
        self.rx = bytearray()
        self.tx = bytearray()
        self.corrupt_every = corrupt_every
        self.n_out = 0
        self.timeout = 1.0

    def reset_input_buffer(self):
        self.tx.clear()

    def reset_output_buffer(self):
        pass

    def write(self, data):
        self.rx += data
        while True:
            i = self.rx.find(b"\xC3")
            if i < 0 or len(self.rx) < i + 2:
                break
            if self.rx[i + 1] == 0xE1 and len(self.rx) >= i + 3:
                del self.rx[: i + 3]
                self.core = self.fm.AttitudeCore()
                self.seq = 0
                continue
            if self.rx[i + 1] == 0x3C and len(self.rx) >= i + 13:
                frame = bytes(self.rx[i: i + 13])
                del self.rx[: i + 13]
                chk = 0
                for b in frame[2:12]:
                    chk ^= b
                if chk != frame[12]:
                    continue
                ax, ay, az, gx, gy = struct.unpack(">hhhhh", frame[2:12])
                roll, pitch, _, _ = self.core.step(ax, ay, az, gx, gy)
                out = bytearray(result_frame(self.seq, roll, pitch))
                self.n_out += 1
                if self.corrupt_every and self.n_out % self.corrupt_every == 0:
                    out[5] ^= 0x01                     # flip a bit on the wire
                self.tx += out
                self.seq = (self.seq + 1) & 0xFF
                continue
            if self.rx[i + 1] in (0x3C, 0xE1):
                break                                  # wait for the rest of the frame
            del self.rx[: i + 1]
        return len(data)

    def read(self, n):
        out = bytes(self.tx[:n])
        del self.tx[:n]
        return out

    @property
    def in_waiting(self):
        return len(self.tx)

    def close(self):
        pass


# ---------------------------------------------------------------- run
def run(port, stim, exp, chunk=200, csv_path=None, verbose=True):
    """Stream all samples and return a summary dict.

    Results are placed by their 8-bit sequence number, so a lost or corrupted
    result frame shows up as a gap at the right sample instead of shifting every
    later comparison."""
    port.reset_input_buffer()
    port.write(RESET_FRAME)
    time.sleep(0.05)
    port.reset_input_buffer()

    n = len(stim)
    results = {}                 # sample index -> (roll, pitch)
    next_index = 0               # index the next in-order frame should carry
    buf = bytearray()
    skipped_total = 0
    t0 = time.time()
    for start in range(0, n, chunk):
        block = stim[start: start + chunk]
        port.write(b"".join(sample_frame(*s) for s in block))
        want = start + len(block)
        deadline = time.time() + 1.0 + 0.001 * len(block)
        while next_index < want and time.time() < deadline:
            data = port.read(max(1, getattr(port, "in_waiting", 0) or RESULT_LEN))
            if not data:
                continue
            buf += data
            got, skipped = parse_results(buf)
            skipped_total += skipped
            for seq, roll, pitch in got:
                idx = next_index + ((seq - next_index) & 0xFF)
                results[idx] = (roll, pitch)
                next_index = idx + 1
        if verbose:
            sys.stdout.write(f"\r{len(results)}/{n} samples")
            sys.stdout.flush()
    elapsed = time.time() - t0
    if verbose:
        print()

    lost = [i for i in range(n) if i not in results]
    mismatches = [i for i, rp in sorted(results.items()) if i < len(exp) and rp != tuple(exp[i])]

    if csv_path:
        with open(csv_path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["sample", "roll_fpga_deg", "pitch_fpga_deg", "roll_model_deg", "pitch_model_deg", "match"])
            for i, (roll, pitch) in sorted(results.items()):
                w.writerow([i, f"{bam_deg(roll):.6f}", f"{bam_deg(pitch):.6f}",
                            f"{bam_deg(exp[i][0]):.6f}", f"{bam_deg(exp[i][1]):.6f}",
                            int((roll, pitch) == tuple(exp[i]))])

    return dict(sent=n, received=len(results), mismatches=mismatches, lost=lost,
                skipped_bytes=skipped_total, elapsed=elapsed, results=results)


def report(s, exp):
    ok = s["received"] == s["sent"] and not s["mismatches"] and not s["lost"]
    print(f"samples sent      : {s['sent']}")
    print(f"results received  : {s['received']}")
    print(f"bit-exact matches : {s['received'] - len(s['mismatches'])}/{s['received']}")
    print(f"lost results      : {len(s['lost'])}" + (f" (first at sample {s['lost'][0]})" if s["lost"] else ""))
    print(f"bytes resynced    : {s['skipped_bytes']}")
    if s["elapsed"] > 0:
        print(f"time              : {s['elapsed']:.2f} s ({s['received'] / s['elapsed']:.0f} samples/s)")
    for i in s["mismatches"][:5]:
        r, p = s["results"][i]
        print(f"  mismatch at sample {i}: FPGA {bam_deg(r):.4f}/{bam_deg(p):.4f} deg, "
              f"model {bam_deg(exp[i][0]):.4f}/{bam_deg(exp[i][1]):.4f} deg")
    print("HIL PASS: hardware output is bit-exact with the model" if ok else "HIL FAIL")
    return ok


def plot(results, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np
    truth = np.loadtxt(os.path.join(VEC, "imu_truth.csv"), delimiter=",", skiprows=1)
    idx = np.array(sorted(results))
    n = int(idx.max()) + 1
    t = truth[idx, 0]
    roll = np.array([bam_deg(results[i][0]) for i in idx])
    pitch = np.array([bam_deg(results[i][1]) for i in idx])
    fig, axs = plt.subplots(2, 1, figsize=(10, 5.5), sharex=True)
    for ax, y, tr, name in ((axs[0], roll, truth[idx, 1], "Roll"), (axs[1], pitch, truth[idx, 2], "Pitch")):
        tr = (tr + 180) % 360 - 180

        def brk(v):                      # break the line at the +/-180 deg wrap
            v = np.asarray(v, dtype=float).copy()
            v[1:][np.abs(np.diff(v)) > 180] = np.nan
            return v
        ax.plot(t, brk(tr), color="#1f1f1e", lw=2, label="Truth")
        ax.plot(t, brk(y), color="#2a78d6", lw=1.2, label="FPGA (hardware)")
        ax.set_ylabel(f"{name} (deg)")
        ax.grid(alpha=0.3)
    axs[0].legend(loc="lower left", frameon=False, ncol=2)
    axs[1].set_xlabel("Time (s)")
    fig.suptitle("Hardware-in-the-loop run on the FPGA", x=0.01, ha="left")
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    print(f"plot saved to {path}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="serial port of the FPGA board (e.g. COM5, /dev/ttyUSB1)")
    ap.add_argument("--baud", type=int, default=1_000_000)
    ap.add_argument("--samples", type=int, help="limit the number of samples (default: all 20,000)")
    ap.add_argument("--chunk", type=int, default=200, help="samples per write")
    ap.add_argument("--csv", help="write per-sample results to this CSV")
    ap.add_argument("--plot", help="save a roll/pitch plot (PNG) of the hardware results")
    ap.add_argument("--selftest", action="store_true", help="run against a fake FPGA built from the model")
    args = ap.parse_args()

    stim, exp = load_vectors(args.samples)
    if args.selftest:
        port = FakeFPGA()
    else:
        if not args.port:
            ap.error("--port is required (or use --selftest)")
        import serial
        port = serial.Serial(args.port, args.baud, timeout=0.05)
        time.sleep(0.1)
    try:
        s = run(port, stim, exp, chunk=args.chunk, csv_path=args.csv)
    finally:
        port.close()
    ok = report(s, exp)
    if args.plot and s["results"]:
        plot(s["results"], args.plot)
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
