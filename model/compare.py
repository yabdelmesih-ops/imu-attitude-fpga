"""
Post-simulation analysis.

  * Confirms the HDL output file matches the bit-exact model (redundant with the
    self-checking testbench, but independent of it).
  * Runs a floating-point version of the same filter to measure quantization error.
  * Reports accuracy vs. ground truth for accel-only, gyro-only and the FPGA filter.
  * Saves docs/attitude_results.png.

Run after the simulation:  python3 model/compare.py
"""
import math
import os
import sys

import numpy as np

import fixed_model as fm

ROOT = os.path.normpath(os.path.join(os.path.dirname(__file__), ".."))
VEC = os.path.join(ROOT, "sim", "vectors")
OUT = os.path.join(ROOT, "sim", "out")


def wrap(d):
    return (d + 180.0) % 360.0 - 180.0


def load_hex(path, bits):
    rows = [[int(v, 16) for v in line.split()] for line in open(path) if line.strip()]
    a = np.array(rows, dtype=np.int64)
    return np.where(a >= (1 << (bits - 1)), a - (1 << bits), a)


def float_filter(raw, alpha_shift=fm.ALPHA_SHIFT):
    """Same algorithm as the HDL (incl. bias calibration), in double precision.
    Also returns raw gyro-only integration (no bias calibration) for comparison."""
    a = 1.0 - 2.0 ** -alpha_shift
    dt = 1.0 / fm.FS_HZ
    ax, ay, az, gx, gy = (raw[:, k].astype(float) for k in range(5))
    acc_r = np.degrees(np.arctan2(ay, az))
    acc_p = np.degrees(np.arctan2(-ax, np.hypot(ay, az)))
    r = np.empty_like(acc_r)
    p = np.empty_like(acc_p)
    ncal = 1 << fm.CAL_LOG2
    bx, by = gx[:ncal].mean(), gy[:ncal].mean()
    r[0], p[0] = acc_r[0], acc_p[0]
    for i in range(1, len(r)):
        cx, cy = (bx, by) if i >= ncal else (0.0, 0.0)
        pr = r[i - 1] + (gx[i] - cx) / fm.GYR_LSB_PER_DPS * dt
        pp = p[i - 1] + (gy[i] - cy) / fm.GYR_LSB_PER_DPS * dt
        r[i] = pr + (1 - a) * wrap(acc_r[i] - pr)
        p[i] = pp + (1 - a) * wrap(acc_p[i] - pp)
    gyro_r = acc_r[0] + np.cumsum(np.r_[0, gx[1:]]) / fm.GYR_LSB_PER_DPS * dt
    gyro_p = acc_p[0] + np.cumsum(np.r_[0, gy[1:]]) / fm.GYR_LSB_PER_DPS * dt
    return wrap(r), wrap(p), acc_r, acc_p, wrap(gyro_r), wrap(gyro_p)


def rms(x):
    return float(np.sqrt(np.mean(np.square(x))))


def main():
    raw = load_hex(os.path.join(VEC, "imu_stimulus.hex"), 16)
    exp = load_hex(os.path.join(VEC, "imu_expected.hex"), 32)
    hdl_path = os.path.join(OUT, "imu_hdl_out.hex")
    if not os.path.exists(hdl_path):
        sys.exit("run the simulation first (sim/run_all.sh)")
    hdl = load_hex(hdl_path, 32)
    truth = np.loadtxt(os.path.join(VEC, "imu_truth.csv"), delimiter=",", skiprows=1)
    t, tr, tp = truth[:, 0], truth[:, 1], truth[:, 2]

    n_bad = int(np.count_nonzero(np.any(hdl != exp, axis=1)))
    print(f"HDL vs bit-exact model : {len(hdl) - n_bad}/{len(hdl)} samples identical")

    to_deg = lambda col: hdl[:, col] * 360.0 / (1 << 32)
    h_r, h_p, h_ar, h_ap = (to_deg(c) for c in range(4))
    f_r, f_p, f_ar, f_ap, g_r, g_p = float_filter(raw)

    q_err = max(np.abs(wrap(h_r - f_r)).max(), np.abs(wrap(h_p - f_p)).max())
    print(f"Fixed vs float filter  : max difference {q_err:.4f} deg (quantization error)")
    print()
    print(f"{'error vs truth (deg)':<24}{'roll rms':>10}{'roll max':>10}{'pitch rms':>11}{'pitch max':>11}")
    for name, r, p in (("accelerometer only", h_ar, h_ap),
                       ("gyro only (no bias cal)", g_r, g_p),
                       ("FPGA filter", h_r, h_p)):
        er, ep = wrap(r - tr), wrap(p - tp)
        print(f"{name:<24}{rms(er):>10.2f}{np.abs(er).max():>10.2f}{rms(ep):>11.2f}{np.abs(ep).max():>11.2f}")

    plot(t, tr, tp, h_r, h_p, h_ar, h_ap, g_r, g_p)
    return 0 if n_bad == 0 else 1


def plot(t, tr, tp, h_r, h_p, h_ar, h_ap, g_r, g_p):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    INK, INK2, GRID, SURF = "#1f1f1e", "#6b6a63", "#e6e5e0", "#fcfcfb"
    BLUE, ORANGE, AQUA = "#2a78d6", "#eb6834", "#1baf7a"
    plt.rcParams.update({
        "font.size": 9, "axes.edgecolor": INK2, "axes.labelcolor": INK2,
        "xtick.color": INK2, "ytick.color": INK2, "axes.spines.top": False,
        "axes.spines.right": False, "figure.facecolor": SURF, "axes.facecolor": SURF,
    })
    fig, axs = plt.subplots(3, 1, figsize=(10, 8.2), sharex=True,
                            gridspec_kw={"height_ratios": [1.15, 1, 0.8]})

    # Break lines at the +/-180 wrap so they don't draw a vertical streak.
    def brk(y):
        y = np.asarray(y, dtype=float).copy()
        y[1:][np.abs(np.diff(y)) > 180] = np.nan
        return y

    for ax, truth, acc, gyro, filt, name in ((axs[0], tr, h_ar, g_r, h_r, "Roll"),
                                             (axs[1], tp, h_ap, g_p, h_p, "Pitch")):
        ax.plot(t, brk(acc), color=ORANGE, lw=0.6, alpha=0.55, label="Accelerometer only")
        ax.plot(t, brk(gyro), color=AQUA, lw=1.5, ls=(0, (4, 2)), label="Gyro only, uncalibrated (drifts)")
        ax.plot(t, brk(wrap(truth)), color=INK, lw=2.0, label="Truth")
        ax.plot(t, brk(filt), color=BLUE, lw=1.5, label="FPGA filter (HDL sim)")
        ax.set_ylabel(f"{name} (deg)")
        ax.grid(True, color=GRID, lw=0.6)
        ax.set_axisbelow(True)
    axs[0].set_ylim(-190, 190)
    axs[0].set_yticks([-180, -90, 0, 90, 180])
    axs[0].legend(loc="lower left", ncol=4, frameon=False, fontsize=8.5,
                  bbox_to_anchor=(0, 1.0))

    er, ep = wrap(h_r - tr), wrap(h_p - tp)
    axs[2].plot(t, er, color=BLUE, lw=1.2, label="Roll error")
    axs[2].plot(t, ep, color=ORANGE, lw=1.2, ls=(0, (4, 2)), label="Pitch error")
    axs[2].axhline(0, color=INK2, lw=0.6)
    axs[2].set_ylabel("Filter error (deg)")
    axs[2].set_xlabel("Time (s)")
    axs[2].grid(True, color=GRID, lw=0.6)
    axs[2].legend(loc="lower left", ncol=2, frameon=False, fontsize=8.5)

    axs[2].set_ylim(top=1.6)
    for x0, x1, lab in ((0, 1.024, "gyro cal"), (2, 6, "roll sweep"), (6, 10, "pitch sweep"), (10, 14, "full 360° roll"),
                        (14, 18, "combined"), (18.5, 18.7, "bump")):
        for ax in axs:
            ax.axvspan(x0, x1, color=GRID, alpha=0.45, lw=0)
        axs[2].text((x0 + x1) / 2, axs[2].get_ylim()[1], lab, ha="center", va="top",
                    fontsize=7.5, color=INK2)

    fig.suptitle("Fixed-point complementary filter: 20 s synthetic IMU run, 1 kHz",
                 x=0.01, ha="left", color=INK, fontsize=11)
    fig.tight_layout()
    os.makedirs(os.path.join(ROOT, "docs"), exist_ok=True)
    path = os.path.join(ROOT, "docs", "attitude_results.png")
    fig.savefig(path, dpi=150)
    print(f"\nplot saved to {os.path.relpath(path, ROOT)}")


if __name__ == "__main__":
    sys.exit(main())
