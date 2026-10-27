"""
Generate synthetic IMU data, golden expected outputs, and CORDIC unit-test vectors.

Outputs (in ../sim/vectors/):
  imu_stimulus.hex     one sample per line: ax ay az gx gy   (16-bit hex each)
  imu_expected.hex     bit-exact expected: roll pitch acc_roll acc_pitch (32-bit hex)
  imu_truth.csv        true roll/pitch in degrees, for error/plots
  cordic_vectors.hex   x y expected_angle expected_mag   (24/24/32/24-bit hex)

Trajectory (20 s at 1 kHz) is built to exercise the cases that break naive designs:
  * gyro bias (gyro-only integration drifts)
  * accelerometer noise and a linear-acceleration bump (accel-only angle jumps)
  * a full 360 deg roll, crossing +/-180 deg (tests BAM wrap-around)
"""
import math
import os
import random

import numpy as np

import fixed_model as fm

OUT = os.path.join(os.path.dirname(__file__), "..", "sim", "vectors")
SEED = 1258


def trajectory(t):
    """True roll/pitch in degrees as a function of time (s)."""
    roll = np.zeros_like(t)
    pitch = np.zeros_like(t)

    seg = (t >= 2) & (t < 6)                       # roll sweep +/-45 deg, 0.5 Hz
    roll[seg] = 45 * np.sin(2 * np.pi * 0.5 * (t[seg] - 2))

    seg = (t >= 6) & (t < 10)                      # pitch sweep +/-30 deg, 0.5 Hz
    pitch[seg] = 30 * np.sin(2 * np.pi * 0.5 * (t[seg] - 6))

    seg = (t >= 10) & (t < 14)                     # full 360 deg roll, smooth ramp
    u = (t[seg] - 10) / 4
    roll[seg] = 360 * (u - np.sin(2 * np.pi * u) / (2 * np.pi))

    seg = (t >= 14) & (t < 18)                     # small combined motion
    roll[seg] = 15 * np.sin(2 * np.pi * 0.75 * (t[seg] - 14))   # whole cycles, so the
    pitch[seg] = 10 * np.sin(2 * np.pi * 0.5 * (t[seg] - 14))   # motion ends at 0 deg
    return roll, pitch


def make_imu(n_sec=20.0):
    rng = np.random.default_rng(SEED)
    dt = 1.0 / fm.FS_HZ
    t = np.arange(int(n_sec * fm.FS_HZ)) * dt
    roll_d, pitch_d = trajectory(t)
    phi, th = np.radians(roll_d), np.radians(pitch_d)

    # Physically correct body rates (ZYX Euler, zero yaw rate)
    phid = np.gradient(phi, dt)
    thd = np.gradient(th, dt)
    p = phid
    q = thd * np.cos(phi)

    # Gravity in body frame (g units)
    ax = -np.sin(th)
    ay = np.sin(phi) * np.cos(th)
    az = np.cos(phi) * np.cos(th)

    # Linear acceleration bump while level (someone bumps the board), 18.5-18.7 s
    bump = (t >= 18.5) & (t < 18.7)
    ax = ax + 0.3 * bump

    # Sensor errors
    ax = ax + rng.normal(0, 0.02, t.size)
    ay = ay + rng.normal(0, 0.02, t.size)
    az = az + rng.normal(0, 0.02, t.size)
    gx_dps = np.degrees(p) + 1.5 + rng.normal(0, 0.1, t.size)    # 1.5 dps bias
    gy_dps = np.degrees(q) - 1.0 + rng.normal(0, 0.1, t.size)    # -1.0 dps bias

    q16 = lambda v: np.clip(np.round(v), -32768, 32767).astype(int)
    raw = dict(
        ax=q16(ax * fm.ACC_LSB_PER_G), ay=q16(ay * fm.ACC_LSB_PER_G), az=q16(az * fm.ACC_LSB_PER_G),
        gx=q16(gx_dps * fm.GYR_LSB_PER_DPS), gy=q16(gy_dps * fm.GYR_LSB_PER_DPS),
    )
    return t, roll_d, pitch_d, raw


def h(v, bits):
    return format(v & ((1 << bits) - 1), f"0{bits // 4}x")


def write_imu_vectors():
    t, roll_d, pitch_d, raw = make_imu()
    core = fm.AttitudeCore()
    with open(os.path.join(OUT, "imu_stimulus.hex"), "w") as fs, \
         open(os.path.join(OUT, "imu_expected.hex"), "w") as fe, \
         open(os.path.join(OUT, "imu_truth.csv"), "w") as ft:
        ft.write("t,roll_deg,pitch_deg\n")
        for i in range(t.size):
            s = [int(raw[k][i]) for k in ("ax", "ay", "az", "gx", "gy")]
            r, p, ar, ap = core.step(*s)
            fs.write(" ".join(h(v, 16) for v in s) + "\n")
            fe.write(" ".join(h(v, 32) for v in (r, p, ar, ap)) + "\n")
            ft.write(f"{t[i]:.4f},{roll_d[i]:.6f},{pitch_d[i]:.6f}\n")
    return t.size


def write_cordic_vectors():
    rng = random.Random(SEED)
    W = fm.CORDIC_W
    lim = 1 << (fm.IN_SHIFT + 15)              # inputs as produced by v << 4
    pts = [(0, 0), (lim - 16, 0), (-lim, 0), (0, lim - 16), (0, -lim),
           (-lim, -lim), (lim - 16, lim - 16), (-lim, lim - 16), (lim - 16, -lim),
           (-16, 0), (-16, 16), (-16, -16), (1, -1)]
    for k in range(720):                       # unit circle, every 0.5 deg
        a = math.radians(k * 0.5)
        pts.append((int(lim * 0.99 * math.cos(a)), int(lim * 0.99 * math.sin(a))))
    for _ in range(2000):                      # random
        pts.append((rng.randrange(-lim, lim), rng.randrange(-lim, lim)))
    with open(os.path.join(OUT, "cordic_vectors.hex"), "w") as f:
        for x, y in pts:
            z, m = fm.cordic_vec(x, y)
            f.write(f"{h(x, W)} {h(y, W)} {h(z, 32)} {h(m, W)}\n")
    return len(pts)


if __name__ == "__main__":
    os.makedirs(OUT, exist_ok=True)
    n = write_imu_vectors()
    c = write_cordic_vectors()
    print(f"wrote {n} IMU samples and {c} CORDIC vectors to {os.path.normpath(OUT)}")
