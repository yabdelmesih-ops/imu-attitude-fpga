"""
Bit-exact reference model of the FPGA attitude core.

Every arithmetic step here mirrors rtl/attitude_core.sv and rtl/cordic_vec.sv
exactly (same widths, same shifts, same rounding), so the HDL output must match
this model bit-for-bit. A floating-point version of the same filter lives in compare.py
and is used to measure quantization error.

Number formats
--------------
  accel  : int16, ACC_LSB_PER_G  LSB per g    (MPU-6500/9250 at +/-2 g)
  gyro   : int16, GYR_LSB_PER_DPS LSB per deg/s (MPU-6500/9250 at +/-250 dps)
  angle  : 32-bit binary angle (BAM). 2**32 counts = 360 deg, wraps naturally,
           so +/-180 deg crossings need no special handling.
"""
import math

# ---- Design parameters (must match the HDL parameters) ----------------------
FS_HZ           = 1000          # IMU sample rate
ACC_LSB_PER_G   = 16384
GYR_LSB_PER_DPS = 131.0

CORDIC_W     = 24               # CORDIC datapath width (signed)
CORDIC_N     = 16               # iterations
IN_SHIFT     = 4                # raw int16 -> CORDIC input: v << 4 (4 guard bits)
KINV_SHIFT   = 16
GYRO_SHIFT   = 8
ALPHA_SHIFT  = 9                # alpha = 1 - 2**-9  (tau ~= 0.51 s at 1 kHz)
CAL_LOG2     = 10               # gyro bias: average first 1024 samples (~1 s, held still)

M32 = (1 << 32) - 1


def _atan_table(n):
    return [int(round(math.atan(2.0 ** -i) * (1 << 32) / (2 * math.pi))) for i in range(n)]


ATAN_TABLE = _atan_table(CORDIC_N)

# CORDIC gain and its reciprocal in Q0.16
_K = 1.0
for _i in range(CORDIC_N):
    _K *= math.sqrt(1 + 2.0 ** (-2 * _i))
CORDIC_GAIN = _K
KINV = int(round((1 << KINV_SHIFT) / _K))                 # 39797

# Gyro LSB -> BAM32 counts per sample, as integer * 2**-GYRO_SHIFT
GYRO_K = int(round((1 << 32) / (360.0 * GYR_LSB_PER_DPS * FS_HZ) * (1 << GYRO_SHIFT)))


# ---- Helpers -----------------------------------------------------------------
def wrap_signed(v, bits):
    """Wrap an integer to a two's-complement value of the given width."""
    m = 1 << bits
    v &= m - 1
    return v - m if v >= (m >> 1) else v


def bam_to_deg(b):
    return wrap_signed(b, 32) * 360.0 / (1 << 32)


def deg_to_bam(d):
    return int(round(d / 360.0 * (1 << 32))) & M32


# ---- CORDIC (vectoring mode): returns (angle_bam32, magnitude*K) -------------
def cordic_vec(x, y):
    """x, y: signed CORDIC_W-bit inputs. Returns (z: uint32 BAM, x_final: signed)."""
    W = CORDIC_W
    x = wrap_signed(x, W)
    y = wrap_signed(y, W)
    z = 0
    if x < 0:                       # pre-rotate by 180 deg into the right half-plane
        x, y = wrap_signed(-x, W), wrap_signed(-y, W)
        z = 1 << 31
    for i in range(CORDIC_N):
        xs, ys = x >> i, y >> i     # Python >> on ints == arithmetic shift (floor)
        if y >= 0:
            x, y, z = x + ys, y - xs, z + ATAN_TABLE[i]
        else:
            x, y, z = x - ys, y + xs, z - ATAN_TABLE[i]
        x, y, z = wrap_signed(x, W), wrap_signed(y, W), z & M32
    return z, x


# ---- Accelerometer tilt (roll from CORDIC #1, pitch from CORDIC #2) ---------
def accel_angles(ax, ay, az):
    """Returns (roll_bam, pitch_bam) computed purely from the accelerometer."""
    roll, mag_k = cordic_vec(az << IN_SHIFT, ay << IN_SHIFT)       # atan2(ay, az)
    mag = (mag_k * KINV) >> KINV_SHIFT                              # sqrt(ay^2+az^2)
    pitch, _ = cordic_vec(mag, -(ax << IN_SHIFT))                   # atan2(-ax, mag)
    return roll, pitch


class AttitudeCore:
    """Complementary filter, one update per IMU sample.

    Gyro bias calibration: the first 2**cal_log2 samples (board held still at
    power-up) are averaged per gyro axis; that average is subtracted from every
    later sample. Before calibration completes, bias = 0.
    """

    def __init__(self, alpha_shift=ALPHA_SHIFT, cal_log2=CAL_LOG2):
        self.alpha_shift = alpha_shift
        self.cal_log2 = cal_log2
        self.roll = 0
        self.pitch = 0
        self.init = False
        self.n = 0
        self.sum_x = self.sum_y = 0
        self.bias_x = self.bias_y = 0
        self.cal_done = False

    def _update(self, angle, acc_angle, gyro):
        d = (gyro * GYRO_K) >> GYRO_SHIFT                 # gyro increment, BAM32
        pred = (angle + d) & M32
        err = wrap_signed(acc_angle - pred, 32)           # shortest signed difference
        return (pred + (err >> self.alpha_shift)) & M32

    def step(self, ax, ay, az, gx, gy):
        acc_roll, acc_pitch = accel_angles(ax, ay, az)
        rx, ry = gx - self.bias_x, gy - self.bias_y        # 17-bit signed in HDL
        if not self.init:
            self.roll, self.pitch, self.init = acc_roll, acc_pitch, True
        else:
            self.roll = self._update(self.roll, acc_roll, rx)
            self.pitch = self._update(self.pitch, acc_pitch, ry)
        if not self.cal_done:
            self.sum_x += gx
            self.sum_y += gy
            self.n += 1
            if self.n == (1 << self.cal_log2):
                self.bias_x = self.sum_x >> self.cal_log2
                self.bias_y = self.sum_y >> self.cal_log2
                self.cal_done = True
        return self.roll, self.pitch, acc_roll, acc_pitch


if __name__ == "__main__":
    print(f"KINV={KINV}  GYRO_K={GYRO_K}  CORDIC gain={CORDIC_GAIN:.6f}")
    for a in (0, 30, 45, 90, 135, 179.9, -179.9, -90, -30):
        r = math.radians(a)
        z, _ = cordic_vec(int(16384 * 16 * math.cos(r)), int(16384 * 16 * math.sin(r)))
        print(f"{a:8.2f} deg -> {bam_to_deg(z):9.4f}")
