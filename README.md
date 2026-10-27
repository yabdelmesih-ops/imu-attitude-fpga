# FPGA IMU Attitude Estimator

Real-time roll/pitch estimation in fixed-point hardware. An FPGA (Alchitry Au,
Xilinx Artix-7 XC7A35T) reads a 6-axis IMU over SPI at 1 kHz, computes tilt from the accelerometer with a time-shared
CORDIC, fuses it with the integrated gyro in a complementary filter, and streams
the attitude to a PC over UART.

**Status:** validated on hardware: the hardware-in-the-loop build on the Alchitry Au (Xilinx Artix-7) returns all 20,000 test samples bit-exact with the reference model, and meets timing at 100 MHz (WNS +1.163 ns). Live-IMU bring-up is next.

The design is verified **bit-exact** against a Python reference model, from the
CORDIC up through the full SPI → filter → UART chain.

```
          SPI (mode 3, 1 MHz)                         UART 115200, 100 Hz frames
 IMU ─────────────────────────▶ imu_spi_reader ──▶ attitude_core ──▶ telemetry_tx ──▶ PC
 MPU-6500/9250   init + 14-byte burst @ 1 kHz       │                               view_serial.py
                                                    ├─ CORDIC #1: roll  = atan2(ay, az)
                                                    ├─ ×1/K   : r = √(ay²+az²)
                                                    ├─ CORDIC #2: pitch = atan2(−ax, r)
                                                    ├─ gyro bias calibration (1 s at power-up)
                                                    └─ complementary filter, α = 1 − 2⁻⁹
```

![Simulation results](docs/attitude_results.png)

## Results (simulation, 20 s synthetic IMU run)

| Error vs. ground truth (deg) | Roll RMS | Roll max | Pitch RMS | Pitch max |
|---|---|---|---|---|
| Accelerometer only            | 1.17  | 4.71  | 2.02  | 19.59 |
| Gyro only (no bias cal)       | 16.44 | 28.71 | 12.68 | 21.29 |
| **FPGA filter**               | **0.12** | **0.74** | **0.73** | **5.42** |

* From t = 2 s (after calibration and settling) up to the deliberate 0.3 g, 0.2 s bump,
  filter RMS error is **0.08° roll / 0.07° pitch**.
* Fixed-point vs. double-precision filter: **0.0018°** worst-case difference.
* Latency: **40 clock cycles** per sample (0.40 µs at the Alchitry Au's 100 MHz clock),
  against a 1 ms budget.
* The test trajectory includes a full 360° roll through the ±180° wrap.

## Synthesis check (Alchitry Au, Artix-7 XC7A35T)

Open-source Yosys 0.70 `synth_xilinx` run (`yosys -s boards/alchitry_au/synth_check.ys`),
before place-and-route. It accepts all the SystemVerilog with no errors.

| Resource (XC7A35T) | Used | Available | |
|---|---|---|---|
| LUTs | ~1,060 | 20,800 | ~5 % |
| Flip-flops | 920 | 41,600 | ~2 % |
| DSP48E1 slices | 3 | 90 | ~3 % |

Vivado 2026.1 place-and-route of the hardware-in-the-loop build meets timing at 100 MHz with
**WNS +1.163 ns** (after pipelining the filter update; the first build failed at −1.463 ns). On the Tang Nano 9K
(Gowin GW1NR-9), the same design uses ~1,800 logic cells (~21 %) and 941 flip-flops
(`boards/tang_nano_9k/synth_check.ys`).

## Verification

Golden vectors come from `model/fixed_model.py`, a bit-exact Python model of the
hardware (same widths, shifts and rounding). Four self-checking testbenches:

| Testbench | What it checks | Vectors |
|---|---|---|
| `tb_cordic`        | CORDIC angle + magnitude, incl. axes, ±full-scale corners, all four quadrants | 2,733 |
| `tb_attitude_core` | every roll / pitch / accel-angle output word of the filter | 20,000 samples |
| `tb_top`           | full board top level (Alchitry Au by default; `-DTANG9K` or `-DBASYS3` for the other boards): IMU init register writes, WHO_AM_I readback, SPI burst reads from a behavioral MPU-6500 model, UART frames decoded and checked (header, checksum, sequence, cal flag, angles) | 2,400 samples / 600 frames |
| `tb_hil`           | hardware-in-the-loop build: UART sample frames in, 32-bit result frames out; reset command, a corrupted input frame (dropped and counted), PC clock 1 % off nominal, no dropped results | 3,600 results |

The testbenches were mutation-tested: a one-LSB change to the gyro scale constant,
a ±1 error in one CORDIC branch, and a swapped byte order in the SPI reader each
make the relevant test fail.

```bash
cd sim && ./run_all.sh                 # needs Icarus Verilog 12+, Python 3, numpy, matplotlib
cd sim && ./run_all.sh --all-boards    # also runs the system test on the Tang Nano 9K and Basys 3 tops
```

## Design notes

* **Binary angle (BAM) representation.** Angles are 32-bit unsigned with
  2³² = 360°. Wrap-around at ±180° is free modular arithmetic, so the filter
  needs no special case when the board rolls upside down.
* **One CORDIC, used twice.** An iterative vectoring CORDIC (16 iterations,
  24-bit datapath, no multipliers) computes roll and the magnitude
  √(ay²+az²) in one pass; the magnitude feeds a second pass for pitch. The
  CORDIC gain K ≈ 1.6468 is removed with one constant multiply.
* **Fixed-point formats.** Raw int16 samples get 4 guard bits in the CORDIC
  (worst-case growth is 1.65 × √2, which fits in 24 bits). The gyro-to-angle scale
  is `GYRO_K / 2⁸` BAM counts per LSB per sample (scale error 0.002 %).
* **Filter.** `pred = angle + gyro·dt` and `angle = pred + (accel_angle − pred) >> 9`.
  α = 1 − 2⁻⁹ makes the blend a shift instead of a multiply (time constant ≈ 0.51 s).
* **Timing closure.** The first Vivado build of the update step (multiply, add, subtract, shift,
  add in one cycle) failed at 100 MHz with WNS = −1.463 ns across 14 logic levels. The update is now
  pipelined into registered stages that run while the CORDIC computes pitch, costing one cycle
  of latency (39 → 40) with bit-identical results. Vivado 2026.1 then reports **WNS +1.163 ns** at
  100 MHz for the hardware-in-the-loop build on the XC7A35T: timing met.
* **Gyro bias calibration.** The first 1,024 samples are averaged per gyro axis
  and subtracted afterwards. Hold the board still for ~1 s after reset (LED2 lights on the Au).
* **CDC.** MISO and the reset button pass through synchronizers; the SPI master
  samples MISO late in the SCLK-high phase.

## Repository layout

```
rtl/                  board-independent design: cordic_vec, attitude_core, spi_master,
                      imu_spi_reader, uart_tx, telemetry_tx, imu_attitude_system,
                      uart_rx, hil_rx, hil_tx (hardware-in-the-loop)
boards/alchitry_au/   top_alchitry_au.sv + alchitry_au.xdc (IMU build),
                      top_alchitry_au_hil.sv + alchitry_au_hil.xdc (HIL build), synth checks
boards/tang_nano_9k/  top_tang_nano_9k.sv, tang_nano_9k.cst, tang_nano_9k.sdc (alternate board)
boards/basys3/        top_basys3.sv, basys3.xdc   (alternate board)
tb/                   tb_cordic, tb_attitude_core, tb_top, mpu_spi_model (IMU slave model)
model/                fixed_model.py (bit-exact), gen_stimulus.py, compare.py (+ float model, plot)
sim/                  run_all.sh, generated vectors/ and out/
tools/                view_serial.py (live telemetry viewer / CSV logger),
                      hil_run.py (hardware-in-the-loop streamer and checker)
docs/                 attitude_results.png
```

---

## Hardware bring-up (Alchitry Au, in progress)

### What to get
* **Alchitry Au** (Xilinx Artix-7 XC7A35T-1FTG256) and its USB-C cable.
* **Alchitry Br breakout**: the Au's I/O pins are on board-to-board connectors, and the Br
  turns them into 0.1" headers for jumper wires.
  (No Br? The SparkFun edition of the Au has a Qwiic I²C connector; an I²C version of the
  IMU reader for a Qwiic/STEMMA QT sensor is planned. Until then, use the hardware-in-the-loop
  build below.)
* **An SPI-capable 6/9-axis IMU in the MPU-6500 / MPU-9250 family**, e.g. a
  "GY-6500" or "GY-9250" breakout. The reader expects this register map.
  * The common **GY-521 (MPU-6050) is I²C-only and will not work** with this SPI reader.
  * ST LSM6DS3/LSM6DSOX boards can work, but need edits: `init_step()` (different
    registers), the burst start (0x22: gyro X/Y/Z then accel X/Y/Z, little-endian, so
    the byte unpacking in `S_OUT` changes), and `GYRO_K = 26724` (8.75 mdps/LSB at ±250 dps).
* Female–female jumper wires.

### Wiring (Alchitry Br, header bank A)

All four signals are on FPGA bank 14, which is fixed at 3.3 V. Match pin names against the
Br reference card / silkscreen before powering up.

| IMU pin | Function | Br pin | FPGA pin |
|---|---|---|---|
| VCC | 3.3 V | A7 (+V) | — |
| GND | ground | A4 (G) | — |
| NCS | chip select | A2 | T8 |
| SDA / SDI | MOSI | A3 | T7 |
| AD0 / SDO | MISO | A5 | T5 |
| SCL / SCLK | SPI clock | A6 | R5 |

Use a **+3.3 V** pin for VCC, not the +5 V pins (A1, A50).

### LEDs (active high)

| LED | Meaning |
|---|---|
| LED0 | IMU detected (WHO_AM_I = 0x70 / 0x71 / 0x73) |
| LED1 | IMU streaming |
| LED2 | gyro bias calibration done |
| LED3 | heartbeat, blinks at 1 Hz |
| LED4 / LED5 / LED6 | tilted left / level / tilted right (±5.6° threshold) |
| LED7 | upside down (\|roll\| > 90°) |

### Checklist
1. **Vivado**: new RTL project, part `xc7a35tftg256-1`. Add `rtl/*.sv` and
   `boards/alchitry_au/top_alchitry_au.sv` (file type SystemVerilog) and
   `boards/alchitry_au/alchitry_au.xdc`; top = `top_alchitry_au`. In *Settings → Bitstream*,
   enable **-bin_file**. Run synthesis and implementation, then record the timing summary
   (100 MHz) and utilization.
2. **Program** the generated `.bin` with the Alchitry Loader (part of Alchitry Labs): RAM for
   quick tests, flash to keep the design across power cycles.
3. **LED0 lights** when the IMU answers. If it stays dark, the SPI link is wrong: check the four
   signal wires and that VCC is on a +3.3 V pin.
4. Press **reset** with the board still: LED2 lights after ~1 s (gyro calibrated), LED1 =
   streaming, LED3 blinks at 1 Hz.
5. `python3 tools/view_serial.py --port <port> --plot`. The Au shows up as two serial ports;
   the UART is usually the second one.
6. Tilt and check signs. If an axis is mirrored for your mounting, negate or swap the
   samples between `u_imu` and `u_att` in `imu_attitude_system.sv`.
7. **Debugging**: `att_valid`, `roll` and `pitch` are marked `mark_debug` for Vivado's
   *Set Up Debug* (ILA), if the Vivado Hardware Manager detects the board. Otherwise, put a scope
   or cheap logic analyzer on the Br header pins to confirm the 1 MHz SPI clock and the 1 kHz
   burst rate.
8. Record timing slack and utilization from the implemented design for this README.

### Alternate boards
* **Tang Nano 9K** (Gowin GW1NR-9, 27 MHz): `boards/tang_nano_9k/`. IMU on header pins
  25 (CS), 26 (MOSI), 27 (MISO), 28 (SCLK) on a 3.3 V bank; six active-low LEDs.
  Latency is 1.48 µs at 27 MHz.
* **Basys 3** (Artix-7, Vivado): `boards/basys3/`. IMU on Pmod JA: JA1 = NCS, JA2 = MOSI,
  JA3 = MISO, JA4 = SCLK, JA5 = GND, JA6 = 3.3 V. LEDs 0–7 show the WHO_AM_I byte.

Any other board needs only a thin top level like these and a constraints file.

## Hardware-in-the-loop (no IMU needed)

**Result on the Alchitry Au (Oct 27, 2026):**

```
samples sent      : 20000
results received  : 20000
bit-exact matches : 20000/20000
lost results      : 0
time              : 3.94 s (5079 samples/s)
HIL PASS: hardware output is bit-exact with the model
```

![Roll and pitch computed by the FPGA hardware vs. ground truth](docs/hardware/hil_results.png)

![Alchitry Au running the hardware-in-the-loop build](docs/hardware/alchitry_au_running.png)

Vivado 2026.1 post-implementation utilization on the XC7A35T: **1,112 LUTs** (5 % of 20,800),
**871 flip-flops** (2 % of 41,600), **3 DSP48E1** (of 90), no block RAM.

The HIL build runs the real filter on the FPGA with recorded sensor data instead of a live IMU.
`tools/hil_run.py` streams all 20,000 test samples to the board over the Au's USB-serial link at
1 Mbaud; the FPGA returns full 32-bit roll/pitch for every sample, and the script checks each one
against the bit-exact model. It needs only the Au and its USB cable.

```
PC (hil_run.py) --USB--> uart_rx -> hil_rx -> attitude_core -> hil_tx -> uart_tx --USB--> PC
```

| Frame | Bytes |
|---|---|
| PC → FPGA, sample | `C3 3C` ax ay az gx gy (int16, big-endian) chk (XOR of the 10 data bytes) |
| PC → FPGA, reset | `C3 E1 1E` (restarts the filter, including gyro calibration) |
| FPGA → PC, result | `A5 5A` seq roll(u32) pitch(u32) chk (XOR of bytes 2..10) |

Yosys estimate for the HIL build on the XC7A35T: ~980 LUTs, 825 flip-flops, 3 DSP48E1.

**Steps**
1. **Vivado (scripted):** in the Tcl Console (*Window → Tcl Console*), run
   `source C:/path/to/imu-attitude-fpga/boards/alchitry_au/build_hil.tcl` (forward slashes).
   It creates the project in `vivado/au_hil/`, runs synthesis and implementation, writes
   `vivado/au_hil.bin`, saves timing and utilization reports, and prints the worst slack.
   Vivado 2026.1 needs the free *Vivado Basic Tier* node-locked license, which covers 7 Series
   devices but not the ILA debug core.

   **Or by hand:** new RTL project, part `xc7a35tftg256-1`. Add `rtl/cordic_vec.sv`,
   `rtl/attitude_core.sv`, `rtl/uart_rx.sv`, `rtl/uart_tx.sv`, `rtl/hil_rx.sv`, `rtl/hil_tx.sv`,
   `boards/alchitry_au/top_alchitry_au_hil.sv` (file type SystemVerilog) and
   `boards/alchitry_au/alchitry_au_hil.xdc`; top = `top_alchitry_au_hil`. In
   *Settings → Bitstream*, enable **-bin_file**. Run synthesis, implementation and bitstream
   generation; note the timing summary (WNS at 100 MHz) and utilization.
2. **Program** the `.bin` with the Alchitry Loader.
3. **On the PC** (Windows Python, not WSL, so it can open the COM port):
   ```
   py -m pip install pyserial numpy matplotlib
   py tools\hil_run.py --port COM5 --plot hil_results.png --csv hil_results.csv
   ```
   The Au appears as two COM ports in Device Manager; the UART is usually the higher number.
4. Expect `HIL PASS: hardware output is bit-exact with the model`. On the board, LED0 is the top LED
   of the column and LED7 the bottom. Verified on hardware: after reset only LED5 (level) is lit; after a
   passing 20,000-sample run, LED1 (calibrated) and LED5 are lit and the error LEDs (LED2, LED3) are off. LED0 blinks while data
   flows, LED1 lights after gyro calibration, LED2 = a corrupted input frame was seen,
   LED3 = a result was dropped (should never light).

`py tools\hil_run.py --selftest` runs the same script against a software model of the FPGA,
to check the PC side without a board.

## Limitations and next steps
* The filter treats gyro rates as Euler-angle rates, which is exact for single-axis
  motion and a small-angle approximation for combined roll and pitch. Next step:
  full Euler kinematics, using a CORDIC in rotation mode for sin/cos.
* Sustained linear acceleration pulls pitch (the bump in the plot). Next step: gate
  the accelerometer correction when |a| differs from 1 g.
* Pitch is limited to ±90° (roll is undefined straight up); yaw needs a magnetometer.
* **Fault tolerance (stretch goal):** triple the filter state registers, add a
  bitwise majority voter, and add a switch-driven fault injector that flips a bit in
  one copy. Show on hardware that the output is unaffected (TMR, as used in
  radiation-tolerant designs).
