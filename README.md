# FPGA IMU Attitude Estimator

Real-time roll/pitch estimation in fixed-point hardware. An FPGA (Sipeed Tang Nano 9K,
Gowin GW1NR-9) reads a 6-axis
IMU over SPI at 1 kHz, computes tilt from the accelerometer with a time-shared
CORDIC, fuses it with the integrated gyro in a complementary filter, and streams
the attitude to a PC over UART.

**Status:** design and verification complete in simulation; hardware bring-up on a Tang Nano 9K in progress.

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
* Latency: **39 clock cycles** per sample (1.44 µs at the Tang Nano 9K's 27 MHz; 0.39 µs at
  100 MHz on a Basys 3), against a 1 ms budget.
* The test trajectory includes a full 360° roll through the ±180° wrap.

## Synthesis check (Tang Nano 9K)

Open-source Yosys 0.70 `synth_gowin` run (`yosys -s boards/tang_nano_9k/synth_check.ys`),
before place-and-route. It accepts all the SystemVerilog with no errors.

| Resource (GW1NR-9) | Used | Available | |
|---|---|---|---|
| Logic (LUT4 + ALU cells) | ~1,800 | 8,640 | ~21 % |
| Flip-flops | 941 | 6,480 | ~15 % |
| Hardware multipliers | 3 × 18×18 + 2 × 9×9 | 20 blocks | |

Timing at 27 MHz still has to be confirmed by place-and-route in Gowin EDA.

## Verification

Golden vectors come from `model/fixed_model.py`, a bit-exact Python model of the
hardware (same widths, shifts and rounding). Three self-checking testbenches:

| Testbench | What it checks | Vectors |
|---|---|---|
| `tb_cordic`        | CORDIC angle + magnitude, incl. axes, ±full-scale corners, all four quadrants | 2,733 |
| `tb_attitude_core` | every roll / pitch / accel-angle output word of the filter | 20,000 samples |
| `tb_top`           | full board top level (Tang Nano 9K by default, `-DBASYS3` for Basys 3): IMU init register writes, WHO_AM_I readback, SPI burst reads from a behavioral MPU-6500 model, UART frames decoded and checked (header, checksum, sequence, cal flag, angles) | 2,400 samples / 600 frames |

The testbenches were mutation-tested: a one-LSB change to the gyro scale constant,
a ±1 error in one CORDIC branch, and a swapped byte order in the SPI reader each
make the relevant test fail.

```bash
cd sim && ./run_all.sh                 # needs Icarus Verilog 12+, Python 3, numpy, matplotlib
cd sim && ./run_all.sh --all-boards    # also runs the system test on the Basys 3 top
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
* **Gyro bias calibration.** The first 1,024 samples are averaged per gyro axis
  and subtracted afterwards. Hold the board still for ~1 s after reset (LED3 lights).
* **CDC.** MISO and the reset button pass through synchronizers; the SPI master
  samples MISO late in the SCLK-high phase.

## Repository layout

```
rtl/                  board-independent design: cordic_vec, attitude_core, spi_master,
                      imu_spi_reader, uart_tx, telemetry_tx, imu_attitude_system
boards/tang_nano_9k/  top_tang_nano_9k.sv, tang_nano_9k.cst (pins), tang_nano_9k.sdc (clock)
boards/basys3/        top_basys3.sv, basys3.xdc   (alternate board)
tb/                   tb_cordic, tb_attitude_core, tb_top, mpu_spi_model (IMU slave model)
model/                fixed_model.py (bit-exact), gen_stimulus.py, compare.py (+ float model, plot)
sim/                  run_all.sh, generated vectors/ and out/
tools/                view_serial.py  (live telemetry viewer / CSV logger)
docs/                 attitude_results.png
```

---

## Hardware bring-up (Tang Nano 9K, in progress)

### What to get
* **Sipeed Tang Nano 9K** (Gowin GW1NR-LV9QN88PC6/I5) and a USB-C data cable.
* **An SPI-capable 6/9-axis IMU in the MPU-6500 / MPU-9250 family**, e.g. a
  "GY-6500" or "GY-9250" breakout. The reader expects this register map.
  * The common **GY-521 (MPU-6050) is I²C-only and will not work** with this SPI reader.
  * ST LSM6DS3/LSM6DSOX boards can work, but need edits: `init_step()` (different
    registers), the burst start (0x22: gyro X/Y/Z then accel X/Y/Z, little-endian, so
    the byte unpacking in `S_OUT` changes), and `GYRO_K = 26724` (8.75 mdps/LSB at ±250 dps).
* Female–female jumper wires (or solder headers onto the Tang Nano 9K and use a breadboard).

### Wiring (Tang Nano 9K header)

The header pads are labelled with FPGA pin numbers. Pins 25–28 are on a **3.3 V bank**.
The pins on the left side of the chip (bank 3, including the LEDs and buttons) are
**1.8 V**: don't connect the 3.3 V IMU to those.

| IMU pin | Function | Tang Nano 9K |
|---|---|---|
| VCC | 3.3 V | 3V3 header pin |
| GND | ground | GND header pin |
| NCS | chip select | pin 25 |
| SDA / SDI | MOSI | pin 26 |
| AD0 / SDO | MISO | pin 27 |
| SCL / SCLK | SPI clock | pin 28 |

### LEDs (active low; "on" below means lit)

| LED | Meaning |
|---|---|
| LED1 | IMU detected (WHO_AM_I = 0x70 / 0x71 / 0x73) |
| LED2 | IMU streaming |
| LED3 | gyro bias calibration done |
| LED4 | heartbeat, blinks at 1 Hz |
| LED5 / LED6 | tilted left / right by more than ~5.6° |

If every LED is inverted from this (all lit at reset), set `LED_ACTIVE_LOW = 0`.

### Checklist
1. **Gowin EDA** (the free Education edition supports the GW1NR-9C): new FPGA design
   project, device `GW1NR-LV9QN88PC6/I5` (device family GW1NR-9C). Add `rtl/*.sv`,
   `boards/tang_nano_9k/top_tang_nano_9k.sv`, `tang_nano_9k.cst` and `tang_nano_9k.sdc`.
   Set *Project → Configuration → Synthesize → Verilog Language* to **SystemVerilog 2017**
   and the top module to `top_tang_nano_9k`. If place-and-route complains about a
   dual-purpose pin, enable that pin as regular I/O under *Place & Route → Dual-Purpose Pin*.
   Run synthesis and place-and-route, then check the timing report (27 MHz should pass with a
   large margin) and the resource usage.
   * Open-source alternative: Yosys (`synth_gowin`) + nextpnr-himbaechel + Apicula, then
     `openFPGALoader -b tangnano9k`.
2. Program with Gowin Programmer (SRAM mode for quick tests, embedded flash to keep the design
   across power cycles).
3. **LED1 lights** when the IMU answers. If it stays dark, the SPI link is wrong: check the four
   signal wires and that VCC is 3.3 V. Use the embedded logic analyzer (step 7).
4. Press **S1** (reset) with the board still: LED3 lights after ~1 s (gyro calibrated),
   LED2 = streaming, LED4 blinks at 1 Hz.
5. `python3 tools/view_serial.py --port <port> --plot`. The Tang Nano 9K shows up as two serial
   ports; the UART is usually the second one.
6. Tilt and check signs. If an axis is mirrored for your mounting, negate or swap the
   samples between `u_imu` and `u_att` in `imu_attitude_system.sv`.
7. **Debugging**: the Gowin Analyzer Oscilloscope (GAO) is Gowin's embedded logic analyzer,
   the equivalent of Vivado's ILA (check that your IDE edition includes it). Capture
   `u_sys.s_valid`, the raw samples, `roll` and `pitch`, trigger on `s_valid`, and compare with
   what the PC shows. Put a scope or cheap logic analyzer on pins 25–28 to confirm the 1 MHz
   SPI clock and the 1 kHz burst rate.
8. Record timing slack and utilization from the implemented design for this README.

### Alternate board: Basys 3
`boards/basys3/` has a top level and constraints for the Digilent Basys 3 (Artix-7, Vivado).
IMU on Pmod JA: JA1 = NCS, JA2 = MOSI, JA3 = MISO, JA4 = SCLK, JA5 = GND, JA6 = 3.3 V.
LEDs 0–7 show the WHO_AM_I byte. Debug nets are marked `mark_debug` for the Vivado ILA.
Any other board needs only a thin top level like these and a constraints file.

### If the IMU doesn't arrive: hardware-in-the-loop
Add a UART receiver that loads `ax..gy` from the PC, and stream
`sim/vectors/imu_stimulus.hex` from a Python script. The board then reproduces the
simulation on real silicon, and you can diff its output against `imu_expected.hex`.

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
