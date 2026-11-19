`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// top_alchitry_au_live.sv: live sensor build for the Alchitry Au.
//
//   MPU-6050 (Qwiic, I2C) -> mpu6050_i2c_reader -> attitude_core -> telemetry_tx
//                                                   -> USB-UART -> PC
//
// Clock  : on-board 100 MHz oscillator (N14)
// Reset  : on-board reset button (P6), active low
// Sensor : Qwiic port, SCL = E6, SDA = K5 (3.3 V), I2C at 350 kHz, 1 kHz samples
// UART   : P16 -> on-board USB-serial -> PC, 115200 8N1, 100 frames/s
//          (view with tools/view_serial.py)
//
// Hold the sensor still for about 2 s after reset: the gyro bias is measured
// from the first 1,024 samples.
//
// LEDs (LED0 = top):
//   LED0  sensor answered with WHO_AM_I = 0x68
//   LED1  sensor streaming
//   LED2  gyro calibration done
//   LED3  heartbeat (1 Hz blink while samples flow)
//   LED4  tilted left  (roll < -5.6 deg)
//   LED5  level        (roll within +/-5.6 deg)
//   LED6  tilted right (roll > +5.6 deg)
//   LED7  I2C error: the sensor missed an ACK at least once since reset
// -----------------------------------------------------------------------------
module top_alchitry_au_live #(
    parameter int CLK_HZ        = 100_000_000,
    parameter int I2C_HZ        = 350_000,
    parameter int SAMPLE_HZ     = 1000,
    parameter int BAUD          = 115_200,
    parameter int DECIM         = 10,
    parameter int RESET_WAIT_MS = 100,
    parameter int STEP_WAIT_US  = 1000
) (
    input  logic       clk,
    input  logic       rst_n,
    output logic       usb_tx,
    output logic [7:0] led,
    inout  wire        qwiic_scl,
    inout  wire        qwiic_sda
);
    logic [2:0] rst_sync = 3'b111;
    always_ff @(posedge clk) rst_sync <= {rst_sync[1:0], ~rst_n};
    wire rst = rst_sync[2];

    // Open-drain I2C pins
    logic scl_oe, sda_oe;
    assign qwiic_scl = scl_oe ? 1'b0 : 1'bz;
    assign qwiic_sda = sda_oe ? 1'b0 : 1'bz;
    logic [1:0] sda_sync;
    always_ff @(posedge clk) sda_sync <= {sda_sync[0], qwiic_sda};

    // Sensor
    logic               s_valid, imu_running, nack_seen;
    logic signed [15:0] ax, ay, az, gx, gy, gz;
    logic        [7:0]  who_am_i;

    mpu6050_i2c_reader #(
        .CLK_HZ(CLK_HZ), .I2C_HZ(I2C_HZ), .SAMPLE_HZ(SAMPLE_HZ),
        .RESET_WAIT_MS(RESET_WAIT_MS), .STEP_WAIT_US(STEP_WAIT_US)
    ) u_imu (
        .clk(clk), .rst(rst),
        .scl_oe(scl_oe), .sda_oe(sda_oe), .sda_in(sda_sync[1]),
        .sample_valid(s_valid), .ax(ax), .ay(ay), .az(az), .gx(gx), .gy(gy), .gz(gz),
        .who_am_i(who_am_i), .running(imu_running), .nack_seen(nack_seen)
    );

    // Filter
    logic        a_busy, att_valid, cal_done;
    logic [31:0] roll, pitch, acc_roll, acc_pitch;

    attitude_core u_att (
        .clk(clk), .rst(rst), .in_valid(s_valid),
        .ax(ax), .ay(ay), .az(az), .gx(gx), .gy(gy),
        .busy(a_busy), .out_valid(att_valid),
        .roll(roll), .pitch(pitch), .acc_roll(acc_roll), .acc_pitch(acc_pitch),
        .cal_done(cal_done)
    );

    // Telemetry to the PC
    telemetry_tx #(.CLK_HZ(CLK_HZ), .BAUD(BAUD), .DECIM(DECIM)) u_tlm (
        .clk(clk), .rst(rst), .in_valid(att_valid), .roll(roll), .pitch(pitch),
        .cal_done(cal_done), .imu_running(imu_running), .txd(usb_tx)
    );

    // Heartbeat: toggles every 500 samples
    logic [9:0] hb_cnt;
    logic       hb;
    always_ff @(posedge clk) begin
        if (rst) begin
            hb_cnt <= '0;
            hb     <= 1'b0;
        end else if (att_valid) begin
            if (hb_cnt == 499) begin hb_cnt <= '0; hb <= ~hb; end
            else hb_cnt <= hb_cnt + 1'b1;
        end
    end

    // roll8: roll in units of 360/256 deg (one step = 1.4 deg)
    wire signed [7:0] roll8 = roll[31:24];
    wire id_ok      = (who_am_i == 8'h68);
    wire tilt_left  = roll8 < -4;
    wire tilt_right = roll8 > 4;

    assign led = {nack_seen, tilt_right, ~tilt_left & ~tilt_right, tilt_left,
                  hb, cal_done, imu_running, id_ok};
endmodule
