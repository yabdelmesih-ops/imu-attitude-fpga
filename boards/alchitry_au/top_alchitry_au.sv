`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// top_alchitry_au.sv — Alchitry Au (Xilinx Artix-7 XC7A35T-1FTG256).
//
// Clock  : on-board 100 MHz oscillator (N14)
// Reset  : on-board reset button (P6), active low
// UART   : P16 -> on-board USB-serial -> PC, 115200 8N1
// IMU    : SPI on header bank A (A2/A3/A5/A6, FPGA bank 14, 3.3 V),
//          reached through an Alchitry Br breakout
// LEDs   : 8 on-board LEDs, active high
//   LED0  IMU detected (WHO_AM_I = 0x70 / 0x71 / 0x73)
//   LED1  IMU streaming
//   LED2  gyro bias calibration done (hold still ~1 s after reset)
//   LED3  heartbeat, toggles every 500 samples (1 Hz blink)
//   LED4  tilted left  (roll < -5.6 deg)
//   LED5  level        (roll within +/-5.6 deg)
//   LED6  tilted right (roll > +5.6 deg)
//   LED7  upside down  (|roll| > 90 deg)
// -----------------------------------------------------------------------------
module top_alchitry_au #(
    parameter int CLK_HZ        = 100_000_000,
    parameter int SCLK_HZ       = 1_000_000,
    parameter int SAMPLE_HZ     = 1000,
    parameter int BAUD          = 115_200,
    parameter int DECIM         = 10,
    parameter int RESET_WAIT_MS = 100,
    parameter int STEP_WAIT_US  = 1000
) (
    input  logic       clk,
    input  logic       rst_n,        // reset button, active low
    output logic       imu_sclk,
    output logic       imu_mosi,
    input  logic       imu_miso,
    output logic       imu_cs_n,
    output logic       usb_tx,
    output logic [7:0] led
);
    // Reset: synchronize the button; hold reset for the first cycles after
    // configuration.
    logic [2:0] rst_sync = 3'b111;
    always_ff @(posedge clk) rst_sync <= {rst_sync[1:0], ~rst_n};
    wire rst = rst_sync[2];

    // mark_debug: these nets show up in Vivado's "Set Up Debug" wizard for the ILA
    (* mark_debug = "true" *) logic        att_valid;
    (* mark_debug = "true" *) logic [31:0] roll, pitch;
    logic [7:0] who_am_i;
    logic       imu_running, cal_done;

    imu_attitude_system #(
        .CLK_HZ(CLK_HZ), .SCLK_HZ(SCLK_HZ), .SAMPLE_HZ(SAMPLE_HZ), .BAUD(BAUD),
        .DECIM(DECIM), .RESET_WAIT_MS(RESET_WAIT_MS), .STEP_WAIT_US(STEP_WAIT_US)
    ) u_sys (
        .clk(clk), .rst(rst),
        .imu_sclk(imu_sclk), .imu_mosi(imu_mosi), .imu_miso(imu_miso), .imu_cs_n(imu_cs_n),
        .uart_txd(usb_tx),
        .who_am_i(who_am_i), .imu_running(imu_running), .cal_done(cal_done),
        .att_valid(att_valid), .roll(roll), .pitch(pitch)
    );

    // Heartbeat
    logic [9:0] hb_cnt;
    logic       hb;
    always_ff @(posedge clk) begin
        if (rst) begin
            hb_cnt <= '0;
            hb     <= 1'b0;
        end else if (att_valid) begin
            if (hb_cnt == 499) begin
                hb_cnt <= '0;
                hb     <= ~hb;
            end else hb_cnt <= hb_cnt + 1'b1;
        end
    end

    // roll8: roll in units of 360/256 deg (one step = 1.4 deg)
    wire signed [7:0] roll8 = roll[31:24];
    wire id_ok      = (who_am_i == 8'h70) || (who_am_i == 8'h71) || (who_am_i == 8'h73);
    wire tilt_left  = roll8 < -4;
    wire tilt_right = roll8 > 4;
    wire upside     = (roll8 > 64) || (roll8 < -64);

    assign led = {upside, tilt_right, ~tilt_left & ~tilt_right, tilt_left,
                  hb, cal_done, imu_running, id_ok};

endmodule
