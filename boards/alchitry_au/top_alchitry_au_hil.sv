`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// top_alchitry_au_hil.sv — hardware-in-the-loop build for the Alchitry Au.
//
// No IMU needed. The PC streams recorded IMU samples over the USB-serial link
// (tools/hil_run.py); the FPGA runs them through the same attitude_core used in
// the real build and returns full 32-bit roll/pitch for every sample. The PC
// compares each result with the bit-exact Python model.
//
//   PC --USB--> uart_rx -> hil_rx -> attitude_core -> hil_tx -> uart_tx --USB--> PC
//
// Clock  : 100 MHz (N14)      Reset : button (P6), active low
// UART   : usb_rx P15 (PC -> FPGA), usb_tx P16 (FPGA -> PC), 1,000,000 baud 8N1
// LEDs   : LED0  activity (toggles every 256 samples)
//          LED1  gyro bias calibration done
//          LED2  checksum error seen on an input frame
//          LED3  result frame dropped (should never light)
//          LED4/5/6 tilted left / level / tilted right, LED7 upside down
// -----------------------------------------------------------------------------
module top_alchitry_au_hil #(
    parameter int CLK_HZ = 100_000_000,
    parameter int BAUD   = 1_000_000
) (
    input  logic       clk,
    input  logic       rst_n,
    input  logic       usb_rx,
    output logic       usb_tx,
    output logic [7:0] led
);
    logic [2:0] rst_sync = 3'b111;
    always_ff @(posedge clk) rst_sync <= {rst_sync[1:0], ~rst_n};
    wire rst = rst_sync[2];

    // UART in
    logic [7:0] rx_data;
    logic       rx_valid, rx_ferr;
    uart_rx #(.CLK_HZ(CLK_HZ), .BAUD(BAUD)) u_rx (
        .clk(clk), .rst(rst), .rxd(usb_rx), .data(rx_data), .valid(rx_valid), .frame_err(rx_ferr)
    );

    // Frame decoder
    (* mark_debug = "true" *) logic        s_valid;
    (* mark_debug = "true" *) logic signed [15:0] ax, ay, az, gx, gy;
    logic       soft_reset;
    logic [7:0] bad_frames;
    hil_rx u_dec (
        .clk(clk), .rst(rst), .rx_data(rx_data), .rx_valid(rx_valid),
        .sample_valid(s_valid), .ax(ax), .ay(ay), .az(az), .gx(gx), .gy(gy),
        .soft_reset(soft_reset), .bad_frames(bad_frames)
    );

    // Attitude core (same module as the IMU build). The PC's reset frame
    // restarts it, including gyro bias calibration.
    logic core_rst;
    always_ff @(posedge clk) core_rst <= rst | soft_reset;

    (* mark_debug = "true" *) logic        a_valid;
    (* mark_debug = "true" *) logic [31:0] roll, pitch;
    logic [31:0] acc_roll, acc_pitch;
    logic        a_busy, cal_done;
    attitude_core u_att (
        .clk(clk), .rst(core_rst), .in_valid(s_valid),
        .ax(ax), .ay(ay), .az(az), .gx(gx), .gy(gy),
        .busy(a_busy), .out_valid(a_valid),
        .roll(roll), .pitch(pitch), .acc_roll(acc_roll), .acc_pitch(acc_pitch),
        .cal_done(cal_done)
    );

    // Results out
    logic [7:0] dropped;
    hil_tx #(.CLK_HZ(CLK_HZ), .BAUD(BAUD)) u_tx (
        .clk(clk), .rst(core_rst), .in_valid(a_valid), .roll(roll), .pitch(pitch),
        .txd(usb_tx), .dropped(dropped)
    );

    // Status LEDs
    logic [7:0] act_cnt;
    logic       act;
    always_ff @(posedge clk) begin
        if (rst) begin
            act_cnt <= '0;
            act     <= 1'b0;
        end else if (a_valid) begin
            act_cnt <= act_cnt + 1'b1;
            if (&act_cnt) act <= ~act;
        end
    end

    wire signed [7:0] roll8 = roll[31:24];
    wire tilt_left  = roll8 < -4;
    wire tilt_right = roll8 > 4;
    wire upside     = (roll8 > 64) || (roll8 < -64);

    assign led = {upside, tilt_right, ~tilt_left & ~tilt_right, tilt_left,
                  dropped != 0, bad_frames != 0, cal_done, act};

endmodule
