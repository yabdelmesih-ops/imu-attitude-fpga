`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// top_basys3.sv — Digilent Basys 3 (Xilinx Artix-7 XC7A35T), alternate board.
//
// LEDs: [7:0] WHO_AM_I byte, [8] gyro calibration done, [9] IMU streaming,
//       [10] heartbeat (toggles every 500 samples), [15:11] roll bar graph.
// -----------------------------------------------------------------------------
module top_basys3 #(
    parameter int CLK_HZ        = 100_000_000,
    parameter int SCLK_HZ       = 1_000_000,
    parameter int SAMPLE_HZ     = 1000,
    parameter int BAUD          = 115_200,
    parameter int DECIM         = 10,
    parameter int RESET_WAIT_MS = 100,
    parameter int STEP_WAIT_US  = 1000
) (
    input  logic        clk,
    input  logic        btn_rst,      // centre button, active high
    output logic        imu_sclk,
    output logic        imu_mosi,
    input  logic        imu_miso,
    output logic        imu_cs_n,
    output logic        uart_txd,
    output logic [15:0] led
);
    logic [2:0] rst_sync = 3'b111;
    always_ff @(posedge clk) rst_sync <= {rst_sync[1:0], btn_rst};
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
        .uart_txd(uart_txd),
        .who_am_i(who_am_i), .imu_running(imu_running), .cal_done(cal_done),
        .att_valid(att_valid), .roll(roll), .pitch(pitch)
    );

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

    // roll8 is roll in units of 360/256 deg: centre LED = within +/-5.6 deg,
    // next LEDs out to +/-16.9 deg, outer LEDs beyond that
    wire signed [7:0] roll8 = roll[31:24];
    logic [4:0] bar;
    always_comb begin
        if      (roll8 < -12) bar = 5'b10000;
        else if (roll8 <  -4) bar = 5'b01000;
        else if (roll8 <=  4) bar = 5'b00100;
        else if (roll8 <= 12) bar = 5'b00010;
        else                  bar = 5'b00001;
    end

    assign led = {bar, hb, imu_running, cal_done, who_am_i};

endmodule
