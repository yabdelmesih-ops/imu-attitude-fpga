`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// imu_attitude_system.sv — board-independent signal chain.
//
//   IMU (SPI) -> imu_spi_reader -> attitude_core -> telemetry_tx -> UART -> PC
//
// Each board gets a thin top level (boards/<board>/) that supplies the clock
// frequency, reset polarity and LED mapping.
// -----------------------------------------------------------------------------
module imu_attitude_system #(
    parameter int CLK_HZ        = 27_000_000,
    parameter int SCLK_HZ       = 1_000_000,
    parameter int SAMPLE_HZ     = 1000,
    parameter int BAUD          = 115_200,
    parameter int DECIM         = 10,
    parameter int RESET_WAIT_MS = 100,
    parameter int STEP_WAIT_US  = 1000
) (
    input  logic        clk,
    input  logic        rst,          // synchronous, active high
    output logic        imu_sclk,
    output logic        imu_mosi,
    input  logic        imu_miso,
    output logic        imu_cs_n,
    output logic        uart_txd,
    // status for LEDs / debug
    output logic [7:0]  who_am_i,
    output logic        imu_running,
    output logic        cal_done,
    output logic        att_valid,    // 1-cycle pulse per filtered sample
    output logic [31:0] roll,
    output logic [31:0] pitch
);
    logic               s_valid, a_busy;
    logic signed [15:0] ax, ay, az, gx, gy, gz;
    logic        [31:0] acc_roll, acc_pitch;

    imu_spi_reader #(
        .CLK_HZ(CLK_HZ), .SCLK_HZ(SCLK_HZ), .SAMPLE_HZ(SAMPLE_HZ),
        .RESET_WAIT_MS(RESET_WAIT_MS), .STEP_WAIT_US(STEP_WAIT_US)
    ) u_imu (
        .clk(clk), .rst(rst),
        .sclk(imu_sclk), .mosi(imu_mosi), .miso(imu_miso), .cs_n(imu_cs_n),
        .sample_valid(s_valid), .ax(ax), .ay(ay), .az(az), .gx(gx), .gy(gy), .gz(gz),
        .who_am_i(who_am_i), .running(imu_running)
    );

    attitude_core u_att (
        .clk(clk), .rst(rst), .in_valid(s_valid),
        .ax(ax), .ay(ay), .az(az), .gx(gx), .gy(gy),
        .busy(a_busy), .out_valid(att_valid),
        .roll(roll), .pitch(pitch), .acc_roll(acc_roll), .acc_pitch(acc_pitch),
        .cal_done(cal_done)
    );

    telemetry_tx #(.CLK_HZ(CLK_HZ), .BAUD(BAUD), .DECIM(DECIM)) u_tlm (
        .clk(clk), .rst(rst), .in_valid(att_valid), .roll(roll), .pitch(pitch),
        .cal_done(cal_done), .imu_running(imu_running), .txd(uart_txd)
    );

endmodule
