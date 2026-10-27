`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// top_tang_nano_9k.sv — Sipeed Tang Nano 9K (Gowin GW1NR-LV9QN88PC6/I5).
//
// Clock  : on-board 27 MHz oscillator (pin 52)
// Reset  : button S1 (pin 3), active low
// UART   : pin 17 -> on-board BL702 USB-serial -> PC, 115200 8N1
// IMU    : SPI on header pins 25-28 (3.3 V bank)
// LEDs   : 6 on-board LEDs, active low (LED_ACTIVE_LOW = 1)
//   LED1  IMU detected (WHO_AM_I = 0x70 / 0x71 / 0x73)
//   LED2  IMU streaming
//   LED3  gyro bias calibration done (hold still ~1 s after reset)
//   LED4  heartbeat, toggles every 500 samples (1 Hz blink)
//   LED5  tilted left  (roll < -5.6 deg)
//   LED6  tilted right (roll > +5.6 deg)
// -----------------------------------------------------------------------------
module top_tang_nano_9k #(
    parameter int CLK_HZ         = 27_000_000,
    parameter int SCLK_HZ        = 1_000_000,
    parameter int SAMPLE_HZ      = 1000,
    parameter int BAUD           = 115_200,
    parameter int DECIM          = 10,
    parameter int RESET_WAIT_MS  = 100,
    parameter int STEP_WAIT_US   = 1000,
    parameter bit LED_ACTIVE_LOW = 1'b1
) (
    input  logic       clk_27m,
    input  logic       btn_s1_n,     // reset, active low
    output logic       imu_sclk,
    output logic       imu_mosi,
    input  logic       imu_miso,
    output logic       imu_cs_n,
    output logic       uart_txd,
    output logic [5:0] led
);
    // Reset: synchronize the button; also hold reset for the first cycles
    // after configuration.
    logic [2:0] rst_sync = 3'b111;
    always_ff @(posedge clk_27m) rst_sync <= {rst_sync[1:0], ~btn_s1_n};
    wire rst = rst_sync[2];

    logic [7:0]  who_am_i;
    logic        imu_running, cal_done, att_valid;
    logic [31:0] roll, pitch;

    imu_attitude_system #(
        .CLK_HZ(CLK_HZ), .SCLK_HZ(SCLK_HZ), .SAMPLE_HZ(SAMPLE_HZ), .BAUD(BAUD),
        .DECIM(DECIM), .RESET_WAIT_MS(RESET_WAIT_MS), .STEP_WAIT_US(STEP_WAIT_US)
    ) u_sys (
        .clk(clk_27m), .rst(rst),
        .imu_sclk(imu_sclk), .imu_mosi(imu_mosi), .imu_miso(imu_miso), .imu_cs_n(imu_cs_n),
        .uart_txd(uart_txd),
        .who_am_i(who_am_i), .imu_running(imu_running), .cal_done(cal_done),
        .att_valid(att_valid), .roll(roll), .pitch(pitch)
    );

    // Heartbeat
    logic [9:0] hb_cnt;
    logic       hb;
    always_ff @(posedge clk_27m) begin
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
    wire id_ok = (who_am_i == 8'h70) || (who_am_i == 8'h71) || (who_am_i == 8'h73);

    wire [5:0] led_on = {roll8 > 4, roll8 < -4, hb, cal_done, imu_running, id_ok};
    assign led = LED_ACTIVE_LOW ? ~led_on : led_on;

endmodule
