`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// imu_spi_reader.sv — configures an MPU-6500 / MPU-9250 family IMU over SPI,
// then burst-reads accel + gyro at a fixed rate.
//
// Register map assumed (MPU-6500 / MPU-9250 / ICM-20602-style):
//   0x3B..0x48  ACCEL_XOUT_H .. GYRO_ZOUT_L, big-endian, 14 bytes incl. temp
//   0x75        WHO_AM_I  (0x70 = MPU-6500, 0x71 = MPU-9250, 0x73 = MPU-9255)
// A read sets bit 7 of the register address.
//
// For a different IMU, edit init_step() and the burst start/order below.
// -----------------------------------------------------------------------------
module imu_spi_reader #(
    parameter int CLK_HZ        = 100_000_000,
    parameter int SCLK_HZ       = 1_000_000,
    parameter int SAMPLE_HZ     = 1000,
    parameter int RESET_WAIT_MS = 100,
    parameter int STEP_WAIT_US  = 1000
) (
    input  logic               clk,
    input  logic               rst,
    // SPI pins
    output logic               sclk,
    output logic               mosi,
    input  logic               miso,
    output logic               cs_n,
    // samples
    output logic               sample_valid,     // 1-cycle pulse
    output logic signed [15:0] ax, ay, az,
    output logic signed [15:0] gx, gy, gz,
    output logic        [7:0]  who_am_i,
    output logic               running           // init finished, streaming
);
    localparam int RESET_WAIT = CLK_HZ / 1000 * RESET_WAIT_MS;
    localparam int STEP_WAIT  = CLK_HZ / 1_000_000 * STEP_WAIT_US;
    localparam int PERIOD     = CLK_HZ / SAMPLE_HZ;
    localparam int N_INIT     = 8;
    localparam int N_BURST    = 14;

    // {register, value} written during init, in order. Step 0 is a device reset,
    // followed by RESET_WAIT; every other step is followed by STEP_WAIT.
    function automatic logic [15:0] init_step(input logic [3:0] i);
        case (i)
            4'd0: init_step = {8'h6B, 8'h80};   // PWR_MGMT_1: device reset
            4'd1: init_step = {8'h6B, 8'h01};   // PWR_MGMT_1: clock = PLL
            4'd2: init_step = {8'h6A, 8'h10};   // USER_CTRL: disable I2C, SPI only
            4'd3: init_step = {8'h19, 8'h00};   // SMPLRT_DIV: 1 kHz / (1+0)
            4'd4: init_step = {8'h1A, 8'h01};   // CONFIG: gyro DLPF 184 Hz
            4'd5: init_step = {8'h1B, 8'h00};   // GYRO_CONFIG: +/-250 dps (131 LSB/dps)
            4'd6: init_step = {8'h1C, 8'h00};   // ACCEL_CONFIG: +/-2 g (16384 LSB/g)
            4'd7: init_step = {8'h1D, 8'h01};   // ACCEL_CONFIG2: accel DLPF 184 Hz
            default: init_step = 16'h0000;
        endcase
    endfunction

    // SPI byte engine
    logic       s_start, s_hold, s_busy, s_done;
    logic [7:0] s_tx, s_rx;
    spi_master #(.CLK_HZ(CLK_HZ), .SCLK_HZ(SCLK_HZ)) u_spi (
        .clk(clk), .rst(rst), .start(s_start), .hold(s_hold), .tx_byte(s_tx), .rx_byte(s_rx),
        .busy(s_busy), .done(s_done), .sclk(sclk), .mosi(mosi), .miso(miso), .cs_n(cs_n)
    );

    typedef enum logic [3:0] {
        S_BOOT, S_W_ADDR, S_W_DATA, S_W_WAIT, S_WAIT,
        S_ID_ADDR, S_ID_DATA, S_ID_WAIT,
        S_RUN, S_R_ADDR, S_R_DATA, S_R_WAIT, S_OUT
    } state_t;
    state_t state;

    logic [31:0] timer, period_cnt;
    logic [3:0]  step;
    logic [3:0]  nbyte;
    logic [7:0]  buf_r [N_BURST];
    wire  [15:0] cur_step = init_step(step);

    always_ff @(posedge clk) begin
        s_start      <= 1'b0;
        sample_valid <= 1'b0;
        if (rst) begin
            state      <= S_BOOT;
            timer      <= '0;
            period_cnt <= '0;
            step       <= '0;
            nbyte      <= '0;
            s_hold     <= 1'b0;
            s_tx       <= '0;
            who_am_i   <= '0;
            running    <= 1'b0;
            {ax, ay, az, gx, gy, gz} <= '0;
        end else begin
            // free-running sample-period counter
            period_cnt <= (period_cnt == PERIOD - 1) ? '0 : period_cnt + 1'b1;

            case (state)
                S_BOOT: begin                       // short power-up delay
                    timer <= STEP_WAIT;
                    state <= S_WAIT;
                    step  <= '0;
                end
                // ---------------- register write: [addr][data] -------------
                S_W_ADDR: if (!s_busy && !s_start) begin
                    s_tx    <= cur_step[15:8];
                    s_hold  <= 1'b1;
                    s_start <= 1'b1;
                    state   <= S_W_DATA;
                end
                S_W_DATA: if (s_done) begin
                    s_tx    <= cur_step[7:0];
                    s_hold  <= 1'b0;
                    s_start <= 1'b1;
                    state   <= S_W_WAIT;
                end
                S_W_WAIT: if (s_done) begin
                    timer <= (step == 0) ? RESET_WAIT : STEP_WAIT;
                    step  <= step + 1'b1;
                    state <= S_WAIT;
                end
                S_WAIT: if (timer == 0) begin
                    state <= (step == N_INIT) ? S_ID_ADDR : S_W_ADDR;
                end else timer <= timer - 1'b1;
                // ---------------- WHO_AM_I --------------------------------
                S_ID_ADDR: if (!s_busy && !s_start) begin
                    s_tx    <= 8'h80 | 8'h75;
                    s_hold  <= 1'b1;
                    s_start <= 1'b1;
                    state   <= S_ID_DATA;
                end
                S_ID_DATA: if (s_done) begin
                    s_tx    <= 8'h00;
                    s_hold  <= 1'b0;
                    s_start <= 1'b1;
                    state   <= S_ID_WAIT;
                end
                S_ID_WAIT: if (s_done) begin
                    who_am_i <= s_rx;
                    running  <= 1'b1;
                    state    <= S_RUN;
                end
                // ---------------- streaming burst reads -------------------
                S_RUN: if (period_cnt == 0) state <= S_R_ADDR;
                S_R_ADDR: if (!s_busy && !s_start) begin
                    s_tx    <= 8'h80 | 8'h3B;       // ACCEL_XOUT_H, read
                    s_hold  <= 1'b1;
                    s_start <= 1'b1;
                    nbyte   <= '0;
                    state   <= S_R_DATA;
                end
                S_R_DATA: if (s_done) begin         // address byte done -> clock data
                    s_tx    <= 8'h00;
                    s_hold  <= 1'b1;
                    s_start <= 1'b1;
                    state   <= S_R_WAIT;
                end
                S_R_WAIT: if (s_done) begin
                    buf_r[nbyte] <= s_rx;
                    if (nbyte == N_BURST - 1) begin
                        state <= S_OUT;
                    end else begin
                        nbyte   <= nbyte + 1'b1;
                        s_tx    <= 8'h00;
                        s_hold  <= (nbyte + 1 != N_BURST - 1);   // release CS after last
                        s_start <= 1'b1;
                    end
                end
                S_OUT: begin
                    ax <= {buf_r[0],  buf_r[1]};
                    ay <= {buf_r[2],  buf_r[3]};
                    az <= {buf_r[4],  buf_r[5]};
                    // buf_r[6], buf_r[7] = temperature (unused)
                    gx <= {buf_r[8],  buf_r[9]};
                    gy <= {buf_r[10], buf_r[11]};
                    gz <= {buf_r[12], buf_r[13]};
                    sample_valid <= 1'b1;
                    state <= S_RUN;
                end
                default: state <= S_BOOT;
            endcase
        end
    end

endmodule
