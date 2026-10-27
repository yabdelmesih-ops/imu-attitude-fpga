`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// attitude_core.sv — fixed-point roll/pitch complementary filter.
//
// Gyro bias is calibrated at power-up from the first 2^CAL_LOG2 samples
// (hold the board still for ~1 s) and subtracted from every later sample.
//
// Per IMU sample (in_valid pulse):
//   1. CORDIC #1:  acc_roll  = atan2(ay, az),   r = K*sqrt(ay^2 + az^2)
//   2. scale:      r'        = r * (1/K)                    (one constant multiply)
//   3. CORDIC #2:  acc_pitch = atan2(-ax, r')
//   4. filter:     pred  = angle + gyro * GYRO_K / 2^GYRO_SHIFT
//                  angle = pred + (acc_angle - pred) / 2^ALPHA_SHIFT
//
// A single CORDIC instance is time-shared for both tilt angles (~40 clocks per
// sample, vs 100,000 clocks available at 100 MHz / 1 kHz).
//
// Angles are 32-bit binary angles (BAM): 2^32 counts = 360 deg, so the filter
// wraps correctly through +/-180 deg using plain modular arithmetic.
//
// Bit-exact with model/fixed_model.py::AttitudeCore.
// -----------------------------------------------------------------------------
module attitude_core #(
    parameter int          ALPHA_SHIFT = 9,      // alpha = 1 - 2^-9, tau ~= 0.51 s at 1 kHz
    parameter int          GYRO_SHIFT  = 8,
    parameter int          GYRO_K      = 23314,  // round(2^32/(360*131*1000) * 2^8)
    parameter int          KINV        = 39797,  // round(2^16 / CORDIC gain)
    parameter int          IN_SHIFT    = 4,
    parameter int          CAL_LOG2    = 10      // gyro bias = mean of first 2^10 samples
) (
    input  logic               clk,
    input  logic               rst,
    input  logic               in_valid,
    input  logic signed [15:0] ax, ay, az,       // raw accelerometer, LSB
    input  logic signed [15:0] gx, gy,           // raw gyro (roll rate, pitch rate), LSB
    output logic               busy,
    output logic               out_valid,        // 1-cycle pulse
    output logic        [31:0] roll,             // filtered, BAM32
    output logic        [31:0] pitch,
    output logic        [31:0] acc_roll,         // accelerometer-only, for debug/plots
    output logic        [31:0] acc_pitch,
    output logic               cal_done          // gyro bias calibration finished
);
    localparam int W = 24;

    typedef enum logic [2:0] {S_IDLE, S_ROLL, S_SCALE, S_PITCH, S_UPDATE} state_t;
    state_t state;

    // Latched sample
    logic signed [15:0] ax_r, gx_r, gy_r;
    logic               init_done;

    // Gyro bias calibration (board must be still for the first 2^CAL_LOG2 samples)
    logic signed [31:0]   sum_x, sum_y;
    logic signed [15:0]   bias_x, bias_y;
    logic [CAL_LOG2-1:0]  cal_cnt;
    wire  signed [31:0]   sum_x_next = sum_x + 32'(gx_r);
    wire  signed [31:0]   sum_y_next = sum_y + 32'(gy_r);
    wire  signed [16:0]   rate_x = 17'(gx_r) - 17'(bias_x);
    wire  signed [16:0]   rate_y = 17'(gy_r) - 17'(bias_y);

    // CORDIC interface
    logic                c_start, c_busy, c_done;
    logic signed [W-1:0] c_x, c_y, c_mag;
    logic        [31:0]  c_angle;
    logic signed [W-1:0] mag_r;

    cordic_vec #(.W(W), .N(16)) u_cordic (
        .clk(clk), .rst(rst), .start(c_start), .x_in(c_x), .y_in(c_y),
        .busy(c_busy), .done(c_done), .angle(c_angle), .mag(c_mag)
    );

    // Sign-extend a raw 16-bit sample into the CORDIC width with guard bits.
    function automatic logic signed [W-1:0] to_cordic(input logic signed [15:0] v);
        to_cordic = W'(v) <<< IN_SHIFT;
    endfunction

    // One complementary-filter update. All angle math is modulo 2^32.
    function automatic logic [31:0] cf_update(input logic [31:0] angle,
                                              input logic [31:0] acc_angle,
                                              input logic signed [16:0] rate);
        logic signed [47:0] prod;
        logic        [31:0] d, pred;
        logic signed [31:0] err;
        prod = 48'(rate) * 48'(GYRO_K);           // signed 17-bit x constant
        d    = 32'(prod >>> GYRO_SHIFT);
        pred = angle + d;
        err  = $signed(acc_angle - pred);
        cf_update = pred + 32'(err >>> ALPHA_SHIFT);
    endfunction

    // Magnitude correction r' = (r * KINV) >> 16
    logic signed [47:0] mag_prod;
    assign mag_prod = 48'(mag_r) * 48'(KINV);

    always_ff @(posedge clk) begin
        c_start   <= 1'b0;
        out_valid <= 1'b0;
        if (rst) begin
            state     <= S_IDLE;
            init_done <= 1'b0;
            roll      <= '0;
            pitch     <= '0;
            acc_roll  <= '0;
            acc_pitch <= '0;
            ax_r      <= '0;
            gx_r      <= '0;
            gy_r      <= '0;
            mag_r     <= '0;
            sum_x     <= '0;
            sum_y     <= '0;
            bias_x    <= '0;
            bias_y    <= '0;
            cal_cnt   <= '0;
            cal_done  <= 1'b0;
            c_x       <= '0;
            c_y       <= '0;
        end else begin
            case (state)
                S_IDLE: if (in_valid) begin
                    ax_r    <= ax;
                    gx_r    <= gx;
                    gy_r    <= gy;
                    c_x     <= to_cordic(az);       // roll = atan2(ay, az)
                    c_y     <= to_cordic(ay);
                    c_start <= 1'b1;
                    state   <= S_ROLL;
                end
                S_ROLL: if (c_done) begin
                    acc_roll <= c_angle;
                    mag_r    <= c_mag;
                    state    <= S_SCALE;
                end
                S_SCALE: begin                      // pitch = atan2(-ax, sqrt(ay^2+az^2))
                    c_x     <= W'(mag_prod >>> 16);
                    c_y     <= -to_cordic(ax_r);
                    c_start <= 1'b1;
                    state   <= S_PITCH;
                end
                S_PITCH: if (c_done) begin
                    acc_pitch <= c_angle;
                    state     <= S_UPDATE;
                end
                S_UPDATE: begin
                    if (!init_done) begin           // seed from the accelerometer
                        roll      <= acc_roll;
                        pitch     <= acc_pitch;
                        init_done <= 1'b1;
                    end else begin
                        roll  <= cf_update(roll,  acc_roll,  rate_x);
                        pitch <= cf_update(pitch, acc_pitch, rate_y);
                    end
                    if (!cal_done) begin
                        sum_x   <= sum_x_next;
                        sum_y   <= sum_y_next;
                        cal_cnt <= cal_cnt + 1'b1;
                        if (&cal_cnt) begin         // last calibration sample
                            bias_x   <= 16'(sum_x_next >>> CAL_LOG2);
                            bias_y   <= 16'(sum_y_next >>> CAL_LOG2);
                            cal_done <= 1'b1;
                        end
                    end
                    out_valid <= 1'b1;
                    state     <= S_IDLE;
                end
                default: state <= S_IDLE;
            endcase
        end
    end

    assign busy = (state != S_IDLE);

endmodule
