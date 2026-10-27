`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// cordic_vec.sv — iterative CORDIC in vectoring mode.
//
// Computes   angle = atan2(y_in, x_in)          (32-bit binary angle, 2^32 = 360 deg)
//            mag   = K * sqrt(x_in^2 + y_in^2)  (K ~= 1.64676, removed by the caller)
//
// One iteration per clock, no multipliers: N+1 cycles from start to done.
// Inputs in the left half-plane are pre-rotated by 180 deg so the full
// +/-180 deg range converges.
//
// Bit-exact with model/fixed_model.py::cordic_vec().
// -----------------------------------------------------------------------------
module cordic_vec #(
    parameter int W = 24,   // datapath width (signed)
    parameter int N = 16    // iterations (max 16 with this LUT)
) (
    input  logic                clk,
    input  logic                rst,
    input  logic                start,     // ignored while busy
    input  logic signed [W-1:0] x_in,
    input  logic signed [W-1:0] y_in,
    output logic                busy,
    output logic                done,      // 1-cycle pulse; outputs valid from here on
    output logic        [31:0]  angle,
    output logic signed [W-1:0] mag
);

    // atan(2^-i) in BAM32 units, round(atan(2^-i) * 2^32 / 2pi)
    function automatic logic [31:0] atan_lut(input logic [4:0] i);
        case (i)
            5'd0:  atan_lut = 32'd536870912;
            5'd1:  atan_lut = 32'd316933406;
            5'd2:  atan_lut = 32'd167458907;
            5'd3:  atan_lut = 32'd85004756;
            5'd4:  atan_lut = 32'd42667331;
            5'd5:  atan_lut = 32'd21354465;
            5'd6:  atan_lut = 32'd10679838;
            5'd7:  atan_lut = 32'd5340245;
            5'd8:  atan_lut = 32'd2670163;
            5'd9:  atan_lut = 32'd1335087;
            5'd10: atan_lut = 32'd667544;
            5'd11: atan_lut = 32'd333772;
            5'd12: atan_lut = 32'd166886;
            5'd13: atan_lut = 32'd83443;
            5'd14: atan_lut = 32'd41722;
            5'd15: atan_lut = 32'd20861;
            default: atan_lut = 32'd0;
        endcase
    endfunction

    logic signed [W-1:0] x, y;
    logic        [31:0]  z;
    logic        [4:0]   iter;

    wire signed [W-1:0] xs = x >>> iter;
    wire signed [W-1:0] ys = y >>> iter;

    always_ff @(posedge clk) begin
        done <= 1'b0;
        if (rst) begin
            busy  <= 1'b0;
            iter  <= '0;
            x     <= '0;
            y     <= '0;
            z     <= '0;
        end else if (!busy) begin
            if (start) begin
                busy <= 1'b1;
                iter <= '0;
                if (x_in[W-1]) begin          // x < 0: rotate by 180 deg
                    x <= -x_in;
                    y <= -y_in;
                    z <= 32'h8000_0000;
                end else begin
                    x <= x_in;
                    y <= y_in;
                    z <= 32'h0000_0000;
                end
            end
        end else begin
            if (!y[W-1]) begin                // y >= 0: rotate clockwise
                x <= x + ys;
                y <= y - xs;
                z <= z + atan_lut(iter);
            end else begin                    // y < 0: rotate counter-clockwise
                x <= x - ys;
                y <= y + xs;
                z <= z - atan_lut(iter);
            end
            if (iter == N - 1) begin
                busy <= 1'b0;
                done <= 1'b1;
            end
            iter <= iter + 1'b1;
        end
    end

    // Results: take the final iteration's values the cycle done is asserted.
    always_ff @(posedge clk) begin
        if (rst) begin
            angle <= '0;
            mag   <= '0;
        end else if (busy && iter == N - 1) begin
            mag   <= (!y[W-1]) ? (x + ys) : (x - ys);
            angle <= (!y[W-1]) ? (z + atan_lut(iter)) : (z - atan_lut(iter));
        end
    end

endmodule
