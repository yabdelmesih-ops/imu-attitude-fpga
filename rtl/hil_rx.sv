`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// hil_rx.sv — decodes hardware-in-the-loop command frames from the PC.
//
//   Sample frame (13 bytes):  C3 3C  axH axL ayH ayL azH azL gxH gxL gyH gyL  chk
//   Reset  frame ( 3 bytes):  C3 E1  1E
//     chk = XOR of the 10 data bytes (sample frame); the reset frame's last byte
//     is the fixed value 0x1E.
//
// A good sample frame produces one sample_valid pulse with the five values.
// A good reset frame produces one soft_reset pulse. Bad checksums are counted
// and the frame is dropped.
// -----------------------------------------------------------------------------
module hil_rx (
    input  logic               clk,
    input  logic               rst,
    input  logic [7:0]         rx_data,
    input  logic               rx_valid,
    output logic               sample_valid,
    output logic signed [15:0] ax, ay, az, gx, gy,
    output logic               soft_reset,
    output logic [7:0]         bad_frames      // saturating count of checksum errors
);
    typedef enum logic [2:0] {H0, H1, PAYLOAD, CHK, RST_CHK} state_t;
    state_t state;

    logic [7:0] buf_r [10];
    logic [3:0] idx;
    logic [7:0] x;                              // running XOR of payload

    always_ff @(posedge clk) begin
        sample_valid <= 1'b0;
        soft_reset   <= 1'b0;
        if (rst) begin
            state      <= H0;
            idx        <= '0;
            x          <= '0;
            bad_frames <= '0;
            {ax, ay, az, gx, gy} <= '0;
            for (int i = 0; i < 10; i++) buf_r[i] <= '0;
        end else if (rx_valid) begin
            case (state)
                H0: if (rx_data == 8'hC3) state <= H1;
                H1: begin
                    if (rx_data == 8'h3C) begin
                        idx   <= '0;
                        x     <= '0;
                        state <= PAYLOAD;
                    end else if (rx_data == 8'hE1) begin
                        state <= RST_CHK;
                    end else if (rx_data == 8'hC3) begin
                        state <= H1;                    // repeated header byte
                    end else begin
                        state <= H0;
                    end
                end
                PAYLOAD: begin
                    buf_r[idx] <= rx_data;
                    x          <= x ^ rx_data;
                    if (idx == 4'd9) state <= CHK;
                    idx <= idx + 1'b1;
                end
                CHK: begin
                    if (rx_data == x) begin
                        ax <= {buf_r[0], buf_r[1]};
                        ay <= {buf_r[2], buf_r[3]};
                        az <= {buf_r[4], buf_r[5]};
                        gx <= {buf_r[6], buf_r[7]};
                        gy <= {buf_r[8], buf_r[9]};
                        sample_valid <= 1'b1;
                    end else if (bad_frames != 8'hFF) begin
                        bad_frames <= bad_frames + 1'b1;
                    end
                    state <= H0;
                end
                RST_CHK: begin
                    if (rx_data == 8'h1E) soft_reset <= 1'b1;
                    state <= H0;
                end
                default: state <= H0;
            endcase
        end
    end
endmodule
