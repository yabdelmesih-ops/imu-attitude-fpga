`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// hil_tx.sv — sends one result frame per filtered sample back to the PC.
//
//   Result frame (12 bytes):  A5 5A  seq  roll[31:24..7:0]  pitch[31:24..7:0]  chk
//     seq  : 8-bit sample counter (wraps), lets the PC detect lost frames
//     roll, pitch : full 32-bit binary angles, big-endian, for a bit-exact check
//     chk  : XOR of bytes 2..10
//
// A result that arrives while a frame is still being sent is dropped and
// counted. With the PC sending 13-byte input frames and the FPGA answering with
// 12-byte frames at the same baud rate, this cannot happen in normal use.
// -----------------------------------------------------------------------------
module hil_tx #(
    parameter int CLK_HZ = 100_000_000,
    parameter int BAUD   = 1_000_000
) (
    input  logic        clk,
    input  logic        rst,
    input  logic        in_valid,
    input  logic [31:0] roll,
    input  logic [31:0] pitch,
    output logic        txd,
    output logic [7:0]  dropped        // saturating count of dropped results
);
    logic [7:0] frame [12];
    logic [3:0] idx;
    logic       sending, u_start, u_busy;
    logic [7:0] seq;

    uart_tx #(.CLK_HZ(CLK_HZ), .BAUD(BAUD)) u_uart (
        .clk(clk), .rst(rst), .start(u_start), .data(frame[idx]), .busy(u_busy), .txd(txd)
    );

    wire [7:0] chk = seq ^ roll[31:24] ^ roll[23:16] ^ roll[15:8] ^ roll[7:0]
                         ^ pitch[31:24] ^ pitch[23:16] ^ pitch[15:8] ^ pitch[7:0];

    always_ff @(posedge clk) begin
        u_start <= 1'b0;
        if (rst) begin
            sending <= 1'b0;
            idx     <= '0;
            seq     <= '0;
            dropped <= '0;
            for (int i = 0; i < 12; i++) frame[i] <= '0;
        end else begin
            if (in_valid) begin
                if (!sending) begin
                    frame[0]  <= 8'hA5;
                    frame[1]  <= 8'h5A;
                    frame[2]  <= seq;
                    frame[3]  <= roll[31:24];
                    frame[4]  <= roll[23:16];
                    frame[5]  <= roll[15:8];
                    frame[6]  <= roll[7:0];
                    frame[7]  <= pitch[31:24];
                    frame[8]  <= pitch[23:16];
                    frame[9]  <= pitch[15:8];
                    frame[10] <= pitch[7:0];
                    frame[11] <= chk;
                    idx       <= '0;
                    sending   <= 1'b1;
                end else if (dropped != 8'hFF) begin
                    dropped <= dropped + 1'b1;
                end
                seq <= seq + 1'b1;              // advances even on a drop, so the PC sees the gap
            end
            if (sending && !u_busy && !u_start) u_start <= 1'b1;
            if (u_start) begin
                if (idx == 4'd11) sending <= 1'b0;
                else              idx <= idx + 1'b1;
            end
        end
    end
endmodule
