`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// uart_tx.sv — 8N1 UART transmitter.
// -----------------------------------------------------------------------------
module uart_tx #(
    parameter int CLK_HZ = 100_000_000,
    parameter int BAUD   = 115_200
) (
    input  logic       clk,
    input  logic       rst,
    input  logic       start,      // ignored while busy
    input  logic [7:0] data,
    output logic       busy,
    output logic       txd
);
    localparam int DIV = CLK_HZ / BAUD;

    logic [$clog2(DIV):0] cnt;
    logic [3:0]           bitn;     // 0 = start, 1..8 = data, 9 = stop
    logic [9:0]           frame;

    always_ff @(posedge clk) begin
        if (rst) begin
            busy  <= 1'b0;
            txd   <= 1'b1;
            cnt   <= '0;
            bitn  <= '0;
            frame <= '1;
        end else if (!busy) begin
            txd <= 1'b1;
            if (start) begin
                frame <= {1'b1, data, 1'b0};    // stop, data (LSB first), start
                busy  <= 1'b1;
                cnt   <= '0;
                bitn  <= '0;
                txd   <= 1'b0;                  // start bit
            end
        end else if (cnt == DIV - 1) begin
            cnt <= '0;
            if (bitn == 4'd9) begin
                busy <= 1'b0;
                txd  <= 1'b1;
            end else begin
                bitn <= bitn + 1'b1;
                txd  <= frame[bitn + 1];
            end
        end else begin
            cnt <= cnt + 1'b1;
        end
    end
endmodule
