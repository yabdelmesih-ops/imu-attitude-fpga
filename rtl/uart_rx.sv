`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// uart_rx.sv — 8N1 UART receiver.
//
// RXD comes from another clock domain (the USB-serial chip), so it passes
// through a 2-FF synchronizer. A start bit is confirmed at its midpoint; each
// data bit is then sampled one bit period later, at the middle of the bit.
// -----------------------------------------------------------------------------
module uart_rx #(
    parameter int CLK_HZ = 100_000_000,
    parameter int BAUD   = 1_000_000
) (
    input  logic       clk,
    input  logic       rst,
    input  logic       rxd,
    output logic [7:0] data,
    output logic       valid,       // 1-cycle pulse per received byte
    output logic       frame_err    // 1-cycle pulse: stop bit was low
);
    localparam int DIV = CLK_HZ / BAUD;

    logic [2:0] sync = 3'b111;
    always_ff @(posedge clk) sync <= {sync[1:0], rxd};
    wire rx = sync[2];

    typedef enum logic [1:0] {IDLE, START, DATA, STOP} state_t;
    state_t state;
    logic [$clog2(DIV):0] cnt;
    logic [2:0]           bitn;
    logic [7:0]           sh;

    always_ff @(posedge clk) begin
        valid     <= 1'b0;
        frame_err <= 1'b0;
        if (rst) begin
            state <= IDLE;
            cnt   <= '0;
            bitn  <= '0;
            sh    <= '0;
            data  <= '0;
        end else begin
            case (state)
                IDLE: if (!rx) begin                // falling edge: possible start bit
                    cnt   <= '0;
                    state <= START;
                end
                START: if (cnt == DIV / 2 - 1) begin
                    cnt   <= '0;
                    bitn  <= '0;
                    state <= rx ? IDLE : DATA;      // glitch, not a start bit
                end else cnt <= cnt + 1'b1;
                DATA: if (cnt == DIV - 1) begin
                    cnt <= '0;
                    sh  <= {rx, sh[7:1]};           // LSB first
                    if (bitn == 3'd7) state <= STOP;
                    bitn <= bitn + 1'b1;
                end else cnt <= cnt + 1'b1;
                STOP: if (cnt == DIV - 1) begin
                    cnt   <= '0;
                    state <= IDLE;
                    if (rx) begin
                        data  <= sh;
                        valid <= 1'b1;
                    end else begin
                        frame_err <= 1'b1;
                    end
                end else cnt <= cnt + 1'b1;
                default: state <= IDLE;
            endcase
        end
    end
endmodule
