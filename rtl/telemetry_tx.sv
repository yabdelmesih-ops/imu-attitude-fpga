`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// telemetry_tx.sv — packs every DECIM-th attitude sample into an 8-byte frame
// and sends it over the UART.
//
// Frame:  A5 5A  rollH rollL  pitchH pitchL  status  chk
//   roll/pitch : top 16 bits of the BAM32 angle, signed; deg = v * 360 / 65536
//   status     : {seq[5:0], imu_running, cal_done}
//   chk        : XOR of bytes 2..6
// At 115200 baud, 8 bytes take 0.69 ms, so DECIM >= 1 works up to ~1.4 kHz;
// the default 10 gives a 100 Hz telemetry stream.
// -----------------------------------------------------------------------------
module telemetry_tx #(
    parameter int CLK_HZ = 100_000_000,
    parameter int BAUD   = 115_200,
    parameter int DECIM  = 10
) (
    input  logic        clk,
    input  logic        rst,
    input  logic        in_valid,
    input  logic [31:0] roll,
    input  logic [31:0] pitch,
    input  logic        cal_done,
    input  logic        imu_running,
    output logic        txd
);
    logic [7:0] frame [8];
    logic [2:0] idx;
    logic       sending, u_start, u_busy;
    logic [$clog2(DECIM + 1):0] dcnt;
    logic [5:0] seq;

    uart_tx #(.CLK_HZ(CLK_HZ), .BAUD(BAUD)) u_uart (
        .clk(clk), .rst(rst), .start(u_start), .data(frame[idx]), .busy(u_busy), .txd(txd)
    );

    wire [7:0] status = {seq, imu_running, cal_done};
    wire [7:0] chk    = roll[31:24] ^ roll[23:16] ^ pitch[31:24] ^ pitch[23:16] ^ status;

    always_ff @(posedge clk) begin
        u_start <= 1'b0;
        if (rst) begin
            sending <= 1'b0;
            idx     <= '0;
            dcnt    <= '0;
            seq     <= '0;
            for (int i = 0; i < 8; i++) frame[i] <= '0;
        end else begin
            if (in_valid) begin
                if (dcnt == DECIM - 1) begin
                    dcnt <= '0;
                    if (!sending) begin               // drop the frame if still busy
                        frame[0] <= 8'hA5;
                        frame[1] <= 8'h5A;
                        frame[2] <= roll[31:24];
                        frame[3] <= roll[23:16];
                        frame[4] <= pitch[31:24];
                        frame[5] <= pitch[23:16];
                        frame[6] <= status;
                        frame[7] <= chk;
                        seq      <= seq + 1'b1;
                        idx      <= '0;
                        sending  <= 1'b1;
                    end
                end else dcnt <= dcnt + 1'b1;
            end
            if (sending && !u_busy && !u_start) begin
                u_start <= 1'b1;                      // uart latches frame[idx]
            end
            if (u_start) begin                        // byte accepted this cycle
                if (idx == 3'd7) sending <= 1'b0;
                else             idx <= idx + 1'b1;
            end
        end
    end
endmodule
