`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// i2c_master.sv: byte-level I2C master. A controller module strings these
// commands together into full transactions:
//
//   CMD_START : START, or a repeated START if a transaction is already open
//   CMD_WRITE : send wdata (8 bits, MSB first), then read the device's ACK.
//               nack = 1 if the device did not acknowledge.
//   CMD_READ  : read 8 bits into rdata, then send ACK (ack_out = 1: "send more")
//               or NACK (ack_out = 0: "that was the last byte")
//   CMD_STOP  : STOP, leaving both lines released (bus idle)
//
// Issue a command with a 1-cycle cmd_valid while ready = 1; done pulses for
// one cycle when it has finished. Reused by every I2C device in the project
// (MPU-6050 reader now, servo driver later).
//
// Open-drain bus: *_oe = 1 pulls the line low, 0 lets the pull-up raise it.
// Every SCL period is 4 quarters, the same scheme as i2c_read_reg.sv:
//   q0 SCL low: set SDA   q1 SCL rises   q2 SCL high: sample SDA   q3 SCL falls
//
// Speed note: with equal high and low halves, SCL low time = 1 / (2 * I2C_HZ).
// Fast mode (400 kHz) needs >= 1.3 us low, so stay at or below ~380 kHz.
// -----------------------------------------------------------------------------
module i2c_master #(
    parameter int CLK_HZ = 100_000_000,
    parameter int I2C_HZ = 350_000
) (
    input  logic       clk,
    input  logic       rst,
    input  logic       cmd_valid,
    input  logic [1:0] cmd,
    input  logic [7:0] wdata,
    input  logic       ack_out,
    output logic       ready,
    output logic       done,
    output logic [7:0] rdata,
    output logic       nack,
    output logic       scl_oe,
    output logic       sda_oe,
    input  logic       sda_in
);
    localparam logic [1:0] CMD_START = 2'd0, CMD_WRITE = 2'd1, CMD_READ = 2'd2, CMD_STOP = 2'd3;

    // ---- quarter-period ticks (only while busy) ----
    localparam int QUARTER = CLK_HZ / (I2C_HZ * 4);
    logic [$clog2(QUARTER + 1)-1:0] tick_count;
    logic tick, busy;

    always_ff @(posedge clk) begin
        tick <= 1'b0;
        if (rst || !busy) tick_count <= '0;
        else if (tick_count == QUARTER - 1) begin
            tick_count <= '0;
            tick       <= 1'b1;
        end else tick_count <= tick_count + 1'b1;
    end

    assign ready = !busy && !cmd_valid;

    logic [1:0] cur;          // command in progress
    logic [1:0] quarter;
    logic [3:0] bit_num;      // 0-7 data bits, 8 = ACK bit
    logic [7:0] shift_reg;
    logic       ack_r;

    always_ff @(posedge clk) begin
        done <= 1'b0;
        if (rst) begin
            busy      <= 1'b0;
            cur       <= CMD_STOP;
            quarter   <= '0;
            bit_num   <= '0;
            shift_reg <= '0;
            ack_r     <= 1'b0;
            rdata     <= '0;
            nack      <= 1'b0;
            scl_oe    <= 1'b0;
            sda_oe    <= 1'b0;
        end else if (!busy) begin
            if (cmd_valid) begin
                busy      <= 1'b1;
                cur       <= cmd;
                quarter   <= '0;
                bit_num   <= '0;
                shift_reg <= wdata;
                ack_r     <= ack_out;
                if (cmd == CMD_WRITE) nack <= 1'b0;
            end
        end else if (tick) begin
            quarter <= quarter + 1'b1;
            case (cur)
                // START / repeated START: release SDA, raise SCL, then pull SDA
                // low while SCL is high, then pull SCL low.
                CMD_START: case (quarter)
                    2'd0: sda_oe <= 1'b0;
                    2'd1: scl_oe <= 1'b0;
                    2'd2: sda_oe <= 1'b1;
                    2'd3: begin scl_oe <= 1'b1; busy <= 1'b0; done <= 1'b1; end
                endcase

                // STOP: pull SDA low, raise SCL, then release SDA while SCL is high.
                CMD_STOP: case (quarter)
                    2'd0: sda_oe <= 1'b1;
                    2'd1: scl_oe <= 1'b0;
                    2'd2: sda_oe <= 1'b0;
                    2'd3: begin busy <= 1'b0; done <= 1'b1; end
                endcase

                // WRITE and READ: 9 SCL periods (8 data bits + the ACK bit)
                default: case (quarter)
                    2'd0: begin
                        if (bit_num == 8)                       // ACK bit
                            sda_oe <= (cur == CMD_READ) ? ack_r : 1'b0;
                        else
                            sda_oe <= (cur == CMD_WRITE) ? ~shift_reg[7] : 1'b0;
                    end
                    2'd1: scl_oe <= 1'b0;
                    2'd2: begin
                        if (bit_num == 8) begin
                            if (cur == CMD_WRITE) nack <= sda_in;   // high = no ACK
                        end else if (cur == CMD_READ)
                            rdata <= {rdata[6:0], sda_in};
                    end
                    2'd3: begin
                        scl_oe    <= 1'b1;
                        shift_reg <= {shift_reg[6:0], 1'b0};
                        if (bit_num == 8) begin busy <= 1'b0; done <= 1'b1; end
                        else bit_num <= bit_num + 1'b1;
                    end
                endcase
            endcase
        end
    end
endmodule
