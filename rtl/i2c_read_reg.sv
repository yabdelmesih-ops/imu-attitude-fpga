`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// i2c_read_reg.sv: read one 8-bit register from an I2C device.
//
// YOUR MODULE TO WRITE. The ports below are fixed so the testbench
// (tb/tb_i2c_read_reg.sv) and the board top level can use it. Replace the
// placeholder logic at the bottom.
//
// The transaction to produce (each byte is 8 bits, MSB first, then 1 ACK bit):
//
//   START | dev_addr,0 (write) | ACK | reg_addr | ACK |
//   repeated START | dev_addr,1 (read) | ACK | data from device | NACK | STOP
//
//   START          : SDA goes high -> low while SCL is high
//   STOP           : SDA goes low -> high while SCL is high
//   data bits      : change SDA only while SCL is low; the receiver samples it
//                    while SCL is high
//   ACK / NACK     : the receiver pulls SDA low for ACK, leaves it high for NACK.
//                    After the data byte YOU send NACK (leave SDA high) to tell
//                    the device you want only one byte.
//   If the device NACKs its address or the register, set ack_error, send STOP,
//   and finish.
//
// Open-drain bus: you never drive a 1. *_oe = 1 pulls the line low;
// *_oe = 0 lets go and the pull-up resistor makes it 1.
//
// Suggested approach (not required):
//   - a counter that ticks 4 times per SCL period (I2C_HZ * 4), so each bit
//     has 4 phases: SCL low/set SDA, SCL rising, SCL high/sample SDA, SCL falling
//   - a state machine: IDLE, START, SEND_BYTE, GET_ACK, RESTART, READ_BYTE,
//     SEND_NACK, STOP, DONE
//   - a bit counter and an 8-bit shift register
// -----------------------------------------------------------------------------
module i2c_read_reg #(
    parameter int CLK_HZ = 100_000_000,
    parameter int I2C_HZ = 100_000          // 100 kHz standard mode for bring-up
) (
    input  logic       clk,
    input  logic       rst,
    input  logic       start,               // 1-cycle pulse: begin one read
    input  logic [6:0] dev_addr,            // 7'h68 for the MPU-6050
    input  logic [7:0] reg_addr,            // 8'h75 = WHO_AM_I
    output logic       busy,
    output logic       done,                // 1-cycle pulse when finished
    output logic       ack_error,           // device did not ACK (valid with done)
    output logic [7:0] data,                // byte read (valid with done)
    output logic       scl_oe,              // 1 = pull SCL low
    output logic       sda_oe,              // 1 = pull SDA low
    input  logic       sda_in               // SDA as seen on the bus
);
    // ---- placeholder: replace everything below with your design ----
    assign busy      = 1'b0;
    assign done      = 1'b0;
    assign ack_error = 1'b0;
    assign data      = 8'h00;
    assign scl_oe    = 1'b0;
    assign sda_oe    = 1'b0;
endmodule
