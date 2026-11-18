`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// top_alchitry_au_whoami.sv: I2C bring-up test for the MPU-6050 on the Qwiic port.
//
// Every 100 ms the FPGA reads the sensor's WHO_AM_I register (0x75) and shows
// the answer on the 8 LEDs (LED0 = bit 0 = top LED).
//
//   LEDs steady    : the byte the sensor returned. 0x68 = genuine MPU-6050
//                    (LED3, LED5, LED6 on). Other values usually mean a clone.
//   LEDs alternate : 0x0F / 0xF0 flashing = no ACK from address 0x68
//                    (check wiring, power and the SDA/SCL order)
//   LED0 blinking  : no read finished yet
//
// Qwiic port: SCL = E6, SDA = K5 (Alchitry pins A24 / A23), 3.3 V.
// -----------------------------------------------------------------------------
module top_alchitry_au_whoami #(
    parameter int CLK_HZ      = 100_000_000,
    parameter int READ_PERIOD = CLK_HZ / 10        // 100 ms between reads
) (
    input  logic       clk,
    input  logic       rst_n,
    output logic [7:0] led,
    inout  wire        qwiic_scl,
    inout  wire        qwiic_sda
);
    logic [2:0] rst_sync = 3'b111;
    always_ff @(posedge clk) rst_sync <= {rst_sync[1:0], ~rst_n};
    wire rst = rst_sync[2];

    // Open-drain pins: pull low or let go
    logic scl_oe, sda_oe;
    assign qwiic_scl = scl_oe ? 1'b0 : 1'bz;
    assign qwiic_sda = sda_oe ? 1'b0 : 1'bz;

    logic [1:0] sda_sync;
    always_ff @(posedge clk) sda_sync <= {sda_sync[0], qwiic_sda};

    // Start a read every READ_PERIOD cycles (the first one 100 ms after reset,
    // which also covers the sensor's power-up time)
    logic [$clog2(READ_PERIOD)-1:0] tmr;
    logic go;
    always_ff @(posedge clk) begin
        go <= 1'b0;
        if (rst) tmr <= '0;
        else if (tmr == READ_PERIOD - 1) begin tmr <= '0; go <= 1'b1; end
        else tmr <= tmr + 1'b1;
    end

    logic       busy, done, ack_error;
    logic [7:0] data;
    i2c_read_reg #(.CLK_HZ(CLK_HZ), .I2C_HZ(100_000)) u_i2c (
        .clk(clk), .rst(rst), .start(go && !busy),
        .dev_addr(7'h68), .reg_addr(8'h75),
        .busy(busy), .done(done), .ack_error(ack_error), .data(data),
        .scl_oe(scl_oe), .sda_oe(sda_oe), .sda_in(sda_sync[1])
    );

    logic [7:0] id;
    logic       have_id, no_ack;
    always_ff @(posedge clk) begin
        if (rst) begin
            id <= '0; have_id <= 1'b0; no_ack <= 1'b0;
        end else if (done) begin
            id      <= data;
            have_id <= !ack_error;
            no_ack  <= ack_error;
        end
    end

    logic [25:0] blink = '0;
    always_ff @(posedge clk) blink <= blink + 1'b1;

    always_comb begin
        if (have_id)     led = id;
        else if (no_ack) led = blink[25] ? 8'hF0 : 8'h0F;
        else             led = {7'b0, blink[24]};
    end
endmodule
