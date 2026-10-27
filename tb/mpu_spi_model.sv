`timescale 1ns/1ps
// Behavioral model of an MPU-6500-style SPI slave (mode 3) for simulation.
// 128-byte register file, auto-incrementing burst reads/writes.
// Logs every register write and raises `burst_done` when a read
// transaction ends, so the testbench can load the next sensor sample.
module mpu_spi_model (
    input  logic sclk,
    input  logic mosi,
    output logic miso,
    input  logic cs_n
);
    logic [7:0] regs [128];
    logic [7:0] sh_in, sh_out;
    logic [2:0] bitn;
    integer     byte_idx;
    logic [6:0] addr, start_addr;
    logic       is_read;
    event       burst_done;

    // write log for the testbench to check the init sequence
    logic [15:0] wlog [64];
    integer      n_writes = 0;

    initial begin
        for (int i = 0; i < 128; i++) regs[i] = 8'h00;
        regs[7'h75] = 8'h70;          // WHO_AM_I = MPU-6500
        miso = 1'b1;
    end

    always @(negedge cs_n) begin
        bitn     = 3'd7;
        byte_idx = 0;
        is_read  = 1'b0;
    end

    always @(posedge cs_n) begin
        miso = 1'b1;
        if (is_read && byte_idx > 1) -> burst_done;
    end

    // sample MOSI on the rising edge
    always @(posedge sclk) if (!cs_n) begin
        sh_in[bitn] = mosi;
        if (bitn == 0) begin
            if (byte_idx == 0) begin
                is_read    = sh_in[7];
                addr       = sh_in[6:0];
                start_addr = sh_in[6:0];
            end else if (!is_read) begin
                regs[addr] = sh_in;
                wlog[n_writes] = {1'b0, addr, sh_in};
                n_writes++;
                addr = addr + 1;
            end
            if (is_read) begin            // prepare next output byte
                sh_out = regs[addr];
                addr   = addr + 1;
            end
            byte_idx++;
        end
        bitn = bitn - 1;
    end

    // drive MISO on the falling edge
    always @(negedge sclk) if (!cs_n && is_read && byte_idx >= 1) begin
        miso   <= sh_out[7];
        sh_out =  {sh_out[6:0], 1'b0};
    end
endmodule
