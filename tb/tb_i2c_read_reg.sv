// Self-checking test for rtl/i2c_read_reg.sv against a simulated MPU-6050.
//
//   1. read WHO_AM_I (0x75)        -> expect 0x68, no ack_error
//   2. read PWR_MGMT_1 (0x6B)      -> expect 0x40 (its reset value)
//   3. read from address 0x69      -> nobody answers: expect ack_error
//   and for every read: the right number of STARTs and STOPs, the bus left
//   idle (both lines high), and SCL no faster than the I2C standard allows.
`timescale 1ns/1ps
module tb_i2c_read_reg;
    localparam int CLK_HZ = 100_000_000;
    localparam int I2C_HZ = 100_000;

    logic clk = 0, rst = 1, start = 0;
    always #5 clk = ~clk;

    logic [6:0] dev_addr;
    logic [7:0] reg_addr, data;
    logic busy, done, ack_error, scl_oe, sda_oe;

    // Open-drain bus with pull-up resistors
    tri1 scl, sda;
    assign scl = scl_oe ? 1'b0 : 1'bz;
    assign sda = sda_oe ? 1'b0 : 1'bz;

    i2c_read_reg #(.CLK_HZ(CLK_HZ), .I2C_HZ(I2C_HZ)) dut (
        .clk(clk), .rst(rst), .start(start), .dev_addr(dev_addr), .reg_addr(reg_addr),
        .busy(busy), .done(done), .ack_error(ack_error), .data(data),
        .scl_oe(scl_oe), .sda_oe(sda_oe), .sda_in(sda === 1'b1)
    );

    mpu6050_i2c_model #(.ADDR(7'h68), .WHO_AM_I(8'h68)) sensor (.scl(scl), .sda(sda));

    // SCL timing check (standard mode: low >= 4.7 us, high >= 4.0 us)
    realtime t_rise = 0, t_fall = 0, min_high = 1e9, min_low = 1e9;
    always @(posedge scl) begin
        t_rise = $realtime;
        if (t_fall > 0 && t_rise - t_fall < min_low) min_low = t_rise - t_fall;
    end
    always @(negedge scl) begin
        t_fall = $realtime;
        if (t_rise > 0 && t_fall - t_rise < min_high) min_high = t_fall - t_rise;
    end

    integer errors = 0;

    task automatic read_reg(input logic [6:0] a, input logic [7:0] r,
                            input bit expect_err, input logic [7:0] expect_data,
                            input integer expect_starts);
        integer s0, p0, waited;
        s0 = sensor.starts; p0 = sensor.stops;
        @(posedge clk);
        dev_addr <= a; reg_addr <= r; start <= 1'b1;
        @(posedge clk);
        start <= 1'b0;
        waited = 0;
        while (!done && waited < 200_000) begin @(posedge clk); waited++; end
        if (!done) begin
            errors++; $display("FAIL addr %h reg %h: done never pulsed", a, r);
            disable read_reg;
        end
        if (ack_error !== expect_err) begin
            errors++; $display("FAIL addr %h reg %h: ack_error = %b, expected %b", a, r, ack_error, expect_err);
        end
        if (!expect_err && data !== expect_data) begin
            errors++; $display("FAIL addr %h reg %h: data = %h, expected %h", a, r, data, expect_data);
        end
        repeat (1000) @(posedge clk);            // let the STOP finish
        if (sensor.starts - s0 != expect_starts) begin
            errors++; $display("FAIL addr %h reg %h: %0d STARTs, expected %0d", a, r, sensor.starts - s0, expect_starts);
        end
        if (sensor.stops - p0 != 1) begin
            errors++; $display("FAIL addr %h reg %h: %0d STOPs, expected 1", a, r, sensor.stops - p0);
        end
        if (scl !== 1'b1 || sda !== 1'b1) begin
            errors++; $display("FAIL addr %h reg %h: bus not released (scl=%b sda=%b)", a, r, scl, sda);
        end
        if (busy) begin errors++; $display("FAIL addr %h reg %h: still busy", a, r); end
        $display("read addr %h reg %h: data %h ack_error %b", a, r, data, ack_error);
    endtask

    initial begin
        repeat (10) @(posedge clk);
        rst = 0;
        repeat (10) @(posedge clk);
        if (scl !== 1'b1 || sda !== 1'b1) begin errors++; $display("FAIL: bus not idle after reset"); end

        read_reg(7'h68, 8'h75, 0, 8'h68, 2);     // WHO_AM_I
        read_reg(7'h68, 8'h6B, 0, 8'h40, 2);     // PWR_MGMT_1 reset value
        read_reg(7'h69, 8'h75, 1, 8'h00, 1);     // wrong address: NACK, then STOP

        if (min_low < 4700.0 || min_high < 4000.0) begin
            errors++;
            $display("FAIL: SCL too fast (shortest low %0.0f ns, high %0.0f ns)", min_low, min_high);
        end
        if (errors == 0) $display("I2C TEST PASS: 3 reads correct, timing within standard mode");
        else             $display("I2C TEST FAIL: %0d errors", errors);
        $finish;
    end
endmodule
