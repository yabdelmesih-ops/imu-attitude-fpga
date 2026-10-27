// Self-checking unit test: cordic_vec vs. golden vectors from model/gen_stimulus.py
`timescale 1ns/1ps
module tb_cordic;
    localparam int W = 24;
    logic clk = 0, rst = 1, start = 0;
    logic signed [W-1:0] x_in, y_in, mag;
    logic [31:0] angle;
    logic busy, done;

    always #5 clk = ~clk;

    cordic_vec #(.W(W), .N(16)) dut (.*);

    integer fd, n, errors = 0, count = 0;
    logic [W-1:0] fx, fy, fmag;
    logic [31:0]  fang;

    initial begin
        fd = $fopen("vectors/cordic_vectors.hex", "r");
        if (fd == 0) $fatal(1, "cannot open vectors/cordic_vectors.hex");
        repeat (3) @(posedge clk);
        rst = 0;
        n = $fscanf(fd, "%h %h %h %h\n", fx, fy, fang, fmag);
        while (n == 4) begin
            @(posedge clk);
            x_in <= fx; y_in <= fy; start <= 1'b1;
            @(posedge clk);
            start <= 1'b0;
            @(posedge done);
            @(negedge clk);
            count++;
            if (angle !== fang || mag !== $signed(fmag)) begin
                errors++;
                if (errors <= 10)
                    $display("MISMATCH #%0d x=%h y=%h  angle %h (exp %h)  mag %h (exp %h)",
                             count, fx, fy, angle, fang, mag, fmag);
            end
            n = $fscanf(fd, "%h %h %h %h\n", fx, fy, fang, fmag);
        end
        $fclose(fd);
        if (errors == 0) $display("CORDIC TEST PASS: %0d/%0d vectors bit-exact", count, count);
        else             $display("CORDIC TEST FAIL: %0d mismatches in %0d vectors", errors, count);
        $finish;
    end
endmodule
