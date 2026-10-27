// Self-checking system test: attitude_core vs. bit-exact Python model.
// Drives 20,000 synthetic IMU samples (20 s of flight at 1 kHz), checks every
// output word, and writes the HDL results to out/imu_hdl_out.hex for plotting.
//
// Samples are fed back-to-back as fast as the core accepts them; the filter's
// behaviour depends only on sample count, not on wall-clock spacing.
`timescale 1ns/1ps
module tb_attitude_core;
    logic clk = 0, rst = 1, in_valid = 0;
    logic signed [15:0] ax, ay, az, gx, gy;
    logic busy, out_valid, cal_done;
    logic [31:0] roll, pitch, acc_roll, acc_pitch;

    always #5 clk = ~clk;   // 100 MHz

    attitude_core dut (.*);

    integer fs, fe, fo, n, errors = 0, count = 0, cyc_start, latency = 0;
    logic [15:0] s_ax, s_ay, s_az, s_gx, s_gy;
    logic [31:0] e_r, e_p, e_ar, e_ap;

    initial begin
        fs = $fopen("vectors/imu_stimulus.hex", "r");
        fe = $fopen("vectors/imu_expected.hex", "r");
        fo = $fopen("out/imu_hdl_out.hex", "w");
        if (fs == 0 || fe == 0 || fo == 0) $fatal(1, "cannot open vector files");
        repeat (3) @(posedge clk);
        rst = 0;
        @(posedge clk);
        n = $fscanf(fs, "%h %h %h %h %h\n", s_ax, s_ay, s_az, s_gx, s_gy);
        while (n == 5) begin
            n = $fscanf(fe, "%h %h %h %h\n", e_r, e_p, e_ar, e_ap);
            ax <= s_ax; ay <= s_ay; az <= s_az; gx <= s_gx; gy <= s_gy;
            in_valid <= 1'b1;
            cyc_start = $time;
            @(posedge clk);
            in_valid <= 1'b0;
            @(posedge out_valid);
            latency = ($time - cyc_start) / 10;
            @(negedge clk);
            count++;
            $fdisplay(fo, "%h %h %h %h", roll, pitch, acc_roll, acc_pitch);
            if ({roll, pitch, acc_roll, acc_pitch} !== {e_r, e_p, e_ar, e_ap}) begin
                errors++;
                if (errors <= 10)
                    $display("MISMATCH sample %0d: got %h %h %h %h  exp %h %h %h %h",
                             count - 1, roll, pitch, acc_roll, acc_pitch, e_r, e_p, e_ar, e_ap);
            end
            @(posedge clk);
            n = $fscanf(fs, "%h %h %h %h %h\n", s_ax, s_ay, s_az, s_gx, s_gy);
        end
        $fclose(fs); $fclose(fe); $fclose(fo);
        $display("latency per sample: %0d clock cycles, gyro bias calibrated: %0d", latency, cal_done);
        if (errors == 0) $display("ATTITUDE TEST PASS: %0d/%0d samples bit-exact", count, count);
        else             $display("ATTITUDE TEST FAIL: %0d mismatches in %0d samples", errors, count);
        $finish;
    end
endmodule
