// End-to-end test of the hardware-in-the-loop build (top_alchitry_au_hil).
//
//   stimulus file -> UART frames -> [ FPGA HIL top ] -> UART result frames -> checker
//
// Plays the role of tools/hil_run.py: sends a reset frame and back-to-back
// sample frames, decodes every result frame (header, sequence, checksum) and
// checks the full 32-bit roll/pitch against the bit-exact model.
//
// Also checks:
//   * the PC's reset frame restarts the filter (phase 1 runs part of the data,
//     then a reset, then phase 2 replays from sample 0 and must match again)
//   * a frame with a bad checksum is dropped, counted (LED2), and does not
//     disturb the samples around it
//   * the PC's bit rate 1 % slower than nominal still works
//   * no result frames are dropped (LED3 stays off)
// UART runs at 5 Mbaud here to keep the simulation short; the logic is the
// same as the 1 Mbaud build.
`timescale 1ns/1ps
module tb_hil;
    localparam int  N1      = 600;     // phase 1 samples
    localparam int  N2      = 3000;    // phase 2 samples (> 1024 for gyro cal)
    localparam int  BAD_AT  = 1500;    // inject a corrupt frame before this sample
    localparam int  BAUD    = 5_000_000;
    localparam real BIT_NS  = 1.0e9 / BAUD;
    localparam real PC_BIT_NS = BIT_NS * 1.01;   // PC side 1 % slow

    logic clk = 0, rst_n = 0, usb_rx = 1;
    logic usb_tx;
    logic [7:0] led;
    always #5 clk = ~clk;

    top_alchitry_au_hil #(.CLK_HZ(100_000_000), .BAUD(BAUD)) dut (
        .clk(clk), .rst_n(rst_n), .usb_rx(usb_rx), .usb_tx(usb_tx), .led(led)
    );

    // ---------------- vectors ----------------
    logic [15:0] stim [N2][5];
    logic [31:0] expv [N2][2];
    logic [15:0] t16 [5];
    logic [31:0] t32 [4];
    integer fs, fe, n, errors = 0;

    // ---------------- PC -> FPGA ----------------
    task automatic send_byte(input logic [7:0] b);
        usb_rx = 1'b0;  #(PC_BIT_NS);
        for (int i = 0; i < 8; i++) begin usb_rx = b[i]; #(PC_BIT_NS); end
        usb_rx = 1'b1;  #(PC_BIT_NS);
    endtask

    task automatic send_sample(input integer i, input bit corrupt);
        logic [7:0] d [10];
        logic [7:0] x;
        x = 8'h00;
        for (int k = 0; k < 5; k++) begin
            d[2*k]   = stim[i][k][15:8];
            d[2*k+1] = stim[i][k][7:0];
        end
        for (int k = 0; k < 10; k++) x = x ^ d[k];
        send_byte(8'hC3); send_byte(8'h3C);
        for (int k = 0; k < 10; k++) send_byte(d[k]);
        send_byte(corrupt ? ~x : x);
    endtask

    task automatic send_reset();
        send_byte(8'hC3); send_byte(8'hE1); send_byte(8'h1E);
    endtask

    // ---------------- FPGA -> PC ----------------
    task automatic rx_byte(output logic [7:0] b);
        @(negedge usb_tx);
        #(BIT_NS / 2);
        for (int i = 0; i < 8; i++) begin #(BIT_NS); b[i] = usb_tx; end
        #(BIT_NS);
        if (usb_tx !== 1'b1) begin errors++; $display("UART framing error from FPGA"); end
    endtask

    integer got = 0, phase_base = 0;
    logic [7:0] exp_seq = 0;
    initial begin : monitor
        logic [7:0] b, f [12], chk;
        logic [31:0] r, p;
        integer s;
        forever begin
            rx_byte(b);
            while (b != 8'hA5) rx_byte(b);
            rx_byte(b);
            if (b != 8'h5A) begin errors++; $display("bad second header byte %h", b); end
            for (int i = 2; i < 12; i++) rx_byte(f[i]);
            chk = 0;
            for (int i = 2; i < 11; i++) chk = chk ^ f[i];
            r = {f[3], f[4], f[5], f[6]};
            p = {f[7], f[8], f[9], f[10]};
            s = got - phase_base;
            if (chk != f[11]) begin errors++; $display("result %0d: bad checksum", got); end
            if (f[2] != exp_seq) begin errors++; $display("result %0d: seq %0d, expected %0d", got, f[2], exp_seq); end
            if (r !== expv[s][0] || p !== expv[s][1]) begin
                errors++;
                if (errors < 10) $display("result %0d (sample %0d): roll %h pitch %h, expected %h %h",
                                          got, s, r, p, expv[s][0], expv[s][1]);
            end
            exp_seq = exp_seq + 1;
            got++;
        end
    end

    // ---------------- main ----------------
    initial begin
        fs = $fopen("vectors/imu_stimulus.hex", "r");
        fe = $fopen("vectors/imu_expected.hex", "r");
        if (fs == 0 || fe == 0) $fatal(1, "cannot open vector files");
        for (int i = 0; i < N2; i++) begin
            n = $fscanf(fs, "%h %h %h %h %h\n", t16[0], t16[1], t16[2], t16[3], t16[4]);
            n = $fscanf(fe, "%h %h %h %h\n", t32[0], t32[1], t32[2], t32[3]);
            for (int j = 0; j < 5; j++) stim[i][j] = t16[j];
            expv[i][0] = t32[0]; expv[i][1] = t32[1];
        end
        $fclose(fs); $fclose(fe);

        repeat (20) @(posedge clk);
        rst_n = 1;
        #(PC_BIT_NS * 20);

        // Phase 1: partial run
        send_reset();
        for (int i = 0; i < N1; i++) send_sample(i, 0);
        #(BIT_NS * 200);
        if (got != N1) begin errors++; $display("phase 1: %0d results, expected %0d", got, N1); end

        // Phase 2: reset, then replay from the start with one corrupt frame inserted
        send_reset();
        #(BIT_NS * 20);
        phase_base = got;
        exp_seq    = 0;                         // hil_tx restarts its sequence on reset
        for (int i = 0; i < N2; i++) begin
            if (i == BAD_AT) send_sample(i, 1); // corrupt copy: must be ignored
            send_sample(i, 0);
        end
        #(BIT_NS * 200);

        if (got - phase_base != N2) begin errors++; $display("phase 2: %0d results, expected %0d", got - phase_base, N2); end
        if (!led[1]) begin errors++; $display("cal_done LED off"); end
        if (!led[2]) begin errors++; $display("bad-frame LED off after a corrupt frame"); end
        if ( led[3]) begin errors++; $display("result frames were dropped"); end
        if (dut.bad_frames != 1) begin errors++; $display("bad_frames = %0d, expected 1", dut.bad_frames); end

        $display("results: %0d (phase 1) + %0d (phase 2), bad frames counted: %0d, dropped: %0d",
                 N1, got - phase_base, dut.bad_frames, dut.dropped);
        if (errors == 0) $display("HIL TEST PASS: %0d results bit-exact over the UART loop", got);
        else             $display("HIL TEST FAIL: %0d errors", errors);
        $finish;
    end

    initial begin
        #(64'd2_000_000_000);
        $display("HIL TEST FAIL: timeout (results=%0d)", got);
        $finish;
    end
endmodule
