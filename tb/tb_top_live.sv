// End-to-end test of the live sensor build (top_alchitry_au_live).
//
//   stimulus file -> simulated MPU-6050 (I2C) -> [ FPGA top ] -> UART -> frame decoder
//
// Checks: the 6 setup register writes, WHO_AM_I readback, status LEDs, no I2C
// errors, every telemetry frame (header, checksum, sequence number, cal flag),
// and that each frame's roll/pitch equal the bit-exact model's output for that
// sample. Same checks as tb_top.sv, with I2C in place of SPI.
//
// Timing is scaled up (5 MHz I2C, 20 kHz samples, 5 Mbaud UART) to keep the
// simulation short; the logic is identical to the 350 kHz / 1 kHz / 115200 build.
`timescale 1ns/1ps
module tb_top_live;
    localparam int NS     = 2400;       // samples to run (> 1024 for gyro cal)
    localparam int DECIM  = 4;
    localparam int BAUD   = 5_000_000;
    localparam int BIT_NS = 1_000_000_000 / BAUD;

    logic clk = 0, btn = 0, txd;
    always #5 clk = ~clk;               // 100 MHz

    logic [7:0] led;
    tri1 scl, sda;                      // pull-up resistors
    top_alchitry_au_live #(
        .CLK_HZ(100_000_000), .I2C_HZ(5_000_000), .SAMPLE_HZ(20_000),
        .BAUD(BAUD), .DECIM(DECIM), .RESET_WAIT_MS(1), .STEP_WAIT_US(10)
    ) dut (
        .clk(clk), .rst_n(~btn), .usb_tx(txd), .led(led),
        .qwiic_scl(scl), .qwiic_sda(sda)
    );

    mpu6050_i2c_model #(.ADDR(7'h68), .WHO_AM_I(8'h68)) imu (.scl(scl), .sda(sda));

    // ---------------- stimulus & expected data ----------------
    logic [15:0] stim [NS][5];
    logic [31:0] expv [NS][4];
    integer fs, fe, n, errors = 0;
    logic [15:0] t16 [5];
    logic [31:0] t32 [4];

    task automatic load_sample(input integer i);
        // 0x3B.. AX AY AZ, 0x41 TEMP, 0x43.. GX GY GZ, high byte first
        imu.regs[7'h3B] = stim[i][0][15:8]; imu.regs[7'h3C] = stim[i][0][7:0];
        imu.regs[7'h3D] = stim[i][1][15:8]; imu.regs[7'h3E] = stim[i][1][7:0];
        imu.regs[7'h3F] = stim[i][2][15:8]; imu.regs[7'h40] = stim[i][2][7:0];
        imu.regs[7'h41] = 8'h12;            imu.regs[7'h42] = 8'h34;
        imu.regs[7'h43] = stim[i][3][15:8]; imu.regs[7'h44] = stim[i][3][7:0];
        imu.regs[7'h45] = stim[i][4][15:8]; imu.regs[7'h46] = stim[i][4][7:0];
        imu.regs[7'h47] = 8'h00;            imu.regs[7'h48] = 8'h00;   // gz unused
    endtask

    integer bursts = 0;
    always @(imu.burst_done) begin
        if (imu.start_addr == 8'h3B) begin
            bursts++;
            if (bursts < NS) load_sample(bursts);
        end
    end

    // ---------------- UART receiver + frame checker ----------------
    logic [7:0] rx [8];
    integer frames = 0, sidx;
    logic [7:0] chk;

    task automatic uart_rx_byte(output logic [7:0] b);
        @(negedge txd);
        #(BIT_NS / 2);
        for (int i = 0; i < 8; i++) begin #(BIT_NS); b[i] = txd; end
        #(BIT_NS);
        if (txd !== 1'b1) begin errors++; $display("UART framing error"); end
    endtask

    initial begin : rx_proc
        logic [7:0] b;
        forever begin
            uart_rx_byte(b);
            while (b != 8'hA5) uart_rx_byte(b);
            uart_rx_byte(b);
            if (b != 8'h5A) begin errors++; $display("frame %0d: bad second header byte %h", frames, b); end
            for (int i = 2; i < 8; i++) uart_rx_byte(rx[i]);
            chk  = rx[2] ^ rx[3] ^ rx[4] ^ rx[5] ^ rx[6];
            sidx = frames * DECIM + DECIM - 1;
            if (chk != rx[7]) begin errors++; $display("frame %0d: bad checksum", frames); end
            if (rx[6][7:2] != 6'(frames & 63)) begin errors++; $display("frame %0d: bad seq %0d", frames, rx[6][7:2]); end
            if (rx[6][0] != (sidx >= 1023)) begin
                errors++; $display("frame %0d (sample %0d): cal flag %0d", frames, sidx, rx[6][0]);
            end
            if ({rx[2], rx[3]} != expv[sidx][0][31:16] || {rx[4], rx[5]} != expv[sidx][1][31:16]) begin
                errors++;
                if (errors < 10)
                    $display("frame %0d (sample %0d): roll %h pitch %h, expected %h %h",
                             frames, sidx, {rx[2], rx[3]}, {rx[4], rx[5]},
                             expv[sidx][0][31:16], expv[sidx][1][31:16]);
            end
            frames++;
        end
    end

    // ---------------- main sequence ----------------
    function automatic logic [15:0] init_exp(input integer i);
        case (i)
            0: init_exp = 16'h6B80;  1: init_exp = 16'h6B01;
            2: init_exp = 16'h1900;  3: init_exp = 16'h1A01;
            4: init_exp = 16'h1B00;  5: init_exp = 16'h1C00;
            default: init_exp = 16'hxxxx;
        endcase
    endfunction

    initial begin
        fs = $fopen("vectors/imu_stimulus.hex", "r");
        fe = $fopen("vectors/imu_expected.hex", "r");
        if (fs == 0 || fe == 0) $fatal(1, "cannot open vector files");
        for (int i = 0; i < NS; i++) begin
            n = $fscanf(fs, "%h %h %h %h %h\n", t16[0], t16[1], t16[2], t16[3], t16[4]);
            n = $fscanf(fe, "%h %h %h %h\n", t32[0], t32[1], t32[2], t32[3]);
            for (int j = 0; j < 5; j++) stim[i][j] = t16[j];
            for (int j = 0; j < 4; j++) expv[i][j] = t32[j];
        end
        $fclose(fs); $fclose(fe);
        load_sample(0);

        btn = 1;
        repeat (10) @(posedge clk);
        btn = 0;

        wait (bursts >= NS);
        #(BIT_NS * 100);

        if (imu.n_writes != 6) begin errors++; $display("expected 6 setup writes, saw %0d", imu.n_writes); end
        for (int i = 0; i < 6; i++)
            if (imu.wlog[i] != init_exp(i)) begin
                errors++; $display("setup write %0d = %h, expected %h", i, imu.wlog[i], init_exp(i));
            end
        if (!led[0]) begin errors++; $display("WHO_AM_I LED not lit (read %h)", dut.who_am_i); end
        if (!led[1]) begin errors++; $display("streaming LED not lit"); end
        if (!led[2]) begin errors++; $display("cal_done LED not lit"); end
        if ( led[7]) begin errors++; $display("I2C error LED lit"); end
        if (frames != NS / DECIM) begin errors++; $display("received %0d frames, expected %0d", frames, NS / DECIM); end

        $display("setup writes: %0d, WHO_AM_I: %h, samples: %0d, frames: %0d",
                 imu.n_writes, dut.who_am_i, bursts, frames);
        if (errors == 0) $display("LIVE TOP-LEVEL TEST PASS: I2C setup, %0d frames bit-exact over UART", frames);
        else             $display("LIVE TOP-LEVEL TEST FAIL: %0d errors", errors);
        $finish;
    end

    initial begin
        #(64'd600_000_000);
        $display("LIVE TOP-LEVEL TEST FAIL: timeout (bursts=%0d frames=%0d)", bursts, frames);
        $finish;
    end
endmodule
