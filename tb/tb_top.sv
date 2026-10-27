// End-to-end integration test of the full board top level.
//
//   stimulus file -> MPU-6500 SPI model -> [ FPGA top ] -> UART -> frame decoder
//
// Checks: the IMU init register writes, WHO_AM_I readback, status LEDs, every
// telemetry frame (header, checksum, sequence number, cal flag) and that each
// frame's roll/pitch equal the bit-exact model's output for that sample.
//
// Default DUT: top_alchitry_au at 100 MHz. Compile with -DTANG9K to test
// top_tang_nano_9k at 27 MHz, or -DBASYS3 for top_basys3 at 100 MHz.
// Timing is scaled up (faster SCLK, 20 kHz samples, Mbaud UART) to keep the
// simulation short; the logic is identical to the 1 kHz / 115200 build.
`timescale 1ns/1ps
module tb_top;
    localparam int NS     = 2400;       // samples to run (> 1024 for gyro cal)
    localparam int DECIM  = 4;
`ifdef TANG9K
    localparam int CLK_HZ  = 27_000_000;
    localparam int SCLK_HZ = 4_500_000;
    localparam int BAUD    = 1_000_000;  // 27 clocks per bit, exact
    localparam real HALF_PERIOD_NS = 18.5185;
`else
    localparam int CLK_HZ  = 100_000_000;
    localparam int SCLK_HZ = 10_000_000;
    localparam int BAUD    = 5_000_000;
    localparam real HALF_PERIOD_NS = 5.0;
`endif
    localparam int BIT_NS = 1_000_000_000 / BAUD;

    logic clk = 0, btn = 0;
    logic sclk, mosi, miso, cs_n, txd;
    always #(HALF_PERIOD_NS) clk = ~clk;

`ifdef BASYS3
    logic [15:0] led;
    wire btn_rst = btn;                  // active high
    top_basys3 #(
        .CLK_HZ(CLK_HZ), .SCLK_HZ(SCLK_HZ), .SAMPLE_HZ(20_000),
        .BAUD(BAUD), .DECIM(DECIM), .RESET_WAIT_MS(1), .STEP_WAIT_US(10)
    ) dut (
        .clk(clk), .btn_rst(btn_rst), .imu_sclk(sclk), .imu_mosi(mosi),
        .imu_miso(miso), .imu_cs_n(cs_n), .uart_txd(txd), .led(led)
    );
    wire       led_id_ok  = (led[7:0] == 8'h70);
    wire       led_cal    = led[8];
    wire       led_run    = led[9];
`elsif TANG9K
    logic [5:0] led;
    wire btn_s1_n = ~btn;                // active low
    top_tang_nano_9k #(
        .CLK_HZ(CLK_HZ), .SCLK_HZ(SCLK_HZ), .SAMPLE_HZ(20_000),
        .BAUD(BAUD), .DECIM(DECIM), .RESET_WAIT_MS(1), .STEP_WAIT_US(10)
    ) dut (
        .clk_27m(clk), .btn_s1_n(btn_s1_n), .imu_sclk(sclk), .imu_mosi(mosi),
        .imu_miso(miso), .imu_cs_n(cs_n), .uart_txd(txd), .led(led)
    );
    wire       led_id_ok  = ~led[0];     // LEDs are active low
    wire       led_run    = ~led[1];
    wire       led_cal    = ~led[2];
`else
    logic [7:0] led;
    wire rst_n = ~btn;                   // active low
    top_alchitry_au #(
        .CLK_HZ(CLK_HZ), .SCLK_HZ(SCLK_HZ), .SAMPLE_HZ(20_000),
        .BAUD(BAUD), .DECIM(DECIM), .RESET_WAIT_MS(1), .STEP_WAIT_US(10)
    ) dut (
        .clk(clk), .rst_n(rst_n), .imu_sclk(sclk), .imu_mosi(mosi),
        .imu_miso(miso), .imu_cs_n(cs_n), .usb_tx(txd), .led(led)
    );
    wire       led_id_ok  = led[0];      // LEDs are active high
    wire       led_run    = led[1];
    wire       led_cal    = led[2];
`endif

    mpu_spi_model imu (.sclk(sclk), .mosi(mosi), .miso(miso), .cs_n(cs_n));

    // ---------------- stimulus & expected data ----------------
    logic [15:0] stim [NS][5];
    logic [31:0] expv [NS][4];
    integer fs, fe, n, errors = 0;
    logic [15:0] t16 [5];
    logic [31:0] t32 [4];

    task automatic load_sample(input integer i);
        logic [15:0] v [6];
        v[0] = stim[i][0]; v[1] = stim[i][1]; v[2] = stim[i][2];
        v[3] = stim[i][3]; v[4] = stim[i][4]; v[5] = 16'h0000;   // gz unused
        // 0x3B.. AX AY AZ, 0x41 TEMP, 0x43.. GX GY GZ, big-endian
        imu.regs[7'h3B] = v[0][15:8]; imu.regs[7'h3C] = v[0][7:0];
        imu.regs[7'h3D] = v[1][15:8]; imu.regs[7'h3E] = v[1][7:0];
        imu.regs[7'h3F] = v[2][15:8]; imu.regs[7'h40] = v[2][7:0];
        imu.regs[7'h41] = 8'h12;      imu.regs[7'h42] = 8'h34;
        imu.regs[7'h43] = v[3][15:8]; imu.regs[7'h44] = v[3][7:0];
        imu.regs[7'h45] = v[4][15:8]; imu.regs[7'h46] = v[4][7:0];
        imu.regs[7'h47] = v[5][15:8]; imu.regs[7'h48] = v[5][7:0];
    endtask

    integer bursts = 0;
    always @(imu.burst_done) begin
        if (imu.start_addr == 7'h3B) begin
            bursts++;
            if (bursts < NS) load_sample(bursts);
        end
    end

    // ---------------- UART receiver + frame checker ----------------
    logic [7:0] rx [8];
    integer frames = 0, sidx;
    logic [7:0] chk;

    task automatic uart_rx_byte(output logic [7:0] b);
        @(negedge txd);                       // start bit
        #(BIT_NS / 2);
        for (int i = 0; i < 8; i++) begin
            #(BIT_NS);
            b[i] = txd;
        end
        #(BIT_NS);                            // stop bit
        if (txd !== 1'b1) begin
            errors++;
            $display("UART framing error");
        end
    endtask

    initial begin : rx_proc
        logic [7:0] b;
        forever begin
            uart_rx_byte(b);
            while (b != 8'hA5) uart_rx_byte(b);      // hunt for header
            uart_rx_byte(b);
            if (b != 8'h5A) begin
                errors++; $display("frame %0d: bad second header byte %h", frames, b);
            end
            for (int i = 2; i < 8; i++) uart_rx_byte(rx[i]);
            chk  = rx[2] ^ rx[3] ^ rx[4] ^ rx[5] ^ rx[6];
            sidx = frames * DECIM + DECIM - 1;      // sample this frame carries
            if (chk != rx[7]) begin
                errors++; $display("frame %0d: bad checksum", frames);
            end
            if (rx[6][7:2] != 6'(frames & 63)) begin
                errors++; $display("frame %0d: bad seq %0d", frames, rx[6][7:2]);
            end
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
            2: init_exp = 16'h6A10;  3: init_exp = 16'h1900;
            4: init_exp = 16'h1A01;  5: init_exp = 16'h1B00;
            6: init_exp = 16'h1C00;  7: init_exp = 16'h1D01;
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

        btn = 1;                              // press reset
        repeat (10) @(posedge clk);
        btn = 0;

        wait (bursts >= NS);
        #(BIT_NS * 100);                      // last frame is 80 bits; next one is 4 samples away

        // init sequence
        if (imu.n_writes != 8) begin
            errors++; $display("expected 8 init writes, saw %0d", imu.n_writes);
        end
        for (int i = 0; i < 8; i++)
            if (imu.wlog[i] != init_exp(i)) begin
                errors++; $display("init write %0d = %h, expected %h", i, imu.wlog[i], init_exp(i));
            end
        if (!led_id_ok) begin errors++; $display("IMU-detected LED not lit (WHO_AM_I = %h)", dut.u_sys.who_am_i); end
        if (!led_run)   begin errors++; $display("streaming LED not lit"); end
        if (!led_cal)   begin errors++; $display("cal_done LED not lit"); end
        if (frames != NS / DECIM) begin
            errors++; $display("received %0d frames, expected %0d", frames, NS / DECIM);
        end

        $display("init writes: %0d, WHO_AM_I: %h, samples: %0d, frames: %0d", imu.n_writes, dut.u_sys.who_am_i, bursts, frames);
        if (errors == 0) $display("TOP-LEVEL TEST PASS: SPI init, %0d frames bit-exact over UART", frames);
        else             $display("TOP-LEVEL TEST FAIL: %0d errors", errors);
        $finish;
    end

    initial begin
        #(64'd600_000_000);                    // 600 ms watchdog
        $display("TOP-LEVEL TEST FAIL: timeout (bursts=%0d frames=%0d)", bursts, frames);
        $finish;
    end
endmodule
