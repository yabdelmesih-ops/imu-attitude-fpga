`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// mpu6050_i2c_reader.sv: configures an MPU-6050 over I2C, then burst-reads
// accel + gyro at a fixed rate. Same sample outputs as imu_spi_reader, so it
// drops into the same chain in front of attitude_core.
//
// Each register write is one transaction:
//   START, addr+W, register, value, STOP
// Each sample is one burst read of 14 bytes starting at ACCEL_XOUT_H (0x3B):
//   START, addr+W, 0x3B, repeated START, addr+R, 14 bytes (ACK all but the
//   last, NACK the last), STOP
//   bytes: AX AY AZ TEMP GX GY GZ, each high byte first
// At 350 kHz a burst takes about 0.45 ms, inside the 1 ms sample period.
//
// If the sensor does not ACK, the transaction is ended with STOP and
// nack_seen is set. During setup the whole setup is retried; while running,
// that sample is skipped.
// -----------------------------------------------------------------------------
module mpu6050_i2c_reader #(
    parameter int         CLK_HZ        = 100_000_000,
    parameter int         I2C_HZ        = 350_000,
    parameter int         SAMPLE_HZ     = 1000,
    parameter int         RESET_WAIT_MS = 100,
    parameter int         STEP_WAIT_US  = 1000,
    parameter logic [6:0] DEV_ADDR      = 7'h68       // AD0 low (left unconnected on the GY-521)
) (
    input  logic               clk,
    input  logic               rst,
    // I2C pins (open-drain)
    output logic               scl_oe,
    output logic               sda_oe,
    input  logic               sda_in,
    // samples
    output logic               sample_valid,      // 1-cycle pulse
    output logic signed [15:0] ax, ay, az,
    output logic signed [15:0] gx, gy, gz,
    output logic        [7:0]  who_am_i,
    output logic               running,           // setup finished, streaming
    output logic               nack_seen          // the sensor failed to ACK at least once
);
    localparam int RESET_WAIT = CLK_HZ / 1000 * RESET_WAIT_MS;
    localparam int STEP_WAIT  = CLK_HZ / 1_000_000 * STEP_WAIT_US;
    localparam int PERIOD     = CLK_HZ / SAMPLE_HZ;
    localparam int N_INIT     = 6;
    localparam int N_BURST    = 14;

    // {register, value} written during setup, in order. Step 0 resets the
    // chip and is followed by RESET_WAIT; the rest are followed by STEP_WAIT.
    function automatic logic [15:0] init_step(input logic [2:0] i);
        case (i)
            3'd0: init_step = {8'h6B, 8'h80};   // PWR_MGMT_1: device reset
            3'd1: init_step = {8'h6B, 8'h01};   // PWR_MGMT_1: wake up, clock = gyro PLL
            3'd2: init_step = {8'h19, 8'h00};   // SMPLRT_DIV: 1 kHz / (1+0)
            3'd3: init_step = {8'h1A, 8'h01};   // CONFIG: DLPF 184 Hz gyro / 188 Hz accel
            3'd4: init_step = {8'h1B, 8'h00};   // GYRO_CONFIG: +/-250 dps (131 LSB/dps)
            3'd5: init_step = {8'h1C, 8'h00};   // ACCEL_CONFIG: +/-2 g (16384 LSB/g)
            default: init_step = 16'h0000;
        endcase
    endfunction

    // ---- I2C byte engine ----
    localparam logic [1:0] CMD_START = 2'd0, CMD_WRITE = 2'd1, CMD_READ = 2'd2, CMD_STOP = 2'd3;
    logic       m_valid, m_ready, m_done, m_nack, m_ack;
    logic [1:0] m_cmd;
    logic [7:0] m_wdata, m_rdata;

    i2c_master #(.CLK_HZ(CLK_HZ), .I2C_HZ(I2C_HZ)) u_i2c (
        .clk(clk), .rst(rst),
        .cmd_valid(m_valid), .cmd(m_cmd), .wdata(m_wdata), .ack_out(m_ack),
        .ready(m_ready), .done(m_done), .rdata(m_rdata), .nack(m_nack),
        .scl_oe(scl_oe), .sda_oe(sda_oe), .sda_in(sda_in)
    );

    // ---- transaction sequencer ----
    // One transaction = a run of byte commands. "phase" says which part we are in;
    // each phase issues its command once, then waits for m_done.
    typedef enum logic [3:0] {
        X_IDLE, X_START, X_ADDR_W, X_REG, X_VALUE, X_RESTART, X_ADDR_R, X_READ, X_STOP
    } xphase_t;
    xphase_t    xph;
    logic       issued;          // command sent, waiting for m_done
    logic       x_read;          // this transaction reads (else writes x_value)
    logic [7:0] x_reg, x_value;
    logic [3:0] x_len, x_count;  // bytes to read / bytes read so far
    logic       x_failed;        // the device did not ACK somewhere
    logic       x_done;          // 1-cycle pulse: transaction finished
    logic [7:0] rbuf [N_BURST];

    // ---- top-level control ----
    typedef enum logic [2:0] {C_BOOT, C_INIT, C_WAIT, C_ID, C_RUN, C_BURST} cstate_t;
    cstate_t     cst;
    logic [31:0] timer, period_cnt;
    logic [2:0]  step;
    logic        x_go;           // 1-cycle pulse: start the transaction set up below
    wire  [15:0] cur_step = init_step(step);

    always_ff @(posedge clk) begin
        m_valid      <= 1'b0;
        x_done       <= 1'b0;
        x_go         <= 1'b0;
        sample_valid <= 1'b0;

        if (rst) begin
            xph        <= X_IDLE;
            issued     <= 1'b0;
            x_read     <= 1'b0;
            x_reg      <= '0;
            x_value    <= '0;
            x_len      <= '0;
            x_count    <= '0;
            x_failed   <= 1'b0;
            m_cmd      <= CMD_STOP;
            m_wdata    <= '0;
            m_ack      <= 1'b0;
            cst        <= C_BOOT;
            timer      <= '0;
            period_cnt <= '0;
            step       <= '0;
            who_am_i   <= '0;
            running    <= 1'b0;
            nack_seen  <= 1'b0;
            {ax, ay, az, gx, gy, gz} <= '0;
            for (int i = 0; i < N_BURST; i++) rbuf[i] <= '0;
        end else begin
            // ================= transaction sequencer =================
            if (xph == X_IDLE) begin
                if (x_go) begin
                    x_failed <= 1'b0;
                    x_count  <= '0;
                    xph      <= X_START;
                end
            end else if (!issued) begin
                if (m_ready) begin                       // issue this phase's command
                    issued <= 1'b1;
                    m_valid <= 1'b1;
                    case (xph)
                        X_START, X_RESTART: m_cmd <= CMD_START;
                        X_ADDR_W: begin m_cmd <= CMD_WRITE; m_wdata <= {DEV_ADDR, 1'b0}; end
                        X_REG:    begin m_cmd <= CMD_WRITE; m_wdata <= x_reg; end
                        X_VALUE:  begin m_cmd <= CMD_WRITE; m_wdata <= x_value; end
                        X_ADDR_R: begin m_cmd <= CMD_WRITE; m_wdata <= {DEV_ADDR, 1'b1}; end
                        X_READ:   begin m_cmd <= CMD_READ;  m_ack <= (x_count != x_len - 1); end
                        default:  m_cmd <= CMD_STOP;     // X_STOP
                    endcase
                end
            end else if (m_done) begin                   // command finished: pick the next phase
                issued <= 1'b0;
                case (xph)
                    X_START:   xph <= X_ADDR_W;
                    X_ADDR_W:  if (m_nack) begin x_failed <= 1'b1; xph <= X_STOP; end
                               else xph <= X_REG;
                    X_REG:     if (m_nack) begin x_failed <= 1'b1; xph <= X_STOP; end
                               else xph <= x_read ? X_RESTART : X_VALUE;
                    X_VALUE:   begin if (m_nack) x_failed <= 1'b1; xph <= X_STOP; end
                    X_RESTART: xph <= X_ADDR_R;
                    X_ADDR_R:  if (m_nack) begin x_failed <= 1'b1; xph <= X_STOP; end
                               else xph <= X_READ;
                    X_READ: begin
                        rbuf[x_count] <= m_rdata;
                        x_count       <= x_count + 1'b1;
                        if (x_count == x_len - 1) xph <= X_STOP;
                    end
                    default: begin                       // X_STOP
                        xph    <= X_IDLE;
                        x_done <= 1'b1;
                        if (x_failed) nack_seen <= 1'b1;
                    end
                endcase
            end

            // ================= top-level control =================
            period_cnt <= (period_cnt == PERIOD - 1) ? '0 : period_cnt + 1'b1;

            case (cst)
                C_BOOT: begin                            // short power-up delay, then setup
                    step    <= '0;
                    running <= 1'b0;
                    timer   <= STEP_WAIT;
                    cst     <= C_WAIT;
                end
                C_WAIT: if (timer != 0) timer <= timer - 1'b1;
                        else if (step == N_INIT) begin   // setup done: read WHO_AM_I
                            x_read <= 1'b1; x_reg <= 8'h75; x_len <= 4'd1;
                            x_go   <= 1'b1;
                            cst    <= C_ID;
                        end else begin                   // next setup write
                            x_read  <= 1'b0;
                            x_reg   <= cur_step[15:8];
                            x_value <= cur_step[7:0];
                            x_go    <= 1'b1;
                            cst     <= C_INIT;
                        end
                C_INIT: if (x_done) begin
                    if (x_failed) begin                  // no answer: start setup over later
                        step  <= '0;
                        timer <= RESET_WAIT;
                    end else begin
                        timer <= (step == 0) ? RESET_WAIT : STEP_WAIT;
                        step  <= step + 1'b1;
                    end
                    cst <= C_WAIT;
                end
                C_ID: if (x_done) begin
                    if (x_failed) begin
                        step <= '0; timer <= RESET_WAIT; cst <= C_WAIT;
                    end else begin
                        who_am_i <= rbuf[0];
                        running  <= 1'b1;
                        cst      <= C_RUN;
                    end
                end
                C_RUN: if (period_cnt == 0) begin        // start one burst per sample period
                    x_read <= 1'b1; x_reg <= 8'h3B; x_len <= 4'(N_BURST);
                    x_go   <= 1'b1;
                    cst    <= C_BURST;
                end
                C_BURST: if (x_done) begin
                    if (!x_failed) begin
                        ax <= {rbuf[0],  rbuf[1]};
                        ay <= {rbuf[2],  rbuf[3]};
                        az <= {rbuf[4],  rbuf[5]};
                        // rbuf[6], rbuf[7] = temperature (unused)
                        gx <= {rbuf[8],  rbuf[9]};
                        gy <= {rbuf[10], rbuf[11]};
                        gz <= {rbuf[12], rbuf[13]};
                        sample_valid <= 1'b1;
                    end
                    cst <= C_RUN;
                end
                default: cst <= C_BOOT;
            endcase
        end
    end
endmodule
