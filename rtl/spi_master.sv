`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// spi_master.sv — byte-wide SPI master, mode 3 (CPOL=1, CPHA=1), MSB first.
//
// Mode 3 is what the MPU-6500/9250 and ST LSM6DS-family IMUs accept.
// SCLK idles high; MOSI changes on the falling edge, MISO is sampled on the
// rising edge. Chip select is held low across bytes while `hold` is set at the
// time `start` is issued, so multi-byte bursts are one transaction.
//
// MISO comes from another device, so it passes through a 2-FF synchronizer
// before it is sampled.
// -----------------------------------------------------------------------------
module spi_master #(
    parameter int CLK_HZ  = 100_000_000,
    parameter int SCLK_HZ = 1_000_000
) (
    input  logic       clk,
    input  logic       rst,
    input  logic       start,     // begin one byte (ignored while busy)
    input  logic       hold,      // keep CS low after this byte
    input  logic [7:0] tx_byte,
    output logic [7:0] rx_byte,
    output logic       busy,
    output logic       done,      // 1-cycle pulse when rx_byte is valid
    output logic       sclk,
    output logic       mosi,
    input  logic       miso,
    output logic       cs_n
);
    localparam int HALF   = (CLK_HZ / SCLK_HZ) / 2;   // clocks per SCLK half period
    localparam int CS_GAP = HALF;                       // CS setup / hold / idle time

    typedef enum logic [2:0] {IDLE, CS_SETUP, LOW, HIGH, CS_HOLD, CS_IDLE} state_t;
    state_t state;

    logic [$clog2(HALF + 1):0] cnt;
    logic [2:0]  bitn;
    logic [7:0]  sh_tx, sh_rx;
    logic        hold_r;
    logic [1:0]  miso_sync;

    always_ff @(posedge clk) miso_sync <= {miso_sync[0], miso};

    always_ff @(posedge clk) begin
        done <= 1'b0;
        if (rst) begin
            state   <= IDLE;
            sclk    <= 1'b1;
            mosi    <= 1'b1;
            cs_n    <= 1'b1;
            cnt     <= '0;
            bitn    <= '0;
            sh_tx   <= '0;
            sh_rx   <= '0;
            hold_r  <= 1'b0;
            rx_byte <= '0;
        end else begin
            case (state)
                IDLE: if (start) begin
                    sh_tx  <= tx_byte;
                    hold_r <= hold;
                    bitn   <= 3'd7;
                    cnt    <= '0;
                    if (cs_n) begin                    // new transaction
                        cs_n  <= 1'b0;
                        state <= CS_SETUP;
                    end else begin                     // continuing a burst
                        state <= LOW;
                    end
                end
                CS_SETUP: if (cnt == CS_GAP - 1) begin
                    cnt   <= '0;
                    state <= LOW;
                end else cnt <= cnt + 1'b1;
                LOW: begin                             // falling edge: drive MOSI
                    if (cnt == 0) begin
                        sclk <= 1'b0;
                        mosi <= sh_tx[7];
                    end
                    if (cnt == HALF - 1) begin
                        cnt   <= '0;
                        state <= HIGH;
                    end else cnt <= cnt + 1'b1;
                end
                HIGH: begin                            // rising edge: sample MISO
                    if (cnt == 0) begin
                        sclk  <= 1'b1;
                        sh_tx <= {sh_tx[6:0], 1'b0};
                    end
                    // sample late in the high phase: slave has had a full half
                    // period plus synchronizer delay to settle MISO
                    if (cnt == HALF - 1) begin
                        sh_rx <= {sh_rx[6:0], miso_sync[1]};
                        cnt   <= '0;
                        if (bitn == 0) begin
                            state <= hold_r ? IDLE : CS_HOLD;
                            if (hold_r) begin
                                rx_byte <= {sh_rx[6:0], miso_sync[1]};
                                done    <= 1'b1;
                            end
                        end else begin
                            bitn  <= bitn - 1'b1;
                            state <= LOW;
                        end
                    end else cnt <= cnt + 1'b1;
                end
                CS_HOLD: if (cnt == CS_GAP - 1) begin
                    cnt     <= '0;
                    cs_n    <= 1'b1;
                    rx_byte <= sh_rx;
                    done    <= 1'b1;
                    state   <= CS_IDLE;
                end else cnt <= cnt + 1'b1;
                CS_IDLE: if (cnt == CS_GAP - 1) begin   // minimum CS-high time
                    cnt   <= '0;
                    state <= IDLE;
                end else cnt <= cnt + 1'b1;
                default: state <= IDLE;
            endcase
        end
    end

    assign busy = (state != IDLE);

endmodule
