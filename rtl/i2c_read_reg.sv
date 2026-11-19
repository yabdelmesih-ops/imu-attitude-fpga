`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// i2c_read_reg.sv: read one 8-bit register from an I2C device.
//
// Transaction (each byte is 8 bits, MSB first, followed by 1 ACK bit):
//
//   START | dev_addr,0 (write) | ACK | reg_addr | ACK |
//   repeated START | dev_addr,1 (read) | ACK | data from device | NACK | STOP
//
// In words: "device 0x68, I'm writing: register 0x75. Now device 0x68, I'm
// reading: send me one byte, and that's all."
//
// Open-drain bus: this module never drives a 1. *_oe = 1 pulls the line low,
// *_oe = 0 lets go and the pull-up resistor makes it 1.
//
// Timing: every SCL period is split into 4 quarters ("ticks"):
//   quarter 0: SCL low   -> put the next bit on SDA
//   quarter 1: SCL rises
//   quarter 2: SCL high  -> the receiver reads SDA (we read here when receiving)
//   quarter 3: SCL falls
// START and STOP are the only times SDA changes while SCL is high.
// -----------------------------------------------------------------------------
module i2c_read_reg #(
    parameter int CLK_HZ = 100_000_000,
    parameter int I2C_HZ = 100_000          // 100 kHz standard mode for bring-up
) (
    input  logic       clk,
    input  logic       rst,
    input  logic       start,               // 1-cycle pulse: begin one read
    input  logic [6:0] dev_addr,            // 7'h68 for the MPU-6050
    input  logic [7:0] reg_addr,            // 8'h75 = WHO_AM_I
    output logic       busy,
    output logic       done,                // 1-cycle pulse when finished
    output logic       ack_error,           // device did not ACK (valid with done)
    output logic [7:0] data,                // byte read (valid with done)
    output logic       scl_oe,              // 1 = pull SCL low
    output logic       sda_oe,              // 1 = pull SDA low
    input  logic       sda_in               // SDA as seen on the bus
);
    // ------------------------------------------------------------------
    // Tick generator: one pulse every quarter of an SCL period.
    // 100 MHz / (100 kHz * 4) = 250 clock cycles per quarter.
    // ------------------------------------------------------------------
    localparam int QUARTER = CLK_HZ / (I2C_HZ * 4);

    logic [$clog2(QUARTER)-1:0] tick_count;
    logic                       tick;

    always_ff @(posedge clk) begin
        tick <= 1'b0;
        if (rst || !busy)                       // only count during a transaction
            tick_count <= '0;
        else if (tick_count == QUARTER - 1) begin
            tick_count <= '0;
            tick       <= 1'b1;
        end else
            tick_count <= tick_count + 1'b1;
    end

    // ------------------------------------------------------------------
    // State machine. Every state except IDLE and FINISH lasts 4 ticks
    // (one SCL period); SEND_BYTE and READ_BYTE repeat for 8 bits.
    // ------------------------------------------------------------------
    typedef enum logic [3:0] {
        IDLE,        // waiting for start
        START,       // SDA falls while SCL is high
        SEND_BYTE,   // send 8 bits from shift_reg, MSB first
        GET_ACK,     // release SDA, check the device pulls it low
        RESTART,     // repeated START before switching to read
        READ_BYTE,   // read 8 bits from the device
        SEND_NACK,   // leave SDA high: "no more bytes"
        STOP,        // SDA rises while SCL is high
        FINISH       // report the result
    } state_t;

    state_t     state;
    logic [1:0] quarter;      // which quarter of the current bit (0-3)
    logic [2:0] bit_index;    // bits left to send/receive (7 down to 0)
    logic [7:0] shift_reg;    // byte being sent
    logic [1:0] byte_num;     // which byte we just sent: 0 = addr+W, 1 = register, 2 = addr+R

    always_ff @(posedge clk) begin
        done <= 1'b0;                           // default: done is a 1-cycle pulse

        if (rst) begin
            state     <= IDLE;
            busy      <= 1'b0;
            ack_error <= 1'b0;
            data      <= '0;
            scl_oe    <= 1'b0;                  // bus idle: both lines released (high)
            sda_oe    <= 1'b0;
            quarter   <= '0;
            bit_index <= '0;
            shift_reg <= '0;
            byte_num  <= '0;

        end else if (state == IDLE) begin
            if (start) begin
                busy      <= 1'b1;
                ack_error <= 1'b0;
                quarter   <= '0;
                state     <= START;
            end

        end else if (state == FINISH) begin
            if (tick) begin
                busy  <= 1'b0;
                done  <= 1'b1;
                state <= IDLE;
            end

        end else if (tick) begin
            quarter <= quarter + 1'b1;          // 0,1,2,3,0,... (2 bits wrap by themselves)

            case (state)
                // START: both lines are high; pull SDA low while SCL stays high,
                // then pull SCL low. Load the first byte: address + write bit (0).
                START: case (quarter)
                    2'd0: begin scl_oe <= 1'b0; sda_oe <= 1'b0; end
                    2'd1: sda_oe <= 1'b1;                       // the START condition
                    2'd2: ;
                    2'd3: begin
                        scl_oe    <= 1'b1;
                        shift_reg <= {dev_addr, 1'b0};
                        byte_num  <= 2'd0;
                        bit_index <= 3'd7;
                        state     <= SEND_BYTE;
                    end
                endcase

                // SEND_BYTE: put the top bit of shift_reg on SDA while SCL is low,
                // pulse SCL, then shift to the next bit. After 8 bits, check ACK.
                SEND_BYTE: case (quarter)
                    2'd0: sda_oe <= ~shift_reg[7];              // bit 0 -> pull low, bit 1 -> let go
                    2'd1: scl_oe <= 1'b0;
                    2'd2: ;                                     // device reads SDA here
                    2'd3: begin
                        scl_oe    <= 1'b1;
                        shift_reg <= {shift_reg[6:0], 1'b0};
                        if (bit_index == 0) state <= GET_ACK;
                        else                bit_index <= bit_index - 1'b1;
                    end
                endcase

                // GET_ACK: let go of SDA; the device pulls it low to acknowledge.
                // Then decide what comes next based on which byte this was.
                GET_ACK: case (quarter)
                    2'd0: sda_oe <= 1'b0;
                    2'd1: scl_oe <= 1'b0;
                    2'd2: if (sda_in) ack_error <= 1'b1;        // still high = NACK
                    2'd3: begin
                        scl_oe <= 1'b1;
                        if (ack_error)
                            state <= STOP;                      // nobody answered: give up
                        else if (byte_num == 2'd0) begin        // address done -> send register
                            shift_reg <= reg_addr;
                            byte_num  <= 2'd1;
                            bit_index <= 3'd7;
                            state     <= SEND_BYTE;
                        end else if (byte_num == 2'd1)          // register done -> restart
                            state <= RESTART;
                        else begin                              // addr+R done -> read data
                            bit_index <= 3'd7;
                            state     <= READ_BYTE;
                        end
                    end
                endcase

                // RESTART: release SDA (SCL low), raise SCL, then pull SDA low
                // while SCL is high: a new START. Load address + read bit (1).
                RESTART: case (quarter)
                    2'd0: sda_oe <= 1'b0;
                    2'd1: scl_oe <= 1'b0;
                    2'd2: sda_oe <= 1'b1;                       // the repeated START
                    2'd3: begin
                        scl_oe    <= 1'b1;
                        shift_reg <= {dev_addr, 1'b1};
                        byte_num  <= 2'd2;
                        bit_index <= 3'd7;
                        state     <= SEND_BYTE;
                    end
                endcase

                // READ_BYTE: let go of SDA so the device can drive it, and read
                // one bit each time SCL is high. Shift in from the right (MSB first).
                READ_BYTE: case (quarter)
                    2'd0: sda_oe <= 1'b0;
                    2'd1: scl_oe <= 1'b0;
                    2'd2: data   <= {data[6:0], sda_in};
                    2'd3: begin
                        scl_oe <= 1'b1;
                        if (bit_index == 0) state <= SEND_NACK;
                        else                bit_index <= bit_index - 1'b1;
                    end
                endcase

                // SEND_NACK: leave SDA high for the 9th clock: "that's all I want".
                SEND_NACK: case (quarter)
                    2'd0: sda_oe <= 1'b0;
                    2'd1: scl_oe <= 1'b0;
                    2'd2: ;
                    2'd3: begin scl_oe <= 1'b1; state <= STOP; end
                endcase

                // STOP: pull SDA low while SCL is low, raise SCL, then let SDA rise
                // while SCL is high. Both lines end released (bus idle).
                STOP: case (quarter)
                    2'd0: sda_oe <= 1'b1;
                    2'd1: scl_oe <= 1'b0;
                    2'd2: sda_oe <= 1'b0;                       // the STOP condition
                    2'd3: state  <= FINISH;
                endcase

                default: state <= IDLE;
            endcase
        end
    end
endmodule
