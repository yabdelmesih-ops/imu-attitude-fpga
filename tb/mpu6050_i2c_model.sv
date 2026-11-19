// Simulated MPU-6050 on an I2C bus (testbench only, not synthesizable).
//
// Behaves like the real sensor for register reads and writes:
//   write:  START, addr+W, reg, data, data, ..., STOP      (register pointer auto-increments)
//   read:   START, addr+W, reg, repeated START, addr+R, data, data, ..., NACK, STOP
// It ACKs only its own address, so a wrong address gets a NACK.
// For system tests it also logs every register write (wlog / n_writes) and
// fires `burst_done` at the STOP that ends a read, with `start_addr` = the
// first register read, so the testbench can load the next sensor sample.
//
// The bus lines are open-drain: this model only ever pulls SDA low or lets it
// go. The testbench supplies the pull-up resistors.
`timescale 1ns/1ps
module mpu6050_i2c_model #(
    parameter logic [6:0] ADDR     = 7'h68,
    parameter logic [7:0] WHO_AM_I = 8'h68     // clone chips may report something else
) (
    input wire scl,
    inout wire sda
);
    reg sda_low = 1'b0;
    assign sda = sda_low ? 1'b0 : 1'bz;

    reg [7:0] regs [0:127];
    integer i;
    initial begin
        for (i = 0; i < 128; i = i + 1) regs[i] = 8'h00;
        regs[7'h6B] = 8'h40;           // PWR_MGMT_1 reset value (sleep bit set)
        regs[7'h75] = WHO_AM_I;
    end

    localparam S_IDLE = 0, S_ADDR = 1, S_ADDR_ACK = 2, S_WRITE = 3, S_WRITE_ACK = 4,
               S_READ = 5, S_READ_ACK = 6;
    reg [2:0] st = S_IDLE;
    reg [3:0] bitcnt = 0;
    reg [7:0] shreg = 0, rdata = 0, ptr = 0;
    reg       rw = 0, first_byte = 0, master_nack = 0;

    // Counters and logs the testbench can check
    integer starts = 0, stops = 0, acks_given = 0;
    reg [15:0] wlog [0:63];
    integer    n_writes = 0;
    reg [7:0]  start_addr = 0;
    reg        was_read = 0;
    event      burst_done;

    // START or repeated START: SDA falls while SCL is high
    always @(negedge sda) if (scl === 1'b1) begin
        starts = starts + 1;
        st = S_ADDR; bitcnt = 0; sda_low = 1'b0; was_read = 0;
    end
    // STOP: SDA rises while SCL is high
    always @(posedge sda) if (scl === 1'b1) begin
        stops = stops + 1;
        st = S_IDLE; sda_low = 1'b0;
        if (was_read) -> burst_done;
        was_read = 0;
    end

    // Sample on SCL rising edges
    always @(posedge scl) begin
        case (st)
            S_ADDR, S_WRITE: begin shreg = {shreg[6:0], sda === 1'b1}; bitcnt = bitcnt + 1; end
            S_READ_ACK:      master_nack = (sda === 1'b1);
            default: ;
        endcase
    end

    // Change SDA only on SCL falling edges
    always @(negedge scl) begin
        case (st)
            S_ADDR: if (bitcnt == 8) begin
                if (shreg[7:1] == ADDR) begin
                    rw = shreg[0]; sda_low = 1'b1; acks_given = acks_given + 1;
                    st = S_ADDR_ACK;
                end else begin
                    st = S_IDLE;                 // not our address: stay off the bus
                end
            end
            S_ADDR_ACK: begin
                bitcnt = 0;
                if (rw) begin                    // read: put the first data bit on SDA
                    was_read = 1; start_addr = ptr;
                    rdata = regs[ptr[6:0]]; ptr = ptr + 1;
                    sda_low = ~rdata[7]; bitcnt = 1;
                    st = S_READ;
                end else begin
                    sda_low = 1'b0; first_byte = 1; st = S_WRITE;
                end
            end
            S_WRITE: if (bitcnt == 8) begin
                if (first_byte) ptr = shreg;     // first byte = register address
                else begin
                    if (n_writes < 64) wlog[n_writes] = {ptr, shreg};
                    n_writes = n_writes + 1;
                    regs[ptr[6:0]] = shreg; ptr = ptr + 1;
                end
                first_byte = 0;
                sda_low = 1'b1; acks_given = acks_given + 1;
                st = S_WRITE_ACK;
            end
            S_WRITE_ACK: begin sda_low = 1'b0; bitcnt = 0; st = S_WRITE; end
            S_READ: begin
                if (bitcnt == 8) begin           // release SDA for the master's ACK/NACK
                    sda_low = 1'b0; st = S_READ_ACK;
                end else begin
                    sda_low = ~rdata[7 - bitcnt]; bitcnt = bitcnt + 1;
                end
            end
            S_READ_ACK: begin
                if (master_nack) begin st = S_IDLE; sda_low = 1'b0; end
                else begin                       // ACK: send the next register
                    rdata = regs[ptr[6:0]]; ptr = ptr + 1;
                    sda_low = ~rdata[7]; bitcnt = 1; st = S_READ;
                end
            end
            default: ;
        endcase
    end
endmodule
