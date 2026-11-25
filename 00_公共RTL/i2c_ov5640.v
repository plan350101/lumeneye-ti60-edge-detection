// ============================================================
//  i2c_ov5640.v
//  SCCB / I2C master that programs the OV5640 sensor.
//
//  Frame written per register:
//      DEV_ADDR(0x78) -> addr[15:8] -> addr[7:0] -> data
//
//  SCL = clk / (2*CLK_DIV).  With clk = 24 MHz and CLK_DIV = 120
//  the SCL runs at 100 kHz.
//
//  SDA is open-drain, but the tristate itself is NOT created here:
//  Titanium devices do not allow an inferred bidirectional net, so
//  this module only exports three plain single-bit signals
//
//      sda_o  : value to drive on SDA (only meaningful when sda_oe=1)
//      sda_oe : 1 = drive, 0 = release (Hi-Z, pull-up takes over)
//      sda_i  : value read back from the pad (registered inside the pad)
//
//  The three are bound to ONE bidirectional pad from the interface
//  script (camera_top.isf); the interface builder creates the
//  EFX_IO_BUF.  sda_i is used only to check the 9th (ACK) bit of
//  every byte, so a missing camera module or a wiring fault shows up
//  immediately on led[0].
// ============================================================

module i2c_ov5640 #(
    parameter        CLK_DIV  = 120,
    parameter        REG_NUM  = 374,
    parameter [23:0] PWR_WAIT = 24'd1_200_000   // ~50 ms @ 24 MHz
)(
    input  wire        clk,
    input  wire        rst_n,
    output reg         scl,
    input  wire        sda_i,      // read-back from the pad (ACK)
    output wire        sda_o,      // drive value
    output wire        sda_oe,     // output enable, 1 = drive
    output reg         cfg_done,
    output reg         ack_ok,     // sticky: 1 = every write was ACKed
    output reg         any_ack,    // sticky: 1 = at least one byte was ACKed
    output wire [9:0]  rom_addr,
    input  wire [23:0] rom_data,

    // ---- 诊断用的「额外单寄存器写」通道 ----
    // ROM 走完之后，cfg_done=1。此后只要 ext_req 拉高，就再写寄存器一次
    // (ext_addr, ext_data)，写完把 ext_done 拉高一拍、回到 S_DONE 待命。
    // 握手约定（顶层请照此实现，避免重入）：
    //   1) 顶层把 ext_req 拉高并保持；
    //   2) 本模块写完一拍后把 ext_done 拉高；
    //   3) 顶层看到 ext_done 后把 ext_req 拉低；
    //   4) ext_req 拉低期间本模块自动清 ext_done，重新武装。
    // 不做握手的后果：ext_req 一直高会**连续重写同一条寄存器**，虽然不会
    // 写坏别的东西，但会把 SCCB 总线占满、拖慢一切。
    input  wire        ext_req,
    input  wire [15:0] ext_addr,
    input  wire [7:0]  ext_data,
    output reg         ext_done
);

    localparam [7:0] DEV_ADDR = 8'h78;

    localparam [3:0] S_PWR   = 4'd0,
                     S_LOAD  = 4'd1,
                     S_START = 4'd2,
                     S_SEND  = 4'd3,
                     S_ACK   = 4'd4,
                     S_STOP  = 4'd5,
                     S_DONE  = 4'd6;

    reg [3:0]  st;
    reg [23:0] pwr_cnt;
    reg [15:0] div_cnt;
    reg        tick;

    reg [3:0]  bitc;
    reg [2:0]  bytec;
    reg [9:0]  regidx;
    reg [31:0] shreg;
    reg [1:0]  ph;

    reg        sda_oe_r;
    reg        sda_out_r;

    assign sda_oe   = sda_oe_r;
    assign sda_o    = sda_out_r;
    assign rom_addr = regidx;

    // ---------------- tick generator ----------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            div_cnt <= 16'd0;
            tick    <= 1'b0;
        end else if (div_cnt == CLK_DIV[15:0] - 16'd1) begin
            div_cnt <= 16'd0;
            tick    <= 1'b1;
        end else begin
            div_cnt <= div_cnt + 16'd1;
            tick    <= 1'b0;
        end
    end

    // ---------------- main FSM ----------------
    // Two ticks make one SCL cycle: ph=0 -> SCL low (SDA may change),
    // ph=1 -> SCL high (data is sampled by the sensor).
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st        <= S_PWR;
            scl       <= 1'b1;
            sda_oe_r  <= 1'b0;
            sda_out_r <= 1'b0;
            cfg_done  <= 1'b0;
            ack_ok    <= 1'b1;
            any_ack   <= 1'b0;
            bitc      <= 4'd0;
            bytec     <= 3'd0;
            regidx    <= 10'd0;
            shreg     <= 32'd0;
            ph        <= 2'd0;
            pwr_cnt   <= 24'd0;
            ext_done  <= 1'b0;
        end else begin
            case (st)

            // -------- wait for sensor power / XCLK to settle --------
            S_PWR: begin
                scl       <= 1'b1;
                sda_oe_r  <= 1'b0;
                sda_out_r <= 1'b0;
                if (pwr_cnt == PWR_WAIT) begin
                    pwr_cnt <= 24'd0;
                    regidx  <= 10'd0;
                    st      <= S_LOAD;
                end else begin
                    pwr_cnt <= pwr_cnt + 24'd1;
                end
            end

            // -------- fetch one register from the ROM --------
            S_LOAD: begin
                if (tick) begin
                    shreg <= {DEV_ADDR, rom_data[23:16], rom_data[15:8], rom_data[7:0]};
                    bytec <= 3'd0;
                    bitc  <= 4'd0;
                    ph    <= 2'd0;
                    st    <= S_START;
                end
            end

            // -------- START: SDA falls while SCL is high --------
            S_START: begin
                if (tick) begin
                    if (ph == 2'd0) begin
                        scl       <= 1'b1;
                        sda_oe_r  <= 1'b0;      // release -> SDA high
                        sda_out_r <= 1'b0;
                        ph        <= 2'd1;
                    end else begin
                        sda_oe_r  <= 1'b1;      // pull SDA low, SCL still high
                        sda_out_r <= 1'b0;
                        ph        <= 2'd0;
                        st        <= S_SEND;
                    end
                end
            end

            // -------- 8 data bits, MSB first --------
            S_SEND: begin
                if (tick) begin
                    if (ph == 2'd0) begin
                        scl       <= 1'b0;
                        sda_oe_r  <= ~shreg[31];   // bit=1 -> release, bit=0 -> drive low
                        sda_out_r <= 1'b0;
                        ph        <= 2'd1;
                    end else begin
                        scl <= 1'b1;
                        ph  <= 2'd0;
                        if (bitc == 4'd7) begin
                            bitc <= 4'd8;
                            st   <= S_ACK;
                        end else begin
                            bitc  <= bitc + 4'd1;
                            shreg <= {shreg[30:0], 1'b0};
                        end
                    end
                end
            end

            // -------- 9th clock: ACK slot --------
            //  ph=0 : SCL low, release SDA so the sensor can pull it low
            //  ph=1 : SCL high  (the sensor drives the ACK bit now)
            //  ph=2 : sample SDA -> 0 means ACK, 1 means NACK
            S_ACK: begin
                if (tick) begin
                    if (ph == 2'd0) begin
                        scl       <= 1'b0;
                        sda_oe_r  <= 1'b0;
                        sda_out_r <= 1'b0;
                        ph        <= 2'd1;
                    end else if (ph == 2'd1) begin
                        scl <= 1'b1;
                        ph  <= 2'd2;
                    end else begin
                        if (sda_i) ack_ok  <= 1'b0;   // NACK
                        else       any_ack <= 1'b1;   // 至少收到过一个 ACK
                        scl  <= 1'b0;
                        ph   <= 2'd0;
                        bitc <= 4'd0;
                        if (bytec == 3'd3) begin
                            st <= S_STOP;
                        end else begin
                            bytec <= bytec + 3'd1;
                            shreg <= {shreg[30:0], 1'b0};
                            st    <= S_SEND;
                        end
                    end
                end
            end

            // -------- STOP: SDA rises while SCL is high --------
            S_STOP: begin
                if (tick) begin
                    if (ph == 2'd0) begin
                        scl       <= 1'b0;
                        sda_oe_r  <= 1'b1;
                        sda_out_r <= 1'b0;
                        ph        <= 2'd1;
                    end else begin
                        scl       <= 1'b1;
                        sda_oe_r  <= 1'b0;      // release -> SDA rises
                        sda_out_r <= 1'b0;
                        ph        <= 2'd0;
                        // cfg_done=1 说明 ROM 已经走完，这一帧属于「额外寄存器写」，
                        // 回到 S_DONE 待命即可，千万不要再去动 regidx。
                        if (cfg_done) begin
                            ext_done <= 1'b1;
                            st       <= S_DONE;
                        end else if (regidx == REG_NUM - 1) begin
                            st <= S_DONE;
                        end else begin
                            regidx <= regidx + 10'd1;
                            st     <= S_LOAD;
                        end
                    end
                end
            end

            // -------- done --------
            // cfg_done 一旦拉高就一直是 1；此后这个状态兼作「待命」。
            // ext_req 拉高且还没做过（!ext_done）时，装一条新帧直接发出去。
            S_DONE: begin
                cfg_done  <= 1'b1;
                scl       <= 1'b1;
                sda_oe_r  <= 1'b0;
                sda_out_r <= 1'b0;
                if (ext_req) begin
                    if (!ext_done) begin
                        shreg <= {DEV_ADDR, ext_addr, ext_data};
                        bytec <= 3'd0;
                        bitc  <= 4'd0;
                        ph    <= 2'd0;
                        st    <= S_START;
                    end
                end else begin
                    ext_done <= 1'b0;      // req 撤销 -> 重新武装
                end
            end

            default: st <= S_PWR;
            endcase
        end
    end

endmodule
